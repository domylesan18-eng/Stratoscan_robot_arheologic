import argparse
import cv2
import numpy as np
import time
import csv
import glob
import os
import shutil
import sys
import open3d as o3d
from record3d import Record3DStream
from ultralytics import YOLO


NUME_FEREASTRA_VIDEO = "Scanare Continua Sit - Apasa ESC la final"


def _cale_model_yolo():
    """Calea catre yolov8n.pt - langa script in modul normal de dezvoltare,
    sau in folderul de resurse PyInstaller (langa executabil, in interiorul
    _internal\\) cand aplicatia ruleaza ca executabil impachetat."""
    nume = "yolov8n.pt"
    baza_resurse = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    cale = os.path.join(baza_resurse, nume)
    return cale if os.path.exists(cale) else nume


class ContinuousArchaeoScanner:
    def __init__(self):
        print("═" * 56)
        print("  SCANNER 3D SIT ARHEOLOGIC   ·   iPhone LiDAR + YOLO")
        print("═" * 56)

        # Fiecare scanare isi are propriul folder: scanari\<data_ora>\
        # (structurile vechi - fisiere in radacina, 'scanari_anterioare' - sunt
        # mutate automat in noua structura la prima pornire)
        self.director_scanari = "scanari"
        self.migreaza_structura_veche()

        stamp = time.strftime("%Y-%m-%d_%H-%M")
        director = os.path.join(self.director_scanari, stamp)
        sufix = 1
        while os.path.exists(director):
            sufix += 1
            director = os.path.join(self.director_scanari, f"{stamp}_{sufix}")
        self.director_sesiune = director
        os.makedirs(self.director_sesiune)
        print(f"🗂️ Scanarea aceasta se salveaza in: {self.director_sesiune}\\")

        self.ply_output_path = os.path.join(self.director_sesiune, "harta_completa_sit.ply")
        self.mesh_ply_path = os.path.join(self.director_sesiune, "harta_sit_mesh.ply")
        self.mesh_glb_path = os.path.join(self.director_sesiune, "harta_sit_mesh.glb")
        self.ply_artefacte_path = os.path.join(self.director_sesiune, "artefacte_gasite.ply")
        self.csv_path = os.path.join(self.director_sesiune, "coordonate_sit.csv")

        # Prag de incredere minim pentru a considera o detectie valida
        self.conf_minima = 0.45
        # Ignoram punctele LiDAR mai indepartate de atat (cm) - fundalul din jurul
        # gropii doar adauga zgomot pe harta
        self.dist_max_cm = 350.0

        # --- Reconstructie TSDF (suprafata continua, nu puncte razlete) ---
        # Fiecare cadru LiDAR e fuzionat intr-un volum de voxeli; la final se
        # extrage o suprafata neteda (mesh), exportata ca .ply si .glb.
        self.tsdf_voxel_m = 0.01   # rezolutia suprafetei: 1 cm
        self.tsdf_trunc_m = 0.04   # marginea de fuziune (4 cm) - netezeste zgomotul LiDAR
        # Distanta minima (cm) intre doua detectii ca sa fie considerate artefacte DIFERITE
        self.distanta_duplicat_cm = 15.0

        # --- Poze pentru registrul sapaturii ---
        # La fiecare artefact nou (sau cand e vazut mai clar) se salveaza un decupaj
        # din imagine in acest folder: artefact_001_vase.jpg etc.
        self.director_poze = os.path.join(self.director_sesiune, "poze_artefacte")
        self.margine_poza_px = 25  # margine in jurul chenarului la decupare

        # --- Salvare automata periodica (protectie daca scriptul/telefonul crapa) ---
        self.autosave_interval_s = 120.0
        self.autosave_path = os.path.join(self.director_sesiune, "harta_completa_sit_autosave.ply")
        self.ultimul_autosave = time.time()

        # --- Mini-harta de acoperire (vedere de sus, plan X-Z) ---
        self.minimapa_px = 200          # latura maxima a mini-hartii, in pixeli
        self.minimapa_celula_cm = 5.0   # o celula de acoperire = 5x5 cm de teren
        self.celule_acoperite = set()

        # --- Praguri pentru detectarea pierderii de tracking (salturi ARKit) ---
        # Peste sol uniform, fara textura, ARKit poate pierde urmarirea si "teleporteaza"
        # camera. Cand detectam asta, pornim un SEGMENT NOU de harta in loc sa lipim
        # gresit punctele de cele vechi.
        self.prag_salt_cm = 40.0          # deplasare brusca intre doua cadre consecutive
        self.prag_salt_rotatie_grade = 45.0  # rotatie brusca intre doua cadre consecutive
        self.prag_pauza_s = 2.0           # pauza lunga intre cadre (stream blocat / relocalizare)

        # --- Parametri pentru alinierea automata (ICP) a segmentelor la final ---
        self.icp_voxel_cm = 2.0           # rezolutia la care se face alinierea
        self.icp_fitness_minim = 0.30     # sub acest scor, segmentul ramane nealiniat (cu avertisment)

        # Registrul artefactelor confirmate: fiecare intrare e un dict
        # {id, incredere, nr_detectii, x, y, z, segment, poza}
        # (deocamdata fara clasificare pe tip - doar numerotate 1..n)
        # segment = -1 inseamna "deja in sistemul de coordonate al hartii finale"
        self.artefacte_inregistrate = []
        self.artefact_counter = 0

        os.makedirs(self.director_poze, exist_ok=True)

        self.csv_file = open(self.csv_path, mode='w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.rescrie_csv()

        print("Incarc modelul AI YOLOv8...")
        self.yolo_model = YOLO(_cale_model_yolo())

        self.rgb_frame = None
        self.depth_frame = None
        self.intrinsics = None
        self.camera_pose = None
        self.is_new_frame = False

        # Harta de teren e tinuta pe SEGMENTE: un segment nou incepe automat
        # cand detectam o pierdere de tracking. La final, segmentele se aliniaza
        # intre ele cu ICP si se unesc intr-o singura harta.
        self.segmente = [self.segment_nou()]
        self.pozitie_anterioara = None
        self.quat_anterior = None
        self.timp_cadru_anterior = None

        # --- stare pentru interfata ferestrei video (HUD) ---
        self.fps_curent = 0.0
        self.timp_fps_anterior = None
        self.timp_pornire = None
        self.timp_segment_nou = None   # cand s-a pierdut ultima data trackingul

        # Stream-ul (USB sau Wi-Fi direct) e creat mai tarziu, in start_scanning() -
        # depinde de argumentul --wifi (adresa IP) primit la pornire.
        self.stream = None

    def segment_nou(self):
        """Un segment = un volum TSDF propriu. La pierderea trackingului pornim un
        volum nou, iar la final segmentele se aliniaza intre ele cu ICP."""
        return {
            'volum': o3d.pipelines.integration.ScalableTSDFVolume(
                voxel_length=self.tsdf_voxel_m,
                sdf_trunc=self.tsdf_trunc_m,
                color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8),
            'nr_cadre': 0,
        }

    def migreaza_structura_veche(self):
        """Aduce totul la structura noua 'scanari\\<data>': muta folderele din
        vechiul 'scanari_anterioare' si fisierele ramase in radacina proiectului
        (formatul de dinainte, cand scanarea curenta statea langa scripturi)."""
        os.makedirs(self.director_scanari, exist_ok=True)

        # 1) folderele din vechiul 'scanari_anterioare' -> 'scanari'
        vechi = "scanari_anterioare"
        if os.path.isdir(vechi):
            for d in os.listdir(vechi):
                sursa = os.path.join(vechi, d)
                destinatie = os.path.join(self.director_scanari, d)
                if os.path.isdir(sursa) and not os.path.exists(destinatie):
                    shutil.move(sursa, destinatie)
            try:
                os.rmdir(vechi)
                print(f"🗂️ Am mutat arhivele din '{vechi}' in '{self.director_scanari}\\'.")
            except OSError:
                pass  # a ramas ceva nemutat inauntru - nu fortam

        # 2) fisierele vechi din radacina -> un folder datat in 'scanari'
        fisiere = [f for f in ("harta_completa_sit.ply", "harta_sit_mesh.ply",
                               "harta_sit_mesh.glb", "artefacte_gasite.ply",
                               "coordonate_sit.csv", "harta_completa_sit_autosave.ply")
                   if os.path.exists(f)]
        fisiere += glob.glob("raport_sit_*.html")
        poze_exista = os.path.isdir("poze_artefacte") and os.listdir("poze_artefacte")
        if not fisiere and not poze_exista:
            return

        # Numele folderului: momentul ultimei modificari din scanarea veche
        timpuri = [os.path.getmtime(f) for f in fisiere]
        if poze_exista:
            timpuri.append(os.path.getmtime("poze_artefacte"))
        stamp = time.strftime("%Y-%m-%d_%H-%M", time.localtime(max(timpuri)))

        destinatie = os.path.join(self.director_scanari, stamp)
        sufix = 1
        while os.path.exists(destinatie):
            sufix += 1
            destinatie = os.path.join(self.director_scanari, f"{stamp}_{sufix}")
        os.makedirs(destinatie)

        for f in fisiere:
            shutil.move(f, destinatie)
        if poze_exista:
            shutil.move("poze_artefacte", os.path.join(destinatie, "poze_artefacte"))
        elif os.path.isdir("poze_artefacte"):
            try:
                os.rmdir("poze_artefacte")
            except OSError:
                pass

        print(f"🗂️ Scanarea veche din folderul principal a fost mutata in '{destinatie}'.")

    def process_frame_callback(self):
        self.rgb_frame = self.stream.get_rgb_frame()
        self.depth_frame = self.stream.get_depth_frame()
        self.intrinsics = self.stream.get_intrinsic_mat()
        self.camera_pose = self.stream.get_camera_pose()
        self.is_new_frame = True

    @staticmethod
    def quat_to_rotmat(qx, qy, qz, qw):
        """Converteste quaternionul de rotatie al telefonului (din ARKit/Record3D)
        intr-o matrice de rotatie 3x3."""
        return np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)]
        ])

    @staticmethod
    def intrinsici_pentru_depth(intrinsics, w_depth, h_depth, w_rgb, h_rgb):
        """Intrinsecii camerei vin la rezolutia cadrului RGB; aici ii scalam la
        rezolutia hartii de adancime. IMPORTANT: fx/fy trebuie scalate cu acelasi
        factor ca cx/cy, altfel geometria iese comprimata lateral si harta devine
        o dara alungita si incetosata."""
        sx = w_depth / float(w_rgb)
        sy = h_depth / float(h_rgb)
        return intrinsics.fx * sx, intrinsics.fy * sy, intrinsics.tx * sx, intrinsics.ty * sy

    # ------------------------------------------------------------------
    #  DETECTAREA PIERDERII DE TRACKING → SEGMENT NOU
    # ------------------------------------------------------------------
    def verifica_pierdere_tracking(self, pose):
        """Compara poza curenta cu cea anterioara. Daca telefonul a 'sarit' brusc
        (pozitie sau rotatie) ori a fost o pauza lunga intre cadre, porneste un
        segment nou de harta in loc sa lipeasca gresit punctele."""
        acum = time.time()
        try:
            t = np.array([pose.tx, pose.ty, pose.tz]) * 100.0
            q = np.array([pose.qx, pose.qy, pose.qz, pose.qw])
        except AttributeError:
            return

        motiv = None
        if self.timp_cadru_anterior is not None:
            pauza = acum - self.timp_cadru_anterior
            if pauza > self.prag_pauza_s:
                motiv = f"pauza de {pauza:.1f}s intre cadre"

        if motiv is None and self.pozitie_anterioara is not None:
            salt = float(np.linalg.norm(t - self.pozitie_anterioara))
            if salt > self.prag_salt_cm:
                motiv = f"salt de pozitie de {salt:.0f} cm"

        if motiv is None and self.quat_anterior is not None:
            # unghiul dintre cele doua orientari: cos(theta/2) = |q1·q2|
            dot = min(1.0, abs(float(np.dot(q, self.quat_anterior))))
            unghi = 2.0 * np.degrees(np.arccos(dot))
            if unghi > self.prag_salt_rotatie_grade:
                motiv = f"salt de rotatie de {unghi:.0f}°"

        self.timp_cadru_anterior = acum
        self.pozitie_anterioara = t
        self.quat_anterior = q

        if motiv is not None and self.segmente[-1]['nr_cadre'] > 0:
            self.segmente.append(self.segment_nou())
            self.timp_segment_nou = acum
            print(f"⚠️ Tracking pierdut ({motiv}) → pornesc segmentul de harta #{len(self.segmente)}.")
            print("   Tine telefonul nemiscat cateva secunde deasupra unei zone cu textura, apoi continua.")

    # ------------------------------------------------------------------
    #  COORDONATE GLOBALE
    # ------------------------------------------------------------------
    def calculeaza_coordonate_globale(self, u, v, z_val, intrinsics, pose, w_depth, h_depth, w_rgb, h_rgb):
        """Transforma un pixel (u, v) + adancime intr-o coordonata globala a sitului.
        IMPORTANT: aplica si rotatia telefonului (nu doar translatia), altfel orice
        inclinare a mainii deformeaza harta (banda stramba din poza vine exact de aici)."""
        fx, fy, cx_s, cy_s = self.intrinsici_pentru_depth(intrinsics, w_depth, h_depth, w_rgb, h_rgb)

        x_c = ((u - cx_s) * z_val) / fx
        y_c = ((v - cy_s) * z_val) / fy

        try:
            R = self.quat_to_rotmat(pose.qx, pose.qy, pose.qz, pose.qw)
            t = np.array([pose.tx, pose.ty, pose.tz]) * 100.0
            # Convertim din conventia pinhole (x dreapta, y jos, z=adancime inainte)
            # in conventia ARKit a camerei (x dreapta, y sus, z inapoi) inainte de rotatie
            punct_cam = np.array([x_c, -y_c, -z_val])
            punct_lume = R @ punct_cam + t
            x_g, y_g, z_g = punct_lume
        except AttributeError:
            # fallback daca pose-ul nu are quaternion (nu ar trebui sa se intample normal)
            try:
                tel_x, tel_y, tel_z = pose.tx * 100, pose.ty * 100, pose.tz * 100
            except AttributeError:
                tel_x, tel_y, tel_z = 0.0, 0.0, 0.0
            x_g = tel_x + x_c
            y_g = tel_y + y_c
            z_g = tel_z - z_val

        return x_g, y_g, z_g

    # ------------------------------------------------------------------
    #  REGISTRUL DE ARTEFACTE (deduplicare + medie ponderata)
    # ------------------------------------------------------------------
    def gaseste_artefact_apropiat(self, x, y, z, segment):
        """Cauta in registru un artefact deja inregistrat aproape de (x, y, z),
        DOAR in acelasi segment (intre segmente coordonatele nu sunt inca aliniate;
        duplicatele dintre segmente se unesc la final, dupa ICP).
        Returneaza indexul lui in lista, sau None daca e un artefact nou."""
        cel_mai_apropiat_idx = None
        cea_mai_mica_dist = self.distanta_duplicat_cm
        for idx, art in enumerate(self.artefacte_inregistrate):
            if art['segment'] != segment:
                continue
            dist = np.sqrt((art['x'] - x) ** 2 + (art['y'] - y) ** 2 + (art['z'] - z) ** 2)
            if dist < cea_mai_mica_dist:
                cea_mai_mica_dist = dist
                cel_mai_apropiat_idx = idx
        return cel_mai_apropiat_idx

    def inregistreaza_artefact(self, incredere, x, y, z):
        """Adauga un artefact nou sau actualizeaza media pozitiei unuia existent
        (media creste precizia pe masura ce e vazut din mai multe unghiuri).
        Artefactele nu sunt clasificate pe tip (deocamdata) - sunt doar numerotate
        de la 1 la n, in ordinea descoperirii.
        Returneaza (id, e_nou, poza_merita_refacuta) - al treilea e True cand
        detectia curenta e cea mai clara de pana acum si merita salvata poza."""
        segment_curent = len(self.segmente) - 1
        idx_existent = self.gaseste_artefact_apropiat(x, y, z, segment_curent)

        if idx_existent is not None:
            art = self.artefacte_inregistrate[idx_existent]
            n = art['nr_detectii']
            art['x'] = (art['x'] * n + x) / (n + 1)
            art['y'] = (art['y'] * n + y) / (n + 1)
            art['z'] = (art['z'] * n + z) / (n + 1)
            art['nr_detectii'] += 1
            poza_merita_refacuta = incredere > art['incredere']
            if poza_merita_refacuta:
                art['incredere'] = incredere
            return art['id'], False, poza_merita_refacuta

        self.artefact_counter += 1
        nou = {
            'id': self.artefact_counter,
            'incredere': incredere,
            'nr_detectii': 1,
            'x': x, 'y': y, 'z': z,
            'segment': segment_curent,
            'poza': '',
        }
        self.artefacte_inregistrate.append(nou)
        return nou['id'], True, True

    def dedup_global_artefacte(self):
        """Dupa alinierea ICP a segmentelor, uneste artefactele duplicate care
        provin din segmente diferite (acelasi obiect vazut inainte si dupa un
        salt de tracking, sau intr-o scanare anterioara)."""
        finali = []
        # pornim de la cele vazute de cele mai multe ori (pozitia lor e cea mai sigura)
        for art in sorted(self.artefacte_inregistrate, key=lambda a: -a['nr_detectii']):
            pereche = None
            for f in finali:
                dist = np.sqrt((f['x'] - art['x']) ** 2 + (f['y'] - art['y']) ** 2 + (f['z'] - art['z']) ** 2)
                if dist < self.distanta_duplicat_cm:
                    pereche = f
                    break
            if pereche is not None:
                n1, n2 = pereche['nr_detectii'], art['nr_detectii']
                pereche['x'] = (pereche['x'] * n1 + art['x'] * n2) / (n1 + n2)
                pereche['y'] = (pereche['y'] * n1 + art['y'] * n2) / (n1 + n2)
                pereche['z'] = (pereche['z'] * n1 + art['z'] * n2) / (n1 + n2)
                pereche['nr_detectii'] = n1 + n2
                if art['incredere'] > pereche['incredere']:
                    pereche['incredere'] = art['incredere']
            else:
                finali.append(art)

        unite = len(self.artefacte_inregistrate) - len(finali)
        if unite > 0:
            print(f"🔗 Am unit {unite} artefacte duplicate intre segmente.")
        self.artefacte_inregistrate = sorted(finali, key=lambda a: a['id'])

    def salveaza_poza_artefact(self, art_id, bgr_curat, x1, y1, x2, y2):
        """Decupeaza artefactul din cadrul curat (fara chenarele desenate), cu o
        margine in jur, si salveaza poza pentru registrul sapaturii (numele
        fisierului depinde doar de ID, deci o vedere mai clara il suprascrie)."""
        h, w = bgr_curat.shape[:2]
        m = self.margine_poza_px
        y_a, y_b = max(0, y1 - m), min(h, y2 + m)
        x_a, x_b = max(0, x1 - m), min(w, x2 + m)
        if y_b <= y_a or x_b <= x_a:
            return

        nume_poza = f"artefact_{art_id:03d}.jpg"
        cv2.imwrite(os.path.join(self.director_poze, nume_poza), bgr_curat[y_a:y_b, x_a:x_b])

        for art in self.artefacte_inregistrate:
            if art['id'] == art_id:
                art['poza'] = nume_poza
                break

    def rescrie_csv(self):
        """Rescrie CSV-ul cu starea curenta a registrului (pozitiile mediate)."""
        self.csv_file.seek(0)
        self.csv_file.truncate()
        self.csv_writer.writerow(['ID_Artefact', 'Incredere', 'Nr_Detectii', 'X_Centru', 'Y_Centru', 'Z_Centru', 'Poza'])
        for art in self.artefacte_inregistrate:
            self.csv_writer.writerow([
                art['id'], f"{art['incredere']:.2f}", art['nr_detectii'],
                f"{art['x']:.1f}", f"{art['y']:.1f}", f"{art['z']:.1f}",
                art.get('poza', '')
            ])
        self.csv_file.flush()

    # ------------------------------------------------------------------
    #  FUZIUNEA TSDF A CADRELOR
    # ------------------------------------------------------------------
    def integreaza_cadru_tsdf(self, segment, rgb, depth, fx, fy, cx, cy, R, t, w_depth, h_depth):
        """Fuzioneaza un cadru RGB-D in volumul TSDF al segmentului curent.
        R si t vin din poza ARKit a telefonului (t in METRI - TSDF lucreaza in metri)."""
        rgb_mic = cv2.resize(rgb, (w_depth, h_depth))
        culoare = o3d.geometry.Image(np.ascontiguousarray(rgb_mic))
        adancime = o3d.geometry.Image(np.ascontiguousarray(depth.astype(np.float32)))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            culoare, adancime,
            depth_scale=1.0,                       # adancimea e deja in metri
            depth_trunc=self.dist_max_cm / 100.0,  # taiem fundalul indepartat
            convert_rgb_to_intensity=False)

        intr = o3d.camera.PinholeCameraIntrinsic(w_depth, h_depth, fx, fy, cx, cy)

        # Conventia ARKit a camerei (x dreapta, y sus, z inapoi) -> conventia
        # pinhole ceruta de Open3D (x dreapta, y jos, z inainte)
        R_pinhole = R @ np.diag([1.0, -1.0, -1.0])
        camera_in_lume = np.eye(4)
        camera_in_lume[:3, :3] = R_pinhole
        camera_in_lume[:3, 3] = t
        extrinsic = np.linalg.inv(camera_in_lume)

        segment['volum'].integrate(rgbd, intr, extrinsic)

    def nor_puncte_curent(self):
        """Extrage punctele din volumele TSDF ale tuturor segmentelor, in cm
        (segmentele de dupa o pierdere de tracking sunt inca nealiniate intre ele)."""
        total = o3d.geometry.PointCloud()
        for seg in self.segmente:
            if seg['nr_cadre'] == 0:
                continue
            p = seg['volum'].extract_point_cloud()
            if not p.is_empty():
                p.scale(100.0, center=(0.0, 0.0, 0.0))  # metri -> cm
                total += p
        return total

    # ------------------------------------------------------------------
    #  AUTOSAVE PERIODIC
    # ------------------------------------------------------------------
    def autosave(self):
        """Salveaza periodic punctele extrase din volumele TSDF intr-un fisier
        separat, ca sa nu se piarda nimic daca scriptul sau telefonul se blocheaza.
        La urmatoarea pornire, fisierul e arhivat impreuna cu restul scanarii."""
        total = self.nor_puncte_curent()
        if total.is_empty():
            return
        o3d.io.write_point_cloud(self.autosave_path, total)
        print(f"💾 Autosave: {len(total.points)} puncte puse deoparte in '{self.autosave_path}'.")

    # ------------------------------------------------------------------
    #  MINI-HARTA DE ACOPERIRE (vedere de sus)
    # ------------------------------------------------------------------
    def actualizeaza_acoperirea(self, puncte):
        """Marcheaza celulele de teren (5x5 cm, plan X-Z) atinse de punctele noi."""
        cel = self.minimapa_celula_cm
        ix = np.floor(puncte[:, 0] / cel).astype(int)
        iz = np.floor(puncte[:, 2] / cel).astype(int)
        self.celule_acoperite.update(zip(ix.tolist(), iz.tolist()))

    def actualizeaza_acoperirea_din_cadru(self, depth, fx, fy, cx, cy, R, t, w_depth, h_depth, n=400):
        """Proiecteaza in lume un mic esantion de pixeli din cadru, doar pentru
        mini-harta de acoperire (harta propriu-zisa se construieste prin TSDF)."""
        v = np.random.randint(0, h_depth, n)
        u = np.random.randint(0, w_depth, n)
        z = depth[v, u] * 100.0
        ok = (z > 0.1) & (z < self.dist_max_cm)
        if not np.any(ok):
            return
        u, v, z = u[ok], v[ok], z[ok]
        x_c = (u - cx) * z / fx
        y_c = (v - cy) * z / fy
        puncte_cam = np.vstack((x_c, -y_c, -z)).T
        puncte_lume = puncte_cam @ R.T + t * 100.0
        self.actualizeaza_acoperirea(puncte_lume)

    def deseneaza_minimapa(self, bgr_frame, pose):
        """Deseneaza in coltul din dreapta-sus harta acoperirii, vazuta de sus:
        verde = teren deja scanat, rosu = artefacte, cerc alb = telefonul acum.
        Gaurile negre din pata verde sunt zonele peste care nu ai trecut inca."""
        if not self.celule_acoperite:
            return
        cel = self.minimapa_celula_cm
        celule = np.array(list(self.celule_acoperite), dtype=int)
        ix_min, iz_min = celule.min(axis=0)
        ix_max, iz_max = celule.max(axis=0)
        nx = ix_max - ix_min + 1
        nz = iz_max - iz_min + 1
        if nx > 3000 or nz > 3000:
            return  # coordonate aberante (salt mare de tracking) - nu desenam

        grid = np.zeros((nz, nx), dtype=np.uint8)
        grid[celule[:, 1] - iz_min, celule[:, 0] - ix_min] = 1

        scala = min(self.minimapa_px / nx, self.minimapa_px / nz)
        w_m = max(2, int(nx * scala))
        h_m = max(2, int(nz * scala))
        mini = np.full((h_m, w_m, 3), 35, dtype=np.uint8)
        acoperit = cv2.resize(grid, (w_m, h_m), interpolation=cv2.INTER_NEAREST) > 0
        mini[acoperit] = (70, 200, 70)  # verde (BGR)

        def in_pixeli(x_cm, z_cm):
            return (int((x_cm / cel - ix_min) * scala),
                    int((z_cm / cel - iz_min) * scala))

        for art in self.artefacte_inregistrate:
            px, pz = in_pixeli(art['x'], art['z'])
            if 0 <= px < w_m and 0 <= pz < h_m:
                cv2.circle(mini, (px, pz), 2, (0, 0, 255), -1)

        try:
            px, pz = in_pixeli(pose.tx * 100.0, pose.tz * 100.0)
            if 0 <= px < w_m and 0 <= pz < h_m:
                cv2.circle(mini, (px, pz), 4, (255, 255, 255), 1)
        except AttributeError:
            pass

        h_f, w_f = bgr_frame.shape[:2]
        if h_f < h_m + 80 or w_f < w_m + 20:
            return
        y0, x0 = 46, w_f - w_m - 10  # sub bara de stare a HUD-ului
        zona = bgr_frame[y0:y0 + h_m, x0:x0 + w_m]
        bgr_frame[y0:y0 + h_m, x0:x0 + w_m] = cv2.addWeighted(zona, 0.35, mini, 0.65, 0)
        cv2.rectangle(bgr_frame, (x0 - 1, y0 - 1), (x0 + w_m, y0 + h_m), (255, 255, 255), 1)
        cv2.putText(bgr_frame, "acoperire", (x0, y0 + h_m + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # ------------------------------------------------------------------
    #  INTERFATA FERESTREI VIDEO (HUD)
    #  Nota: fonturile OpenCV nu au diacritice, deci textele de pe imagine
    #  sunt scrise fara ele.
    # ------------------------------------------------------------------
    @staticmethod
    def _text_cu_fundal(img, text, pozitie, scala=0.5,
                        culoare_text=(255, 255, 255), culoare_fundal=None, grosime=1):
        (w, h), sub = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scala, grosime)
        x, y = pozitie
        if culoare_fundal is not None:
            cv2.rectangle(img, (x - 4, y - h - 5), (x + w + 4, y + sub + 3), culoare_fundal, -1)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scala,
                    culoare_text, grosime, cv2.LINE_AA)
        return w

    @staticmethod
    def _banda_translucida(img, y_sus, y_jos):
        img[y_sus:y_jos] = (img[y_sus:y_jos] * 0.35).astype(np.uint8)

    def deseneaza_chenar_artefact(self, img, x1, y1, x2, y2, culoare, eticheta):
        """Chenar modern doar cu colturi (nu dreptunghi plin) + eticheta pe fundal
        colorat, mereu lizibila indiferent de imaginea de dedesubt."""
        lungime = max(10, int(0.22 * min(x2 - x1, y2 - y1)))
        for px, py, dx, dy in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
            cv2.line(img, (px, py), (px + dx * lungime, py), culoare, 2, cv2.LINE_AA)
            cv2.line(img, (px, py), (px, py + dy * lungime), culoare, 2, cv2.LINE_AA)
        self._text_cu_fundal(img, eticheta, (x1 + 2, max(y1 - 7, 16)),
                             0.5, (0, 0, 0), culoare, 1)

    def deseneaza_hud(self, img):
        h, w = img.shape[:2]
        acum = time.time()

        # ---- bara de sus: REC, cronometru, statistici, stare tracking ----
        self._banda_translucida(img, 0, 36)
        if int(acum * 2) % 2 == 0:  # punct rosu care pulseaza
            cv2.circle(img, (17, 18), 7, (40, 40, 230), -1, cv2.LINE_AA)
        x = self._text_cu_fundal(img, "SCANARE", (30, 24), 0.6, (255, 255, 255), None, 2) + 46

        if self.timp_pornire is not None:
            durata = int(acum - self.timp_pornire)
            x += self._text_cu_fundal(img, f"{durata // 60:02d}:{durata % 60:02d}",
                                      (x, 24), 0.6, (200, 200, 200), None, 2) + 24

        x += self._text_cu_fundal(img, f"Artefacte: {len(self.artefacte_inregistrate)}",
                                  (x, 24), 0.55, (120, 230, 120), None, 1) + 20
        x += self._text_cu_fundal(img, f"Segmente: {len(self.segmente)}",
                                  (x, 24), 0.55, (200, 200, 200), None, 1) + 20
        if self.fps_curent > 0:
            x += self._text_cu_fundal(img, f"{self.fps_curent:.0f} cadre/s",
                                      (x, 24), 0.55, (200, 200, 200), None, 1) + 20

        # starea trackingului, aliniata la dreapta
        tracking_ok = (self.timp_segment_nou is None or acum - self.timp_segment_nou > 3.0)
        text_stare = "tracking stabil" if tracking_ok else "TRACKING PIERDUT - stai pe loc!"
        culoare_stare = (120, 230, 120) if tracking_ok else (60, 130, 255)
        (ws, _), _ = cv2.getTextSize(text_stare, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.circle(img, (w - ws - 26, 18), 6, culoare_stare, -1, cv2.LINE_AA)
        self._text_cu_fundal(img, text_stare, (w - ws - 14, 24), 0.55, culoare_stare, None, 2)

        # ---- bara de jos: comenzi + autosave ----
        self._banda_translucida(img, h - 28, h)
        self._text_cu_fundal(img, "ESC = incheie si salveaza harta",
                             (10, h - 9), 0.5, (255, 255, 255), None, 1)
        ramas = int(max(0.0, self.autosave_interval_s - (acum - self.ultimul_autosave)))
        text_as = f"autosave in {ramas}s"
        (wa, _), _ = cv2.getTextSize(text_as, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        self._text_cu_fundal(img, text_as, (w - wa - 10, h - 9), 0.5, (170, 170, 170), None, 1)

    # ------------------------------------------------------------------
    #  BUCLA PRINCIPALA DE SCANARE
    # ------------------------------------------------------------------
    def start_scanning(self, wifi_ip=None):
        """wifi_ip: daca e dat, se conecteaza DIRECT prin Wi-Fi (WebRTC) la adresa
        IP afisata in Record3D (Settings > Live RGBD Video Streaming > Wi-Fi,
        apoi butonul rosu) - necesita extensia platita 'Wi-Fi Streaming &
        RGBD video export' din Record3D. Nu foloseste USB si nici 'Sync over
        Wi-Fi'/iTunes - conexiunea e complet fara cablu, de la prima pornire."""
        if wifi_ip:
            import record3d_wifi
            self.stream = record3d_wifi.Record3DWifiStream()
            self.stream.on_new_frame = self.process_frame_callback
            print("─" * 56)
            print(f"📶 MOD WIFI DIRECT - ma conectez la {wifi_ip}...")
            print("   (Record3D > Settings > Live RGBD Video Streaming > Wi-Fi,")
            print("    apoi butonul rosu - adresa IP e afisata deasupra lui)")
            print("─" * 56)
            try:
                self.stream.connect(wifi_ip)
            except Exception as e:
                print(f"❌ Conectarea Wi-Fi la {wifi_ip} a esuat: {e}")
                return
        else:
            self.stream = Record3DStream()
            self.stream.on_new_frame = self.process_frame_callback
            devices = Record3DStream.get_connected_devices()
            if not devices:
                print("❌ Nu s-a gasit iPhone-ul.")
                return
            if len(devices) > 1:
                print(f"ℹ️ Am gasit {len(devices)} dispozitive conectate - folosesc primul.")
            self.stream.connect(devices[0])

        eticheta_conexiune = f"WiFi direct ({wifi_ip})" if wifi_ip else "USB"
        print("─" * 56)
        print(f"🎥 SCANARE ACTIVA ({eticheta_conexiune}) - plimba telefonul lent peste tot situl")
        print("   · tine-l la 1-2 m de sol, cu suprapunere intre treceri")
        print("   · urmareste mini-harta din colt: petele negre = zone nescanate")
        print("   · ESC (cu fereastra video selectata) = incheie si salveaza")
        print("─" * 56)

        try:
            frame_skip = 0
            start_asteptare = time.time()
            am_primit_cadre = False
            avertisment_afisat = False
            while True:
                if not am_primit_cadre and not avertisment_afisat and time.time() - start_asteptare > 10:
                    avertisment_afisat = True
                    print("⚠️ Sunt conectat la iPhone, dar nu primesc niciun cadru de 10 secunde.")
                    print("   Pe telefon: deschide aplicatia Record3D, activeaza 'USB Streaming mode'")
                    print("   in setarile aplicatiei, apoi apasa butonul rosu de inregistrare.")
                if self.is_new_frame and self.rgb_frame is not None and self.depth_frame is not None:
                    am_primit_cadre = True
                    acum_cadru = time.time()
                    if self.timp_pornire is None:
                        self.timp_pornire = acum_cadru
                    if self.timp_fps_anterior is not None and acum_cadru > self.timp_fps_anterior:
                        fps_instant = 1.0 / (acum_cadru - self.timp_fps_anterior)
                        self.fps_curent = (0.85 * self.fps_curent + 0.15 * fps_instant
                                           if self.fps_curent else fps_instant)
                    self.timp_fps_anterior = acum_cadru
                    rgb = self.rgb_frame
                    depth = self.depth_frame
                    intrinsics = self.intrinsics
                    pose = self.camera_pose
                    self.is_new_frame = False

                    h_depth, w_depth = depth.shape[:2]
                    h_rgb, w_rgb = rgb.shape[:2]

                    # 0. VERIFICAM DACA TELEFONUL A PIERDUT TRACKINGUL (salt/pauza)
                    self.verifica_pierdere_tracking(pose)
                    segment = self.segmente[-1]

                    # 1. FUZIUNE TSDF CONTINUA (fiecare al 3-lea cadru intra in volumul 3D)
                    frame_skip += 1
                    if frame_skip % 3 == 0:
                        fx, fy, cx_s, cy_s = self.intrinsici_pentru_depth(
                            intrinsics, w_depth, h_depth, w_rgb, h_rgb)
                        try:
                            R = self.quat_to_rotmat(pose.qx, pose.qy, pose.qz, pose.qw)
                            t = np.array([pose.tx, pose.ty, pose.tz])  # metri (TSDF lucreaza in metri)
                            self.integreaza_cadru_tsdf(segment, rgb, depth, fx, fy, cx_s, cy_s,
                                                       R, t, w_depth, h_depth)
                            segment['nr_cadre'] += 1
                            self.actualizeaza_acoperirea_din_cadru(depth, fx, fy, cx_s, cy_s,
                                                                   R, t, w_depth, h_depth)
                        except AttributeError:
                            # fara orientarea telefonului (quaternion) nu putem fuziona corect cadrul
                            pass

                    # 2. DETECTIE YOLO + INREGISTRARE COORDONATE ARTEFACTE
                    bgr_frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    # copie fara chenarele desenate, din care decupam pozele artefactelor
                    bgr_curat = bgr_frame.copy()
                    scale_x, scale_y = w_depth / w_rgb, h_depth / h_rgb
                    rezultate = self.yolo_model(bgr_frame, verbose=False)

                    for cutie in rezultate[0].boxes:
                        incredere = float(cutie.conf[0])
                        if incredere < self.conf_minima:
                            continue

                        x1, y1, x2, y2 = map(int, cutie.xyxy[0].cpu().numpy())
                        u_centru = int(((x1 + x2) / 2) * scale_x)
                        v_centru = int(((y1 + y2) / 2) * scale_y)

                        if 0 <= u_centru < w_depth and 0 <= v_centru < h_depth:
                            # Mediana adancimii pe o mica zona din jurul centrului,
                            # ca un singur pixel zgomotos sa nu strice coordonata
                            patch = depth[max(0, v_centru - 2):v_centru + 3,
                                          max(0, u_centru - 2):u_centru + 3] * 100
                            valori_z = patch[patch > 0.1]
                            z_d = float(np.median(valori_z)) if valori_z.size > 0 else 0.0
                            if 0.1 < z_d < self.dist_max_cm:
                                x_g, y_g, z_g = self.calculeaza_coordonate_globale(
                                    u_centru, v_centru, z_d, intrinsics, pose, w_depth, h_depth, w_rgb, h_rgb
                                )

                                art_id, e_nou, poza_noua = self.inregistreaza_artefact(
                                    incredere, x_g, y_g, z_g)

                                if poza_noua:
                                    self.salveaza_poza_artefact(art_id, bgr_curat, x1, y1, x2, y2)
                                if e_nou or poza_noua:
                                    self.rescrie_csv()
                                if e_nou:
                                    print(f"🏺 Artefact nou #{art_id} "
                                          f"({incredere:.0%}) la X={x_g:.1f} Y={y_g:.1f} Z={z_g:.1f} cm")

                                culoare = (80, 220, 80) if e_nou else (60, 200, 255)
                                self.deseneaza_chenar_artefact(bgr_frame, x1, y1, x2, y2, culoare,
                                                               f"Artefact #{art_id}")

                    self.deseneaza_minimapa(bgr_frame, pose)
                    self.deseneaza_hud(bgr_frame)

                    cv2.imshow(NUME_FEREASTRA_VIDEO, bgr_frame)

                    # 3. AUTOSAVE PERIODIC (protectie la crash / telefon blocat)
                    if time.time() - self.ultimul_autosave > self.autosave_interval_s:
                        self.autosave()
                        self.ultimul_autosave = time.time()

                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC opreste scanarea si salveaza harta continua
                    break

        except KeyboardInterrupt:
            pass
        finally:
            self.finalizeaza_si_salveaza()
            self.cleanup()

    # ------------------------------------------------------------------
    #  ALINIEREA SEGMENTELOR (ICP) + SALVAREA HARTII FINALE
    # ------------------------------------------------------------------
    def _pregateste_pentru_icp(self, pcd):
        p = pcd.voxel_down_sample(self.icp_voxel_cm)
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.icp_voxel_cm * 4, max_nn=30))
        return p

    def aliniaza_segment(self, sursa, tinta):
        """Aliniaza un segment (sursa) peste harta acumulata (tinta).
        Intai incearca ICP direct (segmentele sunt de obicei aproape unul de altul);
        daca nu prinde, incearca o aliniere globala RANSAC pe trasaturi FPFH
        (cazul repornirii scriptului, cand originea ARKit s-a resetat complet).
        Returneaza (matrice_transformare_4x4, fitness)."""
        reg = o3d.pipelines.registration
        s = self._pregateste_pentru_icp(sursa)
        t = self._pregateste_pentru_icp(tinta)
        dist_max = self.icp_voxel_cm * 4

        icp = reg.registration_icp(
            s, t, dist_max, np.eye(4),
            reg.TransformationEstimationPointToPlane(),
            reg.ICPConvergenceCriteria(max_iteration=60))
        cel_mai_bun = (icp.transformation, icp.fitness)

        if icp.fitness < self.icp_fitness_minim:
            try:
                print("   ICP direct a esuat - incerc alinierea globala RANSAC "
                      "(poate dura cateva minute, e normal)...")
                raza_fpfh = self.icp_voxel_cm * 10
                fpfh_s = reg.compute_fpfh_feature(
                    s, o3d.geometry.KDTreeSearchParamHybrid(radius=raza_fpfh, max_nn=100))
                fpfh_t = reg.compute_fpfh_feature(
                    t, o3d.geometry.KDTreeSearchParamHybrid(radius=raza_fpfh, max_nn=100))
                ransac = reg.registration_ransac_based_on_feature_matching(
                    s, t, fpfh_s, fpfh_t, True, dist_max * 1.5,
                    reg.TransformationEstimationPointToPoint(False), 3,
                    [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                     reg.CorrespondenceCheckerBasedOnDistance(dist_max * 1.5)],
                    reg.RANSACConvergenceCriteria(100000, 0.999))
                icp2 = reg.registration_icp(
                    s, t, dist_max, ransac.transformation,
                    reg.TransformationEstimationPointToPlane(),
                    reg.ICPConvergenceCriteria(max_iteration=60))
                if icp2.fitness > cel_mai_bun[1]:
                    cel_mai_bun = (icp2.transformation, icp2.fitness)
            except Exception as e:
                print(f"   (alinierea globala RANSAC a esuat: {e})")

        return cel_mai_bun

    def transforma_artefacte_segment(self, idx_segment, T):
        """Aplica transformarea gasita de ICP si artefactelor din acel segment,
        ca sa ramana la locul corect pe harta aliniata."""
        for art in self.artefacte_inregistrate:
            if art['segment'] == idx_segment:
                p = T @ np.array([art['x'], art['y'], art['z'], 1.0])
                art['x'], art['y'], art['z'] = float(p[0]), float(p[1]), float(p[2])
                art['segment'] = -1  # de acum e in sistemul de coordonate al hartii finale

    def finalizeaza_si_salveaza(self):
        print("\n💾 Construiesc harta finala a sitului...")

        # Extragem din fiecare volum TSDF atat suprafata (mesh), cat si punctele
        # (pe puncte se face alinierea ICP; transformarea gasita se aplica si meshului)
        segmente_gata = []
        for idx, seg in enumerate(self.segmente):
            if seg['nr_cadre'] == 0:
                continue
            pcd = seg['volum'].extract_point_cloud()
            if pcd.is_empty():
                continue
            mesh = seg['volum'].extract_triangle_mesh()
            pcd.scale(100.0, center=(0.0, 0.0, 0.0))   # metri -> cm
            mesh.scale(100.0, center=(0.0, 0.0, 0.0))
            segmente_gata.append((idx, pcd, mesh))

        if not segmente_gata:
            print("⚠️ Nu am acumulat niciun punct de teren.")
            self.rescrie_csv()
            self.save_artefact_map()
            return

        # Referinta: primul segment; artefactele lui sunt deja in sistemul final
        idx0, harta, mesh_total = segmente_gata[0]
        for art in self.artefacte_inregistrate:
            if art['segment'] == idx0:
                art['segment'] = -1

        for idx, pcd, mesh in segmente_gata[1:]:
            if len(pcd.points) < 200:
                print(f"   Segment #{idx + 1}: prea mic ({len(pcd.points)} puncte), il adaug fara aliniere.")
                self.transforma_artefacte_segment(idx, np.eye(4))
                harta += pcd
                mesh_total += mesh
                continue

            print(f"   Aliniez segmentul #{idx + 1} ({len(pcd.points)} puncte) cu ICP...")
            T, fitness = self.aliniaza_segment(pcd, harta)
            if fitness >= self.icp_fitness_minim:
                pcd.transform(T)
                mesh.transform(T)
                self.transforma_artefacte_segment(idx, T)
                print(f"   ✅ Segment #{idx + 1} aliniat (fitness {fitness:.2f}).")
            else:
                self.transforma_artefacte_segment(idx, np.eye(4))
                print(f"   ⚠️ Segment #{idx + 1}: alinierea nu a prins (fitness {fitness:.2f}) - "
                      f"il pastrez nealiniat. Rescaneaza zona cu suprapunere mai mare peste restul hartii.")
            harta += pcd
            mesh_total += mesh

        # Uneste artefactele duplicate ramase dupa alinierea segmentelor
        self.dedup_global_artefacte()
        self.rescrie_csv()

        o3d.io.write_point_cloud(self.ply_output_path, harta)
        print(f"✅ Harta de puncte ({len(harta.points)} puncte) "
              f"a fost salvata in: {self.ply_output_path}")

        mesh_total.compute_vertex_normals()
        o3d.io.write_triangle_mesh(self.mesh_ply_path, mesh_total)
        print(f"✅ Suprafata 3D ({len(mesh_total.triangles)} triunghiuri) "
              f"a fost salvata in: {self.mesh_ply_path}")
        try:
            o3d.io.write_triangle_mesh(self.mesh_glb_path, mesh_total)
            print(f"✅ Modelul .glb (pentru telefon / colegi) a fost salvat in: {self.mesh_glb_path}")
        except Exception as e:
            print(f"⚠️ Exportul .glb a esuat ({e}) - foloseste fisierul .ply.")

        # Harta finala e completa - autosave-ul intermediar nu mai e necesar
        if os.path.exists(self.autosave_path):
            try:
                os.remove(self.autosave_path)
            except OSError:
                pass

        self.save_artefact_map()

        # Raportul HTML al scanarii (harta de sus + registrul artefactelor cu poze)
        try:
            import raport
            raport.genereaza(self.director_sesiune)
        except Exception as e:
            print(f"⚠️ Raportul HTML nu a putut fi generat ({e}) - "
                  f"poti incerca manual cu: python raport.py {self.director_sesiune}")

        # Ortofotoplanul (fotografie de sus, cu grila si scara grafica)
        try:
            import ortofoto
            ortofoto.genereaza(self.director_sesiune)
        except Exception as e:
            print(f"⚠️ Ortofotoplanul nu a putut fi generat ({e}) - "
                  f"poti incerca manual cu: python ortofoto.py {self.director_sesiune}")

    def save_artefact_map(self):
        print(f"💾 Salvez {len(self.artefacte_inregistrate)} artefacte gasite...")
        if self.artefacte_inregistrate:
            puncte = np.array([[a['x'], a['y'], a['z']] for a in self.artefacte_inregistrate])
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(puncte)
            # Coloram toate artefactele in rosu, ca sa iasa in evidenta pe teren
            culori_rosii = np.tile(np.array([[1.0, 0.0, 0.0]]), (len(puncte), 1))
            pcd.colors = o3d.utility.Vector3dVector(culori_rosii)
            o3d.io.write_point_cloud(self.ply_artefacte_path, pcd)
            print(f"✅ Artefactele au fost salvate in: {self.ply_artefacte_path}")
        print(f"✅ Coordonatele artefactelor au fost salvate in: {self.csv_path}")

    def cleanup(self):
        try:
            self.stream.disconnect()
        except:
            pass
        if not self.csv_file.closed: self.csv_file.close()
        cv2.destroyAllWindows()

        # Daca sesiunea nu a produs nimic (niciun punct, niciun artefact),
        # nu lasam un folder gol in 'scanari'
        if (not os.path.exists(self.ply_output_path)
                and not os.path.exists(self.autosave_path)
                and not self.artefacte_inregistrate):
            shutil.rmtree(self.director_sesiune, ignore_errors=True)
            print(f"ℹ️ Scanarea nu a produs date - am sters folderul gol '{self.director_sesiune}'.")

        print("Sistem oprit.")


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Scanare 3D sit arheologic (iPhone LiDAR)")
    parser.add_argument("--wifi", metavar="IP", default=None,
                         help="Conecteaza DIRECT prin Wi-Fi la adresa IP afisata in "
                              "Record3D (Settings > Live RGBD Video Streaming > Wi-Fi), "
                              "fara cablu USB. Necesita extensia platita Record3D "
                              "'Wi-Fi Streaming & RGBD video export'.")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    scanner = ContinuousArchaeoScanner()
    scanner.start_scanning(wifi_ip=args.wifi)


if __name__ == "__main__":
    main()
