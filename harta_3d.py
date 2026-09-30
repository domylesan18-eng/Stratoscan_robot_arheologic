import copy
import csv
import os
import sys
import time

import numpy as np
import open3d as o3d
from open3d.visualization import gui, rendering

FISIER_MESH = "harta_sit_mesh.ply"
FISIER_PUNCTE = "harta_completa_sit.ply"
FISIER_CSV = "coordonate_sit.csv"
DIRECTOR_SCANARI = "scanari"
DIRECTOR_ARHIVA_VECHE = "scanari_anterioare"  # structura veche, pana la migrare

CULOARE_FUNDAL = (0.075, 0.085, 0.105, 1.0)   # albastru-gri foarte inchis
CULOARE_MARCAJ = [0.90, 0.15, 0.15]
CULOARE_PIN = [0.65, 0.10, 0.10]

# Corectia de expunere pentru culorile reale ale terenului: Filament aplica
# implicit o expunere fotografica (calibrata pentru scene HDR reale) care
# distorsioneaza culorile brute ale meshului chiar si pe materialul unlit.
# Constanta a fost calibrata empiric (masurata cu o masca de adancime, pe
# date reale din proiect) ca vertex-colorii sa iasa cat mai aproape de poza
# originala. Folosita atat de vizualizator, cat si de ortofoto.py.
EXPUNERE_TEREN = 0.44


# Peste acest numar de triunghiuri, mesh-ul e simplificat DOAR pentru afisare
# in fereastra (fisierele salvate pe disc raman la rezolutia completa). Un mesh
# de milioane de triunghiuri incarcat dintr-o data in memoria video a provocat
# page fault-uri reproductibile ale driverului grafic (verificat cu Event Viewer:
# LiveKernelEvent 141 / igdkmdn64.sys pe placi Intel integrate) - mai ales cand
# fereastra mai are deja continut activ randat (cazul meniului STRATOSCAN).
LIMITA_TRIUNGHIURI_AFISARE = 800_000


def simplifica_pentru_afisare(mesh):
    """Reduce numarul de triunghiuri al unei copii a mesh-ului, doar pentru
    randare interactiva - vezi LIMITA_TRIUNGHIURI_AFISARE mai sus.
    Foloseste vertex clustering (grupare pe o grila de voxeli), nu quadric
    decimation - pe un mesh de milioane de triunghiuri, quadric decimation
    dureaza multe minute (testat: peste 5 minute, tot nu terminase pentru
    2.6M triunghiuri); vertex clustering e o singura trecere si dureaza sub
    o secunda, indiferent de marimea mesh-ului."""
    nr = len(mesh.triangles)
    if nr <= LIMITA_TRIUNGHIURI_AFISARE:
        return mesh
    print(f"⚙️ Simplific terenul pentru afisare: {nr} triunghiuri "
          f"(fisierele salvate pe disc raman neatinse)...")
    voxel = 2.0  # cm - marimea initiala a celulei de grupare
    simplu = mesh
    for _ in range(6):
        simplu = mesh.simplify_vertex_clustering(
            voxel, contraction=o3d.geometry.SimplificationContraction.Average)
        if 0 < len(simplu.triangles) <= LIMITA_TRIUNGHIURI_AFISARE:
            break
        voxel *= 1.6
    simplu.compute_vertex_normals()
    print(f"   → {len(simplu.triangles)} triunghiuri (celula de grupare: {voxel:.1f} cm).")
    return simplu


def material_teren_real():
    """Materialul terenului cu culori reale, cu corectia de expunere aplicata."""
    m = rendering.MaterialRecord()
    m.shader = "defaultUnlit"
    m.base_color = (EXPUNERE_TEREN, EXPUNERE_TEREN, EXPUNERE_TEREN, 1.0)
    return m


def aplica_grading_neutru(scene):
    """Dezactiveaza curba de tonemapping (filmic/ACES) pe scena data, ca sa nu
    mai spalaceasca/desatureze culorile peste corectia de expunere de mai sus."""
    try:
        grading = rendering.ColorGrading(rendering.ColorGrading.Quality.HIGH,
                                         rendering.ColorGrading.ToneMapping.LINEAR)
        scene.view.set_color_grading(grading)
    except Exception:
        pass  # daca API-ul de color grading nu e disponibil, raman setarile implicite


def incarca_artefacte(fisier_csv=FISIER_CSV):
    """Citeste coordonate_sit.csv si intoarce lista de artefacte gasite."""
    artefacte = []
    if not os.path.exists(fisier_csv):
        print(f"ℹ️ Nu am gasit '{fisier_csv}' - nu afisez artefacte pe harta.")
        return artefacte

    with open(fisier_csv, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for rand in reader:
            try:
                artefacte.append({
                    'id': rand['ID_Artefact'],
                    'incredere': rand.get('Incredere', ''),
                    'nr_detectii': rand.get('Nr_Detectii', '1'),
                    'x': float(rand['X_Centru']),
                    'y': float(rand['Y_Centru']),
                    'z': float(rand['Z_Centru']),
                })
            except (KeyError, ValueError):
                continue
    return artefacte


def incarca_teren(folder="."):
    """Incarca terenul din folderul dat: suprafata neteda (mesh TSDF) daca exista,
    altfel punctele. Returneaza (geometrie, este_mesh) sau (None, False)."""
    cale_mesh = os.path.join(folder, FISIER_MESH)
    if os.path.exists(cale_mesh):
        mesh = o3d.io.read_triangle_mesh(cale_mesh)
        if not mesh.is_empty():
            mesh.compute_vertex_normals()
            print(f"📂 Am incarcat suprafata 3D din '{cale_mesh}' "
                  f"({len(mesh.triangles)} triunghiuri).")
            return mesh, True

    cale_puncte = os.path.join(folder, FISIER_PUNCTE)
    if os.path.exists(cale_puncte):
        pcd = o3d.io.read_point_cloud(cale_puncte)
        if not pcd.is_empty():
            print(f"📂 Am incarcat harta de puncte din '{cale_puncte}' "
                  f"({len(pcd.points)} puncte).")
            return pcd, False

    print(f"❌ Nu am gasit nicio harta in '{folder}'. Ruleaza mai intai scanarea!")
    return None, False


def are_harta(folder):
    return (os.path.exists(os.path.join(folder, FISIER_MESH))
            or os.path.exists(os.path.join(folder, FISIER_PUNCTE)))


def nume_frumos(folder):
    """'scanari/2026-08-25_14-59' -> '25.08.2026 14:59'."""
    if folder in (".", ""):
        return "fisierele vechi din folderul proiectului"
    baza = os.path.basename(os.path.normpath(folder))
    try:
        return time.strftime("%d.%m.%Y %H:%M", time.strptime(baza[:16], "%Y-%m-%d_%H-%M"))
    except ValueError:
        return baza


def numar_artefacte(folder):
    """Cate artefacte are o scanare (0 daca nu exista CSV), fara mesaje in consola."""
    cale = os.path.join(folder, FISIER_CSV)
    if not os.path.exists(cale):
        return 0
    with open(cale, mode='r', newline='', encoding='utf-8') as f:
        return sum(1 for _ in csv.DictReader(f))


def lista_scanari():
    """Toate folderele de scanare cu harta, cele mai noi primele.
    Include si fisierele ramase in radacina din structura veche (ca '.')."""
    foldere = []
    for radacina in (DIRECTOR_SCANARI, DIRECTOR_ARHIVA_VECHE):
        if os.path.isdir(radacina):
            for d in os.listdir(radacina):
                cale = os.path.join(radacina, d)
                if os.path.isdir(cale) and are_harta(cale):
                    foldere.append(cale)
    foldere.sort(key=lambda c: os.path.basename(c), reverse=True)
    if are_harta("."):
        foldere.append(".")
    return foldere


def alege_scanare():
    """Meniu in consola: ultima scanare (Enter) sau oricare alta.
    Returneaza folderul ales, sau None daca nu exista nimic de afisat."""
    scanari = lista_scanari()
    if not scanari:
        print("❌ Nu exista nicio scanare salvata. Ruleaza mai intai scanarea!")
        return None
    if len(scanari) == 1:
        print(f"ℹ️ Deschid singura scanare existenta: {nume_frumos(scanari[0])}.")
        return scanari[0]

    ultima, restul = scanari[0], scanari[1:]
    print("\n" + "═" * 56)
    print("  HARTA 3D  ·  alege scanarea")
    print("═" * 56)
    print(f"  [Enter]  ULTIMA: {nume_frumos(ultima):<22}{numar_artefacte(ultima):>3} artefacte")
    for i, folder in enumerate(restul, start=1):
        print(f"  [{i:>5}]  {nume_frumos(folder):<29}{numar_artefacte(folder):>3} artefacte")
    print("─" * 56)

    while True:
        alegere = input("Alegerea ta: ").strip()
        if alegere == "":
            return ultima
        if alegere.isdigit() and 1 <= int(alegere) <= len(restul):
            return restul[int(alegere) - 1]
        print(f"   Tasteaza un numar intre 1 si {len(restul)} sau Enter pentru ultima scanare.")


def varfuri(geometrie, este_mesh):
    return np.asarray(geometrie.vertices if este_mesh else geometrie.points)


def coloreaza_dupa_adancime(geometrie, este_mesh):
    """Copie a terenului colorata dupa cota Y (albastru = adanc, rosu = sus),
    ca sa iasa in evidenta stratigrafia si denivelarile."""
    import matplotlib
    copie = copy.deepcopy(geometrie)
    v = varfuri(copie, este_mesh)
    y = v[:, 1]
    interval = np.ptp(y)
    norm = (y - y.min()) / (interval if interval > 1e-6 else 1.0)
    culori = matplotlib.colormaps['turbo'](norm)[:, :3]
    if este_mesh:
        copie.vertex_colors = o3d.utility.Vector3dVector(culori)
    else:
        copie.colors = o3d.utility.Vector3dVector(culori)
    return copie


def creeaza_scara_adancime(bbox_min, bbox_max, intindere):
    """Legenda modului adancime: o bara verticala cu gradientul de culori, asezata
    langa marginea hartii, de la cota cea mai joasa la cea mai inalta a terenului.
    Returneaza (mesh_bara, info) - info are pozitiile pentru etichetele cu cm."""
    import matplotlib
    y0, y1 = float(bbox_min[1]), float(bbox_max[1])
    inaltime = max(y1 - y0, 10.0)
    lat = max(3.0, intindere * 0.012)
    x = float(bbox_max[0]) + max(20.0, intindere * 0.06)
    z = float(bbox_min[2])

    n = 32
    cmap = matplotlib.colormaps['turbo']
    bara = o3d.geometry.TriangleMesh()
    for i in range(n):
        segment = o3d.geometry.TriangleMesh.create_box(lat, inaltime / n, lat)
        segment.translate([x, y0 + inaltime * i / n, z])
        segment.paint_uniform_color(cmap((i + 0.5) / n)[:3])
        bara += segment
    bara.compute_vertex_normals()

    info = {'x_eticheta': x + lat * 2.5, 'z': z, 'y0': y0, 'y1': y1}
    return bara, info


def creeaza_text_3d(text, pozitie, inaltime, culoare):
    """Text 3D real (mesh vopsit), centrat la 'pozitie'. Etichetele 2D din
    Open3D 0.19 nu pot fi colorate din Python si ies negre pe fundal inchis,
    asa ca scrisul e construit ca geometrie, cu culoarea garantata."""
    t = o3d.t.geometry.TriangleMesh.create_text(text, depth=0.8).to_legacy()
    bb = t.get_axis_aligned_bounding_box()
    h = float(bb.get_extent()[1])
    t.scale(inaltime / max(h, 1e-6), center=bb.get_center())
    bb = t.get_axis_aligned_bounding_box()
    t.translate(np.asarray(pozitie, dtype=float) - bb.get_center())
    t.paint_uniform_color(culoare)
    t.compute_vertex_normals()
    return t


def creeaza_marcaje(artefacte, y_sol):
    """Pentru fiecare artefact: o sfera rosie + un 'pin' vertical pana la nivelul
    solului, ca marcajul sa fie vizibil si cand sfera e ascunsa in relief."""
    marcaje = []
    for art in artefacte:
        sfera = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
        sfera.translate([art['x'], art['y'], art['z']])
        sfera.paint_uniform_color(CULOARE_MARCAJ)
        sfera.compute_vertex_normals()
        marcaje.append((f"marcaj_{art['id']}", sfera))

        inaltime = max(6.0, art['y'] - y_sol)
        pin = o3d.geometry.TriangleMesh.create_cylinder(radius=0.5, height=inaltime)
        pin.rotate(o3d.geometry.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0)),
                   center=(0, 0, 0))
        pin.translate([art['x'], art['y'] - inaltime / 2.0, art['z']])
        pin.paint_uniform_color(CULOARE_PIN)
        pin.compute_vertex_normals()
        marcaje.append((f"pin_{art['id']}", pin))
    return marcaje


def _fonturi_panou(app):
    """Inregistreaza fonturile proprii pentru panou. Trebuie apelat o SINGURA
    data per proces, inainte de prima fereastra - reinregistrarea fonturilor
    dupa ce o fereastra anterioara s-a inchis a provocat un crash nativ
    reproductibil (verificat empiric), de-asta orice fereastra noua din
    aceeasi sesiune trebuie sa refoloseasca fonturile deja inregistrate,
    nu sa le ceara din nou."""
    cale_roboto_bold = os.path.join(os.path.dirname(o3d.__file__), "resources", "Roboto-Bold.ttf")
    font_titlu = app.add_font(gui.FontDescription(cale_roboto_bold, gui.FontStyle.NORMAL, 22))
    font_sectiune = app.add_font(gui.FontDescription(cale_roboto_bold, gui.FontStyle.NORMAL, 14))
    return font_titlu, font_sectiune


def afiseaza_profesional(teren, este_mesh, artefacte, titlu="Harta 3D - Sit Arheologic",
                         _app=None, _window=None, _fonturi=None, _porneste_bucla=True,
                         _gestioneaza_layout=True, _pe_inapoi=None):
    """Construieste vizualizatorul complet. In mod normal (_window=None) isi
    creeaza propria fereastra si porneste bucla principala. Poate fi apelat
    si pentru a POPULA o fereastra deja existenta (vezi afiseaza_cu_selector),
    caz in care ferestrea/fonturile sunt refolosite si bucla nu se porneste
    aici (apelantul o porneste o singura data, la finalul lui).

    _gestioneaza_layout=False: apelantul isi are deja propriul on_layout pe
    fereastra (de exemplu STRATOSCAN, care comuta intre mai multe ecrane in
    aceeasi fereastra) - in loc sa il suprascriem, intoarcem (scene_widget,
    panel, pozitioneaza) ca apelantul sa plaseze el insusi cele doua widget-uri
    in propriul on_layout.
    _pe_inapoi: daca e dat, adauga un buton 'Inapoi la meniu' in varful
    panoului - folosit doar cand vizualizatorul e incorporat in alta fereastra."""
    app = _app or gui.Application.instance

    if este_mesh:
        teren = simplifica_pentru_afisare(teren)

    if _window is None:
        app.initialize()
        font_titlu, font_sectiune = _fonturi_panou(app)
        window = gui.Application.instance.create_window(titlu, 1600, 900)
        window.show(True)
    else:
        window = _window
        font_titlu, font_sectiune = _fonturi
    em = window.theme.font_size

    scene_widget = gui.SceneWidget()
    scene_widget.scene = rendering.Open3DScene(window.renderer)
    scene_widget.scene.set_background(CULOARE_FUNDAL)
    scene_widget.scene.show_skybox(False)
    scene_widget.scene.show_axes(True)
    scene_widget.scene.show_ground_plane(True, rendering.Scene.GroundPlane.XZ)
    scene_widget.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
    try:
        scene_widget.scene.set_lighting(rendering.Open3DScene.LightingProfile.SOFT_SHADOWS,
                                        np.array([0.577, -0.577, -0.577], dtype=np.float32))
    except Exception:
        pass  # daca profilul de lumina nu e disponibil, raman setarile implicite
    aplica_grading_neutru(scene_widget.scene)
    window.add_child(scene_widget)
    # Cadru NEDEGENERAT INAINTE de a incarca geometria si a apela setup_camera
    # mai jos - altfel scene_widget ramane cu dimensiune 0x0 in tot intervalul
    # (sincron) al constructiei scenei, ceea ce e inofensiv pe o fereastra noua
    # (modul de sine statator, unde nimic altceva nu are inca layout), dar pe
    # fereastra unica STRATOSCAN - unde meniul are deja un layout valid de mult
    # timp - starea asta inconsistenta intre widget-uri a provocat reproductibil
    # un page fault al driverului grafic (verificat cu Event Viewer:
    # LiveKernelEvent 141 / igdkmdn64.sys). Vizibilitatea reala (ascuns/aratat)
    # ramane controlata de apelant (STRATOSCAN) dupa ce functia se intoarce.
    scene_widget.frame = window.content_rect

    # Culorile reale se afiseaza FARA iluminare artificiala - exact cum au fost
    # capturate de camera (iluminarea scenei le albea/spalacea).
    mat_real = rendering.MaterialRecord()
    mat_real.shader = "defaultUnlit"

    # Terenul are propriul material, cu corectia de expunere calibrata aplicata.
    mat_teren_real = material_teren_real()

    # Modul adancime foloseste iluminare mata (fara luciu), ca relieful sa aiba
    # umbre si denivelarile sa se citeasca usor.
    mat_relief = rendering.MaterialRecord()
    mat_relief.shader = "defaultLit" if este_mesh else "defaultUnlit"
    mat_relief.base_reflectance = 0.0
    mat_relief.base_roughness = 1.0

    mat_marcaj = rendering.MaterialRecord()
    mat_marcaj.shader = "defaultLit"

    mat_linie = rendering.MaterialRecord()
    mat_linie.shader = "unlitLine"
    mat_linie.line_width = 5.0

    # Modul adancime (copie recolorata a intregului teren) si etichetele 3D ale
    # artefactelor NU se construiesc aici, la deschidere - pe un teren mare
    # (sute de mii - milioane de triunghiuri) constructia lor sincrona, pe firul
    # principal, poate bloca fereastra destul de mult cat Windows sa o marcheze
    # "Not Responding" (reprodus cu o harta de 1M+ triunghiuri si 77 artefacte).
    # Se construiesc LENES, la prima activare - vezi comuta_adancime() mai jos.
    teren_adancime_cache = {'geom': None}
    stare = {'adancime': False, 'artefact': -1, 'masurare': False, 'etichete': True, 'zbor': False}

    v = varfuri(teren, este_mesh)
    y_sol = float(v[:, 1].min())
    bbox_min, bbox_max = v.min(axis=0), v.max(axis=0)
    centru = ((bbox_min + bbox_max) / 2.0).astype(np.float32)
    intindere = float(np.linalg.norm(bbox_max - bbox_min))
    sus = np.array([0.0, 1.0, 0.0], dtype=np.float32)

    scara_bara, scara_info = creeaza_scara_adancime(bbox_min, bbox_max, intindere)
    inaltime_text = max(5.0, intindere * 0.018)

    # --- Populam scena: terenul (ambele variante, doar una vizibila), marcajele,
    #     etichetele artefactelor si legenda de adancime (ascunsa la pornire) ---
    scene_widget.scene.add_geometry("teren_real", teren, mat_teren_real)

    for nume, geom in creeaza_marcaje(artefacte, y_sol):
        scene_widget.scene.add_geometry(nume, geom, mat_marcaj)

    # Etichetele 3D (text triangulat, cate o mesh per artefact) sunt scumpe la
    # constructie - pe un sit cu multe artefacte le construim doar la prima
    # activare (comuta_etichete), nu neaparat aici. Sub prag, comportamentul
    # ramane cel dintotdeauna: construite imediat, vizibile din prima.
    LIMITA_ETICHETE_3D_AUTO = 25
    nume_etichete = []
    etichete_construite = len(artefacte) <= LIMITA_ETICHETE_3D_AUTO

    def construieste_etichete():
        for art in artefacte:
            text = creeaza_text_3d(f"Artefact #{art['id']}",
                                   [art['x'], art['y'] + 8.0 + inaltime_text, art['z']],
                                   inaltime_text, [1.0, 0.85, 0.3])
            nume = f"text_art_{art['id']}"
            scene_widget.scene.add_geometry(nume, text, mat_real)
            nume_etichete.append(nume)

    if etichete_construite:
        construieste_etichete()

    si = scara_info
    y_mij = (si['y0'] + si['y1']) / 2.0
    nume_scara = ["scara_adancime"]
    scene_widget.scene.add_geometry("scara_adancime", scara_bara, mat_real)
    for i, (y, text) in enumerate(((si['y0'], f"{si['y0']:.0f} cm - adanc"),
                                   (y_mij, f"{y_mij:.0f} cm"),
                                   (si['y1'], f"{si['y1']:.0f} cm - sus"))):
        nume = f"text_scara_{i}"
        geom = creeaza_text_3d(text, [si['x_eticheta'] + inaltime_text * 4.0, y, si['z']],
                               inaltime_text, [0.95, 0.95, 0.95])
        scene_widget.scene.add_geometry(nume, geom, mat_real)
        nume_scara.append(nume)
    for nume in nume_scara:
        scene_widget.scene.show_geometry(nume, False)

    # Camera: stabilim intai proiectia pe baza intregii scene (include si
    # scara, chiar ascunsa, ca zoom-ul sa nu se schimbe la comutarea modului)
    scene_widget.setup_camera(60.0, scene_widget.scene.bounding_box, centru)

    # --- Vederi predefinite: un click te duce mereu intr-un unghi cunoscut,
    #     iar rotirea cu mouse-ul pivoteaza in jurul punctului ales ---
    def vedere_generala():
        ochi = centru + np.array([0.0, 0.75, 0.9], dtype=np.float32) * intindere
        scene_widget.look_at(centru, ochi, sus)
        scene_widget.center_of_rotation = centru

    def vedere_de_sus():
        ochi = centru + np.array([0.0, 1.4 * max(intindere, 50.0), 0.0], dtype=np.float32)
        # privim drept in jos; "sus" devine directia nordului hartii (-Z)
        scene_widget.look_at(centru, ochi, np.array([0.0, 0.0, -1.0], dtype=np.float32))
        scene_widget.center_of_rotation = centru

    def salt_la_artefact(index):
        stare['artefact'] = index
        art = artefacte[index]
        tinta = np.array([art['x'], art['y'], art['z']], dtype=np.float32)
        ochi = tinta + np.array([60.0, 80.0, 60.0], dtype=np.float32)
        scene_widget.look_at(tinta, ochi, sus)
        scene_widget.center_of_rotation = tinta
        lbl_info.text = f"Camera pe Artefact #{art['id']} ({index + 1}/{len(artefacte)})"
        print(f"🔎 Camera pe artefactul #{art['id']} - rotirea pivoteaza acum in jurul lui.")

    def artefact_urmator():
        if not artefacte:
            print("ℹ️ Niciun artefact pe harta.")
            return
        salt_la_artefact((stare['artefact'] + 1) % len(artefacte))

    def opreste_masurarea():
        if stare['masurare']:
            stare['masurare'] = False
            btn_masurare.is_on = False
            scene_widget.set_view_controls(gui.SceneWidget.Controls.FLY if stare['zbor']
                                           else gui.SceneWidget.Controls.ROTATE_CAMERA)

    def comuta_zbor(is_on):
        stare['zbor'] = is_on
        opreste_masurarea()
        scene_widget.set_view_controls(gui.SceneWidget.Controls.FLY if is_on
                                       else gui.SceneWidget.Controls.ROTATE_CAMERA)
        if is_on:
            lbl_info.text = "Mod zbor: W/S inainte-inapoi, A/D stanga-dreapta, Q/Z sus-jos, mouse = privire"
        else:
            lbl_info.text = "Mod rotire: mouse stanga = rotire, dreapta = deplasare, scroll = zoom"

    def comuta_adancime(is_on):
        stare['adancime'] = is_on
        if is_on and teren_adancime_cache['geom'] is None:
            lbl_info.text = "Calculez modul adancime..."
            teren_adancime_cache['geom'] = coloreaza_dupa_adancime(teren, este_mesh)
            scene_widget.scene.add_geometry("teren_adancime", teren_adancime_cache['geom'], mat_relief)
        scene_widget.scene.show_geometry("teren_real", not is_on)
        scene_widget.scene.show_geometry("teren_adancime", is_on)
        for nume in nume_scara:
            scene_widget.scene.show_geometry(nume, is_on)
        lbl_info.text = ("Adancime: albastru = adanc, rosu = sus" if is_on
                         else "Culori reale, exact cum au fost capturate de camera")

    def comuta_etichete(is_on):
        nonlocal etichete_construite
        stare['etichete'] = is_on
        if is_on and not etichete_construite:
            lbl_info.text = f"Construiesc {len(artefacte)} etichete 3D..."
            construieste_etichete()
            etichete_construite = True
        for nume in nume_etichete:
            scene_widget.scene.show_geometry(nume, is_on)

    def salveaza_captura():
        nume = time.strftime("captura_sit_%Y-%m-%d_%H-%M-%S.png")

        def scris(imagine):
            o3d.io.write_image(nume, imagine)
            print(f"📸 Captura salvata: {os.path.abspath(nume)}")

        scene_widget.scene.scene.render_to_image(scris)

    # --- Unealta de masurat distante: click pe teren in doua puncte. Raza de
    #     clic e calculata analitic din matricea camerei (camera.unproject,
    #     sincron) si intersectata pe CPU cu suprafata (RaycastingScene) -
    #     nu depinde de randarea GPU, deci reactioneaza instant la click. ---
    puncte_masurate = []
    # constructia acceleratiei de raycast (BVH) peste tot terenul poate dura
    # cateva secunde pe un mesh mare - o amanam pana la primul click de
    # masurare in loc s-o platim mereu la deschidere, chiar daca nimeni nu
    # foloseste unealta asta in sesiunea curenta
    raycast_scene_cache = {'scena': None, 'incercat': False}

    def obtine_raycast_scene():
        if not raycast_scene_cache['incercat']:
            raycast_scene_cache['incercat'] = True
            if este_mesh:
                try:
                    rs = o3d.t.geometry.RaycastingScene()
                    rs.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(teren))
                    raycast_scene_cache['scena'] = rs
                except Exception as e:
                    print(f"⚠️ Nu am putut pregati masurarea pe aceasta harta: {e}")
        return raycast_scene_cache['scena']

    def comuta_masurare(is_on):
        stare['masurare'] = is_on
        if is_on:
            stare['zbor'] = False
            btn_zbor.is_on = False
            scene_widget.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
            lbl_info.text = "Mod masurare: click pe teren in doua puncte"
        else:
            lbl_info.text = "Mod rotire: mouse stanga = rotire, dreapta = deplasare, scroll = zoom"

    def deseneaza_masuratoarea():
        p1, p2 = puncte_masurate[-2], puncte_masurate[-1]
        dist = float(np.linalg.norm(p2 - p1))
        dist_oriz = float(np.hypot(p2[0] - p1[0], p2[2] - p1[2]))
        dif_cota = float(abs(p2[1] - p1[1]))

        linie = o3d.geometry.LineSet()
        linie.points = o3d.utility.Vector3dVector([p1, p2])
        linie.lines = o3d.utility.Vector2iVector([[0, 1]])
        linie.colors = o3d.utility.Vector3dVector([[1.0, 0.9, 0.1]])

        eticheta = creeaza_text_3d(f"{dist:.1f} cm", (p1 + p2) / 2.0 + np.array([0, inaltime_text, 0]),
                                   inaltime_text, [1.0, 0.9, 0.1])

        for nume in ("masurare_linie", "masurare_text"):
            if scene_widget.scene.has_geometry(nume):
                scene_widget.scene.remove_geometry(nume)
        scene_widget.scene.add_geometry("masurare_linie", linie, mat_linie)
        scene_widget.scene.add_geometry("masurare_text", eticheta, mat_real)

        lbl_distanta.text = (f"Distanta: {dist:.1f} cm\n"
                             f"orizontal {dist_oriz:.1f} cm\n"
                             f"diferenta cota {dif_cota:.1f} cm")
        print(f"📏 Distanta: {dist:.1f} cm  (orizontal {dist_oriz:.1f} cm, "
              f"diferenta de cota {dif_cota:.1f} cm)")

    def sterge_masuratoarea():
        puncte_masurate.clear()
        for nume in ("masurare_linie", "masurare_text"):
            if scene_widget.scene.has_geometry(nume):
                scene_widget.scene.remove_geometry(nume)
        lbl_distanta.text = "Nicio masuratoare inca"

    def on_mouse(event):
        if not stare['masurare']:
            return gui.SceneWidget.EventCallbackResult.IGNORED
        if not (event.type == gui.MouseEvent.Type.BUTTON_DOWN
                and event.is_button_down(gui.MouseButton.LEFT)):
            return gui.SceneWidget.EventCallbackResult.CONSUMED
        raycast_scene = obtine_raycast_scene()
        if raycast_scene is None:
            lbl_distanta.text = "Masurarea are nevoie de o harta tip suprafata (mesh)."
            return gui.SceneWidget.EventCallbackResult.CONSUMED

        w, h = scene_widget.frame.width, scene_widget.frame.height
        if w <= 0 or h <= 0:
            return gui.SceneWidget.EventCallbackResult.CONSUMED
        x = event.x - scene_widget.frame.x
        y = event.y - scene_widget.frame.y

        cam = scene_widget.scene.camera
        p_apropiat = np.asarray(cam.unproject(x, y, 0.0, w, h), dtype=np.float64)
        p_mijloc = np.asarray(cam.unproject(x, y, 0.5, w, h), dtype=np.float64)
        directie = p_mijloc - p_apropiat
        norma = float(np.linalg.norm(directie))
        if norma < 1e-9:
            return gui.SceneWidget.EventCallbackResult.CONSUMED
        directie /= norma

        raza = o3d.core.Tensor(
            [[p_apropiat[0], p_apropiat[1], p_apropiat[2], directie[0], directie[1], directie[2]]],
            dtype=o3d.core.Dtype.Float32)
        t_hit = float(raycast_scene.cast_rays(raza)['t_hit'].numpy()[0])
        if not np.isfinite(t_hit):
            return gui.SceneWidget.EventCallbackResult.CONSUMED  # click pe fundal

        punct = p_apropiat + directie * t_hit
        puncte_masurate.append(punct)
        if len(puncte_masurate) > 2:
            del puncte_masurate[:-2]
        if len(puncte_masurate) == 2:
            deseneaza_masuratoarea()
        else:
            lbl_distanta.text = "Primul punct ales - mai click o data."
        return gui.SceneWidget.EventCallbackResult.CONSUMED

    scene_widget.set_on_mouse(on_mouse)

    # ================================================================
    #  PANOUL LATERAL
    # ================================================================
    CUL_ACCENT = gui.Color(0.85, 0.65, 0.26)     # auriu-arheologic, culoarea de brand a panoului
    CUL_TEXT_SLAB = gui.Color(0.72, 0.72, 0.72)  # gri pentru text secundar, destul de
                                                 # luminos pe fundal inchis (contrast dark mode)

    panel = gui.Vert(0.65 * em, gui.Margins(0.7 * em, 0.7 * em, 0.7 * em, 0.7 * em))

    if _pe_inapoi is not None:
        buton_inapoi_meniu = gui.Button("Inapoi la meniu")
        buton_inapoi_meniu.vertical_padding_em = 0.4
        buton_inapoi_meniu.set_on_clicked(_pe_inapoi)
        panel.add_child(buton_inapoi_meniu)
        panel.add_fixed(0.2 * em)

    lbl_titlu = gui.Label("SIT ARHEOLOGIC")
    lbl_titlu.font_id = font_titlu
    lbl_titlu.text_color = CUL_ACCENT
    panel.add_child(lbl_titlu)

    lbl_subtitlu = gui.Label(titlu.split("(", 1)[-1].rstrip(")") if "(" in titlu else "")
    lbl_subtitlu.text_color = CUL_TEXT_SLAB
    panel.add_child(lbl_subtitlu)

    panel.add_fixed(0.15 * em)
    lbl_info = gui.Label("Mouse stanga = rotire  ·  scroll = zoom  ·  dreapta = deplasare")
    panel.add_child(lbl_info)
    panel.add_fixed(0.3 * em)

    def buton(text, callback, sectiune, comutabil=False):
        b = gui.Button(text)
        b.toggleable = comutabil
        b.vertical_padding_em = 0.45
        # set_on_clicked cheama mereu callback-ul FARA argumente; pentru
        # butoanele comutabile citim starea noua din chiar butonul care a
        # fost apasat (Open3D ii comuta singur .is_on la fiecare click).
        if comutabil:
            b.set_on_clicked(lambda: callback(b.is_on))
        else:
            b.set_on_clicked(callback)
        sectiune.add_child(b)
        return b

    def sectiune_noua(text):
        s = gui.CollapsableVert(text, 0.35 * em, gui.Margins(em, 0.2 * em, 0, 0.2 * em))
        s.font_id = font_sectiune
        s.set_is_open(True)
        return s

    sec_navigare = sectiune_noua("NAVIGARE")
    buton("Vedere generala", vedere_generala, sec_navigare)
    buton("Vedere de sus", vedere_de_sus, sec_navigare)
    nr_art_txt = f" ({len(artefacte)})" if artefacte else ""
    buton(f"Artefactul urmator{nr_art_txt}", artefact_urmator, sec_navigare)
    btn_zbor = buton("Mod zbor (WASD)", comuta_zbor, sec_navigare, comutabil=True)
    panel.add_child(sec_navigare)

    if artefacte:
        sec_lista = sectiune_noua(f"ARTEFACTE ({len(artefacte)})")

        def eticheta_artefact(art):
            try:
                incredere_txt = f"{float(art['incredere']) * 100:.0f}%"
            except (KeyError, ValueError):
                incredere_txt = "?"
            return f"#{art['id']:<4} {incredere_txt:>4}    {art.get('nr_detectii', '?')} det."

        lista_artefacte = gui.ListView()
        lista_artefacte.set_items([eticheta_artefact(a) for a in artefacte])
        lista_artefacte.set_max_visible_items(10)

        def pe_selectie(valoare_noua, e_dublu_click):
            salt_la_artefact(lista_artefacte.selected_index)

        lista_artefacte.set_on_selection_changed(pe_selectie)
        sec_lista.add_child(lista_artefacte)
        panel.add_child(sec_lista)

    sec_afisare = sectiune_noua("AFISARE")
    buton("Adancime (culori dupa cota)", comuta_adancime, sec_afisare, comutabil=True)
    btn_etichete = buton("Etichete artefacte", comuta_etichete, sec_afisare, comutabil=True)
    # pe siturile mari, etichetele nu sunt construite din start (vezi mai sus) -
    # comutatorul porneste stins, ca starea lui sa reflecte ce se vede cu adevarat
    btn_etichete.is_on = etichete_construite
    panel.add_child(sec_afisare)

    sec_unelte = sectiune_noua("INSTRUMENTE")
    btn_masurare = buton("Mod masurare (click 2 puncte)", comuta_masurare, sec_unelte, comutabil=True)
    buton("Sterge masuratoarea", sterge_masuratoarea, sec_unelte)
    panel_distanta = gui.Vert(0, gui.Margins(0.6 * em, 0.4 * em, 0.6 * em, 0.4 * em))
    panel_distanta.background_color = gui.Color(0.16, 0.16, 0.14)
    lbl_distanta = gui.Label("Nicio masuratoare inca")
    lbl_distanta.text_color = gui.Color(1.0, 0.82, 0.25)
    panel_distanta.add_child(lbl_distanta)
    sec_unelte.add_child(panel_distanta)
    panel.add_child(sec_unelte)

    sec_export = sectiune_noua("EXPORT")
    buton("Salveaza captura PNG", salveaza_captura, sec_export)
    panel.add_child(sec_export)

    panel.add_stretch()
    lbl_footer = gui.Label("Scanare LiDAR · iPhone")
    lbl_footer.text_color = CUL_TEXT_SLAB
    panel.add_child(lbl_footer)

    window.add_child(panel)

    lat_panou = 23 * em

    def pozitioneaza(r):
        scene_widget.frame = gui.Rect(r.x, r.y, r.width - lat_panou, r.height)
        panel.frame = gui.Rect(r.get_right() - lat_panou, r.y, lat_panou, r.height)

    if _gestioneaza_layout:
        def on_layout(layout_context):
            pozitioneaza(window.content_rect)
        window.set_on_layout(on_layout)

    # asiguram un cadru valid INAINTE de a pozitiona camera - fara asta,
    # setup_camera/look_at (in vedere_generala) ruleaza pe un widget cu
    # dimensiune 0x0 (inca n-a trecut niciun layout), iar Filament respinge
    # proiectia ("camera preconditions not met, using default projection").
    # In modul de sine statator, window.set_on_layout(...) de mai sus declanseaza
    # deja un layout imediat - dar cand vizualizatorul e incorporat in fereastra
    # altcuiva (_gestioneaza_layout=False), on_layout-ul acela nu ruleaza aici.
    pozitioneaza(window.content_rect)

    vedere_generala()
    if _porneste_bucla:
        app.run()

    return scene_widget, panel, pozitioneaza


def afiseaza_clasic(teren, este_mesh, artefacte, titlu="Harta 3D - Sit Arheologic"):
    """Varianta de rezerva (vizualizatorul simplu), daca interfata noua nu porneste."""
    v = varfuri(teren, este_mesh)
    y_sol = float(v[:, 1].min())
    geometrii = [teren, o3d.geometry.TriangleMesh.create_coordinate_frame(size=50.0)]
    geometrii += [geom for _, geom in creeaza_marcaje(artefacte, y_sol)]

    viz = o3d.visualization.Visualizer()
    viz.create_window(window_name=titlu, width=1440, height=810)
    for g in geometrii:
        viz.add_geometry(g)
    optiuni = viz.get_render_option()
    optiuni.background_color = np.array(CULOARE_FUNDAL[:3])
    optiuni.point_size = 3.0
    viz.run()
    viz.destroy_window()


def afiseaza_info_consola(artefacte):
    if artefacte:
        print(f"\n🏺 {len(artefacte)} artefacte gasite pe sit:")
        print(f"{'ID':<5}{'Incredere':<12}{'Detectii':<10}{'X (cm)':<10}{'Y (cm)':<10}{'Z (cm)':<10}")
        print("-" * 57)
        for art in artefacte:
            print(f"{art['id']:<5}{art['incredere']:<12}{art['nr_detectii']:<10}"
                  f"{art['x']:<10.1f}{art['y']:<10.1f}{art['z']:<10.1f}")
    else:
        print("\nℹ️ Niciun artefact inregistrat inca.")

    print("\n" + "─" * 56)
    print("  NAVIGARE          mouse stanga = rotire · scroll = zoom")
    print("                    mouse dreapta = deplasare")
    print("  PANOU LATERAL (din dreapta ferestrei)")
    print("    Navigare    - vederi predefinite, salt la artefactul urmator,")
    print("                  mod zbor (WASD + mouse)")
    print("    Artefacte   - lista clicabila: click pe un rand = camera sare direct la el")
    print("    Afisare     - culori reale / adancime (cu scara in cm),")
    print("                  arata/ascunde etichetele artefactelor")
    print("    Instrumente - mod masurare: click pe teren in doua puncte,")
    print("                  distanta apare direct in panou")
    print("    Export      - salveaza captura curenta ca PNG")
    if artefacte:
        print("  PE HARTA          sfere rosii + etichete galbene = artefacte")
    print("─" * 56)


def deschide_folder(folder):
    """Incarca si afiseaza scanarea din folderul dat, intr-o fereastra proprie
    (calea normala: argument din linia de comanda, sau fallback la meniul din
    consola). Deschide vizualizatorul simplu daca cel avansat esueaza."""
    eticheta = nume_frumos(folder)
    titlu = f"Harta 3D - Sit Arheologic ({eticheta})"
    print(f"\n🗺️ Deschid scanarea: {eticheta}.")

    teren, este_mesh = incarca_teren(folder)
    if teren is None:
        return

    artefacte = incarca_artefacte(os.path.join(folder, FISIER_CSV))
    afiseaza_info_consola(artefacte)

    try:
        afiseaza_profesional(teren, este_mesh, artefacte, titlu)
    except Exception as e:
        print(f"⚠️ Vizualizatorul avansat nu a pornit ({e}) - deschid varianta simpla.")
        afiseaza_clasic(teren, este_mesh, artefacte, titlu)


def afiseaza_cu_selector():
    """Deschide fereastra principala imediat si arata o lista GRAFICA (nu
    meniu de consola, nici dialog de rasfoit tot calculatorul) doar cu
    scanarile reale gasite in `scanari\\`, etichetate cu data. Dupa alegere,
    ACEEASI fereastra se populeaza cu harta - nu se deschide o fereastra noua:
    inchiderea si redeschiderea unei ferestre Open3D in aceeasi sesiune s-a
    dovedit instabila la testare (blocaje native reproductibile)."""
    app = gui.Application.instance
    app.initialize()
    fonturi = _fonturi_panou(app)

    window = app.create_window("Harta 3D - Sit Arheologic", 1600, 900)
    window.show(True)
    em = window.theme.font_size

    scanari_disp = lista_scanari()

    selector = gui.Vert(0.6 * em, gui.Margins(2 * em, 2 * em, 2 * em, 2 * em))
    lbl_titlu = gui.Label("ALEGE SCANAREA")
    lbl_titlu.font_id = fonturi[0]
    lbl_titlu.text_color = gui.Color(0.85, 0.65, 0.26)
    selector.add_child(lbl_titlu)

    if not scanari_disp:
        selector.add_child(gui.Label(
            "Nu exista nicio scanare salvata in 'scanari\\'. Ruleaza mai intai scaner.py."))
        window.add_child(selector)

        def layout_gol(ctx):
            r = window.content_rect
            selector.frame = gui.Rect(r.x, r.y, r.width, r.height)
        window.set_on_layout(layout_gol)
        app.run()
        return

    selector.add_child(gui.Label(f"{len(scanari_disp)} scanari gasite - alege una din lista:"))

    lista = gui.ListView()
    lista.set_items([f"{nume_frumos(f)}   ·   {numar_artefacte(f)} artefacte" for f in scanari_disp])
    lista.set_max_visible_items(14)
    lista.selected_index = 0
    selector.add_child(lista)

    lbl_stare = gui.Label("")
    selector.add_child(lbl_stare)

    def deschide_index(idx):
        folder = scanari_disp[idx]
        selector.visible = False
        eticheta = nume_frumos(folder)
        titlu = f"Harta 3D - Sit Arheologic ({eticheta})"
        print(f"\n🗺️ Deschid scanarea: {eticheta}.")

        teren, este_mesh = incarca_teren(folder)
        if teren is None:
            window.close()
            return
        artefacte = incarca_artefacte(os.path.join(folder, FISIER_CSV))
        afiseaza_info_consola(artefacte)

        try:
            afiseaza_profesional(teren, este_mesh, artefacte, titlu,
                                 _app=app, _window=window, _fonturi=fonturi,
                                 _porneste_bucla=False)
        except Exception as e:
            print(f"⚠️ Vizualizatorul avansat a esuat ({e}).")
            window.close()

    def pe_selectie(valoare_noua, e_dublu_click):
        if e_dublu_click:
            deschide_index(lista.selected_index)
        else:
            lbl_stare.text = f"Selectat: {valoare_noua}  (dublu-click sau butonul de mai jos ca s-o deschizi)"
    lista.set_on_selection_changed(pe_selectie)

    buton_deschide = gui.Button("Deschide scanarea selectata")
    buton_deschide.vertical_padding_em = 0.4
    buton_deschide.set_on_clicked(lambda: deschide_index(lista.selected_index))
    selector.add_child(buton_deschide)

    window.add_child(selector)

    def on_layout(ctx):
        r = window.content_rect
        lat = min(50 * em, r.width)
        x = r.x + (r.width - lat) / 2.0
        selector.frame = gui.Rect(x, r.y, lat, r.height)
    window.set_on_layout(on_layout)

    print(f"📂 {len(scanari_disp)} scanari disponibile - alege una din fereastra care s-a deschis.")

    app.run()


def afiseaza_harta_sit(argumente=None):
    # Folderul se poate da si ca argument: python harta_3d.py scanari\2026-08-27_16-30
    if argumente is None:
        argumente = sys.argv[1:]
    if argumente:
        deschide_folder(argumente[0])
        return

    try:
        afiseaza_cu_selector()
    except Exception as e:
        print(f"⚠️ Selectorul grafic nu a pornit ({e}) - trec pe meniul din consola.")
        folder = alege_scanare()
        if folder is None:
            return
        deschide_folder(folder)


if __name__ == "__main__":
    afiseaza_harta_sit()
