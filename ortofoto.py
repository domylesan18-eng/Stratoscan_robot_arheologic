# -*- coding: utf-8 -*-
"""Genereaza ortofotoplanul unei scanari: o fotografie de sus, perfect
perpendiculara, cu grila precisa, rigla grafica de scara si artefactele
marcate - formatul standard pentru documentatia de santier arheologic.

Rulare:
    python ortofoto.py                            -> pentru ultima scanare
    python ortofoto.py scanari\\2026-08-25_14-59   -> pentru o anumita scanare
"""
import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harta_3d
import raport


def alege_pas_grid_cm(intindere_cm):
    """Alege un pas de grila 'rotund' (cm), ca sa iasa undeva la 6-10 linii
    pe latura mare a sitului."""
    candidati = [10, 20, 25, 50, 100, 200, 250, 500, 1000, 2000]
    tinta = 8
    for pas in candidati:
        if intindere_cm / pas <= tinta * 1.6:
            return pas
    return candidati[-1]


def alege_lungime_rigla_cm(intindere_cm):
    """Lungimea totala a riglei grafice - o fractiune 'rotunda' din intinderea
    sitului, ca rigla sa fie nici minuscula, nici mai lata decat harta."""
    tinta = intindere_cm * 0.35
    candidati = [10, 20, 25, 50, 100, 200, 250, 500, 1000, 2000, 5000]
    return min(candidati, key=lambda c: abs(c - tinta))


def randeaza_foto_de_sus(teren, este_mesh, latura_px=2400, padding=0.04, fov_grade=7.0):
    """Randeaza terenul vazut perfect de sus, cu o camera cu FOV ingust
    pozitionata foarte sus (aproximeaza o proiectie ortografica, fara
    paralaxa vizibila, evitand un bug de proiectie ortografica din Open3D).
    Returneaza (imagine RGB uint8, extent=(x_min, x_max, z_min, z_max) cm)."""
    from open3d.visualization import gui, rendering

    v = harta_3d.varfuri(teren, este_mesh)
    bbox_min, bbox_max = v.min(axis=0), v.max(axis=0)
    centru = ((bbox_min + bbox_max) / 2.0).astype(np.float32)
    latime_x = float(bbox_max[0] - bbox_min[0]) * (1.0 + padding * 2)
    latime_z = float(bbox_max[2] - bbox_min[2]) * (1.0 + padding * 2)

    aspect = latime_x / max(latime_z, 1e-6)
    h_px = latura_px if aspect <= 1 else max(1, int(round(latura_px / aspect)))
    w_px = latura_px if aspect >= 1 else max(1, int(round(latura_px * aspect)))

    # distanta camerei: suficient de departe incat AMBELE laturi (X si Z) ale
    # sitului sa incapa in cadru, dat fiind FOV-ul vertical si aspectul (w/h)
    fov_rad = np.radians(fov_grade)
    dist_pt_z = (latime_z / 2.0) / np.tan(fov_rad / 2.0)
    dist_pt_x = (latime_x / 2.0) / (np.tan(fov_rad / 2.0) * (w_px / h_px))
    distanta = max(dist_pt_z, dist_pt_x)

    app = gui.Application.instance
    app.initialize()
    window = app.create_window("ortofoto-render", w_px, h_px)
    window.show(True)
    sw = gui.SceneWidget()
    sw.scene = rendering.Open3DScene(window.renderer)
    sw.scene.set_background((1.0, 1.0, 1.0, 1.0))
    sw.scene.show_skybox(False)
    window.add_child(sw)

    def on_layout(ctx):
        r = window.content_rect
        sw.frame = gui.Rect(r.x, r.y, r.width, r.height)
    window.set_on_layout(on_layout)

    harta_3d.aplica_grading_neutru(sw.scene)
    mat = harta_3d.material_teren_real()
    sw.scene.add_geometry("teren", teren, mat)
    app.run_one_tick()

    eye = centru + np.array([0.0, distanta, 0.0], dtype=np.float32)
    sw.look_at(centru, eye, np.array([0.0, 0.0, -1.0], dtype=np.float32))
    sw.setup_camera(fov_grade, sw.scene.bounding_box, centru)
    sw.look_at(centru, eye, np.array([0.0, 0.0, -1.0], dtype=np.float32))

    rezultat = {}
    def capturat(img):
        rezultat['img'] = np.asarray(img)
    for i in range(400):
        app.run_one_tick()
        window.post_redraw()
        if i == 30:
            sw.scene.scene.render_to_image(capturat)
        if 'img' in rezultat:
            break
        time.sleep(0.005)

    w_real, h_real = sw.frame.width, sw.frame.height
    window.close()
    for i in range(3):
        app.run_one_tick()

    if 'img' not in rezultat:
        return None, None

    # extentul real vizibil (cm), calculat din geometria exacta a camerei
    # (semi-inaltime = distanta * tan(fov/2); semi-latime = semi-inaltime * aspect real)
    semi_h = distanta * np.tan(fov_rad / 2.0)
    semi_w = semi_h * (w_real / h_real)
    extent = (centru[0] - semi_w, centru[0] + semi_w,
             centru[2] - semi_h, centru[2] + semi_h)
    return rezultat['img'], extent


def deseneaza_rigla_grafica(ax, lungime_cm, x0_fr=0.03, y0_fr=0.05, latime_fr=0.30):
    """Rigla grafica (scara cartografica): bara alternativ neagra/alba, cu
    subdiviziuni si etichete in cm/m - standardul din planurile arheologice."""
    n = 5
    y1_fr = y0_fr + 0.018
    for i in range(n):
        x_a = x0_fr + latime_fr * i / n
        x_b = x0_fr + latime_fr * (i + 1) / n
        culoare = "black" if i % 2 == 0 else "white"
        ax.add_patch(patches.Rectangle((x_a, y0_fr), x_b - x_a, y1_fr - y0_fr,
                                       transform=ax.transAxes, facecolor=culoare,
                                       edgecolor="black", linewidth=0.8, zorder=10))
    ax.add_patch(patches.Rectangle((x0_fr, y0_fr), latime_fr, y1_fr - y0_fr,
                                   transform=ax.transAxes, facecolor="none",
                                   edgecolor="black", linewidth=0.8, zorder=11))

    eticheta_val = lambda cm: f"{cm/100:g} m" if cm >= 100 else f"{cm:g} cm"
    for i in range(n + 1):
        cm_val = lungime_cm * i / n
        x = x0_fr + latime_fr * i / n
        ax.text(x, y0_fr - 0.012, eticheta_val(cm_val), transform=ax.transAxes,
                ha="center", va="top", fontsize=8, zorder=11)
    ax.text(x0_fr + latime_fr / 2, y1_fr + 0.012, "SCARA GRAFICA", transform=ax.transAxes,
            ha="center", va="bottom", fontsize=7.5, color="#444", zorder=11)


def genereaza(folder=None, latura_px=2400):
    """Construieste ortofotoplanul PNG in folderul scanarii. Returneaza calea lui."""
    if folder is None:
        folder = raport.ultima_scanare()
        if folder is None:
            print("❌ Nu am gasit nicio scanare - nu pot face ortofotoplanul.")
            return None
        print(f"ℹ️ Generez ortofotoplanul pentru ultima scanare: {folder}")

    teren, este_mesh = harta_3d.incarca_teren(folder)
    if teren is None:
        return None
    artefacte = harta_3d.incarca_artefacte(os.path.join(folder, harta_3d.FISIER_CSV))

    v = harta_3d.varfuri(teren, este_mesh)
    bbox_min, bbox_max = v.min(axis=0), v.max(axis=0)
    intindere_x = float(bbox_max[0] - bbox_min[0])
    intindere_z = float(bbox_max[2] - bbox_min[2])
    intindere_mare = max(intindere_x, intindere_z)

    print("🛰️ Randez fotografia de sus (perpendiculara)...")
    foto, extent = randeaza_foto_de_sus(teren, este_mesh, latura_px=latura_px)
    if foto is None:
        print("❌ Randarea a esuat - nu pot construi ortofotoplanul.")
        return None

    pas_grid = alege_pas_grid_cm(intindere_mare)
    lungime_rigla = alege_lungime_rigla_cm(intindere_mare)

    fig, ax = plt.subplots(figsize=(13, 13 * foto.shape[0] / foto.shape[1] + 1.4))
    ax.imshow(foto, extent=extent, origin="upper", interpolation="bilinear")

    # grila precisa, pe multipli exacti ai pasului ales
    x0 = np.ceil(extent[0] / pas_grid) * pas_grid
    x1 = np.floor(extent[1] / pas_grid) * pas_grid
    z0 = np.ceil(extent[2] / pas_grid) * pas_grid
    z1 = np.floor(extent[3] / pas_grid) * pas_grid
    xs = np.arange(x0, x1 + pas_grid / 2, pas_grid)
    zs = np.arange(z0, z1 + pas_grid / 2, pas_grid)
    for x in xs:
        ax.axvline(x, color="yellow", linewidth=0.6, alpha=0.75, zorder=5)
    for z in zs:
        ax.axhline(z, color="yellow", linewidth=0.6, alpha=0.75, zorder=5)
    ax.set_xticks(xs)
    ax.set_yticks(zs)
    ax.tick_params(labelsize=8)
    unitate = "m" if pas_grid >= 100 else "cm"
    div = 100.0 if unitate == "m" else 1.0
    ax.set_xticklabels([f"{x/div:g}" for x in xs])
    ax.set_yticklabels([f"{z/div:g}" for z in zs])
    ax.set_xlabel(f"X ({unitate}) — pas grila {pas_grid/div:g} {unitate}", fontsize=9)
    ax.set_ylabel(f"Z ({unitate})", fontsize=9)

    # artefactele
    for art in artefacte:
        ax.plot(art["x"], art["z"], marker="o", markersize=7,
                markerfacecolor="red", markeredgecolor="white", markeredgewidth=1.2, zorder=8)
        ax.annotate(f"#{art['id']}", (art["x"], art["z"]), xytext=(6, 6),
                    textcoords="offset points", color="red", fontsize=8,
                    fontweight="bold", zorder=9)

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[3], extent[2])  # Z creste "in jos" in imagine, ca la harta
    for spine in ax.spines.values():
        spine.set_linewidth(1.2)

    deseneaza_rigla_grafica(ax, lungime_rigla)

    data_scanarii = time.strftime("%d.%m.%Y %H:%M", time.localtime(
        os.path.getmtime(os.path.join(folder, harta_3d.FISIER_MESH))
        if os.path.exists(os.path.join(folder, harta_3d.FISIER_MESH))
        else os.path.getmtime(os.path.join(folder, harta_3d.FISIER_PUNCTE))))
    titlu = f"ORTOFOTOPLAN SIT ARHEOLOGIC — {data_scanarii}"
    subtitlu = (f"Intindere: {intindere_x/100:.2f} × {intindere_z/100:.2f} m   ·   "
               f"{len(artefacte)} artefacte   ·   grila {pas_grid/div:g} {unitate}   ·   "
               f"coordonate relative la punctul de pornire al scanarii (nu geografic)")
    fig.suptitle(titlu, fontsize=13, fontweight="bold", y=0.985)
    ax.set_title(subtitlu, fontsize=8.5, color="#555", pad=8)

    fig.text(0.5, 0.005, "Generat automat de sistemul de scanare LiDAR · fara orientare pe nord magnetic",
             ha="center", fontsize=7, color="#888")

    fig.tight_layout(rect=[0, 0.015, 1, 0.965])

    nume = time.strftime("ortofoto_%Y-%m-%d_%H-%M.png")
    cale = os.path.join(folder, nume)
    fig.savefig(cale, dpi=300)
    plt.close(fig)
    print(f"🖼️ Ortofotoplanul a fost generat: {os.path.abspath(cale)}")
    return cale


def main(argv=None):
    argumente = list(argv) if argv is not None else sys.argv[1:]
    genereaza(argumente[0] if argumente else None)


if __name__ == "__main__":
    main()
