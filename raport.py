# -*- coding: utf-8 -*-
"""Genereaza raportul scanarii: un fisier HTML de sine statator (cu harta
vazuta de sus, statisticile sitului si tabelul artefactelor cu pozele lor),
sau varianta PDF, paginata, pentru arhivare/tiparire.

Rulare:
    python raport.py                            -> raport HTML, ultima scanare
    python raport.py scanari\\2026-08-25_14-59   -> raport HTML, o anumita scanare
    python raport.py --pdf                      -> raport PDF, ultima scanare
    python raport.py --pdf scanari\\2026-08-25_14-59
"""
import base64
import csv
import io
import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MultipleLocator

FISIER_MESH = "harta_sit_mesh.ply"
FISIER_PUNCTE = "harta_completa_sit.ply"
FISIER_CSV = "coordonate_sit.csv"
DIRECTOR_POZE = "poze_artefacte"
PAGINA_A4 = (8.27, 11.69)


def incarca_teren(folder):
    """Returneaza (puncte Nx3, culori Nx3 in [0,1]) din mesh sau din norul de puncte."""
    import open3d as o3d
    cale_mesh = os.path.join(folder, FISIER_MESH)
    if os.path.exists(cale_mesh):
        mesh = o3d.io.read_triangle_mesh(cale_mesh)
        if not mesh.is_empty():
            return np.asarray(mesh.vertices), np.asarray(mesh.vertex_colors)
    cale_pcd = os.path.join(folder, FISIER_PUNCTE)
    if os.path.exists(cale_pcd):
        pcd = o3d.io.read_point_cloud(cale_pcd)
        if not pcd.is_empty():
            return np.asarray(pcd.points), np.asarray(pcd.colors)
    return None, None


def incarca_artefacte(folder):
    artefacte = []
    cale = os.path.join(folder, FISIER_CSV)
    if not os.path.exists(cale):
        return artefacte
    with open(cale, mode='r', newline='', encoding='utf-8') as f:
        for rand in csv.DictReader(f):
            try:
                artefacte.append({
                    'id': rand['ID_Artefact'],
                    'incredere': rand.get('Incredere', ''),
                    'nr_detectii': rand.get('Nr_Detectii', '1'),
                    'x': float(rand['X_Centru']),
                    'y': float(rand['Y_Centru']),
                    'z': float(rand['Z_Centru']),
                    'poza': rand.get('Poza', ''),
                })
            except (KeyError, ValueError):
                continue
    return artefacte


def poza_base64(folder, nume_poza, latime_max=360):
    """Poza artefactului, micsorata si codata base64, pentru includere in HTML."""
    if not nume_poza:
        return None
    cale = os.path.join(folder, DIRECTOR_POZE, nume_poza)
    if not os.path.exists(cale):
        return None
    try:
        import cv2
        img = cv2.imread(cale)
        if img is None:
            return None
        h, w = img.shape[:2]
        if w > latime_max:
            img = cv2.resize(img, (latime_max, int(h * latime_max / w)))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if not ok:
            return None
        return base64.b64encode(buf.tobytes()).decode("ascii")
    except Exception:
        return None


def _marcheaza_artefacte(ax, artefacte):
    for art in artefacte:
        ax.plot(art['x'], art['z'], marker='x', color='red', markersize=9, markeredgewidth=2.5)
        ax.annotate(f"#{art['id']}", (art['x'], art['z']),
                    xytext=(5, 5), textcoords='offset points',
                    color='red', fontsize=9, fontweight='bold')


def _construieste_harta_fig(puncte, culori, artefacte, dupa_adancime=False, figsize=(9.5, 7)):
    """Construieste figura matplotlib a vederii de sus (plan X-Z), cu grila de
    50 cm si artefactele marcate. Refolosita atat pentru raportul HTML (codata
    base64), cat si pentru raportul PDF (adaugata direct ca pagina)."""
    fig, ax = plt.subplots(figsize=figsize)
    # rasterizat explicit: cu sute de mii de puncte, un PDF vectorial (fiecare
    # punct ca obiect separat) ar iesi zeci de MB si greu de deschis - randam
    # norul de puncte ca imagine, dar textul/tabelul raman vectoriale, nete
    if dupa_adancime:
        sc = ax.scatter(puncte[:, 0], puncte[:, 2], s=0.4, c=puncte[:, 1],
                        cmap='turbo', linewidths=0, rasterized=True)
        bara = fig.colorbar(sc, ax=ax, shrink=0.8, pad=0.02)
        bara.set_label("cota (cm): albastru = adanc, rosu = sus")
    else:
        c = culori if culori is not None and len(culori) == len(puncte) else None
        ax.scatter(puncte[:, 0], puncte[:, 2], s=0.4, c=c, linewidths=0, rasterized=True)

    _marcheaza_artefacte(ax, artefacte)
    ax.set_aspect('equal')
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Z (cm)")
    ax.xaxis.set_major_locator(MultipleLocator(50))
    ax.yaxis.set_major_locator(MultipleLocator(50))
    ax.grid(True, linewidth=0.4, alpha=0.5)
    ax.tick_params(labelsize=8)
    ax.set_title("Vedere de sus" + (" - colorata dupa cota" if dupa_adancime else " - culori reale"),
                 fontsize=11)
    fig.tight_layout()
    return fig


def vedere_de_sus(puncte, culori, artefacte, dupa_adancime=False):
    """Vederea de sus a sitului, codata ca imagine PNG base64 (pentru HTML)."""
    fig = _construieste_harta_fig(puncte, culori, artefacte, dupa_adancime)
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def ultima_scanare():
    """Cel mai nou folder de scanare cu harta; fisierele vechi din radacina au
    prioritate (structura de dinainte de migrare)."""
    def are_harta(folder):
        return (os.path.exists(os.path.join(folder, FISIER_MESH))
                or os.path.exists(os.path.join(folder, FISIER_PUNCTE)))

    if are_harta("."):
        return "."
    candidati = []
    for radacina in ("scanari", "scanari_anterioare"):
        if os.path.isdir(radacina):
            for d in os.listdir(radacina):
                cale = os.path.join(radacina, d)
                if os.path.isdir(cale) and are_harta(cale):
                    candidati.append(cale)
    if not candidati:
        return None
    return max(candidati, key=lambda c: os.path.basename(c))


def _pagina_titlu_pdf(pdf, folder, puncte, artefacte, dim, data_scanarii):
    fig = plt.figure(figsize=PAGINA_A4)
    fig.text(0.5, 0.82, "RAPORT SCANARE", ha="center", fontsize=24,
             fontweight="bold", color="#3a2a1a")
    fig.text(0.5, 0.775, "SIT ARHEOLOGIC", ha="center", fontsize=17, color="#8c5a2b")
    fig.text(0.5, 0.72, data_scanarii, ha="center", fontsize=12, color="#555")

    linii = [
        ("Artefacte gasite", f"{len(artefacte)}"),
        ("Intinderea sitului (X x Z)", f"{dim[0] / 100:.2f} x {dim[2] / 100:.2f} m"),
        ("Diferenta de nivel", f"{dim[1]:.0f} cm"),
        ("Puncte 3D", f"{len(puncte):,}"),
    ]
    y = 0.58
    for eticheta, valoare in linii:
        fig.text(0.3, y, eticheta, fontsize=11, color="#555")
        fig.text(0.7, y, valoare, fontsize=11, fontweight="bold", ha="right")
        y -= 0.045

    fig.text(0.5, 0.08, f"Folder scanare: {os.path.abspath(folder)}",
             ha="center", fontsize=7, color="#999")
    fig.text(0.5, 0.055,
             "Coordonatele sunt in centimetri, relative la punctul de pornire al scanarii "
             "(nu geografic).", ha="center", fontsize=7, color="#999")
    fig.text(0.5, 0.03, "Generat automat de sistemul de scanare LiDAR",
             ha="center", fontsize=7, color="#999")
    pdf.savefig(fig)
    plt.close(fig)


def _pagina_tabel_pdf(pdf, artefacte, randuri_per_pagina=32):
    if not artefacte:
        fig = plt.figure(figsize=PAGINA_A4)
        fig.text(0.5, 0.5, "Niciun artefact inregistrat la aceasta scanare.",
                ha="center", va="center", fontsize=12, color="#777")
        pdf.savefig(fig)
        plt.close(fig)
        return

    antet = ["Artefact", "Incredere", "Detectii", "X (cm)", "Y/cota (cm)", "Z (cm)"]
    for start in range(0, len(artefacte), randuri_per_pagina):
        bucata = artefacte[start:start + randuri_per_pagina]
        fig = plt.figure(figsize=PAGINA_A4)
        ax = fig.add_axes([0.06, 0.05, 0.88, 0.88])
        ax.axis("off")
        titlu = "Registrul artefactelor"
        if start:
            titlu += f" (continuare, {start + 1}-{start + len(bucata)})"
        ax.set_title(titlu, fontsize=13, fontweight="bold", color="#8c5a2b", loc="left")

        randuri = []
        for art in bucata:
            incredere = art["incredere"]
            try:
                incredere = f"{float(incredere) * 100:.0f}%"
            except ValueError:
                pass
            randuri.append([f"#{art['id']}", incredere, str(art["nr_detectii"]),
                           f"{art['x']:.1f}", f"{art['y']:.1f}", f"{art['z']:.1f}"])

        tabel = ax.table(cellText=randuri, colLabels=antet, loc="upper left",
                         cellLoc="center", colLoc="center")
        tabel.auto_set_font_size(False)
        tabel.set_fontsize(8)
        tabel.scale(1, 1.35)
        for (rand, col), celula in tabel.get_celld().items():
            if rand == 0:
                celula.set_facecolor("#8c5a2b")
                celula.set_text_props(color="white", fontweight="bold")
            elif rand % 2 == 0:
                celula.set_facecolor("#faf8f5")

        pdf.savefig(fig)
        plt.close(fig)


def genereaza_pdf(folder=None):
    """Construieste raportul PDF, paginat (pagina de titlu, cele doua harti
    de sus, registrul artefactelor) - format de arhivat/tiparit. Returneaza
    calea lui."""
    if folder is None:
        folder = ultima_scanare()
        if folder is None:
            print("Nu am gasit nicio scanare - nu pot face raportul PDF.")
            return None
        print(f"Generez raportul PDF pentru ultima scanare: {folder}")

    puncte, culori = incarca_teren(folder)
    if puncte is None or len(puncte) == 0:
        print("Nu am gasit nicio harta in folderul dat - nu pot face raportul PDF.")
        return None
    artefacte = incarca_artefacte(folder)

    mn, mx = puncte.min(axis=0), puncte.max(axis=0)
    dim = mx - mn
    data_scanarii = time.strftime("%d.%m.%Y %H:%M", time.localtime(
        os.path.getmtime(os.path.join(folder, FISIER_MESH))
        if os.path.exists(os.path.join(folder, FISIER_MESH))
        else os.path.getmtime(os.path.join(folder, FISIER_PUNCTE))))

    nume = time.strftime("raport_sit_%Y-%m-%d_%H-%M.pdf")
    cale = os.path.join(folder, nume)

    with PdfPages(cale) as pdf:
        _pagina_titlu_pdf(pdf, folder, puncte, artefacte, dim, data_scanarii)

        for dupa_adancime in (False, True):
            fig = _construieste_harta_fig(puncte, culori, artefacte,
                                          dupa_adancime=dupa_adancime, figsize=PAGINA_A4)
            pdf.savefig(fig, dpi=150)
            plt.close(fig)

        _pagina_tabel_pdf(pdf, artefacte)

        info = pdf.infodict()
        info["Title"] = f"Raport scanare sit arheologic - {data_scanarii}"
        info["Author"] = "Sistem de scanare LiDAR"

    print(f"Raportul PDF a fost generat: {os.path.abspath(cale)}")
    return cale


def genereaza(folder=None):
    """Construieste raportul HTML in folderul scanarii. Returneaza calea lui."""
    if folder is None:
        folder = ultima_scanare()
        if folder is None:
            print("❌ Nu am gasit nicio scanare - nu pot face raportul.")
            return None
        print(f"ℹ️ Generez raportul pentru ultima scanare: {folder}")
    puncte, culori = incarca_teren(folder)
    if puncte is None or len(puncte) == 0:
        print("❌ Nu am gasit nicio harta in folderul dat - nu pot face raportul.")
        return None
    artefacte = incarca_artefacte(folder)

    mn, mx = puncte.min(axis=0), puncte.max(axis=0)
    dim = mx - mn
    data_scanarii = time.strftime("%d.%m.%Y %H:%M", time.localtime(
        os.path.getmtime(os.path.join(folder, FISIER_MESH))
        if os.path.exists(os.path.join(folder, FISIER_MESH))
        else os.path.getmtime(os.path.join(folder, FISIER_PUNCTE))))

    img_real = vedere_de_sus(puncte, culori, artefacte, dupa_adancime=False)
    img_adancime = vedere_de_sus(puncte, culori, artefacte, dupa_adancime=True)

    randuri = []
    for art in artefacte:
        b64 = poza_base64(folder, art.get('poza', ''))
        celula_poza = (f'<img src="data:image/jpeg;base64,{b64}" alt="poza">'
                       if b64 else '<span class="fara">fara poza</span>')
        incredere = art['incredere']
        try:
            incredere = f"{float(incredere) * 100:.0f}%"
        except ValueError:
            pass
        randuri.append(f"""
        <tr>
          <td class="id">Artefact #{art['id']}</td>
          <td>{celula_poza}</td>
          <td>{incredere}</td>
          <td>{art['nr_detectii']}</td>
          <td>{art['x']:.1f}</td>
          <td>{art['y']:.1f}</td>
          <td>{art['z']:.1f}</td>
        </tr>""")

    tabel = f"""
      <table>
        <thead><tr>
          <th>Artefact</th><th>Fotografie</th><th>Incredere</th>
          <th>Detectii</th><th>X (cm)</th><th>Y / cota (cm)</th><th>Z (cm)</th>
        </tr></thead>
        <tbody>{''.join(randuri)}</tbody>
      </table>""" if randuri else "<p class='fara'>Niciun artefact inregistrat la aceasta scanare.</p>"

    html = f"""<!DOCTYPE html>
<html lang="ro">
<head>
<meta charset="utf-8">
<title>Raport scanare sit - {data_scanarii}</title>
<style>
  body {{ font-family: 'Segoe UI', Arial, sans-serif; margin: 0; background: #f4f3f0; color: #232323; }}
  .pagina {{ max-width: 960px; margin: 0 auto; padding: 32px 40px 60px; background: #fff; }}
  header {{ border-bottom: 3px solid #8c5a2b; padding-bottom: 14px; margin-bottom: 24px; }}
  h1 {{ margin: 0 0 4px; font-size: 26px; }}
  .subtitlu {{ color: #6b6b6b; font-size: 14px; }}
  h2 {{ font-size: 18px; margin: 32px 0 10px; color: #8c5a2b; }}
  .statistici {{ display: flex; flex-wrap: wrap; gap: 12px; margin-top: 18px; }}
  .stat {{ flex: 1 1 130px; background: #faf7f2; border: 1px solid #e8e0d4; border-radius: 8px;
           padding: 10px 14px; }}
  .stat .valoare {{ font-size: 20px; font-weight: 600; }}
  .stat .nume {{ font-size: 12px; color: #6b6b6b; }}
  img.harta {{ width: 100%; border: 1px solid #ddd; border-radius: 6px; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
  th {{ background: #8c5a2b; color: #fff; padding: 7px 8px; text-align: left; }}
  td {{ border-bottom: 1px solid #e5e0d8; padding: 6px 8px; vertical-align: middle; }}
  tr:nth-child(even) td {{ background: #faf8f5; }}
  td.id {{ font-weight: 700; color: #8c5a2b; }}
  td img {{ max-width: 180px; border-radius: 4px; display: block; }}
  .fara {{ color: #999; font-style: italic; }}
  footer {{ margin-top: 40px; font-size: 11px; color: #999; border-top: 1px solid #eee; padding-top: 10px; }}
  @media print {{ body {{ background: #fff; }} .pagina {{ padding: 0; }} }}
</style>
</head>
<body>
<div class="pagina">
  <header>
    <h1>Raport scanare sit arheologic</h1>
    <div class="subtitlu">Scanare finalizata la {data_scanarii} &middot; generat automat de sistemul de scanare LiDAR</div>
    <div class="statistici">
      <div class="stat"><div class="valoare">{len(artefacte)}</div><div class="nume">artefacte gasite</div></div>
      <div class="stat"><div class="valoare">{dim[0] / 100:.1f} &times; {dim[2] / 100:.1f} m</div><div class="nume">intinderea sitului (X &times; Z)</div></div>
      <div class="stat"><div class="valoare">{dim[1]:.0f} cm</div><div class="nume">diferenta de nivel</div></div>
      <div class="stat"><div class="valoare">{len(puncte):,}</div><div class="nume">puncte 3D</div></div>
    </div>
  </header>

  <h2>Harta sitului - vedere de sus</h2>
  <p>Grila are pasul de <b>50 cm</b>. Artefactele sunt marcate cu <span style="color:red">&#10005;</span> rosu si numarul lor de identificare.</p>
  <img class="harta" src="data:image/png;base64,{img_real}" alt="vedere de sus culori reale">

  <h2>Relieful sitului (cote)</h2>
  <img class="harta" src="data:image/png;base64,{img_adancime}" alt="vedere de sus dupa cota">

  <h2>Registrul artefactelor</h2>
  {tabel}

  <footer>
    Coordonatele sunt in centimetri, relative la punctul de pornire al scanarii.
    Y este cota (negativ = sub punctul de pornire). Fisiere asociate: {FISIER_MESH},
    {FISIER_PUNCTE}, {FISIER_CSV}, {DIRECTOR_POZE}/.
  </footer>
</div>
</body>
</html>"""

    nume = time.strftime("raport_sit_%Y-%m-%d_%H-%M.html")
    cale_raport = os.path.join(folder, nume)
    with open(cale_raport, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"📄 Raportul a fost generat: {os.path.abspath(cale_raport)}")
    return cale_raport


def main(argv=None):
    argumente = list(argv) if argv is not None else sys.argv[1:]
    vrea_pdf = "--pdf" in argumente
    argumente = [a for a in argumente if a != "--pdf"]
    folder_dat = argumente[0] if argumente else None

    if vrea_pdf:
        genereaza_pdf(folder_dat)
    else:
        genereaza(folder_dat)


if __name__ == "__main__":
    main()
