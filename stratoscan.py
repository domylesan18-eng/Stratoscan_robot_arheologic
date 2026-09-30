# -*- coding: utf-8 -*-
"""STRATOSCAN - meniu principal. Un singur punct de intrare pentru intregul
flux: scanare, vizualizare, raport si ortofotoplan - in loc sa rulezi manual
fiecare script separat.

Rulare:
    python stratoscan.py
"""
import os
import subprocess
import sys
import threading

# Cand aplicatia e impachetata cu PyInstaller intr-un singur executabil,
# sys.executable e chiar Stratoscan.exe (nu exista un python.exe separat de
# gasit) si __file__ nu mai are un folder real de proiect langa el. Detectam
# asta si ne adaptam: folderul de baza devine folderul executabilului, iar
# subprocesele se pornesc reinvocand ACELASI executabil cu un flag --modul,
# in loc sa caute un script .py pe disc.
CONGELAT = getattr(sys, "frozen", False)
BAZA = os.path.dirname(sys.executable) if CONGELAT else os.path.dirname(os.path.abspath(__file__))
PYTHON = sys.executable
sys.path.insert(0, BAZA)


def ruleaza(script, *args):
    """Porneste un script al proiectului ca proces separat (fiecare unealta
    isi are propriile ferestre/librarii grafice - le tinem izolate, nu
    importate in acelasi proces, ca sa nu se interfereze intre ele)."""
    if CONGELAT:
        nume_modul = os.path.splitext(script)[0]
        subprocess.run([sys.executable, "--modul", nume_modul, *args], cwd=BAZA)
    else:
        cale = os.path.join(BAZA, script)
        subprocess.run([PYTHON, cale, *args], cwd=BAZA)


def ruleaza_modul(nume_modul, argumente):
    """Executa un 'modul' al proiectului chiar in acest proces - folosit doar
    cand executabilul congelat e reinvocat cu --modul (vezi ruleaza() de mai
    sus). Fiecare invocare e un proces nou (pornit de subprocess.run), deci
    izolarea ferestrelor ramane aceeasi ca in modul de dezvoltare."""
    if nume_modul == "scaner":
        import scaner
        scaner.main(argumente)
    elif nume_modul == "harta_3d":
        import harta_3d
        harta_3d.afiseaza_harta_sit(argumente)
    elif nume_modul == "raport":
        import raport
        raport.main(argumente)
    elif nume_modul == "ortofoto":
        import ortofoto
        ortofoto.main(argumente)
    else:
        print(f"Modul necunoscut: {nume_modul}")
        sys.exit(1)


def alege_folder_consola():
    """Meniul din consola al harta_3d.py (Enter = ultima scanare, sau un
    numar pentru oricare alta) - folosit doar de fallback-ul din consola."""
    import harta_3d
    return harta_3d.alege_scanare()


def meniu_consola():
    """Fallback text, daca interfata grafica nu poate porni."""
    def _cu_folder(script, *flaguri):
        folder = alege_folder_consola()
        if folder:
            ruleaza(script, *flaguri, folder)

    def _scaneaza_wifi():
        ip = input("Adresa IP afisata in Record3D (Settings > Live RGBD "
                    "Video Streaming > Wi-Fi, apoi butonul rosu): ").strip()
        if ip:
            ruleaza("scaner.py", "--wifi", ip)

    optiuni = {
        "1": ("Scaneaza un sit nou (USB)", lambda: ruleaza("scaner.py")),
        "2": ("Scaneaza un sit nou (WiFi direct, fara cablu)", _scaneaza_wifi),
        "3": ("Vizualizeaza o scanare (harta 3D)", lambda: ruleaza("harta_3d.py")),
        "4": ("Genereaza raport HTML", lambda: _cu_folder("raport.py")),
        "5": ("Genereaza raport PDF", lambda: _cu_folder("raport.py", "--pdf")),
        "6": ("Genereaza ortofotoplan PNG", lambda: _cu_folder("ortofoto.py")),
    }

    while True:
        print("\n" + "=" * 46)
        print("   STRATOSCAN - scanare 3D sit arheologic")
        print("=" * 46)
        for cheie, (eticheta, _) in optiuni.items():
            print(f"  [{cheie}] {eticheta}")
        print("  [0] Iesire")
        print("-" * 46)

        alegere = input("Alegerea ta: ").strip()
        if alegere == "0":
            print("La revedere.")
            return
        if alegere in optiuni:
            optiuni[alegere][1]()
        else:
            print(f"   Tasteaza un numar intre 0 si {len(optiuni)}.")


def meniu_grafic():
    """Interfata grafica STRATOSCAN: o singura fereastra, cu meniul principal
    si o lista clicabila a scanarilor (pentru vizualizare/raport/ortofoto).
    Vizualizatorul 3D NU se construieste in aceasta fereastra/proces - se
    porneste ca proces separat (harta_3d.py, prin porneste_in_fundal), la fel
    ca scaner.py/raport.py/ortofoto.py. A fost incercat si varianta incorporata
    (acelasi proces, acelasi Open3D renderer ca meniul) si a provocat un page
    fault reproductibil al driverului grafic (Event Viewer: LiveKernelEvent
    141 / igdkmdn64.sys) de fiecare data cand se deschidea harta din meniu -
    desi harta_3d.py rulat de sine statator, in propriul proces, ramane stabil."""
    import harta_3d
    from open3d.visualization import gui

    app = gui.Application.instance
    app.initialize()
    font_titlu, font_sectiune = harta_3d._fonturi_panou(app)

    # O singura fereastra pentru meniu + selectorul de scanari; vizibilitatea
    # ecranelor se schimba, fereastra nu se inchide/redeschide niciodata.
    window = app.create_window("STRATOSCAN", 1180, 860)
    window.show(True)
    em = window.theme.font_size
    CUL_ACCENT = gui.Color(0.85, 0.65, 0.26)       # auriu-arheologic, culoarea de brand
    CUL_ACCENT_INCHIS = gui.Color(0.55, 0.38, 0.12)
    # gri deschis pentru text secundar - suficient de luminos pe fundal inchis
    # ca sa treaca de pragul de contrast recomandat pentru dark mode
    CUL_TEXT_SLAB = gui.Color(0.72, 0.72, 0.72)
    CUL_BANNER = gui.Color(0.11, 0.10, 0.09)       # aproape negru, cald
    # canvasul de baza al ferestrei - putin mai inchis decat banner-ul, ca sa
    # dea senzatia de "adancime" cardurilor asezate deasupra (3 niveluri: fundal
    # cel mai inchis, banner la mijloc, carduri/butoane cele mai deschise)
    CUL_FUNDAL = gui.Color(0.075, 0.07, 0.06)
    CUL_CHIP = gui.Color(0.16, 0.16, 0.14)
    # culori semantice de stare: verde = reusita, rosu-portocaliu = eroare,
    # chihlimbar = activitate in curs (starea nu se transmite doar prin text)
    CUL_OK = gui.Color(0.30, 0.78, 0.45)
    CUL_EROARE = gui.Color(0.95, 0.45, 0.30)
    CUL_LUCRU = gui.Color(0.95, 0.78, 0.35)

    def eticheta_sectiune(text):
        # spatiere intre litere - efect discret de eticheta ("eyebrow"),
        # obtinut fara sa cerem un font nou (inregistrarea de fonturi e
        # riscanta - vezi nota de la _fonturi_panou)
        lbl = gui.Label(" ".join(text))
        lbl.font_id = font_sectiune
        lbl.text_color = CUL_ACCENT
        return lbl

    def buton(text, callback, principal_=False):
        b = gui.Button(text)
        b.vertical_padding_em = 0.65 if principal_ else 0.5
        b.background_color = CUL_ACCENT_INCHIS if principal_ else CUL_CHIP
        b.set_on_clicked(callback)
        return b

    def cadru(interior, fundal=CUL_CHIP, bordura=None, pad=0.6, spatiu=0.15):
        """'Card' vizual: container colorat cu padding in jurul unui widget.
        Bordura optionala e simulata prin doua containere imbricate - Open3D
        GUI nu are proprietate nativa de bordura/raza, dar acelasi truc clasic
        de 'bordura de 1px' functioneaza cu widget-urile disponibile aici."""
        corp = gui.Vert(spatiu * em, gui.Margins(pad * em, pad * em, pad * em, pad * em))
        corp.background_color = fundal
        corp.add_child(interior)
        if bordura is None:
            return corp
        exterior = gui.Vert(0, gui.Margins(0.05 * em, 0.05 * em, 0.05 * em, 0.05 * em))
        exterior.background_color = bordura
        exterior.add_child(corp)
        return exterior

    # ---------------------------------------------------------------
    #  ECRANUL PRINCIPAL - banner pe toata latimea + continut cu margini
    # ---------------------------------------------------------------
    principal = gui.Vert(0)   # container exterior, fara margini - banner-ul umple fereastra

    banner = gui.Vert(0.15 * em, gui.Margins(2 * em, 1.4 * em, 2 * em, 1.4 * em))
    banner.background_color = CUL_BANNER
    lbl_titlu = gui.Label("STRATOSCAN")
    lbl_titlu.font_id = font_titlu
    lbl_titlu.text_color = CUL_ACCENT
    banner.add_child(lbl_titlu)
    lbl_subtitlu = gui.Label("Scanare 3D a unui sit arheologic")
    lbl_subtitlu.text_color = gui.Color(0.75, 0.75, 0.75)
    banner.add_child(lbl_subtitlu)
    principal.add_child(banner)

    # margini orizontale generoase - la 1180px latime, continutul meniului
    # (ganditi pentru o fereastra ingusta) ramane o coloana lizibila in loc
    # sa se intinda pe toata latimea
    continut = gui.Vert(0.55 * em, gui.Margins(7 * em, 1.6 * em, 7 * em, 1.4 * em))
    continut.background_color = CUL_FUNDAL

    # cardul cu rezumatul ultimei scanari - context dintr-o privire, fara sa
    # trebuiasca sa deschizi vizualizatorul ca sa afli unde ai ramas. Bordura
    # aurie il marcheaza ca fiind cardul "hero" al ecranului principal.
    lbl_ultima_titlu = gui.Label("U L T I M A   S C A N A R E")
    lbl_ultima_titlu.font_id = font_sectiune
    lbl_ultima_titlu.text_color = CUL_ACCENT
    lbl_ultima = gui.Label("-")
    corp_ultima = gui.Vert(0.25 * em)
    corp_ultima.add_child(lbl_ultima_titlu)
    corp_ultima.add_child(lbl_ultima)
    card_ultima = cadru(corp_ultima, fundal=CUL_CHIP, bordura=CUL_ACCENT_INCHIS, pad=0.8)
    continut.add_child(card_ultima)

    toate_butoanele = []
    butoane_cu_scanari = []  # active doar daca exista cel putin o scanare

    def buton_meniu(text, descriere, callback, sectiune, principal_=False, cere_scanari=False):
        b = buton(text, callback, principal_)
        toate_butoanele.append(b)
        if cere_scanari:
            butoane_cu_scanari.append(b)
        corp = gui.Vert(0.25 * em)
        corp.add_child(b)
        if descriere:
            lbl = gui.Label(descriere)
            lbl.text_color = CUL_TEXT_SLAB
            corp.add_child(lbl)
        # butonul principal (Scaneaza) capata o bordura aurie - il distinge
        # vizual ca actiune principala fata de restul itemilor din meniu
        bordura = CUL_ACCENT_INCHIS if principal_ else None
        sectiune.add_child(cadru(corp, fundal=CUL_CHIP, bordura=bordura, pad=0.55, spatiu=0))
        return b

    continut.add_child(eticheta_sectiune("TEREN"))
    sec_teren = gui.Vert(0.35 * em)
    continut.add_child(sec_teren)

    continut.add_fixed(0.5 * em)
    continut.add_child(eticheta_sectiune("DOCUMENTE"))
    sec_documente = gui.Vert(0.35 * em)
    continut.add_child(sec_documente)

    # bara de stare, jos, ca in aplicatiile desktop clasice: mesajul curent
    # + butonul de deschis folderul rezultatului (apare doar dupa o reusita)
    continut.add_stretch()
    rand_stare = gui.Horiz(0.6 * em)
    lbl_stare = gui.Label("Gata.")
    rand_stare.add_child(lbl_stare)
    rand_stare.add_stretch()
    btn_deschide_folder = gui.Button("Deschide folderul")
    btn_deschide_folder.vertical_padding_em = 0.1
    btn_deschide_folder.background_color = CUL_ACCENT_INCHIS
    btn_deschide_folder.visible = False

    def _deschide_ultimul_folder():
        folder = stare.get("ultim_folder")
        if folder:
            try:
                os.startfile(os.path.abspath(folder))
            except Exception as e:
                print(f"Nu am putut deschide folderul: {e}")

    btn_deschide_folder.set_on_clicked(_deschide_ultimul_folder)
    rand_stare.add_child(btn_deschide_folder)
    continut.add_child(cadru(rand_stare, fundal=CUL_CHIP, pad=0.5, spatiu=0))

    principal.add_child(continut)

    # ---------------------------------------------------------------
    #  ECRANUL DE SELECTIE A SCANARII (pentru raport/ortofoto)
    # ---------------------------------------------------------------
    selector = gui.Vert(0)
    selector.visible = False

    banner_sel = gui.Vert(0.15 * em, gui.Margins(2 * em, 1.4 * em, 2 * em, 1.4 * em))
    banner_sel.background_color = CUL_BANNER
    lbl_titlu_sel = gui.Label("ALEGE SCANAREA")
    lbl_titlu_sel.font_id = font_titlu
    lbl_titlu_sel.text_color = CUL_ACCENT
    banner_sel.add_child(lbl_titlu_sel)
    lbl_subtitlu_sel = gui.Label("")
    lbl_subtitlu_sel.text_color = gui.Color(0.75, 0.75, 0.75)
    banner_sel.add_child(lbl_subtitlu_sel)
    selector.add_child(banner_sel)

    continut_sel = gui.Vert(0.6 * em, gui.Margins(7 * em, 1.8 * em, 7 * em, 1.8 * em))
    continut_sel.background_color = CUL_FUNDAL

    lista = gui.ListView()
    lista.set_max_visible_items(12)
    continut_sel.add_child(cadru(lista, fundal=CUL_CHIP, pad=0.4, spatiu=0))

    lbl_stare_sel = gui.Label("")
    continut_sel.add_child(cadru(lbl_stare_sel, fundal=CUL_CHIP, pad=0.5, spatiu=0))

    randul_butoane_sel = gui.Horiz(0.5 * em)
    buton_inapoi = buton("Inapoi", lambda: None)
    buton_genereaza = buton("Genereaza", lambda: None, principal_=True)
    randul_butoane_sel.add_stretch()
    randul_butoane_sel.add_child(buton_inapoi)
    randul_butoane_sel.add_child(buton_genereaza)
    continut_sel.add_child(randul_butoane_sel)

    selector.add_child(continut_sel)

    window.add_child(principal)
    window.add_child(selector)

    def on_layout(ctx):
        r = window.content_rect
        principal.frame = gui.Rect(r.x, r.y, r.width, r.height)
        selector.frame = gui.Rect(r.x, r.y, r.width, r.height)
    window.set_on_layout(on_layout)

    stare = {"actiune_pendinte": None, "scanari": [], "ultim_folder": None}

    def actualizeaza_rezumat():
        """Reimprospateaza cardul 'ULTIMA SCANARE' si activeaza/dezactiveaza
        butoanele care au nevoie de cel putin o scanare existenta."""
        scanari = harta_3d.lista_scanari()
        if scanari:
            f = scanari[0]
            lbl_ultima.text = (f"{harta_3d.nume_frumos(f)}   ·   "
                               f"{harta_3d.numar_artefacte(f)} artefacte   ·   "
                               f"{len(scanari)} scanari in total")
            lbl_ultima.text_color = CUL_ACCENT
        else:
            lbl_ultima.text = "Nicio scanare inca - incepe cu 'Scaneaza un sit nou'."
            lbl_ultima.text_color = CUL_TEXT_SLAB
        for b in butoane_cu_scanari:
            b.enabled = bool(scanari)
        return scanari

    def arata_principal():
        selector.visible = False
        principal.visible = True

    def arata_selector(eticheta_actiune, script, *flaguri):
        scanari = harta_3d.lista_scanari()
        if not scanari:
            lbl_stare.text = "Nu exista nicio scanare salvata inca."
            return
        stare["actiune_pendinte"] = (eticheta_actiune, script, flaguri)
        stare["scanari"] = scanari
        lista.set_items([f"{harta_3d.nume_frumos(f)}   ·   {harta_3d.numar_artefacte(f)} artefacte"
                         for f in scanari])
        lbl_subtitlu_sel.text = eticheta_actiune
        lbl_stare_sel.text = ""
        buton_genereaza.text = "Deschide" if script == "harta_3d.py" else "Genereaza"
        principal.visible = False
        selector.visible = True

    def porneste_in_fundal(eticheta_activitate, script, *args, folder_rezultat=None):
        """folder_rezultat: folderul in care apare rezultatul activitatii, ca
        butonul 'Deschide folderul' sa stie unde sa duca; sentinela '__ULTIMA__'
        inseamna 'cea mai noua scanare de dupa terminare' (cazul scanarii noi)."""
        for b in toate_butoanele + [buton_genereaza, buton_inapoi]:
            b.enabled = False
        btn_deschide_folder.visible = False
        lbl_stare.text = f"{eticheta_activitate}..."
        lbl_stare_sel.text = f"{eticheta_activitate}..."
        lbl_stare.text_color = CUL_LUCRU
        lbl_stare_sel.text_color = CUL_LUCRU

        def treaba():
            try:
                if CONGELAT:
                    nume_modul = os.path.splitext(script)[0]
                    comanda = [sys.executable, "--modul", nume_modul, *args]
                else:
                    comanda = [PYTHON, os.path.join(BAZA, script), *args]
                rezultat = subprocess.run(comanda, cwd=BAZA)
                if rezultat.returncode == 0:
                    mesaj, e_ok = f"{eticheta_activitate}: gata.", True
                else:
                    mesaj, e_ok = (f"{eticheta_activitate}: procesul s-a incheiat "
                                   f"cu eroare (cod {rezultat.returncode})."), False
            except Exception as e:
                mesaj, e_ok = f"{eticheta_activitate}: eroare ({e}).", False

            def actualizeaza():
                culoare = CUL_OK if e_ok else CUL_EROARE
                lbl_stare.text = mesaj
                lbl_stare_sel.text = mesaj
                lbl_stare.text_color = culoare
                lbl_stare_sel.text_color = culoare
                for b in toate_butoanele + [buton_genereaza, buton_inapoi]:
                    b.enabled = True
                scanari_noi = actualizeaza_rezumat()
                if e_ok:
                    if folder_rezultat == "__ULTIMA__":
                        if scanari_noi:
                            stare["ultim_folder"] = scanari_noi[0]
                            btn_deschide_folder.visible = True
                    elif folder_rezultat:
                        stare["ultim_folder"] = folder_rezultat
                        btn_deschide_folder.visible = True
                window.set_needs_layout()
                window.post_redraw()
            gui.Application.instance.post_to_main_thread(window, actualizeaza)

        threading.Thread(target=treaba, daemon=True).start()

    def pe_genereaza():
        if stare["actiune_pendinte"] is None or lista.selected_index < 0:
            return
        eticheta_actiune, script, flaguri = stare["actiune_pendinte"]
        folder = stare["scanari"][lista.selected_index]
        arata_principal()
        porneste_in_fundal(eticheta_actiune, script, *flaguri, folder,
                           folder_rezultat=folder)

    def pe_selectie_lista(valoare_noua, e_dublu_click):
        if e_dublu_click:
            pe_genereaza()

    lista.set_on_selection_changed(pe_selectie_lista)
    buton_genereaza.set_on_clicked(pe_genereaza)
    buton_inapoi.set_on_clicked(arata_principal)

    def arata_dialog_wifi():
        """Cere adresa IP afisata in Record3D (Settings > Live RGBD Video
        Streaming > Wi-Fi, apoi butonul rosu) - conectare DIRECTA prin Wi-Fi,
        fara cablu si fara Sync-over-WiFi/iTunes."""
        dlg = gui.Dialog("Scaneaza prin Wi-Fi")
        corp = gui.Vert(0.5 * em, gui.Margins(em, em, em, em))
        corp.add_child(gui.Label("Adresa IP afisata pe telefon (Record3D > Wi-Fi > butonul rosu):"))
        camp_ip = gui.TextEdit()
        camp_ip.placeholder_text = "ex: 192.168.1.100"
        corp.add_child(camp_ip)

        rand_butoane = gui.Horiz(0.5 * em)
        rand_butoane.add_stretch()

        def _anuleaza():
            window.close_dialog()

        def _conecteaza():
            ip = camp_ip.text_value.strip()
            window.close_dialog()
            if ip:
                porneste_in_fundal("Scanare WiFi", "scaner.py", "--wifi", ip,
                                   folder_rezultat="__ULTIMA__")

        rand_butoane.add_child(buton("Anuleaza", _anuleaza))
        rand_butoane.add_child(buton("Conecteaza", _conecteaza, principal_=True))
        corp.add_child(rand_butoane)

        dlg.add_child(corp)
        window.show_dialog(dlg)

    buton_meniu("Scaneaza un sit nou",
               "Conecteaza iPhone-ul (Record3D, prin USB) si porneste scanarea LiDAR",
               lambda: porneste_in_fundal("Scanare", "scaner.py", folder_rezultat="__ULTIMA__"),
               sec_teren, principal_=True)
    buton_meniu("Scaneaza prin WiFi (fara cablu)",
               "Conectare directa prin Wi-Fi (necesita extensia platita Record3D "
               "'Wi-Fi Streaming & RGBD video export') - iti cere adresa IP",
               arata_dialog_wifi,
               sec_teren)
    buton_meniu("Vizualizeaza o scanare (harta 3D)",
               "Harta 3D interactiva: navigare, masuratori, artefacte",
               lambda: arata_selector("Vizualizare 3D", "harta_3d.py"),
               sec_teren, cere_scanari=True)
    buton_meniu("Genereaza raport HTML",
               "Pagina cu harti, statistici si pozele artefactelor - gata de trimis",
               lambda: arata_selector("Raport HTML", "raport.py"),
               sec_documente, cere_scanari=True)
    buton_meniu("Genereaza raport PDF",
               "Versiune paginata, de arhivat sau tiparit",
               lambda: arata_selector("Raport PDF", "raport.py", "--pdf"),
               sec_documente, cere_scanari=True)
    buton_meniu("Genereaza ortofotoplan PNG",
               "Fotografie de sus cu grila precisa si scara grafica",
               lambda: arata_selector("Ortofotoplan", "ortofoto.py"),
               sec_documente, cere_scanari=True)

    continut.add_fixed(0.3 * em)
    lbl_footer = gui.Label("STRATOSCAN v1.0  ·  Scanare LiDAR  ·  iPhone")
    lbl_footer.text_color = CUL_TEXT_SLAB
    continut.add_child(lbl_footer)

    actualizeaza_rezumat()
    app.run()


if __name__ == "__main__":
    if "--modul" in sys.argv:
        _idx = sys.argv.index("--modul")
        ruleaza_modul(sys.argv[_idx + 1], sys.argv[_idx + 2:])
    else:
        try:
            meniu_grafic()
        except Exception as e:
            print(f"Interfata grafica nu a pornit ({e}) - trec pe meniul din consola.")
            meniu_consola()
