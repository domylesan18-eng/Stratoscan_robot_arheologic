# STRATOSCAN — Scanare 3D a unui sit arheologic

Sistem complet pentru scanarea 3D a unui sit arheologic cu LiDAR-ul unui iPhone:
capteaza terenul si detecteaza automat artefactele in timp real, reconstruieste
o suprafata 3D neteda a sitului, marcheaza artefactele gasite cu coordonate
precise, si genereaza automat raport HTML/PDF si ortofotoplan — totul dintr-o
singura aplicatie, cu o interfata grafica unificata (`stratoscan.py`) sau, daca
preferi, din linia de comanda, unealta cu unealta.

Momentan in faza de testare cu telefonul tinut manual. Planul pe termen lung e
montarea pe un robot de tip gantry (CNC), cu motoare pas cu pas pe axele
OX/OY si un brat pe axa OZ, pentru scanare autonoma, repetabila, fara operator.

---

## Cuprins

1. [Arhitectura aplicatiei](#arhitectura-aplicatiei)
2. [Interfata grafica unificata (STRATOSCAN)](#interfata-grafica-unificata-stratoscan)
3. [Instalare](#instalare)
4. [Rulare](#rulare)
5. [Scanare prin WiFi (fara cablu)](#scanare-prin-wifi-fara-cablu)
6. [Executabilul Windows (.exe)](#executabilul-windows-exe)
7. [Structura datelor unei scanari](#structura-datelor-unei-scanari)
8. [Cum functioneaza scanarea (`scaner.py`)](#cum-functioneaza-scanarea-scanerpy)
9. [Vizualizatorul 3D](#vizualizatorul-3d)
10. [Raportul si ortofotoplanul](#raportul-si-ortofotoplanul)
11. [Sistem de coordonate](#sistem-de-coordonate)
12. [Performanta pe scanari mari](#performanta-pe-scanari-mari)
13. [Limitari cunoscute](#limitari-cunoscute)
14. [Depanare](#depanare)

---

## Arhitectura aplicatiei

| Fisier | Rol |
|---|---|
| `stratoscan.py` | **Punctul de intrare unificat.** O singura fereastra grafica cu buton pentru fiecare actiune (scanare, vizualizare 3D, raport HTML/PDF, ortofotoplan). Fiecare actiune ruleaza ca **proces de sistem separat** (vezi mai jos), ca fereastra principala sa ramana mereu responsiva. Cade automat pe un meniu din consola daca interfata grafica nu poate porni. |
| `scaner.py` | Scanarea live: se conecteaza la iPhone (LiDAR, prin Record3D) — prin cablu USB sau, cu flagul `--wifi <ip>`, direct in retea fara cablu (vezi [Scanare prin WiFi](#scanare-prin-wifi-fara-cablu)) — fuzioneaza cadrele intr-un volum TSDF (suprafata 3D neteda), ruleaza YOLOv8 pe fiecare cadru, salveaza coordonatele globale (cm) si fotografiile artefactelor, si genereaza automat raportul + ortofotoplanul la final. |
| `harta_3d.py` | Motorul vizualizatorului 3D: incarca o scanare si construieste scena (teren, artefacte, unelte de navigare/masurare). Ruleaza **intotdeauna ca proces separat** (fereastra proprie), fie invocat de sine statator (cu propriul selector de scanari), fie pornit de `stratoscan.py` pentru o scanare aleasa. |
| `record3d_wifi.py` | Conectare **directa** prin Wi-Fi (WebRTC) la Record3D, fara cablu si fara iTunes/Sync-over-WiFi — vezi [Scanare prin WiFi](#scanare-prin-wifi-fara-cablu). |
| `raport.py` | Genereaza raportul unei scanari: harti de sus, statistici, tabel cu artefactele si pozele lor — ca HTML (implicit) sau ca **PDF paginat** (`--pdf`), gata de arhivat/tiparit. |
| `ortofoto.py` | Genereaza ortofotoplanul PNG: fotografie de sus, perfect perpendiculara, cu grila precisa si rigla grafica de scara — formatul standard din documentatia de santier. |
| `requirements.txt` | Dependintele Python fixate la versiunile testate. |
| `yolov8n.pt` | Modelul YOLOv8 folosit la detectia obiectelor/artefactelor (model generic — vezi [Limitari](#limitari-cunoscute)). |
| `Stratoscan.spec` | Reteta PyInstaller pentru executabilul Windows de sine statator — vezi [Executabilul Windows](#executabilul-windows-exe). |

### De ce procese separate, nu un singur script mare

`scaner.py` (captura video OpenCV, buclă strânsă de cadre), `harta_3d.py`
(fereastră Open3D/Filament) și rapoartele (matplotlib, fără fereastră) folosesc
motoare grafice diferite, incompatibile în același proces. `stratoscan.py`
pornește **fiecare unealtă** (inclusiv vizualizatorul 3D) **ca proces de sistem
separat**, pe un fir de fundal, ca fereastra principală să rămână mereu
responsivă.

O versiune anterioară încorpora vizualizatorul 3D direct în fereastra
STRATOSCAN (același proces, același renderer Open3D ca meniul), ca să evite
deschiderea unei ferestre noi. S-a renunțat la asta: pe mesh-uri mari,
combinația "meniu 2D activ + scenă 3D grea în același renderer" provoca un
page fault reproductibil al driverului grafic (confirmat cu Event Viewer —
`LiveKernelEvent 141` / `igdkmdn64.sys` pe plăci Intel integrate), care nu
apărea când `harta_3d.py` rula de sine stătător, ca proces propriu. Izolarea
în proces separat (ca la `scaner.py`) a eliminat problema complet.

---

## Interfata grafica unificata (STRATOSCAN)

`stratoscan.py` deschide o singura fereastra (identic dimensionata pe tot
parcursul sesiunii) cu doua "ecrane", fara sa se inchida si sa se redeschida
niciodata:

1. **Meniul principal** — un card cu rezumatul ultimei scanari (data, numar de
   artefacte, cate scanari existe in total), grupate pe sectiuni **TEREN**
   (Scaneaza un sit nou, Scaneaza prin WiFi, Vizualizeaza o scanare) si
   **DOCUMENTE** (raport HTML, raport PDF, ortofotoplan). Butoanele care au
   nevoie de o scanare existenta sunt dezactivate automat cand nu exista
   niciuna. O bara de stare jos arata activitatea curenta si, dupa o reusita,
   un buton "Deschide folderul" care duce direct la rezultat.
2. **Selectorul de scanari** — pentru raport/ortofoto/vizualizare, o lista
   clicabila doar cu scanarile reale gasite in `scanari\`, etichetate cu
   data si numarul de artefacte.

Fiecare actiune (scanare, vizualizare 3D, raport, ortofoto) porneste
**unealta corespunzatoare ca proces separat** (vezi
[De ce procese separate](#de-ce-procese-separate-nu-un-singur-script-mare)),
intr-o fereastra proprie; fereastra STRATOSCAN ramane deschisa si responsiva
in tot acest timp, cu o bara de stare care arata cand unealta a terminat.

### Identitate vizuala

Tema e intunecata, cu accent auriu-arheologic (`#D9A642`-ish), gandita pentru
citire usoara pe orice fundal de birou. Fiecare item de meniu e un "card" cu
fundal propriu, deosebit de fundalul general, si o descriere scurta sub numele
actiunii; actiunea principala ("Scaneaza un sit nou") are o bordura aurie ca
sa iasa in evidenta ca actiune implicita.

---

## Instalare

```powershell
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Dependinte cheie: `open3d` (randare 3D + interfata grafica), `opencv-python`
(captura video + desen HUD), `record3d` (streaming LiDAR de pe iPhone prin
USB), `ultralytics` (YOLOv8), `matplotlib` (rapoarte HTML/PDF si
ortofotoplan), `numpy`. `ultralytics` aduce si `torch`/`torchvision` ca
dependinte tranzitive (fara ele, detectia YOLO nu poate rula). `aiortc` +
`aiohttp` sunt folosite doar de scanarea **prin WiFi direct** (vezi mai jos).

Pe telefon: aplicatia **Record3D** (App Store), cu modul **USB Streaming**
activat din setarile ei.

Pentru scanare **prin WiFi, fara cablu**, vezi
[Scanare prin WiFi](#scanare-prin-wifi-fara-cablu) mai jos — e nevoie de
extensia platita "Wi-Fi Streaming & RGBD video export" din Record3D.

---

## Rulare

Cel mai simplu, din radacina proiectului:

```powershell
.venv\Scripts\python.exe stratoscan.py
```

Aceasta deschide interfata grafica descrisa mai sus. Daca preferi sa rulezi
fiecare unealta separat (util pentru depanare sau automatizari proprii):

```powershell
# 1. Porneste aplicatia Record3D pe iPhone (USB Streaming activat) si apoi scanarea:
.venv\Scripts\python.exe scaner.py
# Plimba telefonul lent peste sit, la 1-2 m de sol, cu suprapunere intre treceri.
# ESC (cu fereastra video selectata) incheie si salveaza harta, mesh-ul, .glb-ul,
# raportul HTML si ortofotoplanul PNG.

# 2. Vizualizeaza harta 3D — un meniu te lasa sa alegi ultima scanare sau una veche:
.venv\Scripts\python.exe harta_3d.py

# 2b. Sau direct o anumita scanare, fara meniu:
.venv\Scripts\python.exe harta_3d.py scanari\2026-08-27_16-30

# 3. Regenereaza raportul oricand (implicit pentru ultima scanare):
.venv\Scripts\python.exe raport.py                       # HTML
.venv\Scripts\python.exe raport.py --pdf                  # PDF paginat, de arhivat/tiparit
.venv\Scripts\python.exe raport.py scanari\2026-08-27_16-30

# 4. Regenereaza ortofotoplanul PNG oricand (implicit pentru ultima scanare):
.venv\Scripts\python.exe ortofoto.py
.venv\Scripts\python.exe ortofoto.py scanari\2026-08-27_16-30
```

---

## Scanare prin WiFi (fara cablu)

Pe langa scanarea clasica prin cablu USB, `scaner.py` se poate conecta
**direct** la iPhone prin Wi-Fi, de la prima pornire, fara sa fie nevoie de
cablu deloc — nici macar o data.

### Cum functioneaza

Asta foloseste **Wi-Fi Streaming**, o extensie **platita** a aplicatiei
Record3D ("Wi-Fi Streaming & RGBD video export"), diferita de sincronizarea
Wi-Fi gratuita a lui Apple (iTunes/Sync-over-WiFi) — acelea NU dau acces la
fluxul RGBD, doar la sincronizarea generala a telefonului.

Cand activezi Wi-Fi Streaming pe telefon, Record3D porneste pe el un mic
server HTTP/WebRTC. `record3d_wifi.py` se conecteaza direct la acel server,
la adresa IP afisata pe ecranul telefonului:

1. Cere oferta WebRTC (`GET /getOffer`), construieste un raspuns si il trimite
   (`POST /answer`).
2. Primeste un singur flux video H.264: jumatatea **dreapta** a fiecarui cadru
   e imaginea color, jumatatea **stanga** e adancimea, codata ca o imagine
   HSV (Hue 0–1 → 0–3 metri).
3. Fiecare cadru are, ascunse in bitstream-ul H.264 (un mesaj SEI), pozitia
   si rotatia telefonului (quaternion) la momentul acelui cadru exact — extrase
   direct din datele brute, inainte de decodare, la fel ca la USB.

Protocolul e documentat oficial de autorul Record3D in
[record3d-simple-wifi-streaming-demo](https://github.com/marek-simonik/record3d-simple-wifi-streaming-demo)
(un demo in JavaScript/browser) — `record3d_wifi.py` e o reimplementare in
Python a aceluiasi protocol, ca sa se conecteze la exact acelasi server, dar
direct din `scaner.py`, fara sa fie nevoie de un browser.

### Pregatire (de fiecare data cand incepi o sesiune)

1. Pe telefon: **Record3D → Settings → Live RGBD Video Streaming → Wi-Fi**.
2. Inapoi pe ecranul de Record, apasa **butonul rosu** — ar trebui sa apara
   "Started, Waiting for Connection".
3. Deasupra butonului apare adresa IP a telefonului (sau numele mDNS, de
   forma `nume.local`) sub "Device Addresses" — noteaz-o.
4. Telefonul si calculatorul trebuie sa fie pe **aceeasi retea Wi-Fi**.

### Rulare

```powershell
# Din interfata grafica: butonul "Scaneaza prin WiFi (fara cablu)" din
# meniul STRATOSCAN iti cere adresa IP intr-un dialog.

# Sau direct din linia de comanda:
.venv\Scripts\python.exe scaner.py --wifi 192.168.1.100
```

### Diagnostic

Daca ceva nu merge (nu se conecteaza, sau se conecteaza dar cade repede),
`test_wifi_connect.py` e un script separat, minimal, doar pentru verificarea
conexiunii (nu porneste o scanare reala) — afiseaza pas cu pas fiecare etapa
a handshake-ului WebRTC si arata imaginea color + adancimea alaturi:

```powershell
.venv\Scripts\python.exe test_wifi_connect.py 192.168.1.100
```

### Limitari

- Necesita extensia **platita** Record3D "Wi-Fi Streaming & RGBD video
  export" — fara ea, telefonul nu porneste serverul WebRTC si conectarea
  esueaza.
- Telefonul si calculatorul trebuie sa fie pe **aceeasi retea** (nu
  functioneaza intre retele diferite sau prin internet).
- **Calitate mai slaba decat USB**: adancimea vine comprimata intr-o imagine
  video (cuantizare pe 8 biti, plafonata la 3 metri, plus artefacte de
  compresie H.264) — Record3D insusi recomanda WiFi Streaming doar cand
  cablul chiar nu e o optiune, nu cand precizia conteaza cel mai mult.
- Rezolutia efectiva a fluxului scade automat pe o retea Wi-Fi slaba
  (telefonul reduce calitatea ca sa tina pasul in timp real) — `scaner.py`
  se adapteaza singur la rezolutia primita, dar o retea foarte slaba tot
  poate intrerupe conexiunea in mijlocul scanarii.
- **O singura conexiune activa** o data — daca altcineva (alt calculator,
  alt browser) s-a conectat deja la fluxul WebRTC al telefonului, trebuie
  deconectat el intai.

---

## Executabilul Windows (.exe)

Proiectul se poate impacheta intr-un singur executabil de sine statator, care
nu are nevoie de Python instalat pe calculatorul tinta.

### Cum se construieste

```powershell
.venv\Scripts\pyinstaller.exe --noconfirm Stratoscan.spec
```

Rezultatul apare in `dist\Stratoscan\` — `Stratoscan.exe` plus un folder
`_internal\` cu toate dependintele (open3d, torch, ultralytics, modelul
`yolov8n.pt` etc.). E un build `--onedir` (nu `--onefile`): porneste mult mai
repede, cu pretul de a avea un folder intreg in loc de un singur fisier — la
predare/distributie, se copiaza/arhiveaza tot folderul `dist\Stratoscan\`,
nu doar `.exe`-ul.

Prima construire dureaza 10-15 minute (analizeaza toate dependintele grele —
torch, open3d); rebuild-urile ulterioare, dupa modificari de cod, sunt mai
rapide daca refolosesti acelasi `Stratoscan.spec`.

### Cum functioneaza intern

`Stratoscan.exe` e practic Python + toate dependintele + `stratoscan.py`,
lipite intr-un singur pachet. Fiindca uneltele (`scaner.py`, `harta_3d.py`
de sine statator, `raport.py`, `ortofoto.py`) trebuie sa ruleze ca **procese
separate** de sistem (vezi [mai sus](#de-ce-procese-separate-nu-un-singur-script-mare)),
iar un executabil impachetat nu are un `python.exe` separat pe care sa-l
invoce, `Stratoscan.exe` se **reinvoca pe sine insusi** cu un flag intern:

```
Stratoscan.exe --modul raport scanari\2026-08-27_16-30
```

`sys.frozen` (setat automat de PyInstaller) e semnalul ca aplicatia ruleaza
impachetata; codul detecteaza asta si comuta automat pe acest mod de
reinvocare, in loc sa caute un script `.py` pe disc.

### Limitari ale executabilului

- Ruleaza doar pe Windows x64 — nu e portabil pe alt SO sau arhitectura
  (vezi mai jos, Raspberry Pi).
- **Nu poate rula pe Raspberry Pi 5.** Motive: sistemul de operare e diferit
  (Windows vs. Linux) si arhitectura procesorului e diferita (x64 vs. ARM64) —
  un `.exe` Windows compilat pentru x64 pur si simplu nu e un format pe care
  Linux/ARM64 il poate executa, indiferent de emulare. Pentru un flux pe
  Raspberry Pi: ruleaza `scaner.py` direct din sursa Python, pe mediul nativ
  Linux/ARM64 al Pi-ului (dupa instalarea dependintelor potrivite arhitecturii
  ARM), iar uneltele grele de interfata (vizualizator, rapoarte) le rulezi pe
  un PC obisnuit, asupra fisierelor din `scanari\` copiate de pe Pi.

---

## Structura datelor unei scanari

Toate rezultatele unei scanari sunt salvate in `scanari\<data>_<ora>\`, creat
automat la pornirea `scaner.py`. Nu trebuie sa stergi sau sa muti nimic manual
intre scanari.

```
scanari\
  2026-08-27_16-30\
    harta_completa_sit.ply          norul de puncte al terenului
    harta_sit_mesh.ply              suprafata 3D neteda (mesh, din fuziunea TSDF)
    harta_sit_mesh.glb               acelasi mesh, format pentru telefon/colegi/AR
    artefacte_gasite.ply            doar punctele artefactelor (rosii)
    coordonate_sit.csv              registrul artefactelor: ID, incredere,
                                     nr. detectii, X/Y/Z in cm, numele pozei
    poze_artefacte\                 cate o fotografie decupata per artefact
    raport_sit_<data>.html          raportul complet, generat automat la final
    ortofoto_<data>.png             ortofotoplan: foto de sus + grila + scara grafica
```

Scanarile vechi facute inainte de aceasta structura (fisiere direct in
folderul proiectului, sau arhivate in `scanari_anterioare\`) sunt mutate
automat aici la prima rulare a `scaner.py` — nimic nu se pierde.

---

## Cum functioneaza scanarea (`scaner.py`)

1. **Fuziune TSDF a terenului** — fiecare al 3-lea cadru LiDAR e integrat
   (pozitie **si rotatie** a telefonului, din quaternion) intr-un volum de
   voxeli (`ScalableTSDFVolume`, rezolutie 1 cm), care mediaza zgomotul intre
   cadre in loc sa esantioneze puncte razlete. La final se extrage o
   **suprafata neteda (mesh)**, nu doar un nor de puncte.
2. **Detectie artefacte** — YOLOv8 pe fiecare cadru; adancimea centrului se ia
   ca mediana pe un petic 5×5 pixeli. Detectiile la sub 15 cm una de alta sunt
   considerate acelasi artefact, cu pozitia mediata pe masura ce e revazut.
   Deocamdata artefactele **nu sunt clasificate pe tip** (modelul YOLO generic
   nu recunoaste obiecte arheologice) — sunt doar numerotate de la 1 la n, in
   ordinea descoperirii; identificarea reala se face din fotografie. La
   fiecare detectie noua (sau mai clara decat cea anterioara) se salveaza
   automat o fotografie decupata in `poze_artefacte\` (`artefact_001.jpg`,
   care se suprascrie cu o vedere mai clara daca apare una).
3. **Pierderea tracking-ului** — peste sol uniform, fara textura, ARKit poate
   „teleporta" camera. Scriptul detecteaza salturile (pozitie > 40 cm,
   rotatie > 45°, pauza > 2 s) si porneste automat un **segment nou** (propriul
   volum TSDF), afisand un avertisment vizibil pe ecran.
4. **Aliniere ICP la final** — la ESC, segmentele se aliniaza intre ele cu ICP
   (cu fallback RANSAC + FPFH pentru decalaje mari); artefactele si mesh-ul
   urmeaza transformarea segmentului lor, iar artefactele duplicate dintre
   segmente se unesc.
5. **Autosave la 2 minute** — harta acumulata pana atunci se salveaza periodic
   intr-un fisier separat, ca protectie daca scriptul sau telefonul se
   blocheaza. Se sterge automat dupa salvarea finala reusita.
6. **Mini-harta de acoperire** — in coltul ferestrei video, o harta de sus in
   timp real: verde = teren deja scanat, gaurile negre = zone neacoperite,
   puncte rosii = artefacte, cerc alb = pozitia curenta a telefonului.
7. **Interfata ferestrei video (HUD)** — cronometru, numarul de artefacte si
   segmente, cadre/secunda, stare tracking (verde/portocaliu), numaratoare
   pana la urmatorul autosave.
8. **Raport automat** — la final, `raport.py` genereaza un HTML de sine
   statator cu hartile sitului si tabelul artefactelor (poze incluse), gata
   de trimis. Acelasi script poate genera si un **PDF paginat**
   (`raport.py --pdf`), de arhivat sau tiparit, cu pagina de titlu, hartile si
   registrul artefactelor.
9. **Ortofotoplan automat** — la final, `ortofoto.py` genereaza un PNG de
   mare rezolutie: fotografie de sus perfect perpendiculara, cu grila precisa,
   rigla grafica de scara si artefactele numerotate — formatul standard de
   plan de santier arheologic, gata de tiparit.

---

## Vizualizatorul 3D

- **Selector grafic** la pornire (sau la alegerea "Vizualizeaza o scanare"
  din meniul STRATOSCAN): o lista clicabila doar a scanarilor reale din
  `scanari\`, fiecare cu eticheta ei de data/ora si numarul de artefacte —
  nu poti naviga aiurea prin calculator ca la un dialog obisnuit de
  „Open File”, doar alegi dintre scanarile existente. (Dat un folder direct
  ca argument in linia de comanda, la rularea de sine statatoare, selectorul
  e sarit.)
- **Suprafata neteda cu iluminare**, tema inchisa, grila de sol.
- **Panou lateral**, cu sectiuni pliabile:
  - *Navigare* — „Vedere generala” / „Vedere de sus”, „Artefactul urmator”
    (camera sare din artefact in artefact si pivoteaza in jurul lui), „Mod
    zbor” (navigare libera W/A/S/D + mouse).
  - *Artefacte* — lista clicabila cu toate artefactele (ID, incredere, nr.
    detectii); click pe un rand → camera sare direct la el, fara sa tot dai
    „urmatorul”.
  - *Afisare* — comuta intre culorile reale (fara iluminare artificiala, ca
    sa nu fie spalacite) si colorarea dupa cota (albastru = adanc, rosu = sus,
    cu **legenda** langa harta); arata/ascunde etichetele 3D ale artefactelor.
  - *Instrumente* — „Mod masurare”: click pe teren in doua puncte deseneaza
    linia si afiseaza lungimea (totala, orizontala, diferenta de cota) direct
    in panou; raza de clic e calculata analitic din camera (fara randare GPU),
    apoi intersectata cu suprafata pe CPU, deci reactioneaza instant.
  - *Export* — salveaza captura curenta ca PNG.
- Artefactele au sfere rosii si un „pin” vertical pana la sol; pe siturile cu
  putine artefacte (≤ 25), fiecare are si o eticheta 3D galbena („Artefact
  #N”) — vezi [Performanta pe scanari mari](#performanta-pe-scanari-mari)
  pentru siturile cu multe artefacte.
- Ruleaza intr-o fereastra proprie, separata de STRATOSCAN (vezi
  [Arhitectura aplicatiei](#arhitectura-aplicatiei)) — inchide fereastra
  vizualizatorului ca sa te intorci la meniu.

---

## Raportul si ortofotoplanul

- **Raport HTML** (`raport.py`) — pagina de sine statatoare, gata de trimis
  prin email: harti ale sitului, statistici, tabel cu toate artefactele
  (ID, incredere, numar de detectii, coordonate) si fotografia fiecaruia.
- **Raport PDF** (`raport.py --pdf`) — aceleasi date, paginate, cu pagina de
  titlu — de arhivat sau tiparit.
- **Ortofotoplan** (`ortofoto.py`) — PNG de mare rezolutie: fotografie de sus
  perfect perpendiculara pe teren, cu grila metrica precisa, rigla grafica de
  scara si artefactele numerotate — formatul standard de plan de santier.

Ambele ruleaza automat la finalul unei scanari si pot fi regenerate oricand,
pentru orice scanare din `scanari\`, din meniul STRATOSCAN sau din linia de
comanda.

---

## Sistem de coordonate

Toate coordonatele sunt in **centimetri**, in sistemul ARKit al scanarii
curente: originea = pozitia de pornire a telefonului la acea sesiune
(coordonatele nu sunt inca legate de un reper fix pe teren intre sesiuni
diferite). In vizualizator, axele X (rosu) / Y (verde, cota) / Z (albastru)
marcheaza originea (0, 0, 0).

---

## Performanta pe scanari mari

Pe un sit mare (mesh de sute de mii - milioane de triunghiuri, zeci de
artefacte), constructia scenei 3D face cateva lucruri scumpe de calcul:
recolorarea intregului teren pentru modul „Adancime”, o eticheta 3D (text
triangulat) per artefact, si o structura de accelerare pentru masuratori
(raycasting) peste tot terenul. Niciunul dintre ele nu e necesar doar ca sa
*vezi* harta — sunt utile doar daca chiar folosesti modul respectiv.

De aceea, toate trei se calculeaza **lenes** (la prima activare efectiva a
modului „Adancime”, respectiv la primul click de masurare), nu la deschiderea
vizualizatorului. Etichetele 3D ale artefactelor fac exceptie doar in sens
invers: pe siturile cu **peste 25 de artefacte**, nu se construiesc automat
la deschidere (oricum ar fi ilizibile, suprapuse) — se pot activa manual din
panou („Etichete artefacte”), cand chiar ai nevoie de ele.

In plus, terenul afisat (nu si fisierul salvat pe disc, care ramane la
rezolutia completa) e **simplificat automat** cand depaseste 800.000 de
triunghiuri, prin vertex clustering (grupare pe o grila de voxeli) — rapid
(sub o secunda, indiferent de marimea mesh-ului), spre deosebire de quadric
decimation, care pe un mesh de milioane de triunghiuri poate dura minute
intregi fara sa se justifice pentru randare interactiva. Asta reduce
semnificativ presiunea pusa pe driverul grafic la incarcare — utila mai ales
pe placi video integrate (Intel), mai putin robuste decat una dedicata la
mesh-uri foarte mari.

Rezultatul practic: deschiderea vizualizatorului dureaza sub o secunda,
indiferent cat de mare e scanarea, in loc sa blocheze fereastra timp de mai
multe secunde (destul cat Windows sa marcheze aplicatia „Not Responding”).

---

## Limitari cunoscute

- `yolov8n.pt` e modelul generic COCO (80 de clase de obiecte moderne) — nu
  recunoaste ceramica, oase sau unelte litice. Pentru ca numele lui de clasa
  (ex. „cup”, „vase”) ar fi inselator pentru artefacte reale, sistemul **nu
  afiseaza deloc tipul obiectului** — artefactele sunt doar numerotate 1..n,
  iar identificarea se face din fotografia salvata. Pentru clasificare reala
  pe tip e nevoie de un model antrenat pe poze proprii de pe sit (pozele din
  `poze_artefacte\` sunt un bun punct de plecare pentru un set de date).
- Coordonatele nu sunt georeferentiate — o scanare noua porneste din nou de
  la telefon ca origine, deci scanarile din zile diferite nu sunt automat
  aliniate intre ele.
- Fereastra video de scanare (`scaner.py`) si vizualizatorul 3D (`harta_3d.py`)
  raman mereu ferestre separate fata de STRATOSCAN (vezi
  [Arhitectura aplicatiei](#arhitectura-aplicatiei)) — nu pot fi incorporate
  in fereastra principala.
- Scanarea prin WiFi are calitate mai slaba a adancimii decat prin USB — vezi
  [Limitari](#limitari) din sectiunea [Scanare prin WiFi](#scanare-prin-wifi-fara-cablu).
- Executabilul `.exe` ruleaza doar pe Windows x64 (vezi
  [Executabilul Windows](#executabilul-windows-exe)).

---

## Depanare

**Interfata grafica nu porneste deloc.** Aplicatia cade automat pe un meniu
in consola — verifica mesajul de eroare afisat; de obicei indica o dependinta
lipsa (`pip install -r requirements.txt`).

**Fereastra pare inghetata cateva secunde la deschiderea unei scanari mari.**
De la fix-ul descris in [Performanta pe scanari mari](#performanta-pe-scanari-mari),
deschiderea propriu-zisa dureaza sub o secunda; o intarziere ramasa apare doar
daca activezi manual modul „Adancime” sau „Etichete artefacte” pe un sit
foarte mare — asteapta, se calculeaza o singura data (rezultatul ramane in
memorie pentru restul sesiunii).

**`scaner.py` nu primeste cadre de la telefon (USB).** Pe telefon: deschide
aplicatia Record3D, activeaza „USB Streaming mode” din setarile ei, apoi
apasa butonul rosu de inregistrare — in aceasta ordine. Verifica si cablul
USB (unele cabluri sunt doar de incarcare, fara date).

**Conectarea WiFi esueaza sau se opreste rapid.** Ruleaza
`test_wifi_connect.py <ip>` separat (vezi
[Scanare prin WiFi](#scanare-prin-wifi-fara-cablu)) — arata pas cu pas unde
se opreste conectarea. Cel mai des: extensia platita Wi-Fi Streaming nu e
activata pe telefon, adresa IP e gresita/veche (se schimba la fiecare
pornire a streaming-ului), sau telefonul si calculatorul nu sunt pe aceeasi
retea. O retea Wi-Fi foarte slaba poate intrerupe conexiunea in mijlocul
scanarii — nu e un bug, `scaner.py` salveaza automat ce a apucat sa
acumuleze pana atunci (vezi „Autosave” in
[Cum functioneaza scanarea](#cum-functioneaza-scanarea-scanerpy)).

**Vreau sa raportez o problema reproductibila.** Noteaza: ce scanare
(dimensiune mesh, numar de artefacte — vizibile in cardul „Ultima scanare”
sau in titlul panoului vizualizatorului), ce actiune exacta ai facut, si daca
a aparut vreun mesaj in bara de stare sau in consola din spatele ferestrei.
