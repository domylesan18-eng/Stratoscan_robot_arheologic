# -*- coding: utf-8 -*-
"""Conectare DIRECTA prin Wi-Fi la Record3D (extensia platita 'Wi-Fi Streaming
& RGBD video export'), fara cablu si fara 'Sync over Wi-Fi' (Apple/iTunes).

Protocol (WebRTC), documentat de autorul Record3D in:
https://github.com/marek-simonik/record3d-simple-wifi-streaming-demo
    1. GET  http://<ip>/getOffer    -> oferta WebRTC (JSON: {type, sdp})
    2. Se construieste un raspuns WebRTC local si se trimite:
       POST http://<ip>/answer      <- {"type": "answer", "data": <sdp>}
    3. Un singur track video: jumatatea DREAPTA = culoare, jumatatea STANGA =
       adancime codata HSV (Hue 0-1 -> 0-3 metri; valori invalide/prea departe
       ies rosii, adica Hue~1.0 -> ~3m).
    4. Fiecare cadru H.264 are, dupa datele de imagine, un mesaj SEI (payload
       type 5, 'unregistered user data') cu metadate JSON (poza camerei /
       intrinseci) - extras aici direct din bitstream-ul brut, INAINTE de
       decodare (portat dupa receivedVideoMetadataTransformerWorker.js).

IMPORTANT: schema exacta a JSON-ului din SEI (numele campurilor pentru poza
camerei) nu e documentata public - a fost dedusa/verificata empiric rulând
test_wifi_connect.py cu telefonul conectat. Daca Record3D schimba formatul,
ajusteaza _Pose.din_json() mai jos (metadata bruta se printeaza oricum la
prima aparitie, ca sa se vada imediat daca ceva nu se potriveste).
"""
import asyncio
import json
import time
import threading

import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.codecs import h264 as _aiortc_h264

DEPTH_MAX_M = 3.0  # per documentatia Record3D: Hue 0-1 codeaza 0-3 metri


# ----------------------------------------------------------------------
#  Extragerea metadatelor per-cadru din mesajul SEI H.264 (portat dupa
#  demo-ul oficial JS - vezi receivedVideoMetadataTransformerWorker.js -
#  dar cautand NAL-ul dupa TIP, nu dupa un byte de header hardcodat: JS-ul
#  cauta literal secventa de octeti \x06\x05, presupunand ca octetul de
#  header al NAL-ului SEI e mereu 0x06 - insa acel octet mai contine si
#  nal_ref_idc (bitii 5-6), care poate fi diferit de 0, schimband valoarea
#  exacta a byte-ului desi tipul (ultimii 5 biti, 0x1F) ramane 6. Verificat
#  empiric cu telefonul: NAL-ul SEI apare corect (tip 6, payload type 5),
#  dar cu octetul de header diferit de 0x06 exact - de-aia cautarea veche
#  nu gasea niciodata nimic.)
# ----------------------------------------------------------------------
def _ebsp_la_rbsp(ebsp: bytes) -> bytes:
    """Elimina octetii de 'emulation prevention' (0x03 dupa doi 0x00) din
    bitstream-ul H.264, ca in reference JS (ebsp2rbsp)."""
    rbsp = bytearray()
    zerouri_consecutive = 0
    for b in ebsp:
        if not (b == 0x03 and zerouri_consecutive >= 2):
            rbsp.append(b)
        zerouri_consecutive = zerouri_consecutive + 1 if b == 0x00 else 0
    return bytes(rbsp)


def scaneaza_nal_uri(date_cadru: bytes):
    """Gaseste toate NAL-urile din bitstream (start-code de 3 SAU 4 octeti) si
    intoarce o lista de (offset_dupa_start_code, lungime_start_code, nal_type,
    primul_octet_payload). Folosita atat pentru diagnostic, cat si de
    extrage_metadata_sei() de mai jos, ca sa gaseasca NAL-ul SEI dupa tip
    (nu dupa un byte de header hardcodat - vezi nota de mai sus)."""
    rezultate = []
    i = 0
    n = len(date_cadru)
    while i < n - 4:
        e4 = date_cadru[i:i + 4] == b"\x00\x00\x00\x01"
        e3 = not e4 and date_cadru[i:i + 3] == b"\x00\x00\x01"
        if e4 or e3:
            lung = 4 if e4 else 3
            offset_nal = i + lung
            nal_type = date_cadru[offset_nal] & 0x1F
            urmator = date_cadru[offset_nal + 1] if offset_nal + 1 < n else None
            rezultate.append((offset_nal, lung, nal_type, urmator))
            i = offset_nal
        else:
            i += 1
    return rezultate


def extrage_metadata_sei(date_cadru: bytes):
    """Cauta ultimul NAL SEI (tip 6, payload type 5 = 'unregistered user data')
    din cadrul H.264 brut si intoarce dict-ul JSON continut, sau None daca nu
    s-a gasit / nu e JSON valid. `date_cadru` e bitstream-ul Annex-B complet
    al cadrului (asa cum il primeste H264Decoder.decode() din aiortc,
    INAINTE de decodare)."""
    offset_nal = None
    for offset, _lung, tip, urmator in scaneaza_nal_uri(date_cadru):
        if tip == 6 and urmator == 5:
            offset_nal = offset  # ultimul gasit castiga (parcurgere in ordine)
    if offset_nal is None:
        return None
    # +2: sarim peste octetul de header NAL si peste octetul de payload type (0x05)
    ebsp = date_cadru[offset_nal + 2:]
    rbsp = _ebsp_la_rbsp(ebsp)

    # payloadSize e codat pe 1+ octeti: cat timp octetul curent e 0xFF, mai
    # sunt octeti de citit (fiecare 0xFF adauga 255 la marime); primul octet
    # NE-0xFF e ULTIMUL din campul payloadSize (si trebuie sarit peste el, nu
    # doar peste cei 0xFF de dinainte - aici era bug-ul: lipsea exact acest
    # ultim octet, ceea ce dezalinia tot restul cu un octet si strica JSON-ul).
    i = 0
    while i < len(rbsp):
        b = rbsp[i]
        i += 1
        if b != 0xFF:
            break
    i += 16  # UUID-ul de 16 octeti al mesajului SEI 'unregistered user data'
    payload = rbsp[i:-1]  # ultimul octet (0x80) e bitul de "rbsp trailing"
    try:
        return json.loads(payload.decode("utf-8"))
    except Exception as e:
        if _nr_apeluri_decode["n"] <= 10:
            print(f"⚠️ SEI gasit dar JSON-ul nu s-a putut parsa ({e!r}); "
                  f"primii octeti bruti: {payload[:80]!r}")
        return None


# Monkeypatch: H264Decoder.decode() e singurul loc unde mai avem acces la
# bitstream-ul BRUT (encoded_frame.data) - dupa asta aiortc intoarce doar
# cadre deja decodate (pixeli), fara SEI. Extragem metadatele aici si le
# punem intr-un "cutiuta" citita de Record3DWifiStream imediat dupa track.recv().
_ultima_metadata_bruta = {"json": None}
_nr_apeluri_decode = {"n": 0}
_decode_original_h264 = _aiortc_h264.H264Decoder.decode


def _decode_si_extrage_metadata(self, encoded_frame):
    _nr_apeluri_decode["n"] += 1
    n = _nr_apeluri_decode["n"]
    metadata = extrage_metadata_sei(encoded_frame.data)
    if metadata is not None:
        _ultima_metadata_bruta["json"] = metadata
    elif n in (1, 2, 3, 10):
        nal_uri = scaneaza_nal_uri(encoded_frame.data)
        tipuri = [(t, sc, urm) for (_, sc, t, urm) in nal_uri]
        print(f"🔎 [diagnostic decode #{n}] {len(encoded_frame.data)} octeti, "
              f"NAL-uri gasite (tip, lungime_start_code, urmatorul_octet): {tipuri}")
    return _decode_original_h264(self, encoded_frame)


_aiortc_h264.H264Decoder.decode = _decode_si_extrage_metadata


# ----------------------------------------------------------------------
#  Adaptoare mici, ca sa semene cu obiectele intoarse de record3d.Record3DStream
#  (scaner.py foloseste pose.qx/qy/qz/qw/tx/ty/tz si intrinsics.fx/fy/tx/ty)
# ----------------------------------------------------------------------
class _Intrinsici:
    """Container simplu fx/fy/tx/ty (tx/ty = cx/cy, denumire ca in record3d),
    deja scalate la rezolutia REALA a cadrului primit - vezi _intrinsici_brute_din_K
    de mai jos si scalarea facuta in Record3DWifiStream._citeste_cadre."""
    def __init__(self, fx, fy, tx, ty):
        self.fx, self.fy, self.tx, self.ty = fx, fy, tx, ty


def _intrinsici_brute_din_K(K):
    """(fx, fy, cx, cy) la rezolutia ORIGINALA (metadata['originalSize']),
    inainte de orice scalare. K vine COLUMN-major (verificat empiric:
    [fx,0,0, 0,fy,0, cx,cy,1]), nu row-major cum ai crede din conventia
    matematica obisnuita - cx/cy sunt ultimii doi din lista, nu pe pozitiile
    2 si 5."""
    plat = [v for rand in K for v in rand] if isinstance(K[0], (list, tuple)) else list(K)
    return float(plat[0]), float(plat[4]), float(plat[6]), float(plat[7])


class _Pose:
    """Poza camerei per-cadru. Schema reala (verificata empiric, cu telefonul
    conectat): metadata['pose'] e o LISTA de 7 numere [qx, qy, qz, qw, tx, ty, tz]
    - confirmat si prin faptul ca primele 4 au norma ~1.0 (quaternion unitar)."""
    def __init__(self, metadata):
        p = metadata.get("pose", metadata)
        if isinstance(p, (list, tuple)) and len(p) >= 7:
            self.qx, self.qy, self.qz, self.qw, self.tx, self.ty, self.tz = (float(v) for v in p[:7])
            return
        self.qx = float(p.get("qx", 0.0))
        self.qy = float(p.get("qy", 0.0))
        self.qz = float(p.get("qz", 0.0))
        self.qw = float(p.get("qw", 1.0))
        self.tx = float(p.get("tx", 0.0))
        self.ty = float(p.get("ty", 0.0))
        self.tz = float(p.get("tz", 0.0))


async def _json_de_la(sesiune, url):
    async with sesiune.get(url) as resp:
        if resp.status != 200:
            corp = await resp.text()
            raise RuntimeError(f"{url} -> HTTP {resp.status}: {corp[:200]}")
        return await resp.json()


class Record3DWifiStream:
    """Inlocuitor pentru record3d.Record3DStream, care se conecteaza DIRECT
    prin Wi-Fi (WebRTC) la adresa IP afisata in Record3D, in loc de USB.
    Aceeasi interfata (connect/disconnect/get_*_frame/on_new_frame) ca sa
    poata fi folosit fara nicio schimbare in bucla principala din scaner.py."""

    def __init__(self):
        self.on_new_frame = None
        self._rgb = None
        self._depth = None
        self._pose = None
        self._intrinsics = None
        self._K_brut = None            # (fx, fy, cx, cy) la rezolutia originala
        self._original_w = None
        self._original_h = None
        self._fir = None
        self._bucla = None
        self._pc = None
        self._oprit = threading.Event()
        self._prima_metadata_afisata = False
        self._jum_tinta = None         # dimensiunea FIXA (latime jumatate, inaltime) la
        self._h_tinta = None           # care aducem toate cadrele, indiferent de rezolutia
                                        # reala primita - vezi nota din _citeste_cadre

    def connect(self, adresa_ip):
        if not adresa_ip.startswith("http://") and not adresa_ip.startswith("https://"):
            adresa_ip = "http://" + adresa_ip
        gata = threading.Event()
        eroare = {"exceptie": None}

        def ruleaza_bucla():
            self._bucla = asyncio.new_event_loop()
            asyncio.set_event_loop(self._bucla)
            try:
                self._bucla.run_until_complete(self._conecteaza(adresa_ip, gata))
            except Exception as e:
                eroare["exceptie"] = e
                gata.set()

        self._fir = threading.Thread(target=ruleaza_bucla, daemon=True)
        self._fir.start()
        if not gata.wait(timeout=20.0):
            raise RuntimeError(f"Nu am primit raspuns de la {adresa_ip} in 20s.")
        if eroare["exceptie"] is not None:
            raise eroare["exceptie"]

    async def _conecteaza(self, adresa_ip, gata):
        import aiohttp
        timeout = aiohttp.ClientTimeout(total=10.0)
        async with aiohttp.ClientSession(timeout=timeout) as sesiune:
            meta = await _json_de_la(sesiune, adresa_ip + "/metadata")
            print(f"ℹ️ Metadata /metadata: {meta}")
            K = meta.get("K") or meta.get("k") or meta.get("intrinsicMatrix")
            dim_originala = meta.get("originalSize")
            if K is not None and dim_originala is not None:
                self._K_brut = _intrinsici_brute_din_K(K)
                self._original_w, self._original_h = float(dim_originala[0]), float(dim_originala[1])
            else:
                print("⚠️ Nu am gasit K/originalSize in /metadata - "
                      "verifica manual campurile de mai sus.")

            print("… cer oferta WebRTC de la /getOffer")
            oferta = await _json_de_la(sesiune, adresa_ip + "/getOffer")
            print(f"✅ Oferta primita (type={oferta.get('type')}, "
                  f"sdp={len(oferta.get('sdp', ''))} caractere)")
            self._pc = RTCPeerConnection()

            @self._pc.on("track")
            def pe_track(track):
                print(f"🎥 Track primit: kind={track.kind}")
                asyncio.ensure_future(self._citeste_cadre(track))

            @self._pc.on("icegatheringstatechange")
            def pe_schimbare_gathering():
                print(f"   iceGatheringState -> {self._pc.iceGatheringState}")

            @self._pc.on("connectionstatechange")
            def pe_schimbare_conexiune():
                print(f"   connectionState -> {self._pc.connectionState}")

            @self._pc.on("iceconnectionstatechange")
            def pe_schimbare_ice():
                print(f"   iceConnectionState -> {self._pc.iceConnectionState}")

            @self._pc.on("signalingstatechange")
            def pe_schimbare_semnalizare():
                print(f"   signalingState -> {self._pc.signalingState}")

            print("… setRemoteDescription")
            await self._pc.setRemoteDescription(
                RTCSessionDescription(sdp=oferta["sdp"], type=oferta["type"]))
            print("… createAnswer")
            raspuns = await self._pc.createAnswer()
            print("… setLocalDescription")
            await self._pc.setLocalDescription(raspuns)

            print(f"… astept ICE gathering (stare curenta: {self._pc.iceGatheringState})")
            inceput_gathering = time.time()
            while self._pc.iceGatheringState != "complete":
                if time.time() - inceput_gathering > 8.0:
                    print("⚠️ ICE gathering nu s-a terminat in 8s - trimit oricum "
                          "raspunsul cu ce candidati am adunat pana acum.")
                    break
                await asyncio.sleep(0.1)
            else:
                print(f"✅ ICE gathering complet in {time.time() - inceput_gathering:.1f}s")

            print("… trimit POST /answer")
            async with sesiune.post(
                adresa_ip + "/answer",
                json={"type": "answer", "data": self._pc.localDescription.sdp},
            ) as resp:
                if resp.status != 200:
                    corp = await resp.text()
                    raise RuntimeError(f"POST /answer -> HTTP {resp.status}: {corp[:200]}")
            print("✅ Raspuns trimis - astept track-ul video...")

            gata.set()
            while not self._oprit.is_set():
                await asyncio.sleep(0.2)
            await self._pc.close()

    async def _citeste_cadre(self, track):
        nr_cadru = 0
        while not self._oprit.is_set():
            try:
                cadru = await track.recv()
                nr_cadru += 1

                imagine = cadru.to_ndarray(format="bgr24")
                h, w = imagine.shape[:2]
                jum = w // 2
                bgr_adancime = imagine[:, :jum]
                bgr_culoare = imagine[:, jum:]

                # Decodam adancimea (hue -> metri) ACUM, la rezolutia NATIVA primita -
                # INAINTE de orice eventual resize de mai jos. Daca am redimensiona
                # bgr_adancime inainte de decodare, interpolarea ar amesteca pixelii hue
                # de pe cele doua laturi ale unei muchii de adancime (ex. marginea unui
                # obiect din prim-plan) intr-o culoare care NU corespunde niciunei
                # adancimi reale - iar acele adancimi false apar apoi in reconstructia
                # TSDF ca suprafete "fantoma"/suprapuse peste geometria reala.
                culoare_rgb = cv2.cvtColor(bgr_culoare, cv2.COLOR_BGR2RGB)
                hsv = cv2.cvtColor(bgr_adancime, cv2.COLOR_BGR2HSV)
                # OpenCV: H e in [0, 179] pentru imagini pe 8 biti (nu [0, 359])
                hue_normalizat = hsv[:, :, 0].astype(np.float32) / 179.0
                depth_metri = (hue_normalizat * DEPTH_MAX_M).astype(np.float32)

                # Rezolutia primita prin Wi-Fi variaza cadru cu cadru (telefonul o
                # adapteaza la viteza retelei - vezi nota de mai jos). Asta strica orice
                # se deseneaza la dimensiune FIXA in pixeli peste cadru (minimapa/HUD din
                # scaner.py) si da senzatia de "zoom" cand imaginea e scalata apoi la o
                # fereastra de dimensiune fixa. Fixam iesirea la rezolutia PRIMULUI cadru
                # primit si redimensionam restul la ea, ca sa fie stabila tot timpul
                # sesiunii - exact ca pe USB, unde rezolutia nu se schimba niciodata.
                if self._jum_tinta is None:
                    self._jum_tinta, self._h_tinta = jum, h
                elif (jum, h) != (self._jum_tinta, self._h_tinta):
                    culoare_rgb = cv2.resize(culoare_rgb, (self._jum_tinta, self._h_tinta))
                    # INTER_NEAREST, nu liniar: adancimea e deja in metri aici, iar o
                    # interpolare liniara ar amesteca tot doua adancimi diferite de pe o
                    # muchie intr-o valoare falsa, intermediara.
                    depth_metri = cv2.resize(depth_metri, (self._jum_tinta, self._h_tinta),
                                              interpolation=cv2.INTER_NEAREST)
                jum, h = self._jum_tinta, self._h_tinta

                # Rezolutia REALA primita e de obicei mai mica decat cea originala
                # (telefonul reduce calitatea in functie de viteza Wi-Fi - vezi
                # documentatia oficiala) - scalam K in consecinta, la fel cum
                # scaner.py face deja intre RGB si depth pe USB. Preferam K-ul
                # per-cadru din SEI ('intrinsicMatrixRgb' + originalRgbWidth/
                # Height), mai proaspat decat cel static din /metadata; daca
                # SEI-ul inca nu a sosit, folosim /metadata ca prim fallback.
                metadata = _ultima_metadata_bruta["json"]
                K_curent, w_original, h_original = self._K_brut, self._original_w, self._original_h
                if metadata is not None and "intrinsicMatrixRgb" in metadata:
                    K_curent = _intrinsici_brute_din_K(metadata["intrinsicMatrixRgb"])
                    w_original = float(metadata.get("originalRgbWidth", self._original_w))
                    h_original = float(metadata.get("originalRgbHeight", self._original_h))
                if K_curent is not None:
                    fx0, fy0, cx0, cy0 = K_curent
                    sx, sy = jum / w_original, h / h_original
                    self._intrinsics = _Intrinsici(fx0 * sx, fy0 * sy, cx0 * sx, cy0 * sy)

                self._rgb = culoare_rgb
                self._depth = depth_metri

                if metadata is not None:
                    if not self._prima_metadata_afisata:
                        self._prima_metadata_afisata = True
                        print(f"ℹ️ Prima metadata per-cadru (SEI) gasita: {metadata}")
                    self._pose = _Pose(metadata)
                elif nr_cadru in (1, 10, 30):
                    print(f"⚠️ Cadrul #{nr_cadru}: inca nicio metadata SEI gasita.")

                if self.on_new_frame is not None:
                    self.on_new_frame()
            except Exception as e:
                import traceback
                print(f"❌ Cadrul #{nr_cadru + 1}: eroare in bucla de citire ({e!r}):")
                traceback.print_exc()
                break
        print(f"ℹ️ Bucla de citire cadre s-a oprit dupa {nr_cadru} cadre.")

    def get_rgb_frame(self):
        return self._rgb

    def get_depth_frame(self):
        return self._depth

    def get_intrinsic_mat(self):
        return self._intrinsics

    def get_camera_pose(self):
        return self._pose

    def disconnect(self):
        self._oprit.set()
        if self._fir is not None:
            self._fir.join(timeout=5.0)
