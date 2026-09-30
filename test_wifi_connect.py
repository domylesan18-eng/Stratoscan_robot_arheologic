# -*- coding: utf-8 -*-
"""Script de DIAGNOSTIC pentru conectarea directa prin Wi-Fi la Record3D
(fara USB, fara Sync-over-WiFi) - NU e parte din fluxul normal de scanare,
doar verifica pas cu pas ca protocolul WebRTC merge si arata ce date vin.

Rulare:
    .venv\\Scripts\\python.exe test_wifi_connect.py <adresa_ip_sau_hostname>

Pe telefon, inainte de asta: Record3D > Settings > Live RGBD Video Streaming
> Wi-Fi, apoi inapoi pe Record, apasa butonul rosu si citeste adresa IP
afisata deasupra lui ("Device Addresses").

Ce ar trebui sa se intample:
  1. se afiseaza raspunsul /metadata (matricea intrinseca K)
  2. se afiseaza primul JSON de metadata per-cadru gasit in SEI (pose/poza)
  3. apare o fereastra cu doua imagini: culoare (dreapta originala) si
     adancimea decodata (falsa culoare, ca sa se vada daca are sens)
  4. ESC inchide fereastra si opreste conexiunea

Daca pasul 2 arata alte nume de campuri decat qx/qy/qz/qw/tx/ty/tz, spune-mi
JSON-ul exact care apare si ajustez record3d_wifi.py._Pose."""
import sys
import time

import cv2
import numpy as np

from record3d_wifi import Record3DWifiStream, DEPTH_MAX_M


def main():
    if len(sys.argv) < 2:
        print("Foloseste: test_wifi_connect.py <adresa_ip_sau_hostname>")
        print("Exemplu:   test_wifi_connect.py 192.168.1.100")
        sys.exit(1)

    adresa = sys.argv[1]
    stream = Record3DWifiStream()

    print(f"🔌 Ma conectez la {adresa}...")
    try:
        stream.connect(adresa)
    except Exception as e:
        print(f"❌ Conectarea a esuat: {e}")
        sys.exit(1)
    print("✅ Conectat. Astept primul cadru...")

    ultimul_afisaj = 0.0
    cadre_primite = 0

    def pe_cadru_nou():
        nonlocal cadre_primite
        cadre_primite += 1

    stream.on_new_frame = pe_cadru_nou

    inceput = time.time()
    while stream.get_rgb_frame() is None:
        if time.time() - inceput > 15.0:
            print("❌ Niciun cadru dupa 15s - verifica butonul rosu de streaming pe telefon.")
            stream.disconnect()
            sys.exit(1)
        time.sleep(0.1)

    print(f"📷 Primul cadru: rgb={stream.get_rgb_frame().shape}, "
          f"depth={stream.get_depth_frame().shape}")
    intr = stream.get_intrinsic_mat()
    if intr is not None:
        print(f"📐 Intrinseci: fx={intr.fx:.1f} fy={intr.fy:.1f} tx={intr.tx:.1f} ty={intr.ty:.1f}")

    NUME_FEREASTRA = "Test Wi-Fi direct (ESC = iesire)"

    try:
        while True:
            rgb = stream.get_rgb_frame()
            depth = stream.get_depth_frame()
            if rgb is not None and depth is not None:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                depth_norm = np.clip(depth / DEPTH_MAX_M, 0.0, 1.0)
                depth_color = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
                depth_color = cv2.resize(depth_color, (bgr.shape[1], bgr.shape[0]))
                combinat = np.hstack([bgr, depth_color])
                cv2.putText(combinat, f"cadre: {cadre_primite}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow(NUME_FEREASTRA, combinat)

            if time.time() - ultimul_afisaj > 2.0:
                ultimul_afisaj = time.time()
                pose = stream.get_camera_pose()
                if pose is not None:
                    print(f"   poza: x={pose.tx:.2f} y={pose.ty:.2f} z={pose.tz:.2f}  "
                          f"q=({pose.qx:.2f},{pose.qy:.2f},{pose.qz:.2f},{pose.qw:.2f})")

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
    except Exception as e:
        import traceback
        print(f"❌ Eroare in bucla principala: {e!r}")
        traceback.print_exc()
    finally:
        stream.disconnect()
        cv2.destroyAllWindows()
        print("Deconectat.")


if __name__ == "__main__":
    main()
