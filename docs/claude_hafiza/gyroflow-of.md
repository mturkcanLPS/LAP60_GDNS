---
name: gyroflow-of
description: "19_11 OKAD gyroflow entegre edildi — OF1 = infra1, OF2 = infra2 (akış birleşimi kapalı), filtresiz akış − gyro, varsayılan açık; otopilot V1.1.3 bilmiyor → GPS'li modda uç; FTDI radar + hat hataları (commit bekliyor)"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-17T07:42:21.317Z
---

19_11 (kullanıcı onayıyla, commit de8ca8d pushlandı):
- **Gyroflow (OKAD `gyroflow.hpp` + `OKAD_Jetson_Gyroflow.md`, değiştirmeden `include/lps_serial_sender/gyroflow.hpp`):** pakete giden `opticflow` = SADECE infra1, `opticflow2` = SADECE infra2 (kullanıcı: iki kameranın akışı da ayrı gitsin; `flow_fusion` kapalı, tag birleşimi duruyor). Değer `comp_*_kare` = (all_dx/all_dy − gyro dönmesi), px / nominal 1/30 s. İki nesne de AYNI D435i IMU'suyla beslenir (kullanıcı onayı). `gyroflow_enable` varsayılan AÇIK (kullanıcı). infra2_recorder_node yeni `/infra2/optical_flow_raw` (all_dx/all_dy) yayınlar. OF1 GyroGelmedi → VPI thread'inde ≤30 ms bekle; OF2 → bekleyen kare IMU geri çağrısında tekrar. GyroYok/KareAraligi → paket güncellenmez. Log `[GYROFLOW] OF1 tamam … OF2 …`; `flow_sNNNN.csv` sonuna comp_dx_kare, comp_dy_kare, durum; `flow2_sNNNN.csv` (infra2).
- Canlı test (s0204, masa, 40 s): OF1 822/829 tamam, OF2 842/851 (1 gelmedi, kalan kare aralığı). 
- **UYARI (dokümandan): otopilot V1.1.3 bu formatı bilmiyor → otopilot güncellenene kadar GPS'li modda uçulmalı.**
- IMU ve akış logları yine AYRI dosyalar (kullanıcı isteği; birleşik CSV geri alındı).
- **HRVS adaptörü FTDI FT232R** (`/dev/ttyFTDI`, udev latency 1 ms, `hrvs_port`). Ama radar ölçüm vermiyor: hat sayaçları 30 s'de break 15, frame 10, overrun 20 → elektriksel (GND/3.3-5 V jumper/kablo). BYSTerm portu aynı anda açık tutuyordu (baytları bölüyordu) — kapatıldı, sorun sürdü. CP2102 klonunda da aynısıydı. Öneri: GND + jumper kontrol, aynı kabloyla CH340 dene. Test araçları `/home/lapis/huni_kalib/test/` (pty_tut.py, sayac.py = TIOCGICOUNT).
- Jetson boşta 91–94 °C (fan 4080 rpm) — hava akışı/kapak uyarısı.

İlgili: [[hrvs-takeoff-dropout]] [[funnel-offset-heading]] [[autopilot-owns-frame-and-tilt]]

**17_11: radar tekrar CH340.** Aynı kabloyla CH340: 30 s'de 0 hata, 0 reset, 4 radar (72–73 mesaj), ölçüm akıyor → FTDI/CP2102 sorunu adaptör bağlantısındaydı (GND/seviye) ya da adaptörün kendisinde. `hrvs_port` varsayılan + launch `/dev/ttyCH340`. CH340 sürücüsü TIOCGICOUNT desteklemiyor (sayac.py çalışmaz).
