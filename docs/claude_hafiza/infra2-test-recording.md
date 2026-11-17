---
name: infra2-test-recording
description: "infra2 (ikinci IR kamera) — AprilTag + optik akis paketin SONUNDA aprtiltag2/opticflow2 ile seriale gidiyor (2026-09-28), ayrica video kaydi"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-09-28T12:45:59.008Z
---

`infra2_test` launch argumani (bringup.launch.py, varsayilan 'true') D435i'nin
infra2'sini acar. 2026-09-25'te sadece test videosuydu; 2026-09-28'de kullanici
istegiyle infra2 verisi SERIALE eklendi: paketin sonuna `apriltagPack_t aprtiltag2`
+ `opticFlowPack_t opticflow2` (paket 77 -> 120 B; 28_09'da car eklendi -> 133 B,
bkz. [[car-detection-bearing]]; static_assert ile kilitli). infra1 alanlariyla birlestirilmez, ayri alanlar.
infra2 kapaliysa bu alanlar SIFIR gider (boyut degismez).

Veri yolu: cpu_apriltag_infra2 -> /tag_detections_infra2 -> serial_sender tagCb2
(infra1 ile ayni fuseTags + packFromFusion). Akis: infra2_recorder_node (VPI) ->
/infra2/optical_flow (geometry_msgs/Vector3Stamped: x=dx y=dy z=quality) ->
serial_sender flowCb2. Topic isimleri launch'ta INFRA2_*_TOPIC sabitlerinde.
Uctan uca pty testiyle dogrulandi (sahte seri port + bayt cozucu).

**Why:** Kullanici iki IR kameranin verisini otopilotta ayri ayri istiyor.

**How to apply:** Firmware tarafi yeni 120 B struct'i bilmeli (ofsetler: payload
icinde aprtiltag2 @72, opticflow2 @102, chksum paket bayt 119). infra2 pos.x'in
infra1'den ~0.050 m KUCUK cikmasi beklenir (stereo baseline; 28_09'da kamera
degisti: 243222074221 -49.95 mm, eskisi 244222077462 -50.06 mm) — hata degil.
cpu_apriltag_infra2 nice 10'da -> aprtiltag2 ~7-8 Hz (infra1 ~21-26 Hz); daha hizli
istenirse nice kaldirilmali (infra1'den CPU calar). infra2_recorder nice 0 (nice
10'da DDS alimi ac kaldi, 30 Hz'in 4-9 Hz'ini aliyordu — olculdu).
serial_sender'da `hrvs_enabled:=false` radarsiz masa testi icin (canli servis
calisirken ikinci kopya radar UART'ina yazmasin diye de gerekli).
Acik karar: 45/24 modunda FUSION_NODE_SIZE=0.25 iken apriltag size=0.245
(tag 24 pozisyonu ~%2 kucuk) — kullaniciya soruldu, cevap bekliyor.

Kayit analizi tuzaklari (25_09 olculdu):
- Jetson acilista saati son kayitli zamana ceker, NTP sonra ~dakikalarca ileri
  atlar -> NTP oncesi acilan dosya adlarindaki saat YANLIS. Gercek sure icin
  monotonik t_ms (ir_*.csv) kullan, dosya adi/mtime degil.
- COZULDU 28_09: serial_sender'in 30 Hz kameradan ~21 Hz almasinin sebebi FastDDS:
  921 KB kare varsayilan SHM segmentine (~536 KB) sigmiyor -> UDP'ye dusuyor,
  rmem_max 208 KB -> RcvbufErrors (%17.8 datagram). Cozum config/fastdds_shm.xml
  (32 MB SHM segment), launch'ta FASTRTPS_DEFAULT_PROFILES_FILE ile (fastdds_shm
  arg, varsayilan true). Sonrasi UDP kaybi 0, node'lar tam kare hizini aliyor.
- Olcum 28_09 (VS Code acikken, measure_rates.py): 30 fps'de iki kamerada tam
  isleme CPU'ya sigmiyor (%87-91, akislar karelerin %35-42'sini atliyor).
  15 fps + infra2_equal_priority=true + SHM: iki kamera da 15.0 Hz, %100 duzenli,
  CPU %67. Kullanici 'once olc sonra karar' dedi — varsayilanlar (30/false)
  DEGISTIRILMEDI, karar bekliyor.
- DONANIM SAATI (28_09, asil akis darbogazi): Orin Nano Super'da DVFS OFA'yi
  115/537, VIC'i 115/435, GPU'yu 306/1020 MHz'de tutuyordu -> 30 fps'de iki akis
  karelerin ~%33'unu atliyordu (CPU bosken). jetson_clocks (sadece GPU'yu
  kaldiriyor, OFA/VIC'e dokunmuyor) -> %21; + OFA/VIC governor=performance ->
  %0.4-0.6, IKI AKIS DA 30 Hz duzenli. Kalici ayar: scripts/set_vision_clocks.sh,
  servise ExecStartPre drop-in'i (apriltag_bringup.service.d/clocks.conf) ile —
  YENIDEN ACILISTA sifirlanir, drop-in kurulu mu kontrol et.
- KARAR (28_09, kullanici onayi): launch varsayilanlari camera_fps=30,
  infra2_equal_priority=true, apriltag_every_n=2. Olculen: tag iki kamerada sabit
  15.0 Hz AYNI karelerde (aprtiltag/aprtiltag2 ts birebir ayni), akis iki kamerada
  30 Hz %99 duzenli, CPU %77 (VS Code acik). Alternatif every_n=1+esit: tag
  21.7/20.8 Hz sabit degil, CPU %92 doygun — reddedildi.
- Masa testi araclari: scripts/bench_config.sh (servis durmali, seri pty'ye),
  measure_rates.py, serial_packet_check.py.
- 11:10 kaydinda infra2 8.5 dk %100 doygun (beyaz) idi, infra1 normal; sebep
  bilinmiyor (lens ortulu?). infra2 durum satiri 'doygun %' uyarir.

Ilgili: [[autopilot-owns-frame-and-tilt]]

**03_11: infra1 + infra2 birleşimi** (dual_camera_fusion.hpp, serial_sender TX)
- Paketteki `aprtiltag` = iki kameranın ofsetleri düşülmüş avgPos ortalaması.
  - Sadece AYNI kareden (çekim zamanı farkı ≤ tag_pair_sync_ms 10 ms) iki sonuç ortalanır;
    farklı karelerse yeni olan gider. [FUSION]'da "esli kare X/Y". cpu_apriltag
    scheduledFrame iki node'u global kare paritesinde hizalar ama kare kaçırınca
    kayabilir; eşleşme oranı düşük çıkarsa scheduler saf paritaya çevrilebilir.
  - Tutarlılık tag_pair_tol_m 0.30; aşarsa infra1.
  - Bitler OR; açılar dairesel ortalama.
  - Bayatlık: tag_fresh_ms 200.
- `opticflow` = kaliteyle ağırlıklı ortalama.
  - Uyuşmazsa (max(1 px, %30)) kalitesi yüksek olan.
  - Bayatlık: flow_fresh_ms 100.
- `aprtiltag2` infra2'nin kendisi (ofset düşülmüş); `opticflow2` ham infra2.
- Kapatmak için dual_camera_fusion:=false. `[FUSION]` log satırı her saniye kaynak dağılımını
  ve paketteki değeri basar.
- Doğrulama:
  - kayıt: akış işaret uyumu %89, oran 0.93–0.99, fark medyanı 0.11 px;
  - test_dual_fusion geçti;
  - uçtan uca (domain 77): ikisi → (0.970, 0.240), infra1 → 0.967, yok → geçersiz;
  - TX 50.0 Hz.
- Test tuzağı: pty'nin master ucunu okumazsan tampon dolar ve TX ~49.4 Hz / 6 ms jitter
  görünür. Bu sahte bir sorun.
