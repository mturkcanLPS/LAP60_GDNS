---
name: car-detection-bearing
description: "car_detector (DeepStream YOLOv8n VisDrone + NvDCF, renkli kamera) bearing'i paketin SONUNDA carPack_t ile gider (paket 133 B); model dizini ve export tuzaklari"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-05T17:35:36.145Z
---

2026-09-28: diger dronedaki car_detector (GDNS_Tragon_wCarDetection) bizim yigina
eklendi. Orada bearing avgPos'a ENJEKTE ediliyordu (car_avg_inject); bizde paketin
SONUNA ayri alan: `carPack_t car {uint32 timeStamp; bool valid; float_2D_t bearing}`
(irPack_t ile ayni yapi). Paket 120 -> 133 B (payload 128; car payload ofseti 115; 20_10 saglik AYRI mesaj 0x48, ana paket 133 B kaldi, bkz. [[jetson-health-thermal]]),
static_assert ile kilitli. bearing = kilitli aracin X/Z, Y/Z (renkli kamera optik
frame'i, boyutsuz). car_timeout_ms=500: bundan eski bearing valid=0 (node olurse).

Model: test_visdrone_best.pt = YOLOv8n, ultralytics 8.0.71, imgsz 640, VisDrone 10
sinif (3=car, 4=van, 5=truck, 8=bus). vehicle_classes [3,5] (diger dronedan) -> van
ve otobus KILITLENMEZ. Model dizini /home/lapis/DeepStream-Yolo/visdrone (config +
onnx + labels + model_b1_gpu0_fp16.engine); car_detector config dizinine chdir eder,
ozel YOLO lib motoru CWD'ye yazar. Motor ilk derleme ~6 dk, sonra 1 sn.
Kurulum: src/car_detector/scripts/setup_visdrone_model.sh (config'in repo kopyasi asil).

**Why:** Export tuzaklari tekrar yasanmasin: python3-venv yok; sisteme pip ultralytics/
torch kurmak ~/.local'a setuptools 78 gibi paketler koyup colcon'u bozar (28_09
yasandi, geri alindi). Export paketleri /home/lapis/yolo_export_pkgs (pip --target,
PYTHONPATH ile). torch 2.14 dynamo exporter onnxscript ister, opset 18 cikar (17 degil);
ONNX = PyTorch dogrulandi (kutu <=0.002 px).
Renkli kamera intrinsics (20_10, camera_info, seri 243222074221, 1280x720):
fx 909.50, fy 909.53, cx 643.24, cy 359.79; D hep 0 (plumb_bob) → bearing = (u−cx)/fx
birebir. Kenarlarda x −0.707…+0.700, y ±0.396; FOV 70.3°×43.2°.
Otopilot ekibine açıklama sayfası: https://claude.ai/artifact/9a84JWhVmDZQyxWSkZRd4Z
(birimsiz tan, renkli kamera optik çerçevesi, tavan yüksekliği yüzünden ~%30 ölçek uyarısı).
Root servisten topic okumak için UDP-only FastDDS profili gerekiyor (SHM root↔lapis
zaman aşımına düşüyor).
Intrinsics camera_info'dan (fx=0): diger dronun launch'taki SABIT fx/fy onun kamerasiydi.
~/.local'daki numpy 2.2.6 (16:57, DeepStream-Yolo pip3 install onnx... adimi)
apt cv2'yi lapis icin bozuyor; root servis etkilenmez — kullaniciya bildirildi.

Arac videosu (eo_*.avi) record_scale 0.5 (launch car_record_scale, 28_09 olculdu):
car_detector CPU %78 -> %33 (kayit kapaliyken %15); 0.75 kazandirmiyor (INTER_AREA
kesirli oranda yavas yol) — olcek degistirilecekse tam-sayi oran (0.5, 0.25) sec.
Tam yigin + car + 0.5: CPU ~%80, akislar %97-99 duzenli, bellek sizintisi yok (8 dk).

**How to apply:** Firmware 133 B paketi bilmeli. Test: car_detector/scripts/
fake_color_pub.py (izole domain) + serial_packet_check.py (pty, 133 B). Gercek kamerayla
CPU/GPU etkisi olculmeli (renkli kamera + YOLO + EO kaydi eklendi).

Ilgili: [[infra2-test-recording]] [[autopilot-owns-frame-and-tilt]]

**Model giriş boyutu ölçümü (22_10, MAXN_SUPER, 1280×720 giriş)**

| Model | trtexec ortalama çıkarım | Güç | Not |
|---|---|---|---|
| 640 | 4.3 ms | 9.0 W | şimdiki |
| 960² | 7.8 ms | 9.9 W | |
| 736×1280 | 9.7 ms | 9.7 W | |
| 1280² | 16.3 ms (p99 20.6) | 10.8 W | |

- Dördü de 15 fps yetişiyor. car_detector CPU ~%35–38 ve RAM değişmiyor; tek maliyet GPU.
- 736×1280, 720p kameranın tam ayrıntısını kare 1280'in ~%60 hesabıyla veriyor.
- 720p kamera + 1280'lik model → 1080p kameraya gerek yok. 1080p denemesinde (640 modeli)
  car_detector CPU %34 → %60, RAM +300–400 MB, GPU +2.5 puan, tespit kazancı yok.
- Motorlar önceden derlendi: `DeepStream-Yolo/visdrone_960`, `visdrone_736x1280`,
  `visdrone_1280`. Export, setup betiğiyle aynı (`-s H W --simplify`, opset 18).
- **Tuzak:** motor derlenirken car_detector RSS ~1850 MB, `max_rss_mb` (1500) bekçisini
  aşar. Model değişince motoru önceden derle (`max_rss_mb:=0` ile).
- Tespit kalitesi henüz ölçülmedi. EO kaydı 0.5 ölçekte olduğu için karşılaştırmaya uygun
  değil. Öneri: bir test uçuşunda `car_record_scale:=1.0`, sonra modelleri yan yana çalıştır.

**Tespit kazancı ölçümü (22_10)**
- Veri: VisDrone2019-DET-val (`/home/lapis/visdrone_val`), 427 görüntü, 1280×720'ye
  ölçeklenmiş, ~14.6k araç. Aynı ağırlıklar PyTorch'ta imgsz 640 ile 1280 (rect ≈ 736×1280)
  karşılaştırıldı. conf 0.35, IoU 0.5.
- Toplam yakalama: %66.6 → %79.0.
- Boyuta göre (dik bakışta yaklaşık irtifa):

| Araç boyu | ~İrtifa | 640 | 1280 |
|---|---|---|---|
| > 60 px | < 43 m | %94.0 | %93.4 |
| 40–60 px | 43–65 m | %87 | %92 |
| 20–28 px | 93–130 m | %59 | %75 |
| 14–20 px | 130–185 m | %41 | %68 |
| 10–14 px | 185–259 m | %21 | %57 |

- Menzil yaklaşık 2× (%50 yakalama 640'ta ~120 m, 1280'de ~240 m).
- Yanlış alarm 2.14 → 2.78/görüntü; tespit doğruluk oranı aynı (%91).
- 60 m altında fark yok.
- Betikler (geçici): scratchpad/vd/eval.py ve ozet.py. Eski .pt yüklemek için
  `sys.modules['ultralytics.yolo']` takma adları gerekiyor.
- **22_10: varsayılan model 736×1280** (kullanıcı kararı; uçuş sonrası EO kaydına bakıp
  kesinleştirecek).
  - Launch arg `car_model` = 640 | 960 | 736x1280 | 1280.
  - Bilinmeyen ad → 640'a döner, uyarı basar.
  - Motor yoksa `max_rss_mb` = 0 olur (ilk açılış derlemesi kesilmesin).
  - Doğrulama (VisDrone karesi, merkezdeki araba GT (644,280)): 640 modeli (643,280)
    conf 0.68; 736×1280 modeli (644,281) conf 0.89. Dikdörtgen girişin dolgu eşlemesi doğru.
- EO kamera 1280×720 @ 15 fps kalıyor; 1080p'ye geçilmiyor. Kayıt ölçeği 0.5 değişmedi.

**Uçuş #42 (2026-11-01, 736×1280 ile ilk uçuş): yakın menzil kaybı**
- Bearing doğru: 14–64 m'de tek kilit (id 85), kutu aracın üstünde, `px` ile bx/by birebir
  tutuyor. 50–64 m'de std ~0.01 rad.
- by ≈ −0.05 sabit (araç görüntüde ~3° yukarıda). Tespit hatası değil, geometri (eğim/montaj).
- **~11 m altında araç tespit edilmiyor.** Araç kadrajı dolduruyor ama conf <0.35.
- Kayıttan offline test (aynı .pt, CPU, 10–3 m kareleri): 640 giriş conf 0.82–0.98;
  736×1280 giriş 0.25–0.68, 3.2 m'de hiç yok.
- Sebep: 1280 girişte yakındaki araç ağa VisDrone eğitiminden çok büyük görünüyor. Büyük
  model uzak menzili ~2× artırırken yakın menzil sınırını da yukarı itiyor.
- 10–15 m kör bölge: car %0, AprilTag %12–16.
- Kullanıcıya seçenekler sunuldu (640'a dönmek / irtifaya göre model / ikisi birden). Karar bekleniyor.
- **EO kameranın sağ ~%23'ü kahverengi bir cisimle kapalı** (lense yakın braket/kablo/takım?).
  Yerde de var, infra1/2'de yok. Fiziksel kontrol istendi.
- Yerde (kamera çimden 30 cm) çime yanlış kilit, conf 0.43–0.54, valid=1. %34 süre.
- **Aşırı akım olayı model kaynaklı:** 640 ile 1–22/dk, 736×1280 ile 640–1617/dk (#36'dan
  beri). Her GPU patlaması VDD_IN'i 5 A üstüne çıkarıyor.
- Kesik AVI: güç kesilince kapatılmayan videoda başlıktaki kare sayısı yanlış (17565).
  RIFF `00dc` chunk taraması gerçeği verir (21191 ≈ 14.8 fps). Kareyi ofsetten
  `cv2.imdecode` ile al.

**02_11: van sınıfı ve boy filtresi eklendi** (commit 9834d26, fc56e8a; GitHub'da)
- vehicle_classes [3, 4, 5]. Hedef araç Hilux; kasası yeşil cisimle kaplı olacak.
- serial_sender her YENİ radar karesinde `/hrvs/altitude` (sensor_msgs/Range) yayınlar.
- car_detector: L = uzun kenar px × irtifa × √(1+bx²+by²) / fx. 2.5–9 m dışındaki adaylar
  kilitlemeden önce elenir. Kenara değen (kesik) kutuda alt sınır yerine, kenara
  PARALEL boyut ≥ car_width_min_m (1.2 m) aranır; köşeye değen kutu serbest (commit d...).
- Tüm uçuşlarda tekrar oynatıldı (scratchpad size/replay_gate.py; kayıttaki overlay
  kutuları + HRVS, node ile aynı formül):
  - levha/tag hep elendi;
  - yerde 3 m altında kabul 0;
  - #42'de araç 418 gözlem kabul.
  - Eski "sadece üst sınır" kuralı yerdeki 111 kutuyu geçiriyordu.
- İrtifa 1.5 s'den eskiyse ya da K yoksa filtre KAPALI (kullanıcı kararı).
- Parametreler: size_filter_enable, car_len_min_m, car_len_max_m, altitude_stale_s,
  size_edge_px, altitude_topic.
- EO overlay: elenen kutu kırmızı ve "X", kutularda L=…m, sağ üstte "HRVS x m / boy filtresi
  KAPALI".
- Test (domain 77, overlay'i inpaint ile silinmiş kayıt kareleri):
  - levha 6.3 m'de L=1.00 elendi;
  - araç 16 m'de kabul (~5.2 m), 3 m'de elendi;
  - bayat → ~1.3 s sonra filtre kapandı.
- Gerçek radarla /hrvs/altitude yayını henüz görülmedi (test anında radar kapalıydı).
- Tuzak: fake_color_pub.py'yi `python3 -s` ile çalıştır. ~/.local'daki numpy 2.x apt cv2'yi
  bozuyor ve sessizce ölüyor.
- Kayıt karesi testi için overlay'i sil: yeşil/turuncu inRange + üst bant → cv2.inpaint.
  Yoksa çizili kutu tespiti bozuyor. 37.8 m'lik yarım ölçek kare büyütülünce tespit
  edilmiyor; 16 m kullan.

**05_11: İrtifa kaynağı = otopilot EKF, yedek HRVS** (commit 372f4b4, pushlandı; kullanıcı isteği; HRVS kopunca filtre kapanıyordu). Drone aracın ÜSTÜNDEN kalkıp yine ona iner.
- Otopilot → Jetson `0x4A`: `BB 4A 05 F6 | float ekfZ [m] LE | uint8 gecerli | cks` (10 B). EKF baro+LRF+HRVS birleşimi.
- serial_sender (`autopilot_rx.hpp` decodeEkfAlt) sadece gecerli=1 iken `/autopilot/ekf_altitude` (Range, frame "ekf") yayınlar.
- car_detector: EKF tazeyse (altitude_stale_s 1.5) HRVS mesajları yok sayılır; EKF susarsa HRVS'e düşer. range ≤ 0.1 EKF yok sayılır (yerde/NED işareti). Log `IRTIFA KAYNAGI X -> Y`, overlay "EKF x m"/"HRVS x m". Param: ekf_altitude_topic, altitude_prefer_ekf.
- Pty + sahte yayıncıyla test edildi (domain 77/78). Gerçek otopilotla görülmedi. Kullanıcı: ekfZ yere göre (AGL, EKF en çok LRF'e güvenir, kalkış noktası referansı yok), 10 Hz, yukarı pozitif. LRF aracın üstündeyken tavanı okur (~1.8 m kısa) → alçakta L düşük çıkar (3 m'de Hilux ~3.0 m, sınır 2.5); HRVS'te de aynı durum vardı.

**04_11: İKİ KAMERA (commit acb7b9d, launch car_dual=true varsayılan)**
- Yüksekte EO + 736x1280, alçakta infra1 + 640.
- Geçiş HRVS irtifasıyla: alçalırken <12 m IR, yükselirken >18 m EO.
  - Bantta iki hat da beslenir ve kilit tutar, yayını sadece aktif kanal yapar.
  - Kısa irtifa kesintisinde karar tutulur, 10 s sonra EO.
- Neden: EO 1280 girişte ~11 m altında araç fazla büyük görünüyor. Renkli kamera FOV'u
  (70×43°) 5 m'de aracı sığdırmıyor.
- Kayıt testi (#42, kare kare):
  - EO 640: 6–11.5 m %100, <6 m kısmi görüş nedeniyle düşüyor.
  - IR 640: 3–25 m %95–100, 25–35 m %77.
- IR bearing'i `/camera/camera/extrinsics/depth_to_color` (transient_local, sütun-öncelikli R)
  ile renkli çerçeveye çevriliyor. Gelmezse birim dönüşüm + uyarı.
- Masa testi:
  - Geçiş IR→EO 47 ms, EO→IR 175 ms (normal %99 209 ms).
  - RSS bantta 844 MB.
  - Bearing sıçraması 0.009 rad (tahmini IR intrinsics).
- Sahada doğrulanacak: log `KAYNAK`, `[BANT]` farkı, "extrinsics alindi" satırı, EO videosunda [IR] görüntüsü.
- Test tuzağı: test görüntüsünün irtifasıyla sahte irtifa tutarsızsa boy filtresi aracı eler
  (16 m'lik kareye 10 m deyince L=2.0 → elendi).
- Engel: EO'nun sağındaki kahverengi cisim IR'de yok.
