---
name: ir-triangle-marker
description: "LPS IR LED markerı 1.36 m ikizkenar dik üçgen; konum hipotenüs orta noktasından, ölçek üçgenin kendisinden (radar değil)"
metadata: 
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-06T08:54:22.125Z
---

LPS iniş markerı: **3 ayırt edilemeyen IR LED**, ikizkenar dik üçgen, iki dik kenar
da **1.36 m** (hipotenüs 1.92 m). Huni merkezi = hipotenüsün orta noktası.

Tasarım kararları (2026-08-25, saha testi henüz YAPILMADI):
- Konum hipotenüs orta noktasının piksel ortalamasından çıkar — dik açı köşesine
  offset UYGULANMAZ (offset marker düzleminde tanımlı olurdu, oryantasyon
  gerektirirdi; PnP tam da bunu hesaplamamak için atlandı).
- Pakete giden `irpos` **METRE DEĞİL, boyutsuz normalize yön** (X/Z, Y/Z).
  Menzille çarpmayı otopilot kendi EKF-füzyonlu yüksekliğiyle yapıyor. Gerekçe:
  bizim optik Z'mizin sessiz ölçek hataları var (marker ölçüsü, fx kalibrasyonu,
  eğimde %5-10) ve otopilot bunları doğrulayamaz; kendi yüksekliği ise
  baro/GPS/IMU ile çapraz kontrol edilebilir alanında.
- Optik Z (`Z = f/k`, üçgenin piksel boyutundan) **atılmadı ama gönderilmiyor** —
  yanlış üçgen eşleşmelerini elemekte kullanılıyor (doğrulamaya %10 doğruluk
  yetiyor) ve CSV'ye `z_opt` olarak yazılıyor. HRVS radar da kaba çapraz kontrol.
- Kamera pozlaması IR için KISILAMAZ: aynı infra1 stream'ini AprilTag ve optik akış
  da tüketiyor. Ayrım yüksek parlaklık eşiğinden gelir (`ir_min_threshold`).

**Why:** Bu üç karar da konuşarak varıldı ve koda bakınca sebebi görünmüyor;
özellikle "radar Z'sine dön" ya da "offset ekle" yönünde geri alınmaya açık.

**How to apply:** `ros2 run lps_serial_sender test_ir_triangle` donanımsız çalışan
sentetik doğrulama — parametre değiştirmeden önce ve sonra çalıştır. Menzil
göstergesi videodaki `amin=Npx` (LED piksel alanı), eğim göstergesi `fit`.

İlgili: [[autopilot-owns-frame-and-tilt]]

**02_11: gündüz sahte üçgen kuralları** (commit sonrası GitHub'da)
- Sorun: gündüz 16+ blob çıkıyor (C(16,3)=560 üçlü). #42/#49/#53/#61'de 4718 sahte
  valid=1 kare. Radar ≤0.05 m iken Z kontrolü atlanıyordu.
- Eklenen kurallar:
  - A) blob > ir_max_blobs_valid (6) ise geçersiz.
  - B) radar yok/bayatsa (hrvs_last_ms_, ir_radar_stale_ms 1500) geçersiz;
    ir_require_radar:=false ile kapatılır.
  - D) ir_min_consec (5) ardışık karede ≤ ir_max_jump_px (20) kalmalı.
- C (Z toleransı %50 → %25) bilinçli olarak EKLENMEDİ; gece LED verisi bekleniyor.
- CSV'ye aday, neden ve dizi sütunları eklendi. HUD'da "IR:RED <sebep>", videoda kırmızı aday üçgen.
- Tekrar oynatma: A+B+D 4718 → 46 (%1). A+B+C+D → 10.
- Kullanıcı: gece LED testi yapılacak; eşikler o veriyle (gerçek pozitifler) yeniden
  ayarlanacak. Açık sorular: LED'ler yanıp söndürülebilir mi, marker hunide/araçta mı.

**06_11: vinç testi → irtifaya göre LED penceresi** (içeride, LED'ler platform kutusu köşelerinde)
- Sorun: sabit `ir_max_area` 400 px. LED doygun halesi 2.4 m'de 663–723 px → LED'ler eleniyor, huni parlaması kırıntılarından yanlış üçgen (Zopt 22 m). Ayrıca seçim "en düşük hata" idi, ölçek SONRA kontrol ediliyordu → rastgele uçlu kazanıyordu.
- Ölçüm (`huni_kalib/led_kayit.py`, tag z ile): LED alan medyanı 754 @1.75, 723 @2.25, 586 @2.75, 506 @3.25, 478 @3.75 m → A ≈ 750·(1.75/Z)^0.6 (1/Z² DEĞİL). Doluluk 0.60–0.75, en/boy ~1; parlama 0.3–0.4 / 2–2.5. optZ/tagZ ≈ 0.93 (LED'ler plakadan ~20 cm yukarıda). 4 m üstü ÖLÇÜLMEDİ.
- Kod: pencere [A/6, max(400, 2.5A)] (alt sınır sadece Z ≤ 5 m), yuvarlaklık >30 px, ölçek aralığı (Z ± ir_z_tol) SEÇİMİN İÇİNDE, A kuralı penceredeki blob'ları sayar. İrtifa: `ir_alt_source` auto=EKF→HRVS; `tag` sadece yer testi (canli.sh verir). Gerçek karelerde: eski yanlış üçgen, yeni tam 3 LED.
- Canlı test 2 (`kalib_105129`, yeni kod, ir_alt_source tag): geçerlilik 1.5–2 m %78, 2–3.5 m %95–97, 3.5–4.5 m %83–93. 1.5 m altında üçgen kadraja sığmıyor (hipotenüs 1.92 m). AMA 146/704 geçerli satır SAHTEYDİ: iki LED + odadaki beyaz yuvarlak lamba (Zopt/tag 1.39 ve 0.70; doğru üçgen hep 0.90–0.93). → `ir_z_tol` 0.50 → **0.25** (02_11'de ertelenen "C" kuralı, artık LED verisiyle).
- **Gerçek üçgen (kullanıcı ölçtü 06_11): R–sağ 144.5 cm, R–sol 131 cm, hipotenüs 195.0** (ikizkenar DEĞİL). Görüntüyle doğrulandı (oran 1.114 vs 1.103, açı 89.3°). Kod 1.445/1.31'e güncellendi → fit_err 0.039 → 0.007.
- IR/tag Z oranı 0.93, eğime/yaw'a bağlı değil (1–5° test). Model ≈ %5 ölçek + 4–6 cm ofset; kenar düzeltmesi ölçeğin ~%1.4'ünü açıklıyor, kalanı açık.
- **Hipotenüs ortası huni merkezinde DEĞİL: ~10 cm kayık** (kamera frame'inde dx +6, dy +7.6 cm; yükseklikten bağımsız, saçılma <1.5 cm → fiziksel yerleşim). → Yazılımda düzeltildi: kullanıcı ölçtü, huni merkezi R'ye ve sol LED'e 90 cm, sağ LED'e 105.5 cm (hesap 105.56 ✓) → merkez = R + 0.655 m kısa kenar + 0.617 m uzun kenar boyunca (`irMarkerCenter`, `ir_center_on_short_m/long_m`). Kenar oranı < `ir_leg_ambig_ratio` 1.03 ise hipotenüs ortasına düşer. 2 gerçek karede (duvar tag ortalaması = eksen): orta nokta 9.5/8.9 cm → düzeltilmiş 1.4/1.8 cm. HUD "merkez:olcu / ORTA". Canlı doğrulandı (kalib_120058): IR−tag merkez farkı 2.2 cm. Commit 2fdd880 (pushlandı).
- Kullanıcı isteği: videoda SADECE geçerli üçgen çizilir (tüm blob'lar ve kırmızı aday kaldırıldı, CPU).
- Kayıtta çıkan tag bulgusu: [[funnel-tag-calibration]] (küçük tag z'si 2 m üstünde sapıyor).
