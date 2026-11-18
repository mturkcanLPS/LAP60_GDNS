---
name: flights-216-217
description: "18_11 uçuşları s0216 (60 m) / s0217 (40 m) + OKAD .mat (Downloads/ucus_matlab_log): car bearing 55 m'de 0.85 m doğru, kullanıcının gördüğü 5 m = eğim düzeltmesiz + lazer 40 m üstü ölçmüyor; gyroflow tau bugün ~0 ms (10 değil) → akış 2–3× gürültülü; HRVS (CH340) yine kalkışta koptu"
metadata:
  type: project
---

Eşleşme: OKAD_261118_T01_1306.mat = s0216, T02_1405 = s0217, yer_1420 = s0218 (yer testi). `tools/analiz/mat_kontrol.py` (16 bit zaman sayacı artık mat zamanından beklenen artışla açılıyor; akışta yükseklik ekfOutPosZ) + `otopilot_hedef.py` + `tau_tara.py`.
- **Araç yönü (montaj düzeltmesi −2.2/−1.1 dahil):** RTK'ya göre medyan 0.13–0.24 m (8–20 m), 0.36 (20–30), 0.44–0.63 (30–45), **0.85 m (45–70 m, p90 1.18)**. Kalan açı ±0.5°, huni ofseti metre kısmı ≤0.2 m. Kırpık kutu nadir (88 kare). → Bizim yön doğru.
- **Kullanıcının 55 m'de gördüğü ~5 m:** bizim yön × yükseklik EĞİM DÜZELTMESİZ çevrilince 45–70 m'de medyan 5.41 m (p90 6.6) — birebir. Dron rüzgârda ort. roll +3.5° (1.8–5.6), pitch +1.3°. Tam roll/pitch düzeltmesiyle 0.84 m. Ayrıca **LaserAltitude 40 m üstünde geçersiz** (%52 sıfır, RTK'ya göre −56 m); EKF z doğru (+1.0 m). Otopilotun Gud_DLat/DLon'u RTK'ya 0.6–0.9 m yakın ama bizim yöne bağlı değil (GPS modunda RTK'dan). → OKAD: yönü konuma çevirirken roll/pitch uygula, yükseklik EKF z (lazer değil).
- **Optik akış:** dron neredeyse sabit (INS hız std 0.05–0.10) → R² anlamsız; sabit kazançla hız hatası std yükseklikle artıyor (T01 45–70 m: sağ 0.69 / ileri 1.98 m/s kare başı). **tau taraması: s0209 +10, s0210 +7.5, s0216 −2.5, s0217 0 ms** → bugün tau=10 yanlış, kalan gürültü dy/dx 0.55/0.25 → 0.19/0.11 px (≈3×). Hipotez: pozlama süresine bağlı (akşam uzun, gündüz kısa); pozlama loglanmıyor → kanıtlanamadı. Öneri: çevrimiçi otomatik tau seçimi (kullanıcı kararı bekliyor).
- 0x4A tutarlı (EKF z = ekfOutPosZ, başlıklar ±0.2°). NTP kapatma çalıştı (saat atlamadı), kalıcı journal açılışlar arası tutuluyor.
- **HRVS (bu uçuşlar hâlâ CH340):** yerde 4 s'lik reset boşlukları sık; kalkışta (EKF 1.8 / 2.4 / 1.0 m) yine koptu, havada ölçüm oranı s0216 %60 (kısa dönüşler), s0217 %0. Black Pill sonraki uçuşta denenecek.

İlgili: [[flights-209-210]] [[gyroflow-of]] [[hrvs-blackpill]] [[autopilot-owns-frame-and-tilt]]
- **18_11 devam (kullanıcı cevapları):** OKAD tarafında kalibrasyon YOK; otopilot roll/pitch uygulayacak; car bearing'de EKF z kullanıyor; EKF'ye "delta t'ye göre" giriyor (ölçüm yaşı mı, ölçümler arası dt mi — belirsiz; bizim car.timeStamp = Jetson'a varış anı, çekimden ~100 ms sonra).
- 85 cm ayrışımı (45–70 m): sabit açı ~0.75° → ~0.8 m (montaj kalıntısı), dağınık ~0.4 m = gecikme (otopilot eğimi 100 ms geri kaydırınca hızlı dönüşte 0.59→0.31°) + kutu titremesi ~0.3° (~5 px).
- **Yerde montaj (`tools/analiz/montaj_yer.py`, D435i ivme − otopilot roll/pitch, hareketsiz):** x: s0209 +2.97, s0210 +3.24 → s0216 +2.30, s0217 +2.47, s0218 +2.54 → günler arası ~0.6° değişim (RTK ile tutarlı), açılışlar arası ±0.25°. Kalibrasyon yok → mekanik (braket). y gürültülü (D435i ivme sapması).
- **Yapıldı:** boresight launch −1.8/−1.8; gyroflow τ otomatik (gyroflow_oto_tau.hpp, 9 aday, EMA 300, histerezis %5, kalite ≥30; tau_sim ile kayıtlarda doğrulandı; test_gyroflow_oto_tau); pozlama logu (infra1 metadata actual_exposure → flow CSV pozlama_us). Servis yeniden başlatılınca canlı doğrulanacak (metadata topic gerçekten yayınlanıyor mu).
- **18_11 kullanıcı kararları:** pozlama logu BOŞVERİLDİ (stok uvcvideo, metadata'da actual_exposure yok; kod kaldırıldı). Montaj düzeltmesi şimdilik Jetson'da kalıcı (−1.8/−1.8). `YRTK_HedefN/E/D` = HUNİ MERKEZİ (kullanıcı) → tag'ler RTK ile kıyaslanabilir; **koda değişiklik yapmadan önce kullanıcıya mutlaka danış**. car.timeStamp (çekim anı mı) kullanıcı OKAD'a soracak. Açılışta otomatik montaj kontrolü şimdilik ertelendi.
