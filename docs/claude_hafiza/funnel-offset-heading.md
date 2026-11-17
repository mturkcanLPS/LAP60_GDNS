---
name: funnel-offset-heading
description: "15_11 araç kutusu→huni ofseti (y +1.45 m, kamera ekseni) + 0x4A yeni 19 B format (dron/araç başlığı) ile döndürme; overlay'siz EO videosu + IMU/ham akış CSV'leri (841737c)"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-15T12:09:41.724Z
---

15_11 commit 841737c, pushlandı:
- **Huni ofseti:** Otopilota giden car bearing kutu merkeziydi (aracın ortası). Kullanıcı huniyi istiyor. 175/176 uçuşlarından (overlay'li videolardan HUD okuyarak, `huni_kalib/ofset_olc.py`) ölçüldü: x ≈ 0, **y = +1.46 m** (kamera ekseni, görüntüde aşağı = aracın arkası), 4–14 m'de sabit. Alçakta küçülüyor (araç kadraja sığmıyor) → 3 m altında 0, 6 m üstünde tam. Eski kayıtlarda huniye hata 1.47 → 0.42 m (176), 0.83 → 0.50 m (175). Kalan hata: kafa açısı farkı + EKF/tag z oranı (0.78–0.86).
- **0x4A yeni format (OKAD V1.1.3, 19 B, yük 14):** ekfZ, gecerli, dronBaslik, yerBaslik [derece, kuzey 0, saat yönü +], baslikDurum (bit0-1 dron INS, 2-3 yer INS, 4 yer RTK taze). `decodeEkfAltMsg` 5 ve 14 baytı okur. serial_sender başlıkları `/autopilot/heading` (Vector3Stamped x=dron y=araç z=durum) yayınlar + RX CSV'ye yazar. car_detector ofseti R(−dpsi) ile döndürür (dpsi = dron − araç; aşağı bakan kamera aynalanmamış üstten görünüş: dron saga 90 → (0,1.45)→(1.45,0)); iki INS modu 0 DEĞİLSE (kullanıcı kararı) ve yer RTK taze (bit4, kullanıcı: kalsın) ise; yoksa döndürmez.
- **EO videosu overlay'siz** (`car_record_overlay` false), bilgiler `eo_sNNNN_*.csv` (kare başı satır; kutular, bearing, irtifa, baslik_farki). Amaç: kendi araç/huni modelini eğitmek.
- **IMU + ham akış logu:** `imu_sNNNN.csv` (D435i gyro+accel 200 Hz, kamera saatiyle aynı), `flow_sNNNN.csv` (all_* filtresiz ortalama, tex_* dokulu ortalama, raw_* RANSAC sonrası, ema_*, kam_t). `imu_log:=false` kapatır.

İlgili: [[car-detection-bearing]] [[hrvs-takeoff-dropout]]

**İlk uçuş doğrulaması 15_11 (s0192/s0193, IR kanalı, tag'le karşılaştırma, saat kayması EKF eşleştirmesiyle +3 s):** başlık durumu 0x15 (INS 1/1, yer taze) → döndürme aktif, fark −1.7…+2°. Huniye uzaklık medyanı: 192'de 1.50 → **0.16 m**, 193'te 1.54 → **0.36 m** (193'te kalan +0.16/+0.31 m). Ofset işe yarıyor.
