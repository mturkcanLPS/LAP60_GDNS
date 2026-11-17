---
name: hrvs-takeoff-dropout
description: "148-151 uçuşları (08_11): HRVS her kalkışta (0.4-0.8 m) susuyor, havada 39-160 reset cevapsız, inince dönüyor — donanım (besleme/toprak); EKF ~11 m'de bloklar halinde kesik; RX CSV eklendi (cb0d37b)"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-12T15:39:59.093Z
---

08_11 uçuşları s0148–s0151 (aynı saatte açılış 22:16 — RTC yok; s0152 YER testiydi, otopilot bağlı değil — analize katma):
- **HRVS kalkışta ölüyor:** 5 kalkışın 5'inde, 1 m'yi geçmeden 0.3–0.8 s önce (irtifa 0.4–0.8 m). SDK logu (`hrvs_sNNNN/*/hrvsSdkLogs.txt`, journal kaybolsa da kalır): "Timeout … tamponda 0 bayt, okunan 0" — radar tamamen sessiz. Havada 39/160/108 reset+konfig denemesi, hiç cevap yok; inişten sonra yerde (0.4 m) geri geliyor (2–9.5 dk). Kod tarafı doğru (PH.07 darbe + yeniden konfig + bekçi). Sebep donanım: kalkış akımında besleme düşmesi / toprak kayması (en olası) ya da EMI. Kullanıcıya önerildi: yerde gaz testi + radar besleme ölçümü + ayrı bataryayla deneme.
- **EKF (0x4A) sadece ~10–14 m'de kesik:** süreler 5.5 s + 8k s (5.5/13.5/21.6/29.5/53.6), uçuşun %9–24'ü irtifasız (HRVS de ölü). Yerde ve 20–50 m'de geliyor. Otopilota sorulacak: FINAL DESCENT/bekleme sırasında 0x4A gönderiyor mu, gecerli kaç?
- Sonuç: car_detector 10 s sonra EO'ya dönüyordu (~11 m'de, kilit kesik). → `altitude_lost_keep_s` 120 s (uçuşta irtifa kesilirse karar korunur).
- `rx_sNNNN_*.csv` eklendi (E: her 0x4A; S: saniyelik özet). Sahte otopilotla: 45 Hz 0x42 yankısı varken bile 0x4A kaybı yok → yankı tek başına sebep değil.
- Otopilot bizim 0x42 paketini geri yankılıyor (141'de görüldü, otopilot bağlıyken) — sorulacak.
- Journal her uçuşta güç kesilince kayboluyor (HRVS SDK TRACE journal'ın %87'si). Çözüm önerildi, yapılmadı.

İlgili: [[hrvs-startup-hang]] [[car-detection-bearing]] [[jetson-health-thermal]]

**15_11 s0192/s0193 (IMU logu ilk kez var):** Yine kalkışta (EKF 1.2–1.3 m) koptu, inişten sonra döndü; havada 50–67 reset cevapsız. IMU: motorlar 9–20 s çalışırken (ivme RMS 0.7–1.9 m/s²) radar sağlam; kalkış anında RMS **6.4–6.8**, tepe 13–27 m/s² → ≤1 s içinde kopuyor. Elle sallama testi de koparmıştı (12_11) → en güçlü şüphe gevşek konnektör/kablo (kalkış darbesi). SDK logu ikili karakter içeriyor: `grep -a` kullan.
