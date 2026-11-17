---
name: autopilot-owns-frame-and-tilt
description: "LPS Jetson tarafı pozisyonları kamera optik frame'inde gönderir; gövde dönüşümü ve eğim düzeltmesi otopilotun sorumluluğu"
metadata: 
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-08-25T11:27:22.020Z
---

lps_serial_sender'da (ve genel olarak Jetson tarafında) pozisyonlar **kamera optik
frame'inde** (X sağ, Y aşağı, Z lensten dışarı) gönderilir. Gövde frame'ine çevirme
ve drone eğimi (roll/pitch) düzeltmesi **otopilot firmware'inin işi** — Jetson
tarafında R_mount benzeri bir dönüşüm uygulanmaz.

**Why:** Kullanıcı 2026-08-25'te açıkça belirtti ("body dönüşümünü drone kendi
yapıyor, biz kamera frame'inde gönderiyoruz" ve "eğimi dronun gyrosuyla düzeltecek,
o bizim işimiz değil"). Kod tabanında AprilTag huni yolu da aynı konvansiyonu
izliyor (funnel → avgPos.pos, hiçbir mount dönüşümü yok).

**How to apply:** Yeni bir sensör/ölçüm eklerken pozisyonu kamera frame'inde bırak;
"eksen takası eksik" diye R_mount eklemeye kalkma — iki yolu tutarsız yapar ve
otopilot dönüşümü ikinci kez uygular. Eğim kaynaklı hatalar için de Jetson tarafında
telafi yazma, sadece ölçümün eğimde ne kadar bozulduğunu raporla.

İlgili: [[ir-triangle-marker]]

**İstisna (03_11, kullanıcı isteği): kamera → platform merkezi ÖTELEMESİ Jetson'da.**
- AprilTag avgPos'tan her kameranın ofseti, KAMERA ekseninde düşülüyor. Ofset kameradan
  platform merkezine vektör:
  - infra1 (32.962, 45) mm
  - infra2 (−17.962, 45) mm
- Döndürme hâlâ YOK: eksenler kamera ekseni kalıyor. Kamera gövdeye rijit olduğu için
  ofset kamera ekseninde sabit; yaw/eğim onu bozmaz.
- x işareti uçuş kaydıyla doğrulandı: iki kameranın aynı tag'i ham +45 mm → ofsetle −6 mm.
  y (45 mm) stereo ile doğrulanamaz; tag tam merkez altındayken x,y≈0 testi önerildi.
- Akışa ofset uygulanmadı. Birleşik akış iki kameranın orta noktasına karşılık gelir
  (7.5, 45 mm); yaw hızında ω×r kadar fark eder, gyro telafisi otopilotta.
