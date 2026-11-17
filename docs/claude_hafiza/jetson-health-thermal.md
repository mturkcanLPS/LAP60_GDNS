---
name: jetson-health-thermal
description: "Sistem sağlığı AYRI mesaj (msg_id 0x48, 31 B, 1 Hz; ana paket 133 B değişmedi); 20_10 fan dönmeyen açılışlarda Jetson 104 °C termal kapanma yaptı"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-10-20T12:35:20.111Z
---

**Sistem sağlığı mesajı (2026-10-20, kullanıcı isteği: sağlığı uzaktan görmek)**
Önce ana paketin sonuna eklendi, sonra kullanıcı isteğiyle AYRI MESAJA çevrildi.
- Aynı header, **msg_id 0x48**: header 4 + healthPack_t 26 + chksum 1 = **31 B**.
- Her yeni örnekte (~1 Hz) bir kez gönderilir.
- **Ana paket 0x42 değişmedi**: 133 B, 50 Hz.
- Seri portun tek yazarı TX thread'i olduğu için 0x48 ana paketin hemen arkasından aynı
  thread'den yazılıyor (`writeAll` yardımcı fonksiyonu). O tick'te hat 14.2/20 ms.
- Örnekleyici: `lps_serial_sender/include/lps_serial_sender/system_health.hpp`. 1 Hz,
  executor timer'ı. Log satırı `[HEALTH]`.
- Alanlar: timeStamp u32, valid, uptime u32 s, cpuLoad, cpuCoreMax, gpuLoad (%),
  cpu/gpu/tj sıcaklık i8 °C (okunamazsa −128), ram/swap/disk % (disk df ile aynı),
  fanRpm u16, inVoltage u16 mV, inPower u16 mW (VDD_IN), ocEvents u16.
- Decoder `lps_bringup/scripts/serial_packet_check.py` iki mesaj tipini birlikte çözüyor.
- pty testi: 0x42 50.1 Hz, 0x48 1.00 Hz, bozuk paket 0.
- Firmware 0x48'ü tanımasa bile ana paket aynı; bilinmeyen msg_id'yi atlaması yeterli.
  Ama 0x48'ü atlamayıp ayrıştırıcıyı bozuyorsa sorun olur, firmware'e sorulmalı.
- Test tuzağı: `ros2 run`'ı kill -9 yapmak node'u öksüz bırakıyor. SIGINT'i node PID'ine
  gönder.

**Termal kapanma bulgusu (20_10 13:53–15:19)**
- 5 açılışta kernel `pwm-tegra-tachometer: Tachometer Overflow` verdi, yani fandan devir
  sinyali yok.
- O açılışlarda CPU 5 dk'da 99 °C (kısma), soc1/tj 104 °C'de "HARDWARE PROTECTION shutdown
  (Temperature too high)": 14:07:58 (858 s) ve 14:50:52 (240 s).
- Fan dönen açılışlarda (12:31, 15:19) sıcaklık ~68 °C, fan ~3300 rpm.
- Kullanıcı kabloya dokunmadığını söylüyor. Fan konnektörü/temassızlık şüphesi,
  doğrulanmadı.
- Teşhis için `kern.log`'da açılış başına Tachometer / cpu-throttle-alert / HARDWARE
  PROTECTION sayımı yapılır.

**Why:** Uçuşta fan durursa ~14 dk'da LPS tamamen düşer. Otopilot fanRpm==0 && tj yüksekken
uyarmalı.

**How to apply:** Uçuş öncesi fan dönüşü kontrol (fanRpm > 0).

İlgili: [[car-detector-oom-crash]] [[dds-hostid-shm-race]] [[car-detection-bearing]]

**GEÇİCİ (20_10, kullanıcı isteği):** 0x48'in `cpuTemp` alanında CPU değil, **tüm HRVS
radarlarının en yüksek çip sıcaklığı** gidiyor. Paket yapısı değişmedi.
- Kaynak: `min_range.csv`, her saniye okunuyor.
- Radar verisi 5 s'den eskiyse −128.
- Param `health_cpu_temp_source` (radar|cpu, varsayılan radar). Geri almak için `cpu`.
- `[HEALTH]` logu gerçek CPU sıcaklığını basar, sonuna "pakette cpuTemp=RADAR N C" ekler.
- Canlı veriyle test edildi: radar max 69–70 → kaynak kaldırılınca 5 s sonra −128.

**02_11 saha uçuşları (#45–#53): ısı birikmesi, kısmaya 2 °C kaldı**
- Trip noktaları: cpu/gpu/soc passive 99 °C, critical 104 °C. tj active 35/74/95 (fan
  95 °C'de 6000 rpm'e çıkıyor).
- nvfancontrol `FAN_DEFAULT_PROFILE quiet` → 95 °C'ye kadar fan ~4000 rpm.
- Ardışık batarya değişimlerinde Jetson soğumadı. Açılıştan 30 s sonraki sıcaklık
  81 → 85 → 89 → 92 → 95 °C; tepe 89/90/94/97/97.
- #52 ve #53'te sürenin 7–9 dk'sı ≥95 °C. Throttle ya da kapanma olmadı.
- Karşılaştırma: #47 (yerde 45 dk, soğuk başlangıç) tepe 81 °C.
- OC olayları sıcaklıkla artıyor: 788 → 2400/dk.
- Öneriler (kullanıcı kararı bekleniyor): fan profili `cool`, uçuşlar arası soğutma/gölge,
  sahada GUI kapalı, 640 model.

**#57 (02_11, yerde ~19 dk): tj 100 °C, fan TAM devirde iken bile.**
- Soğuk başlangıç 67 °C, sürekli tırmanış. 12. dk'dan sonra fan ~6000 rpm.
- ≥95 °C 511 s, ≥99 °C 160 s (passive trip 99 → kısma bölgesi). Yük CPU %75, GPU %2–55,
  ~13.5 W.
- Sonuç: fan profilini `cool` yapmak tek başına yetmez, fan zaten tavanda. Yük azaltma
  (GUI kapalı, 640 model, 25 W modu) ya da hava akışı/gölge şart.
- Fan profili hâlâ `quiet` (kullanıcı değiştirmedi).
