---
name: post-flight-checks-pending
description: "03_11 özelliklerinin uçuş doğrulaması — #74 (03_11, 6 kısa uçuş, levha üstü) sonuçları ve hâlâ açık kalanlar (KAPAT'ta güç erken kesiliyor, gece IR, araçlı uçuş)"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-03T14:05:01.723Z
---

**#74 uçuşunda doğrulandı (03_11, 29 dk, 6 kısa uçuş, max 11.5 m, sadece tag levhası, araç yok):**
- [FUSION]:
  - esli kare **%92**, saf parite scheduler'a GEREK YOK.
  - Havada tag iki kamerada %54; iki kamera farkı medyan 24–29 mm (3B).
  - Akış iki kamerada %96–98.
- /hrvs/altitude gerçek radarla çalışıyor. EO overlay'inde "HRVS x m boy 2.5-9.0m".
- Boy filtresi:
  - 121 eleme; yerde çim L 0.02–0.06 m, havada levha ~1.0 m.
  - Levhaya kilit 0 (eskiden kilitleniyordu).
  - Araç yoktu, kabul yolu henüz gerçek uçuşta görülmedi.
- IR: 52 950 kare, valid 0 (kalabalık %98, Z uyuşmaz 191, kararsız 66).
- HRVS: 0 hata, 0 reset, 0 kesinti (kablaj kalıcı çözüm). Uçuşta 3.5 Hz.
- Isı: tj tepe 92 °C (≥90 °C 294 s, ≥95 °C 0). Fan hâlâ quiet, max ~4100 rpm.

**KAPAT (0x49) bulgusu:** Otopilot uçuş sonunda gönderdi (18 B, 3 çerçeve), poweroff başladı.
- Ama güç ~1 s içinde kesildi; 3 AVI'de idx1 yok.
- Başlık kare sayısı güncellenmişti, yani kapanış başlamıştı.
- Düzgün kapanış ~20 s sürüyor. Servis stop'u ros2 launch python yüzünden 15 s'lik
  TimeoutStopSec'e kadar SIGKILL bekliyor.
- Öneriler: otopilot/operatör KAPAT'tan sonra beklesin.
- 15 s'lik stop takılması ff73be5 ile düzeltildi (ros2 daemon). Restart sonrası stop süresi ve
  gerçek poweroff süresi ölçülmeli.

**Bir sonraki uçuşta bakılacak (04_11 iki kamera, ff73be5 stop düzeltmesi):**
- `KAYNAK EO -> IR / IR -> EO` satırları ve "kilit HAZIR" mı.
- `[BANT]` EO-IR bearing farkı. Büyükse extrinsics/intrinsics sorunu.
- "IR->renkli extrinsics alindi" satırı (gelmediyse QoS/topic sorunu).
- Alçakta IR tespit oranı. RSS (~850 MB beklenir), CPU.
- Servis stop süresi: SIGKILL'siz, birkaç saniye.

**Hâlâ açık:**
- Araçlı uçuşta boy filtresinin kabul yolu (Hilux/yeşil kasa).
- Gece LED testiyle IR eşikleri.
- Masa oturumu #76'da RX'e kendi 0x42 paketimiz geri geliyordu (masada TX–RX yankısı?);
  uçuşta yoktu.

İlgili: [[infra2-test-recording]] [[car-detection-bearing]] [[ir-triangle-marker]] [[jetson-health-thermal]] [[hrvs-startup-hang]]
