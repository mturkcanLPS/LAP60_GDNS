---
name: hrvs-blackpill
description: "18_11: HRVS radar artık STM32 Black Pill USB-TTL (0483:5740, cdc_acm → ttyACM0, udev /dev/ttyBlackPill); masa 40 s 0 hata; CH340 öncesinde 2–4 dk'da bir bozuk yük/reset"
metadata:
  type: project
---

18_11 kullanıcı: arkadaşı STM32 Black Pill üzerine USB↔TTL köprü yazılımı yazdı, HRVS artık onunla bağlı. USB kimliği 0483:5740 "STM32 Virtual ComPort" (seri no 206A31603839), CDC ACM → `/dev/ttyACM0` (sürücü `cdc_acm` JetPack'te modül olarak hazır, CH340 gibi elle derleme YOK).
- udev: `99-usb-serial.rules`'a `/dev/ttyBlackPill` + `ENV{ID_MM_DEVICE_IGNORE}="1"` (ModemManager bu Jetson'da kurulu değil ama yeni kurulumda ACM'ye AT komutu yollayabilir). Kullanıcı sudo ile kuracak.
- launch: yeni arg `hrvs_port` (varsayılan `/dev/ttyBlackPill`), serial_sender varsayılanı da aynı.
- Masa testi (serial_sender ayrı domain, pty otopilot, ttyACM0): ilk denemede 4 radar hazır, 40 s'de 0 hata, 0 reset, 206 ölçüm, radar başı 103 mesaj, sıcaklık 42–52 °C.
- Öncesi: CH340 18_11 sabahı s0215'te 24 dk'da 10 reset ("payload corrupted" → 4/4 MINRANGE timeout), dron hareketsizken; dün öğlen 0 idi.
- Takip: uzun masa testi + kalkıştaki kopma Black Pill ile devam ediyor mu (uçuş).

İlgili: [[hrvs-takeoff-dropout]] [[ch340-driver-not-in-jetpack]] [[hrvs-startup-hang]]
