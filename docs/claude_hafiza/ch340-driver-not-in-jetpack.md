---
name: ch340-driver-not-in-jetpack
description: "Stok L4T kernel'inde CH341 (CH340) sürücüsü kapalı; HRVS portu için ch341.ko elle derlenmiş (26_08_2025), repoda drivers/ch341/install_ch341.sh ile kuruluyor"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-01T12:08:13.197Z
---

**Durum:** NVIDIA L4T kernel config'inde `# CONFIG_USB_SERIAL_CH341 is not set` (`/proc/config.gz`). Stok JetPack'e CH340 takılınca tty oluşmuyor ve serial_sender `/dev/ttyCH340`'ı açamıyor, yani HRVS radar çalışmıyor. Kullanıcı bunu "kernelde tekrar derlemiştik" diye hatırladı (2026-11-01).

**Bu Jetson'da:**
- `/lib/modules/5.15.148-tegra/kernel/drivers/usb/serial/ch341.ko` 2025-08-26'da `/usr/src/kernel/kernel-jammy-src`'den (L4T kernel kaynağının tamamı, 1.3 GB) `CONFIG_USB_SERIAL_CH341=m` ile derlenip elle kopyalanmış.
- Hiçbir pakete ait değil. Bu yüzden 22_10'daki 36.4.7 kernel paketi güncellemesinden sağ çıktı ve hâlâ yükleniyor.
- Modversions'sız derlenmiş: açılışta "no symbol version for module_layout", taint OE. Çalışıyor.

**Repo çözümü (01_11):** `src/lps_bringup/drivers/ch341/` altında `ch341.c` (değiştirilmedi), Makefile ve `install_ch341.sh`.
- Sadece `nvidia-l4t-kernel-headers` ile out-of-tree, modversions'lı derliyor.
- `updates/` altına kuruyor. Zaten kuruluysa dokunmuyor; `--force` ile yeniden derleyip kuruyor.
- Doğrulama: 24 ortak sembolün CRC'si stok 36.4.7 modülleriyle aynı.
- Bu Jetson'da gerçek kurulum yapılmadı (sudo yok). Sahte sudo ile adım sırası denendi.

**Why:** Kernel sürümü (`uname -r`) değişen bir JetPack güncellemesi ya da yeni flash sonrası radar sessizce gelmez. Kod hatası gibi görünür ama sebep sürücüdür.

**How to apply:**
- Radar 0 bayt veriyorsa ve `/dev/ttyCH340` yoksa önce `lsmod | grep ch341` ve `modinfo ch341` kontrol edilir.
- Kernel güncellemesinden sonra `install_ch341.sh` çalıştırılır.

İlgili: [[hrvs-startup-hang]] [[l4t-3647-interrupted-update]] [[github-repo-lap60-gdns]]
