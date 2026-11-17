---
name: l4t-3647-interrupted-update
description: 2026-10-22 GUI güncellemesi (L4T 36.4.4→36.4.7) packagekit kendini güncellerken kesildi; 2026-11-01 tamamen onarıldı + otomatik güncelleme kapatıldı
metadata:
  node_type: memory
  type: project
  originSessionId: 1f95f232-f282-45a7-85a9-7fb6df1a054f
  modified: 2026-11-01T11:36:09.125Z
---

2026-10-22 19:46'da Yazılım Güncelleyici (packagekit role=update-packages) 907 paketlik güncelleme başlattı, 19:49:33'te packagekit paketi kendini unpack ederken oturum öldü → SSD/rootfs sorunlu kaldı. Kullanıcı SSD'yi başka PC'de onardı, 2026-11-01 13:23'te Jetson'da `dpkg --configure -a` + `apt -f install` ile tamamladı.

**DURUM 2026-11-01 15:00: TAMAMLANDI** — UEFI 36.4.7 (slot B, A yedekte 36.4.4), açılış /boot/initrd, 525+2 paket kuruldu (0 kaldırma), dpkg temiz, reboot-required yok, faz3 tüm maskeler aktif, yığın temiz kalkıyor. Açık kalan (önceden var): Intel librealsense apt repo NO_PUBKEY FB0B24895113F120; lps_serial_sender OpenCV 4.5d/4.8 linker çakışma uyarısı.

İlk kontrolde (2026-11-01) bulunanlar:
- 380/907 paket kuruldu (tüm nvidia-l4t-* 36.4.7, kernel/DTB partition yazımı başarılı); 527 paket hiç kurulmadı, **hiçbir ros-humble paketi güncellenmedi** (ROS tutarlı eski sette, karışık sürüm yok). CUDA 12.6 / TRT 10.3 / cuDNN 9.3 değişmedi → .engine dosyaları geçerli.
- `/boot/extlinux/extlinux.conf` INITRD satırı başka PC'de üretilmiş `/boot/initrd.fixed`'i gösteriyordu; Jetson'ın ürettiği `/boot/initrd` üst küme (nvluks-srv-app vb. ekstra). Sonraki L4T güncellemeleri initrd.fixed'i güncellemez.
- nvidia-l4t-bootloader postinst'te `fwupdtool install-blob` sessizce başarısız (çıktı /dev/null), ESP'de EFI/UpdateCapsule yok, OsIndications=0 → UEFI hâlâ 36.4.4 (bios_version). Payload: /opt/ota_package/t23x/TEGRA_BL_3767_super.Cap, fwupd device id 8519221db67a72ffbce42cdf46a48e049decbcff.
- **Capsule kök sebebi:** fwupd capsule-on-disk ESP'de 2× capsule ister ("required 102.6 MB, need additional 36.7 MB"), ESP (nvme0n1p10) sadece 63 MB → her apt L4T güncellemesinde bootloader sessizce atlanır. Çözüm: NVIDIA elle yöntemi (EFI/UpdateCapsule/TEGRA_BL.Cap + OsIndications'a 07000000 04000000 00000000). 2026-11-01 faz1.sh ile hazırlandı; betikler/loglar ~/sistem_bakim_2026-11-01/ (faz2.sh: apt dist-upgrade, RealSense 2.57.7/4.57.7 hold, colcon --symlink-install lps_serial_sender car_detector).
- extlinux 2026-11-01'de /boot/initrd'ye çevrildi, `LABEL fixed` (initrd.fixed) yedek seçenek; yedek dosya extlinux.conf.bak-20261101-142016.
- NVMe SMART (2026-11-01): media_errors 0, %0 kullanım, ext4 clean, lost+found boş; ama unsafe_shutdowns 456/573 power cycle, Critical Composite Temp Time 154 dk, T2 throttle ~4.2 sa, sensör 2 boştayken 85 °C.
- **Tetikleyici:** gnome-software `download-updates=true` güncellemeleri kendisi indiriyordu; 22_10 19:45:40 reboot'ta GNOME kapanış ekranı `trigger-offline-update` yaptı → açılışta packagekit offline update → /var/lib/PackageKit/offline-update-competed "Success=false, The transaction did not complete".
- **Otomatik güncelleme kapatıldı (2026-11-01):** kullanıcı tarafı yapıldı (gsettings org.gnome.software download-updates/download-updates-notify/allow-updates=false; ~/.config/autostart/{update-notifier,gnome-software-service}.desktop Hidden=true). Sistem tarafı faz3_otomatik_guncelleme_kapat.sh: APT::Periodic 0 (99-otomatik-guncelleme-kapali), apt-daily/apt-daily-upgrade/fwupd-refresh/update-notifier-* timer mask, packagekit + packagekit-offline-update mask, `snap refresh --hold`. Geri alma talimatı betiğin başında. aptdaemon systemd birimi değil (dbus) → Software Updater elle açılırsa hâlâ çalışır.
- Önceden var olan, güncellemeyle ilgisiz: logrotate jetson-logging duplicate entry hatası; TRT "across different models of devices" uyarısı Eylül'den beri var.

**Why:** Sonraki oturumda "güncelleme bitti mi / sistem tutarlı mı" sorusunda aynı adli incelemeyi tekrar yapmamak için.
**How to apply:** Önce extlinux INITRD, `cat /sys/class/dmi/id/bios_version` ve `apt list --upgradable | wc -l` ile bu maddelerin kapatılıp kapatılmadığını doğrula. Jetson'da güncellemeyi GUI'den değil terminalden `sudo apt upgrade` ile yaptır (packagekit kendini güncellerken ölüyor). İlgili: [[jetson-health-thermal]]
