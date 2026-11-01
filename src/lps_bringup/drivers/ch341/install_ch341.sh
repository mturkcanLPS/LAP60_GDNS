#!/bin/bash
# HRVS radarinin CH340 USB-seri cevirici surucusu (ch341.ko) — yeni Jetson'da BIR KEZ.
#   install_ch341.sh [--force]        (kurulum adimlari sudo ister)
#
# Neden: NVIDIA L4T kernel config'inde CONFIG_USB_SERIAL_CH341 KAPALI. Stok JetPack'te
# CH340 takilinca /dev/ttyUSB* olusmaz, serial_sender /dev/ttyCH340'i acamaz (radar yok).
# 26_08_2025'te tum kernel kaynagindan (/usr/src/kernel/kernel-jammy-src, 1.3 GB) derlenip
# elle kopyalanmisti. Simdi ch341.c + JetPack'in kernel header'lari yetiyor; modul sembol
# surumleriyle (modversions) derlenir, kernel ABI uyusmazsa yuklenmeyi reddeder.
#
# Kernel guncellenip surumu degisirse (uname -r) modul yeni dizinde olmaz: tekrar calistir.
# Zaten kuruluysa dokunmaz; --force yeniden derleyip updates/ altina kurar (oncelikli).
set -e
KVER=$(uname -r)
KDIR=/lib/modules/$KVER/build
HERE=$(cd "$(dirname "$0")" && pwd)

if [ "$1" != "--force" ] && modinfo -k "$KVER" ch341 >/dev/null 2>&1; then
    echo "ch341 zaten kurulu: $(modinfo -k "$KVER" -n ch341)  (yeniden derlemek icin --force)"
    exit 0
fi
[ -f "$KDIR/Module.symvers" ] || {
    echo "kernel header'lari yok: sudo apt install nvidia-l4t-kernel-headers" >&2; exit 1; }

B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
cp "$HERE/ch341.c" "$HERE/Makefile" "$B/"
make -C "$KDIR" M="$B" modules
sudo install -D -m 644 "$B/ch341.ko" "/lib/modules/$KVER/updates/ch341.ko"
sudo depmod -a "$KVER"
sudo modprobe ch341
echo "Kuruldu: /lib/modules/$KVER/updates/ch341.ko"
echo "CH340'i cikarip tak (ya da udev kuralini yukle) -> /dev/ttyCH340"
