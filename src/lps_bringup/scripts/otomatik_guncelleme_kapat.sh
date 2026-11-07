#!/bin/bash
# Otomatik guncellemeyi sistem genelinde kapatir (yeni Jetson kurulumunda BIR KEZ).
# Neden: 22_10'da GNOME'un arka planda indirdigi 907 paketlik L4T guncellemesi, packagekit
# kendini guncellerken oldu ve rootfs yarim kaldi (SSD baska PC'de onarildi). Ayrica ESP kucuk
# oldugu icin GUI/apt yoluyla bootloader capsule'u sessizce atlaniyor. Guncelleme bundan sonra
# SADECE terminalden: sudo apt update && sudo apt upgrade  (RealSense paketleri hold'da).
#
# Calistir:  sudo bash src/lps_bringup/scripts/otomatik_guncelleme_kapat.sh
# Kullanici tarafi (masaustu oturumunda, sudo'SUZ) README'de.
#
# Geri almak icin:
#   sudo rm /etc/apt/apt.conf.d/99-otomatik-guncelleme-kapali
#   sudo systemctl unmask packagekit.service packagekit-offline-update.service apt-daily.timer \
#        apt-daily-upgrade.timer fwupd-refresh.timer update-notifier-download.timer update-notifier-motd.timer
#   sudo systemctl enable --now apt-daily.timer apt-daily-upgrade.timer fwupd-refresh.timer \
#        update-notifier-download.timer update-notifier-motd.timer
#   sudo snap refresh --unhold
set -u
[ "$(id -u)" -eq 0 ] || { echo "sudo ile calistir: sudo bash $0"; exit 1; }
if pgrep -x 'apt|apt-get|dpkg' >/dev/null; then
    echo "apt/dpkg calisiyor, bitince tekrar calistir."; exit 1
fi

bolum() { echo; echo "=================== $* ==================="; }
ZAMANLAYICILAR="apt-daily.timer apt-daily-upgrade.timer fwupd-refresh.timer update-notifier-download.timer update-notifier-motd.timer"
GUI_SERVISLER="packagekit.service packagekit-offline-update.service"

bolum "1. APT periyodik isleri kapali"
cat > /etc/apt/apt.conf.d/99-otomatik-guncelleme-kapali <<'EOF'
// Otomatik paket listesi yenileme / indirme / kurulum kapali.
// Guncelleme sadece terminalden elle yapilir. 10periodic ve 20auto-upgrades'i ezer.
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
APT::Periodic::Unattended-Upgrade "0";
EOF
apt-config dump | grep -E '^APT::Periodic::'

bolum "2. Guncelleme zamanlayicilari durdur + maskele"
systemctl disable --now $ZAMANLAYICILAR 2>&1 | grep -v '^Removed' || true
systemctl mask $ZAMANLAYICILAR

bolum "3. GUI guncelleme arka ucu (packagekit + acilista kurulum) maskele"
rm -f /system-update
systemctl stop packagekit.service 2>/dev/null
systemctl mask $GUI_SERVISLER

bolum "4. Snap otomatik yenileme suresiz beklemede"
command -v snap >/dev/null && { snap refresh --hold 2>&1; snap refresh --time 2>&1 | grep -vi warning; }

bolum "DOGRULAMA"
for u in $ZAMANLAYICILAR $GUI_SERVISLER; do printf "%-36s %s\n" "$u" "$(systemctl is-enabled "$u" 2>&1)"; done
echo "/system-update: $([ -e /system-update ] && echo VAR || echo yok)"
echo "apt elle calisiyor mu: $(apt-get check >/dev/null 2>&1 && echo evet || echo HAYIR)"
