#!/bin/bash
# Ucus icin sistem ayarlari (17_11). sudo ile calistir:
#   sudo ./ucus_sistem_ayar.sh          -> ayarlari uygula (bir kez yeterli, tekrar calistirmak zararsiz)
#   sudo ./ucus_sistem_ayar.sh saat     -> masada, internet varken saati BIR KEZ esitle (servisi once durdur)
#   sudo ./ucus_sistem_ayar.sh geri     -> tum ayarlari geri al
#
# 1) KALICI JOURNAL: eskiden journal RAM'deydi (/var/log/journal yok), guc kesilince ucusun tum
#    sistem logu kayboluyordu (s0209/s0210). Simdi diske, 10 s'de bir senkron (guc kesilirse en
#    fazla ~10 s kayip), en fazla 1 GB (~30 MB/saat -> 30+ saat).
# 2) NTP KAPALI: s0209'da Jetson havada internete baglaninca NTP saati +2.5 saat atladi, RealSense
#    zamani yetisene kadar optik akis 18 s kesildi. timesyncd CALISMAYA DEVAM EDER (acilista saati
#    son kayittan geri yukler; Jetson'un RTC'si zamani tutmuyor), sadece NTP sunuculari bos.
#    Saat dosyasi dakikada bir guncellenir -> guc kesilince saat en fazla ~1 dk + kapali kalinan
#    sure kadar geride baslar. Gercek saat gerekiyorsa: 'saat' secenegi.
# 3) Bluetooth debug logu kapali (journal'in ~%25'i bluetoothd -d ciktisiydi).
set -e
[ "$(id -u)" = 0 ] || { echo "sudo ile calistir"; exit 1; }

JCONF=/etc/systemd/journald.conf.d/lps-ucus.conf
# zz- oneki SART: NVIDIA'nin nv-fallback-ntp.conf'u alfabetik olarak sonra okunursa FallbackNTP'yi
# geri koyuyor (18_11: ilk surumde lps-ntp-kapali.conf -> NTP yine calisti, saat 10 saat atladi).
TCONF=/etc/systemd/timesyncd.conf.d/zz-lps-ntp-kapali.conf
TCONF_ESKI=/etc/systemd/timesyncd.conf.d/lps-ntp-kapali.conf
BCONF=/etc/systemd/system/bluetooth.service.d/lps-loglevel.conf
SAAT=/var/lib/systemd/timesync/clock

uygula() {
    mkdir -p /var/log/journal /etc/systemd/journald.conf.d
    systemd-tmpfiles --create --prefix /var/log/journal
    cat > "$JCONF" <<'EOF'
[Journal]
Storage=persistent
SyncIntervalSec=10s
SystemMaxUse=1G
SystemKeepFree=5G
EOF
    systemctl restart systemd-journald
    journalctl --flush || true

    mkdir -p /etc/systemd/timesyncd.conf.d
    rm -f "$TCONF_ESKI"
    cat > "$TCONF" <<'EOF'
[Time]
NTP=
FallbackNTP=
EOF
    timedatectl set-ntp true          # timesyncd acik kalsin (saat geri yukleme), sunucu yok
    systemctl restart systemd-timesyncd

    # saat dosyasini dakikada bir guncelle (timesyncd acilista bu dosyanin zamanina doner)
    cat > /etc/systemd/system/lps-saat-kaydet.service <<EOF
[Unit]
Description=LPS: saat dosyasini guncelle (RTC yok, guc kesintisine karsi)
[Service]
Type=oneshot
ExecStart=/usr/bin/touch $SAAT
EOF
    cat > /etc/systemd/system/lps-saat-kaydet.timer <<'EOF'
[Unit]
Description=LPS: saat dosyasini dakikada bir guncelle
[Timer]
OnBootSec=1min
OnUnitActiveSec=1min
[Install]
WantedBy=timers.target
EOF
    mkdir -p /etc/systemd/system/bluetooth.service.d
    cat > "$BCONF" <<'EOF'
[Service]
LogLevelMax=info
EOF
    systemctl daemon-reload
    systemctl enable --now lps-saat-kaydet.timer
    systemctl try-restart bluetooth || true
    echo "Tamam: kalici journal (/var/log/journal, 1 GB), NTP kapali, saat dosyasi dakikada bir, bluetooth debug kapali."
    timedatectl | grep -E "Local time|synchronized|NTP service"
    echo "NTP sunucusu (bos olmali):"; systemd-analyze cat-config systemd/timesyncd.conf 2>/dev/null | grep -E "^(NTP|FallbackNTP)=" | tail -2
}

saat() {
    if systemctl is-active --quiet apriltag_bringup; then
        echo "UYARI: apriltag_bringup calisiyor; saat atlarsa akis ~20 s kesilir. Once: sudo systemctl stop apriltag_bringup"
        read -r -p "Yine de devam? [e/H] " c; [ "$c" = e ] || exit 1
    fi
    rm -f "$TCONF"; systemctl restart systemd-timesyncd
    for i in $(seq 1 30); do
        timedatectl show -p NTPSynchronized --value | grep -q yes && break; sleep 1
    done
    timedatectl | grep -E "Local time|synchronized"
    mkdir -p /etc/systemd/timesyncd.conf.d
    printf '[Time]\nNTP=\nFallbackNTP=\n' > "$TCONF"; systemctl restart systemd-timesyncd
    touch "$SAAT"
    echo "NTP tekrar kapali."
}

geri() {
    rm -f "$JCONF" "$TCONF" "$TCONF_ESKI" "$BCONF" /etc/systemd/system/lps-saat-kaydet.{service,timer}
    systemctl daemon-reload
    systemctl restart systemd-journald systemd-timesyncd
    systemctl try-restart bluetooth || true
    echo "Geri alindi (/var/log/journal klasoru duruyor; silersen journal tekrar RAM'e doner)."
}

case "${1:-}" in
    "") uygula ;;
    saat) saat ;;
    geri) geri ;;
    *) echo "kullanim: sudo $0 [saat|geri]"; exit 1 ;;
esac
