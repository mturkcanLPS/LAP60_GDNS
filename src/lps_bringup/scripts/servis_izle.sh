#!/bin/bash
# Calisan apriltag_bringup servisini izler: node RAM, kamera/akis/tag/arac hizlari,
# RADAR (basarili olcum, frame hizi, irtifa, reset/timeout) ve car_detector
# dayaniklilik olaylari (GPU MMU hatasi, respawn, OOM). Root gerekmez (syslog/adm).
#   servis_izle.sh CIKTI_DOSYASI [SURE_S=3600]
# 2026-10-20: 19_10 ucusundaki car_detector OOM + radar kopmasi sonrasi masa testleri.
OUT=$1; DUR=${2:-3600}; IV=30
SL0=$(wc -l < /var/log/syslog); KL0=$(wc -l < /var/log/kern.log); T0=$(date +%s)
echo "BASLADI $(date +%T) | dk servis car_pid car_rss | MemAvail swap | akis1 duz% tag1 img2 car | HRVS: n/30s Hz alt_min alt_max reset tmo fail | GPUhata car_olum KAPANIYOR oom servis_start" > "$OUT"
rssof() { local p; p=$(pgrep -x "$1" | head -1); [ -n "$p" ] && echo $(( $(awk '{print $2}' /proc/$p/statm) * 4 / 1024 )) || echo "-"; }
while [ $(( $(date +%s) - T0 )) -lt "$DUR" ]; do
  sleep $IV
  D=$(date -d "-$IV sec" +%H:%M:%S)
  W=$(tail -n 2500 /var/log/syslog | tr -d '\000' | awk -v d="$D" '$3>=d')
  NEW=$(tail -n +"$SL0" /var/log/syslog | tr -d '\000'); KNEW=$(tail -n +"$KL0" /var/log/kern.log | tr -d '\000')
  hz() { echo "$W" | grep -a "$1" | grep -a -o -E "$2 +[0-9.]+ Hz" | grep -o -E "[0-9.]+" | awk '{s+=$1;n++} END {if(n) printf "%.1f", s/n; else print "-"}'; }
  DUZ=$(echo "$W" | grep -a "\[OPTICFLOW\] " | grep -a -o -E "duzenli %[0-9]+" | tr -d '%' | awk '{s+=$2;n++} END {if(n) printf "%.0f", s/n; else print "-"}')
  HR=$(echo "$W" | grep -a "\[HRVS\] frame" | sed -E 's/.*frame=([0-9]+) alt=([-0-9.]+)m.*/\1 \2/')
  HRN=$(echo "$HR" | grep -c "[0-9]")
  HRS=$(echo "$HR" | awk -v iv=$IV 'NF==2 {if(n==0)f0=$1; f1=$1; if(mn==""||$2<mn)mn=$2; if(mx==""||$2>mx)mx=$2; n++} END {if(n>1) printf "%.1f %s %s", (f1-f0)/iv, mn, mx; else printf "- - -"}')
  printf "%5.1f %s %s %s | %s %s | %s %s %s %s %s | %s %s %s %s %s %s %s | %s %s %s %s %s\n" \
    "$(echo "($(date +%s)-$T0)/60" | bc -l)" "$(systemctl is-active apriltag_bringup)" "$(pgrep -x car_detector_no | tr '\n' ',')" "$(rssof car_detector_no)" \
    "$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)" "$(awk '/SwapTotal/{t=$2}/SwapFree/{f=$2}END{print int((t-f)/1024)}' /proc/meminfo)" \
    "$(hz '\[OPTICFLOW\]' 'OPTICFLOW\]')" "$DUZ" "$(hz '\[APRILTAG\] ' 'APRILTAG\]')" \
    "$(echo "$W" | grep -a '\[INFRA2\]' | grep -o -E 'img [0-9.]+' | awk '{s+=$2;n++} END {if(n) printf "%.1f", s/n; else print "-"}')" "$(hz '\[CAR\]' 'CAR\]')" \
    "$HRN" $HRS "$(echo "$W" | grep -a -c 'resetted')" "$(echo "$W" | grep -a -c 'Timeout during')" "$(echo "$W" | grep -a -c 'MEASUREMENT_OK -> MEASUREMENT_FAIL')" \
    "$(echo "$KNEW" | grep -a -c 'MMU FAULT')" "$(echo "$NEW" | grep -a -c 'car_detector.*process has died')" "$(echo "$NEW" | grep -a -c 'KAPANIYOR')" \
    "$(echo "$KNEW" | grep -a -c 'Out of memory')" "$(echo "$NEW" | grep -a -c 'Started LAPIS')" >> "$OUT"
done
{
  echo "BITTI $(date +%T)"
  echo "--- olaylar:"
  tail -n +"$SL0" /var/log/syslog | tr -d '\000' | grep -a -E "KAPANIYOR|process has died|GST ERROR|oom-kill|Started LAPIS|Stopped LAPIS|ardisik hata|hrvsConfiguration basarisiz" | cut -c1-200 | head -60
  tail -n +"$KL0" /var/log/kern.log | tr -d '\000' | grep -a -E "MMU FAULT|Out of memory|error notifier|ch341|ttyUSB" | cut -c1-200 | head -30
  echo "--- radar mesaj ozeti:"
  tail -n +"$SL0" /var/log/syslog | tr -d '\000' | grep -a "RadarSystem" | sed -E 's/^.*RadarSystem/RadarSystem/; s/[0-9]+/N/g' | sort | uniq -c | sort -rn | head -12
} >> "$OUT"
