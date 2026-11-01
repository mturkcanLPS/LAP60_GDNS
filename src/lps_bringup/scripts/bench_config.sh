#!/bin/bash
# Masa testi: yigini verilen launch argumanlariyla calistirip olcer.
# Seri cikis SAHTE porta (pty) gider — otopilota hicbir sey gitmez.
# Servis DURDURULMUS olmali (kamerayi o tutar).
#
#   bench_config.sh ETIKET [launch arg ...]
#   bench_config.sh 30fps_esit camera_fps:=30 infra2_equal_priority:=true
#
# Not: donanim saatleri (set_vision_clocks.sh) sistem geneli ayardir; servis
# durunca da gecerli kalir, ama yeniden acilista sifirlanir.
LABEL=$1; shift
DIR=$(cd "$(dirname "$0")" && pwd)
OUT=${BENCH_OUT:-/tmp/lps_bench}; mkdir -p "$OUT"
cd /home/lapis/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

if systemctl is-active --quiet apriltag_bringup; then
    echo "apriltag_bringup servisi calisiyor — once: sudo systemctl stop apriltag_bringup"; exit 1
fi

rm -f "$OUT/ptyname"
python3 "$DIR/serial_packet_check.py" --pty --ptyfile "$OUT/ptyname" --duration 110 \
    > "$OUT/serial_$LABEL.txt" 2>&1 < /dev/null & DEC=$!
for i in $(seq 25); do [ -s "$OUT/ptyname" ] && break; sleep 0.2; done
PTY=$(cat "$OUT/ptyname")

LOG="$OUT/launch_$LABEL.log"
ros2 launch lps_bringup bringup.launch.py serial_port:=$PTY "$@" > "$LOG" 2>&1 < /dev/null & L=$!
sleep 25                                   # isinma: timer'lar (6.5 s) + kamera + VPI init
grep -q "\[OPTICFLOW\]" "$LOG" || { echo "!!! yigin ayaga kalkmadi:"; tail -15 "$LOG"; }
python3 "$DIR/measure_rates.py" --log "$LOG" --duration 60 --label "$LABEL ($*)"

kill -INT $L 2>/dev/null
for i in $(seq 30); do kill -0 $L 2>/dev/null || break; sleep 0.5; done
kill -9 $L 2>/dev/null; sleep 1
# guvenlik agi: sadece bu kullanicinin yigin surecleri, comm adina gore
pkill -9 -u "$(id -un)" -x "realsense2_came|serial_sender_n|cpu_apriltag_no|infra2_recorder|car_detector_no" 2>/dev/null
wait $DEC 2>/dev/null
echo "--- seri paketler ---"; cat "$OUT/serial_$LABEL.txt"
