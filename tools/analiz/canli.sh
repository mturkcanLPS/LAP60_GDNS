#!/bin/bash
# Canli huni kalibrasyonu: stack'i lapis olarak ac (seri -> pty, otopilota bir sey gitmez),
# arac tespiti kapali, AprilTag ucustaki gibi (every_n 2); tag topic'leri + camera_info + HAM IR
# goruntuleri KALICI diske bag'lenir. Kayit sirasinda bag dosyasini OKUMA (sqlite kilitlenir).
# Kullanim: canli.sh baslat | canli.sh durdur
S=/home/lapis/huni_kalib
source /opt/ros/humble/setup.bash; source /home/lapis/ros2_ws/install/setup.bash
case "$1" in
baslat)
  python3 -s -c "
import os, pty, select
m, s = pty.openpty(); p = '$S/lps_pty'
try: os.unlink(p)
except FileNotFoundError: pass
os.symlink(os.ttyname(s), p)
while True:
    r, _, _ = select.select([m], [], [], 1.0)
    if r: os.read(m, 65536)
" > $S/pty.log 2>&1 &
  echo $! > $S/pty.pid; sleep 1
  # ir_alt_source:=tag -> IR LED penceresi AprilTag yuksekligiyle (radar iceride tavani okur)
  setsid ros2 launch lps_bringup bringup.launch.py enable_car:=false serial_port:=$S/lps_pty ir_alt_source:=tag > $S/stack.log 2>&1 &
  echo $! > $S/stack.pid
  sleep 15
  # HAM=1 canli.sh baslat -> ham IR goruntuleri de (~55 MB/s, isinma!). Varsayilan: sadece tespitler.
  IMG=""; [ "$HAM" = 1 ] && IMG="/camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw"
  setsid ros2 bag record -o $S/kalib_$(date +%H%M%S) /tag_detections /tag_detections_infra2 \
      /camera/camera/infra1/camera_info /camera/camera/infra2/camera_info /hrvs/altitude \
      $IMG > $S/bag.log 2>&1 &
  echo $! > $S/bag.pid ;;
durdur)
  if [ -f $S/bag.pid ]; then
    B=$(cat $S/bag.pid); kill -INT $B 2>/dev/null
    for i in $(seq 1 20); do sleep 1; ps -p $B >/dev/null || break; done
  fi
  if [ -f $S/stack.pid ]; then
    L=$(cat $S/stack.pid); kill -INT $L 2>/dev/null
    for i in $(seq 1 15); do sleep 1; ps -p $L >/dev/null || break; done
    ps -p $L >/dev/null && kill -TERM $L 2>/dev/null && sleep 3
  fi
  for c in cpu_apriltag_no infra2_recorde serial_sender_n realsense2_came; do pkill -INT -u lapis -x $c; done; sleep 4
  for c in cpu_apriltag_no infra2_recorde serial_sender_n realsense2_came; do pkill -KILL -u lapis -x $c; done
  [ -f $S/pty.pid ] && kill $(cat $S/pty.pid) 2>/dev/null
  rm -f $S/*.pid ;;
esac
