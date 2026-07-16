#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# Kayıt dizini: normalde compose host'tan mount eder; mount unutulursa
# container içinde oluştur ki launch (camwd.log, video kaydı) patlamasın.
mkdir -p /home/lapis/lps_records

exec "$@"
