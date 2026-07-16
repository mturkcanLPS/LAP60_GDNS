# LPS iniş destek sistemi — Jetson Orin (L4T R36.4 / JetPack 6.2.x) imajı
#
# Host gereksinimi: JetPack 6.2.x (L4T R36.4) + nvidia-container-runtime.
# Container, GPU/VPI sürücü katmanını host'tan alır; bu yüzden base'deki
# kütüphaneler (vpi3, cuda) host'un L4T major sürümüyle (R36) eşleşmeli.
# Yeni Jetson'a farklı major L4T basılırsa bu imaj VPI tarafında çalışmaz —
# KURTARMA.md'deki sürüm tablosuna bak.
#
# Build (ws kökünden):  docker compose -f docker/docker-compose.yml build

FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# --- NVIDIA apt depoları (host'takiyle birebir aynı) ---------------------
# jetson/common r36.4 : cuda, vpi3, tensorrt (Isaac ROS bağımlılıkları)
# isaac-ros/release-3 : ros-humble-isaac-ros-* paketleri
COPY docker/jetson-ota-public.asc /etc/apt/trusted.gpg.d/
RUN echo "deb https://repo.download.nvidia.com/jetson/common r36.4 main" \
      > /etc/apt/sources.list.d/nvidia-jetson.list \
 && echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0" \
      > /etc/apt/sources.list.d/isaac-ros.list

# --- Çalışma zamanı + derleme bağımlılıkları ------------------------------
# Sürümler bilerek pinsiz: release-3 deposu 3.2.x serisinde sabit; build
# sonunda kurulan sürümler /ws/VERSIONS.txt'ye yazılır (denetim izi).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-isaac-ros-apriltag \
      ros-humble-isaac-ros-image-proc \
      ros-humble-realsense2-camera \
      ros-humble-cv-bridge \
      vpi3-dev \
      libopencv-dev \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# --- Workspace derlemesi --------------------------------------------------
COPY src /ws/src
RUN . /opt/ros/humble/setup.sh \
 && cd /ws \
 && colcon build --event-handlers console_direct+ \
 && rm -rf /ws/build /ws/log \
 && dpkg -l | grep -E "isaac-ros|realsense|vpi|opencv" > /ws/VERSIONS.txt

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "lps_bringup", "bringup.launch.py"]
