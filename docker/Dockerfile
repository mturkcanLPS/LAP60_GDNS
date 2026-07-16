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
COPY docker/jetson-ota-public.asc docker/isaac-ros-key.asc /etc/apt/trusted.gpg.d/
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
      libopencv \
      libopencv-dev \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Isaac ROS buildfarm'ın iki paketleme tuhaflığına birebir host çözümü:
#  - libopencv (4.8 runtime): isaac deposunun libopencv-dev'i cmake'te
#    /usr/lib/libopencv_core.so.4.8.0'ı referanslar ama runtime'ı depends
#    olarak çekmez — açıkça kurulmalı (yukarıda).
#  - isaac_ros_apriltag_interfaces cmake hedefi buildfarm'daki pip-numpy
#    yolunu (/usr/local/.../numpy/core/include) arar; numpy 2.x'te core
#    dizini _core oldu, o yüzden <2 pinli.
# --ignore-installed: apt'nin numpy'si "zaten kurulu" saydırıp pip'i
# atlatıyor; /usr/local kopyasının gerçekten oluşması için şart.
RUN python3 -m pip install --no-cache-dir --ignore-installed "numpy<2"

# --- Workspace derlemesi --------------------------------------------------
COPY src /ws/src
RUN . /opt/ros/humble/setup.sh \
 && cd /ws \
# --allow-shlib-undefined: libnvvpi'nin bagimli oldugu L4T surucu
# kutuphaneleri (NvRm/NvMedia/Pva) imajda yok — onlari calisma zamaninda
# nvidia-container-runtime host'tan mount eder. BuildKit build adimlari
# nvidia runtime kullanmadigi icin linker'a bu sembolleri runtime'a
# birakmasini soyluyoruz; kendi kodumuzun eksik sembolleri yine hata verir.
 && colcon build --event-handlers console_direct+ \
      --cmake-args -DCMAKE_EXE_LINKER_FLAGS=-Wl,--allow-shlib-undefined \
 && rm -rf /ws/build /ws/log \
 && dpkg -l | grep -E "isaac-ros|realsense|vpi|opencv" > /ws/VERSIONS.txt

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "lps_bringup", "bringup.launch.py"]
