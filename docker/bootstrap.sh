#!/usr/bin/env bash
# Sıfır Jetson'u LPS uçuş sistemine hazırlar.
#
# Ön koşul: JetPack 6.2.x (L4T R36.4) flash'lanmış olmalı — sürüm farklıysa
# script uyarır. Ayrıntı: KURTARMA.md
#
# Kullanım:
#   ./bootstrap.sh                    # imajı registry'den çeker (REGISTRY_IMAGE)
#   ./bootstrap.sh lps-jetson-v1.tar.gz   # imajı tar yedeğinden yükler (offline)
set -euo pipefail

# Registry adresi netleşince doldurulacak (örn. docker.io/KULLANICI/lps-jetson:v1)
REGISTRY_IMAGE="ghcr.io/mrctrkcn/lps-jetson:v1"
IMAGE_TAG="lps-jetson:v1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() { echo -e "\n[BOOTSTRAP] $*"; }

# --- 1) L4T sürüm kontrolü -------------------------------------------------
if grep -q "R36" /etc/nv_tegra_release 2>/dev/null; then
    log "L4T R36 dogrulandi: $(head -c 60 /etc/nv_tegra_release)"
else
    log "UYARI: Bu Jetson L4T R36 degil! Imaj VPI/CUDA tarafinda calismayabilir."
    log "Mevcut: $(cat /etc/nv_tegra_release 2>/dev/null || echo 'okunamadi')"
    read -rp "Yine de devam? [e/H] " ans; [[ "${ans,,}" == "e" ]] || exit 1
fi

# --- 2) Docker + NVIDIA runtime ---------------------------------------------
if ! command -v docker >/dev/null; then
    log "Docker kuruluyor..."
    sudo apt-get update && sudo apt-get install -y docker.io docker-compose-v2
fi
if ! command -v nvidia-container-runtime >/dev/null; then
    log "nvidia-container-toolkit kuruluyor..."
    sudo apt-get install -y nvidia-container-toolkit
fi
# --set-as-default sart: VPI'li imaj Jetson'da YENIDEN derlenecekse build
# adimlarinin L4T surucu kutuphanelerini gormesi gerekir (runc'de linker
# NvRm/NvMedia sembollerini bulamaz).
if ! grep -q '"default-runtime": "nvidia"' /etc/docker/daemon.json 2>/dev/null; then
    log "Docker'a nvidia runtime tanitiliyor (default)..."
    sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
    sudo systemctl restart docker
fi
sudo systemctl enable --now docker
if ! groups "$USER" | grep -qw docker; then
    log "$USER docker grubuna ekleniyor (oturumu kapat-ac gerekebilir)..."
    sudo usermod -aG docker "$USER"
fi

# --- 3) RealSense udev kurallari (host tarafi sart) --------------------------
log "RealSense udev kurallari kuruluyor..."
sudo cp "$SCRIPT_DIR/60-librealsense2-udev-rules.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# --- 4) Kalici dizinler -------------------------------------------------------
mkdir -p /home/lapis/lps_records

# --- 5) Imaji getir -----------------------------------------------------------
if [[ $# -ge 1 && -f "$1" ]]; then
    log "Imaj tar yedeginden yukleniyor: $1"
    sudo docker load -i "$1"
elif [[ -n "$REGISTRY_IMAGE" ]]; then
    log "Imaj registry'den cekiliyor: $REGISTRY_IMAGE"
    sudo docker pull "$REGISTRY_IMAGE"
    sudo docker tag "$REGISTRY_IMAGE" "$IMAGE_TAG"
else
    log "UYARI: Ne tar dosyasi verildi ne REGISTRY_IMAGE dolu."
    log "Imaji elle yukle (docker load/pull) ya da yerinde derle:"
    log "  docker compose -f docker/docker-compose.yml build"
fi

# --- 6) Baslat ----------------------------------------------------------------
log "Sistem baslatiliyor (boot'ta otomatik kalkacak)..."
sudo docker compose -f "$SCRIPT_DIR/docker-compose.yml" up -d
log "TAMAM. Kontrol: docker compose -f docker/docker-compose.yml logs -f"
log "Ucus oncesi: camwd.log'da '[CAM-WD] TUM AYARLAR AKTIF' satirini bekle!"
