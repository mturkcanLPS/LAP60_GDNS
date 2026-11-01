#!/bin/bash
# Repo'daki VisDrone modellerini DeepStream-Yolo altina kurar (yeni Jetson / temiz kurulum).
#   install_models.sh
#
# 1) DeepStream-Yolo yoksa upstream'den klonla, test edilmis commit'e sabitle.
#    (Upstream dosyalarina dokunmadik; sadece custom lib derlenir.)
# 2) nvdsinfer_custom_impl_Yolo kutuphanesini derle (yoksa).
# 3) models/<dizin>/ -> $DSY/<dizin>/: ONNX + labels + nvinfer config. ONNX degistiyse
#    eski TensorRT motorunu SIL (yeni modelle uyusmaz).
# Motorlar repoda YOK (TensorRT surumune/GPU'ya ozel): car_detector ilk acilisinda model
# dizinine yazar (~6 dk). Launch motor yokken RSS bekcisini kapatir (insa ~1850 MB).
# Tekrar calistirmak guvenli: ayni ONNX'te motora dokunmaz.
set -e
DSY=${DSY:-/home/lapis/DeepStream-Yolo}      # launch CAR_MODEL_ROOT ve config custom-lib-path
DSY_URL=https://github.com/marcoslucianops/DeepStream-Yolo.git
DSY_COMMIT=2894babce8e75c49115dbe0c7b516289ed853565
CUDA_VER=${CUDA_VER:-12.6}
HERE=$(cd "$(dirname "$0")/.." && pwd)
SRC=$HERE/models

if [ ! -d "$DSY/.git" ]; then
    git clone "$DSY_URL" "$DSY"
    git -C "$DSY" checkout -q "$DSY_COMMIT"
fi
LIB=$DSY/nvdsinfer_custom_impl_Yolo/libnvdsinfer_custom_impl_Yolo.so
[ -f "$LIB" ] || make -C "$DSY/nvdsinfer_custom_impl_Yolo" CUDA_VER="$CUDA_VER" -j"$(nproc)"

for onnx in "$SRC"/*/test_visdrone_best.onnx; do
    d=$DSY/$(basename "$(dirname "$onnx")")
    mkdir -p "$d"
    if ! cmp -s "$onnx" "$d/test_visdrone_best.onnx"; then
        cp "$onnx" "$d/"
        rm -f "$d/model_b1_gpu0_fp16.engine"
    fi
    cp "$SRC/labels.txt" "$HERE/config/config_infer_primary_yoloV8_visdrone.txt" "$d/"
    echo "Hazir: $d$([ -f "$d/model_b1_gpu0_fp16.engine" ] || echo '  (motor ilk acilista olusur)')"
done
