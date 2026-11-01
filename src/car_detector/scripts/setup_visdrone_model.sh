#!/bin/bash
# VisDrone YOLOv8 modelini DeepStream icin hazirlar (bir kez; model degisince tekrar).
#   setup_visdrone_model.sh /yol/test_visdrone_best.pt
#
# 1) Export paketleri IZOLE klasore (pip --target): sistem Python'una/ROS'a dokunmaz.
#    (python3-venv kurulu degil; --target sudo gerektirmez. Sisteme kurmak ultralytics
#     ile numpy/setuptools'u degistirip cv2/colcon'u bozabilir — 28_09'da yasandi.)
# 2) DeepStream-Yolo/utils/export_yoloV8.py -> ONNX (+ labels.txt)
# 3) nvinfer config'i model dizinine kopyala (repo'daki kopya asil kaynak)
# 4) TensorRT motoru car_detector ILK acilisinda model dizinine yazilir (~6 dk),
#    sonra 1 sn'de yuklenir. Model degisirse model_b1_gpu0_fp16.engine'i SIL.
set -e
PT=${1:?kullanim: $0 /yol/model.pt}
DSY=/home/lapis/DeepStream-Yolo
M=$DSY/visdrone
PKG=/home/lapis/yolo_export_pkgs
HERE=$(cd "$(dirname "$0")/.." && pwd)

if [ ! -d "$PKG/torch" ]; then
    python3 -m pip install --no-user --target "$PKG" \
        --extra-index-url https://download.pytorch.org/whl/cpu \
        "torch==2.14.0+cpu" ultralytics onnx onnxslim
    python3 -m pip install --no-user --target "$PKG" --no-deps onnxscript onnx-ir
fi
mkdir -p "$M" && cp "$PT" "$M/" && cd "$M"
PYTHONPATH="$PKG" python3 "$DSY/utils/export_yoloV8.py" -w "$(basename "$PT")" -s 640 --simplify
rm -f "$M"/*.onnx.data                       # dynamo artigi; agirliklar .onnx icinde
cp "$HERE/config/config_infer_primary_yoloV8_visdrone.txt" "$M/"
rm -f "$M/model_b1_gpu0_fp16.engine"         # eski motor yeni modelle uyusmaz
echo "Hazir: $M  (motor car_detector ilk acilisinda olusur)"
