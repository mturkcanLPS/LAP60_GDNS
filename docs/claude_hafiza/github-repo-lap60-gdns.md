---
name: github-repo-lap60-gdns
description: "ros2_ws artık git reposu → github.com/mturkcanLPS/LAP60_GDNS (private olmalı, HRVS SDK içinde); push deploy key ile, modeller src/car_detector/models'ta"
metadata:
  node_type: memory
  type: reference
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-01T11:47:06.797Z
---

**Repo:** https://github.com/mturkcanLPS/LAP60_GDNS, dal `main`. Kullanıcı 2026-11-01'de açtı, ilk push aynı gün.
- Yerel kök `/home/lapis/ros2_ws`. `.gitignore` build/install/log, `__pycache__` ve vim swap dosyalarını dışarıda tutuyor.
- **Push yetkisi:** deploy key `~/.ssh/lap60_gdns_deploy` (yazma izinli, sadece bu repo). Repo içindeki `core.sshCommand` bu key'i kullanıyor, remote `git@github.com:mturkcanLPS/LAP60_GDNS.git`. Sistemde gh ve global git kimliği yok; kimlik repo-local olarak `mturkcanLPS` / kullanıcı e-postası.
- `src/negotiated` osrf'in reposu, alt modül (eac198b). Klonlarken `--recursive` gerekiyor.
- **HRVS SDK** `src/lps_serial_sender/third_party/hrvs_sdk` altında. Kullanıcı "repo private" diyerek onayladı. CMake'teki `HRVS_SDK_ROOT` göreli yol; Desktop/GDNS kopyası artık kullanılmıyor.
- **Modeller** `src/car_detector/models/`: `.pt`, `labels.txt`, 4 ONNX. Motorlar repoda yok. Kurulum için `install_models.sh`; DeepStream-Yolo upstream 2894bab'den klonlanıyor (biz dosyalarına dokunmadık). Export için `setup_visdrone_model.sh <pt> <boyut>`.
- systemd unit ve drop-in'lerin kopyaları `src/lps_bringup/systemd/` altında. `/etc` altındakiler kullanıcının elle kurduğu canlı dosyalar.
- Commit sonuna kullanıcının istediği Co-Authored-By satırı ekleniyor.

İlgili: [[car-detection-bearing]] [[hrvs-startup-hang]]
