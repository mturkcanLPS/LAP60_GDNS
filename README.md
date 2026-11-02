# LAP60 GDNS — Jetson LPS yazılımı

Jetson Orin Nano Super üzerinde çalışan ROS 2 Humble çalışma alanı. Sensörlerden gelen veriyi işleyip otopilota seri porttan gönderir:

- **RealSense** infra1 / infra2 kameralarda AprilTag, optik akış (VPI OFA) ve IR üçgen markerı
- **HRVS radar** ile irtifa
- **EO (renkli) kamerada araç tespiti**: DeepStream YOLOv8n VisDrone + NvDCF ile aracı kilitler, bearing'ini çıkarır
- **Sistem sağlığı**: CPU/GPU yükü, sıcaklıklar, RAM, fan, giriş gücü

Her şey tek bir systemd servisiyle (`apriltag_bringup`) açılışta başlar.

## Gereksinimler

| | Sürüm |
|---|---|
| Kart | Jetson Orin Nano Super, JetPack 6.2 (L4T R36.4) |
| CUDA / DeepStream / VPI | 12.6 / 7.1 / 3.2 |
| ROS 2 | Humble |
| ROS paketleri (apt) | `ros-humble-realsense2-camera`, `ros-humble-isaac-ros-apriltag` (+ `-interfaces`, `-image-proc`) 3.1 |
| Kütüphaneler (apt) | `libapriltag-dev` 3.2, `libopencv-dev`, `nvidia-l4t-kernel-headers` (CH340 sürücüsü için) |

Bağlantılar:

| Ne | Port |
|---|---|
| Otopilot | `/dev/ttyTHS1`, 115200 baud |
| HRVS radar | CH340 USB-seri → `/dev/ttyCH340` (udev kuralı), 921600 baud. **Stok JetPack'te sürücüsü yok**, kurulumda derlenir. |
| HRVS reset/tetik | GPIO PH.07 (gpiochip0, hat 50) |
| RealSense | USB 3 |

> **Yollar `/home/lapis`'e sabit.** Servis dosyası, launch (model ve kayıt klasörleri) ve nvinfer config bu yolu kullanıyor. Kullanıcı `lapis` olmalı ve repo `/home/lapis/ros2_ws`'e klonlanmalı.

## Kurulum (yeni Jetson)

Repo private olduğu için klonlamadan önce GitHub erişimi gerekir: SSH key ya da HTTPS + Personal Access Token.

```bash
# 1) Klonla. --recursive şart: src/negotiated alt modül.
cd /home/lapis
git clone --recursive git@github.com:mturkcanLPS/LAP60_GDNS.git ros2_ws
cd ros2_ws

# 2) Araç tespiti modelleri: DeepStream-Yolo'yu klonlar, kütüphanesini derler,
#    modelleri /home/lapis/DeepStream-Yolo/visdrone_* altına kurar.
src/car_detector/scripts/install_models.sh

# 3) Derle
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 4) CH340 sürücüsü (ch341.ko): NVIDIA kernel'inde kapalı, olmazsa radar portu oluşmaz.
#    Kernel header'larıyla derler, sudo ile kurar. Zaten kuruluysa dokunmaz.
src/lps_bringup/drivers/ch341/install_ch341.sh

# 5) udev kuralı (/dev/ttyCH340 bağlantısı)
sudo cp src/lps_bringup/udev/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger

# 6) systemd servisi + drop-in'ler
sudo cp src/lps_bringup/systemd/apriltag_bringup.service /etc/systemd/system/
sudo mkdir -p /etc/systemd/system/apriltag_bringup.service.d
sudo cp src/lps_bringup/systemd/{robust,dds_yerel,clocks}.conf \
        /etc/systemd/system/apriltag_bringup.service.d/
sudo systemctl daemon-reload
sudo systemctl enable --now apriltag_bringup.service
```

**JetPack/kernel güncellemesi:** Kernel sürümü (`uname -r`) değişirse CH340 sürücüsü yeni sürümde olmaz ve radar gelmez. Güncellemeden sonra `install_ch341.sh` tekrar çalıştırılır. 36.4.x içindeki güncellemelerde sürüm aynı kaldığı için sürücü korunur.

**İlk açılış:** Seçili modelin TensorRT motoru oluşturulurken (~6 dk) araç tespiti çalışmaz. Diğer her şey çalışır. Motor model klasörüne yazılır, sonraki açılışlarda 1 sn'de yüklenir. Motorlar TensorRT sürümüne ve GPU'ya özel olduğu için repoda yoktur.

Drop-in'lerin görevi:

| Dosya | Ne yapar |
|---|---|
| `robust.conf` | `OOMPolicy=continue`: bir node bellekten ölürse tüm servis durmaz, node respawn olur |
| `dds_yerel.conf` | `PrivateNetwork=yes`: ağ değişse de DDS paylaşımlı belleği bozulmaz, kareler kaybolmaz |
| `clocks.conf` | Her başlangıçta `set_vision_clocks.sh` ile VIC/OFA donanım saatlerini ayarlar |

## Sistem ayarları

- **Güç modu:** MAXN_SUPER (`sudo nvpmodel -m 2`).
- **Fan:** Fan dönmezse Jetson 4–14 dk içinde 104 °C termal korumayla kapanır (20_10'da yaşandı). Uçuş öncesi sağlık mesajında `fanRpm > 0` olmalı.
- **Saat:** Sahada internet yok. Node'lar saat sıçramasına dayanıklı, ama kayıt dosyası tarihleri için saat elle doğru ayarlanmalı.

## Paketler

| Paket | İçerik |
|---|---|
| `lps_serial_sender` | `serial_sender_node`: otopilot paketi, HRVS radar, AprilTag füzyonu, optik akış, IR üçgen, sağlık mesajı. `cpu_apriltag_node`: libapriltag ile CPU AprilTag (infra1 ve infra2 için iki kopya). `infra2_recorder_node`: infra2 optik akış + video kaydı. HRVS SDK `third_party/hrvs_sdk` altında. |
| `car_detector` | DeepStream YOLOv8n VisDrone + NvDCF tek araç kilidi → `/car_detection/bearing`. Modeller `models/`, nvinfer config `config/`. |
| `lps_bringup` | `bringup.launch.py` (tüm yığın), systemd ve udev dosyaları, CH340 sürücüsü (`drivers/ch341`), izleme betikleri |
| `vpi_optical_flow`, `lps_apriltag_udp`, `lps_apriltag_test_launch` | Erken dönem test araçları; bringup kullanmıyor |
| `negotiated` | osrf/negotiated, alt modül |

## İşletme

```bash
sudo systemctl restart apriltag_bringup     # yeniden başlat
journalctl -u apriltag_bringup -f           # canlı log
```

Önemli launch argümanları:

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `car_model` | `736x1280` | Araç modeli giriş boyutu: `640 \| 960 \| 736x1280 \| 1280`. Bilinmeyen değer 640'a düşer. |
| `enable_car` | `true` | Araç tespitini aç/kapa |
| `car_record` | `true` | EO debug videosu |
| `apriltag_backend` | `cpu` | `cpu` (libapriltag) veya `gpu` (Isaac ROS) |
| `serial_port` | `/dev/ttyTHS1` | Otopilot portu. Masa testinde sahte port (pty) verilir. |

Servisi durdurup elle çalıştırmak için: `ros2 launch lps_bringup bringup.launch.py car_model:=640`.

**Araç boy filtresi:** serial_sender her yeni radar karesinde irtifayı `/hrvs/altitude`'a yayınlar. car_detector kutunun uzun kenarını bu irtifayla metreye çevirir ve 2.5–9 m dışındaki adayları kilitlemeden eler (örneğin yerdeki ~1 m'lik tag levhası). Son irtifa 1.5 s'den eskiyse filtre kapanır. EO kaydında elenen kutular kırmızı çizilir ve yanlarında `L=…m` yazar. Parametreler: `size_filter_enable`, `car_len_min_m`, `car_len_max_m`, `altitude_stale_s`.

**Kayıtlar** `/home/lapis/lps_records` altına yazılır. Servis her açılışta bir oturum numarası alır (`.oturum_sayaci`). Dosya adları bu numarayı taşır (`eo_s0036_...avi`, `hrvs_s0036/`), restart olunca eski kayıtların üstüne yazılmaz.

**İzleme betikleri** (`src/lps_bringup/scripts/`, root gerekmez):

| Betik | Ne için |
|---|---|
| `servis_izle.sh ÇIKTI [SÜRE_S]` | Node RAM, kamera/akış/tag/araç hızları, radar ölçüm ve reset sayıları, car_detector olayları |
| `sistem_izle.py BİTİŞ_HH:MM:SS ÖNEK` | tegrastats, node bazında CPU, fan, aşırı akım / termal olaylar |
| `serial_packet_check.py` | Seri paketleri firmware gibi çözer; masa testinde pty açar, otopilota bir şey gitmez |
| `bench_config.sh ETİKET [arg ...]` | Yığını verilen argümanlarla sahte porta çalıştırıp ölçer (servis durmuş olmalı) |
| `measure_rates.py`, `mem_watch.py` | Hız/CPU ölçümü, bellek sızıntısı izleme |

## Seri protokol (otopilota)

Little-endian, packed. Yapılar ve `static_assert`'ler `serial_sender_node.cpp`'de.

| msg_id | Boyut | Hız | İçerik |
|---|---|---|---|
| `0x42` | 133 B | 50 Hz | hrvs, apriltag, opticflow, ir, apriltag2, opticflow2, car |
| `0x48` | 31 B | 1 Hz | Sistem sağlığı. Ana paketin hemen arkasından aynı thread'den yazılır. |
| `0x49` (otopilot → Jetson) | 6 B | — | KAPAT: `BB 49 01 FB 01 FF` gelince Jetson düzgün kapanır (`systemctl poweroff`). Kapatmak için `autopilot_poweroff_enable:=false`. |

- **Bearing ve IR alanları metre ya da açı değildir.** İkisi de boyutsuz yön birimidir (X/Z, Y/Z = tan). Değerler kamera optik frame'indedir.
- **Geçici:** `0x48`'in `cpuTemp` alanında şu an CPU yerine HRVS radarlarının en yüksek çip sıcaklığı gidiyor. Geri almak için launch'ta `serial_sender` parametrelerine `'health_cpu_temp_source': 'cpu'` eklenir.

## Modeli değiştirmek

Model yeniden eğitilirse:

```bash
src/car_detector/scripts/setup_visdrone_model.sh /yol/yeni.pt 736x1280   # boyut başına bir kez
cp /home/lapis/DeepStream-Yolo/visdrone_736x1280/test_visdrone_best.onnx \
   src/car_detector/models/visdrone_736x1280/
```

Export paketleri ilk çalıştırmada `/home/lapis/yolo_export_pkgs` altına kurulur (~1.3 GB). Sistem Python'una dokunulmaz. Script eski motoru siler, yeni motor bir sonraki açılışta oluşur.
