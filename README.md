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
| Kart | Jetson Orin Nano Super, JetPack 6.2 (L4T R36.4; bu Jetson 36.4.7) |
| CUDA / DeepStream / VPI / OpenCV | 12.6 / 7.1 / 3.2 / 4.8 (JetPack'ten) |
| ROS 2 | Humble (`ros-humble-desktop`) |
| ROS paketleri (apt) | `ros-humble-realsense2-camera` 4.57.7 (+ `ros-humble-librealsense2` 2.57.7; test edilen sürüm, bkz. Tavsiyeler), `ros-humble-isaac-ros-apriltag` (+ `-interfaces`, `-image-proc`) 3.1.0 |
| Kütüphaneler (apt) | `libapriltag-dev` 3.2, `libopencv-dev`, `nvidia-l4t-kernel-headers` (CH340 sürücüsü için), `python3-colcon-common-extensions` |

Bağlantılar:

| Ne | Port |
|---|---|
| Otopilot | `/dev/ttyTHS1`, 115200 baud |
| HRVS radar | **18_11'den beri STM32 Black Pill USB-TTL** (0483:5740, CDC ACM → `/dev/ttyACM0`, udev → `/dev/ttyBlackPill`), 921600 baud. Sürücüsü (`cdc_acm`) JetPack'te hazır. Port launch argümanı `hrvs_port`. Masa testi 40 s: 0 hata, 0 reset. Önceki adaptör CH340 (`/dev/ttyCH340`; **stok JetPack'te sürücüsü yok**, adım 4) son günlerde 2–4 dk'da bir bozuk yük → reset veriyordu. FTDI (`/dev/ttyFTDI`) ve CP2102 (`/dev/ttyCP210x`) denendi (18–19_11): aynı kabloda hat hataları oldu, radar ölçüm vermedi; udev kuralları duruyor. |
| HRVS reset/tetik | GPIO PH.07 (gpiochip0, hat 50) |
| RealSense | USB 3 |

> **Yollar `/home/lapis`'e sabit.** Servis dosyası, launch (model ve kayıt klasörleri) ve nvinfer config bu yolu kullanıyor. Kullanıcı `lapis` olmalı ve repo `/home/lapis/ros2_ws`'e klonlanmalı.

> **Geliştirme geçmişi ve devam notu:** [docs/DEVAM_NOTU.md](docs/DEVAM_NOTU.md) — Claude ile yürütülen çalışmanın sohbet kaydı, hafıza notları, analiz betikleri ve açık işler.

## Kurulum (yeni Jetson)

> Kurmadan önce **[YENI_JETSON.md](YENI_JETSON.md)**'yi oku: repo'nun kendisinin yaptığı sistem ayarları (saat yükseltme, DDS tamponu, öncelikler), eski Jetson'da elle yapılmış ayarlar, yapılmaması gerekenler ve bilinen sorunlar.

Repo private olduğu için klonlamadan önce GitHub erişimi gerekir: SSH key ya da HTTPS + Personal Access Token.

### 0) Sistem hazırlığı (JetPack kurulu, kullanıcı `lapis`, internet var)

```bash
# JetPack bileşenleri (CUDA, TensorRT, VPI, OpenCV 4.8) ve DeepStream
sudo apt update
sudo apt install -y nvidia-jetpack deepstream-7.1 git curl gnupg wget

# ROS 2 Humble apt kaynağı
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list

# Isaac ROS 3.1 apt kaynağı (sadece apriltag mesaj tipleri + GPU arka ucu için)
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key \
     | sudo gpg --dearmor -o /usr/share/keyrings/isaac-ros-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/isaac-ros-keyring.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy legacy-release-3.1" \
     | sudo tee /etc/apt/sources.list.d/isaac-ros.list

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
     ros-humble-realsense2-camera ros-humble-isaac-ros-apriltag \
     ros-humble-isaac-ros-apriltag-interfaces ros-humble-isaac-ros-image-proc \
     libapriltag-dev libopencv-dev nvidia-l4t-kernel-headers

# Güç modu MAXN_SUPER
sudo nvpmodel -m 2
```

Intel'in kendi librealsense apt deposu **gerekmez**; ROS paketi (`ros-humble-librealsense2`) yeterli.

### 1–6) Repo, derleme, sürücü, servis

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

# 4) CH340 sürücüsü (ch341.ko) — SADECE radar CH340 ile bağlanırsa (18_11'den beri Black Pill, gerekmez).
#    NVIDIA kernel'inde kapalı, olmazsa CH340 portu oluşmaz.
#    (Radar FTDI ya da CP2102 ile bağlansaydı gerekmezdi; onların sürücüleri JetPack'te hazır.)
#    Kernel header'larıyla derler, sudo ile kurar. Zaten kuruluysa dokunmaz.
src/lps_bringup/drivers/ch341/install_ch341.sh

# 5) udev kuralı (/dev/ttyBlackPill [HRVS], /dev/ttyFTDI, /dev/ttyCP210x, /dev/ttyCH340 + FTDI gecikme 1 ms)
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

### 7) Kurulum doğrulaması

```bash
# Donanımsız birim testleri (hepsi "GECTI" demeli)
for t in test_tag_fusion test_ir_triangle test_autopilot_rx test_dual_fusion; do
  ./build/lps_serial_sender/$t | tail -1; done

# Servis logu: aşağıdakiler görünmeli
journalctl -u apriltag_bringup -f
#   [HRVS] frame=... alt=...            radar ölçüyor (port var mı: ls -l /dev/ttyBlackPill)
#   [GYROFLOW] OF1 tamam ... OF2 tamam ...   optik akış gyro ile temizleniyor
#   [APRILTAG] ... Hz                   iki IR kamera geliyor
#   cpu_apriltag ... max_hamming=1 min_margin=30
#   car_detector hazir ...              (ilk açılışta ~6 dk motor derlemesinden sonra)
#   [SERIAL-RX] EKF yuksekligi ... m    otopilot 0x4A gönderiyorsa
```

**JetPack/kernel güncellemesi:** Kernel sürümü (`uname -r`) değişirse CH340 sürücüsü yeni sürümde olmaz ve radar gelmez. Güncellemeden sonra `install_ch341.sh` tekrar çalıştırılır. 36.4.x içindeki güncellemelerde sürüm aynı kaldığı için sürücü korunur.

**İlk açılış:** Seçili modelin TensorRT motoru oluşturulurken (~6 dk) araç tespiti çalışmaz. Diğer her şey çalışır. Motor model klasörüne yazılır, sonraki açılışlarda 1 sn'de yüklenir. Motorlar TensorRT sürümüne ve GPU'ya özel olduğu için repoda yoktur.

Drop-in'lerin görevi:

| Dosya | Ne yapar |
|---|---|
| `robust.conf` | `OOMPolicy=continue`: bir node bellekten ölürse tüm servis durmaz, node respawn olur |
| `dds_yerel.conf` | `PrivateNetwork=yes`: ağ değişse de DDS paylaşımlı belleği bozulmaz, kareler kaybolmaz |
| `clocks.conf` | Her başlangıçta `set_vision_clocks.sh` ile VIC/OFA donanım saatlerini ayarlar |

## Tavsiyeler (kurulumdan sonra)

Zorunlu değil ama bu Jetson'da yaşanan sorunlara göre önerilir.

**Otomatik güncellemeyi kapat.** 22_10'da GNOME'un arka planda başlattığı L4T güncellemesi yarıda kaldı ve sistem diskinin başka bir bilgisayarda onarılması gerekti. Ayrıca ESP bölümü küçük olduğu için GUI/apt yoluyla bootloader güncellemesi sessizce atlanıyor. Kapatılınca güncelleme sadece terminalden `sudo apt update && sudo apt upgrade` ile yapılır. Geri alma talimatı betiğin başında.

```bash
sudo bash src/lps_bringup/scripts/otomatik_guncelleme_kapat.sh

# Kullanıcı tarafı (masaüstü oturumunda, sudo'suz):
gsettings set org.gnome.software download-updates false
gsettings set org.gnome.software download-updates-notify false
gsettings set org.gnome.software allow-updates false
mkdir -p ~/.config/autostart
for f in update-notifier gnome-software-service; do
  [ -f /etc/xdg/autostart/$f.desktop ] && cp /etc/xdg/autostart/$f.desktop ~/.config/autostart/ \
    && echo "Hidden=true" >> ~/.config/autostart/$f.desktop
done
```

**RealSense sürümünü sabitle.** Test edilen sürüm `ros-humble-librealsense2` 2.57.7 / `ros-humble-realsense2-camera` 4.57.7. Sonraki bir `apt upgrade` bunları habersiz değiştirmesin:

```bash
sudo apt-mark hold ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
```

**Güncellemeyi GUI'den yapma.** Yazılım Güncelleyici packagekit kendini güncellerken ölebiliyor (22_10). Terminalden yap; sonra `uname -r` değiştiyse `install_ch341.sh`'yi tekrar çalıştır.

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
| `car_model` | `736x1280` | Yüksek irtifa (EO kamera) modeli: `640 \| 960 \| 736x1280 \| 1280`. Bilinmeyen değer 640'a düşer. |
| `car_dual` | `true` | İki kamera: yüksekte EO + `car_model`, alçakta IR (infra1) + `car_model_low`. `false`: sadece EO. |
| `car_model_low` | `640` | Alçak irtifa (IR) modeli |
| `car_switch_down_m` / `car_switch_up_m` | `12` / `18` | Alçalırken bu irtifanın altında IR'ye, yükselirken üstünde EO'ya geçer. Arada iki model de çalışır, geçiş ~1 kare. |
| `enable_car` | `true` | Araç tespitini aç/kapa |
| `car_record` | `true` | EO debug videosu |
| `car_record_overlay` | `false` | EO videosuna kutu/yazı çizilsin mi. Kapalıyken video temiz (eğitim verisi); kanal, irtifa, kilit, bearing ve kutular aynı adlı `eo_sNNNN_*.csv`'de, video karesiyle satır satır eşleşir. |

**Optik akış ve IMU kaydı (ayrı dosyalar):** `flow_sNNNN_*.csv` her akış karesi bir satır: `kam_t` kamera çekim anı, `all_*` filtresiz ortalama, `tex_*` dokulu hücre ortalaması (ikisi de RANSAC'sız, EMA'sız), `raw_*` RANSAC sonrası, `ema_*` EMA'lı (px/kare). `imu_sNNNN_*.csv` RealSense IMU'su 200 Hz: `stamp_s` (kamera saatiyle aynı), `gx/gy/gz` rad/s, `ax/ay/az` m/s². `imu_log:=false` IMU'yu kapatır.

**Huni ofseti:** Araç kutusunun merkezi huniyi değil aracın ortasını gösterir. car_detector gönderilen yöne `ofset / irtifa` ekler: `funnel_offset_x_m` 0, `funnel_offset_y_m` 1.45 m (kamera ekseni, y + = görüntüde aşağı = aracın arkası; 175/176 uçuşlarında araç kilidi ve tag'in birlikte olduğu 241 andan ölçüldü). 6 m üstünde tam, 3 m altında sıfır. Ofset dron başlığı = araç başlığı iken ölçüldü; 0x4A'daki başlıklarla aradaki fark kadar döndürülür (`funnel_offset_use_heading`, `heading_min_ins` 1 / `heading_max_ins` 3, `heading_stale_s`). İrtifa bayatsa ofset eklenmez. EO CSV'sinde `baslik_farki` sütunu kullanılan farkı yazar.
| `apriltag_backend` | `cpu` | `cpu` (libapriltag) veya `gpu` (Isaac ROS) |
| `serial_port` | `/dev/ttyTHS1` | Otopilot portu. Masa testinde sahte port (pty) verilir. |
| `ir_alt_source` | `auto` | IR LED arama penceresi için irtifa: `auto` = otopilot EKF (0x4A), yoksa HRVS. `tag` = AprilTag z, **sadece yer/vinç testi** (içeride radar tavanı okur). Uçuşta `auto`. |

Servisi durdurup elle çalıştırmak için: `ros2 launch lps_bringup bringup.launch.py car_model:=640`.

**İki kamerayla araç tespiti:** EO + büyük model ~11 m altında aracı kaçırıyor ve renkli kameranın görüş alanı 5 m'de aracı sığdırmıyor. Alçakta IR kamera (87°×58°) + 640 model kullanılır; kayıtlarda 3–25 m arası %95–100 buldu. Bearing her durumda **renkli kamera çerçevesinde** gönderilir: IR'den gelen RealSense kalibrasyonuyla çevrilir. Bantta iki kanal da kilit tutar, `[BANT]` log satırı ikisinin bearing farkını yazar. EO videosu o an aktif kanalı gösterir (üst bantta `[EO]` ya da `[IR]`).

**Araç boy filtresi:** İrtifa kaynağı önce otopilotun EKF yüksekliği (0x4A → serial_sender `/autopilot/ekf_altitude`'a yayınlar; LRF + baro + HRVS birleşimi, yere göre, 10 Hz), o 1.5 s gelmezse HRVS (`/hrvs/altitude`, her yeni radar karesi). Log `IRTIFA KAYNAGI X -> Y`, EO overlay'de "EKF x m" / "HRVS x m". car_detector kutunun uzun kenarını bu irtifayla metreye çevirir ve 2.5–9 m dışındaki adayları kilitlemeden eler (örneğin yerdeki ~1 m'lik tag levhası). Son irtifa 1.5 s'den eskiyse filtre kapanır. EO kaydında elenen kutular kırmızı çizilir ve yanlarında `L=…m` yazar. Kutu görüntü kenarına değiyorsa (araç kesik) alt sınır yerine kenara paralel boyutun en az 1.2 m olması aranır. Parametreler: `size_filter_enable`, `car_len_min_m`, `car_len_max_m`, `car_width_min_m`, `altitude_stale_s`.

**İrtifa kesilirse IR/EO kararı:** Uçuşta irtifa gelmişse ve sonra kesilirse (HRVS kalkışta ölüyor, EKF ~11 m'de bloklar halinde kesilebiliyor) aktif kanal `altitude_lost_keep_s` (120 s) boyunca korunur; daha uzun kesintide EO. Açılışta hiç irtifa gelmediyse `altitude_lost_s` (10 s) sonra EO.

**Kayıtlar** `/home/lapis/lps_records` altına yazılır. Servis her açılışta bir oturum numarası alır (`.oturum_sayaci`). Dosya adları bu numarayı taşır (`eo_s0036_...avi`, `hrvs_s0036/`), restart olunca eski kayıtların üstüne yazılmaz. `rx_sNNNN_*.csv`: otopilottan gelen mesajlar — her 0x4A (`E` satırı: zaman, z, gecerli) ve saniyelik özet (`S`: mesaj sayıları, bozuk çerçeve, bayt). Journal güç kesilince kaybolabildiği için RX teşhisi buradan yapılır (`rx_csv:=false` kapatır).

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
| `0x4A` (otopilot → Jetson) | 19 B | 10 Hz | OKAD V1.1.3: `BB 4A 0E ED` + `float ekfZ` [m, yere göre] + `uint8 gecerli` + `float dronBaslik` + `float yerBaslik` [derece, kuzey 0, saat yönü +] + `uint8 baslikDurum` (bit0-1 dron INS, 2-3 yer INS, 4 yer RTK taze) + checksum. Eski 5 baytlık yük de okunur. Yükseklik sadece `gecerli=1` ve > 0.1 m ise kullanılır. Başlıklar `/autopilot/heading`'e gider; car_detector huni ofsetini `dron − araç` farkı kadar döndürür (iki INS modu da 0 değilse ve yer taze ise, yoksa döndürmez). |

Checksum kuralı her yönde aynı: `headerchk = -(ilk 3 bayt) & 0xFF`, `chksum = -(yük baytları) & 0xFF`.

- **`aprtiltag` iki IR kameranın birleşimidir.** AprilTag konumu **platform merkezine göredir**: her kameranın ofseti (infra1 32.962/45 mm, infra2 −17.962/45 mm) kamera ekseninde düşülür, eksenler kamera ekseni kalır. `aprtiltag2` sadece infra2'dir (karşılaştırma için). Kapatmak için `dual_camera_fusion:=false`.
- **Optik akış (19_11, OKAD gyroflow, `gyroflow.hpp`):** `opticflow` = **sadece infra1**, `opticflow2` = **sadece infra2** (birleşim yok, `flow_fusion` kapalı). **17_11: OF2 varsayılan KAPALI** (`of2:=true` ile açılır); kapalıyken `opticflow2` alanı sıfır / quality 0, infra2 tag ve videosu devam eder. Değer: EMA'sız akış − RealSense gyro'sunun ölçtüğü dönme. Girdi `gyroflow_input`: varsayılan **`all`** (tüm hücre ortalaması; OKAD kazançları buna göre — s0209/s0210 uçuşunda kazanç 728/−854 ≈ OKAD 740/−845, telafi sonrası en temiz). `raw` (RANSAC'lı) uçuşta ölçeği ~%9 küçük ölçüyor (697/−768), masadaki "raw daha iyi" sonucu dokusuz sahneden yanıltıcıydı; raw'da RANSAC model bulamazsa kare geçersiz → durum 4. quality iki girdide de o karenin kendi kalitesi (EMA'sız); birim piksel / nominal 1/30 s kare, kamera ekseni. İki kamera da aynı D435i IMU'suyla beslenir. Gyro gelmezse ya da kare aralığı 15–100 ms dışındaysa pakete yeni timestamp ile `dx = dy = 0, quality = 0` yazılır (`gyroflow_invalid_q0`, varsayılan açık; `false` → OKAD dokümanındaki gibi paket güncellenmez, eski değer eski timestamp ile kalır). Saat 2 s'den fazla ileri/geri sıçrarsa (NTP adımı) gyro tamponu sıfırlanır (17_11). `gyroflow_enable:=false` eski EMA'lı akışa ve akış birleşimine döner. **Otopilot V1.1.3 bu formatı bilmiyor: otopilot güncellenene kadar GPS'li modda uçulmalı.** Kayıt: `flow_sNNNN.csv` (infra1) ve `flow2_sNNNN.csv` (infra2) sonunda `comp_dx_kare, comp_dy_kare, durum` (0 tamam, 1 gyro gelmedi, 2 gyro yok, 3 kare aralığı, 4 akış yok/RANSAC).
- **Bearing ve IR alanları metre ya da açı değildir.** İkisi de boyutsuz yön birimidir (X/Z, Y/Z = tan). Değerler kamera optik frame'indedir.
- **Geçici:** `0x48`'in `cpuTemp` alanında şu an CPU yerine HRVS radarlarının en yüksek çip sıcaklığı gidiyor. Geri almak için launch'ta `serial_sender` parametrelerine `'health_cpu_temp_source': 'cpu'` eklenir.

## Huni AprilTag'leri

- Aile **tag25h9**. Duvar tagleri 0 (Kuzey), 34 (Doğu), 24 (Güney), 20 (Batı), siyah kare **26 cm**; küçük tagler 10/12/14/16, **65 mm**, plakada (±4.5, ±7.0 cm).
- Ofsetler `src/lps_serial_sender/include/lps_serial_sender/tag_fusion.hpp` → `funnelTagMeta()`; kalibrasyon geçmişi aynı dosyada. **Referans nokta = küçük tag plakasının merkezi** (avgPos bunu gösterir).
- Mod `#define TEST_APRITLAG 0` (huni). `1` eski 45/24 düz tag testi; değiştirirsen launch'taki tag boyutu da değişmeli.
- Sahte tespit filtresi (`cpu_apriltag_node`): `max_hamming` 1, `min_decision_margin` 30 (launch `APRILTAG_DETECT`). Log `[TAG-FILTRE]` elenenleri yazar.
- **Geçerlilik kapısı** (`tag_gate.hpp`, serial_sender, kamera başına): tag ancak **3 ardışık** tespitte otopilota geçerli gider; doğrulanmış tag **1.0 s** içinde tekrar görülürse anında geçerli. Neden: ~10 m'de 1–2 karelik tag görüşleri otopilotu inişi askıya almaya götürüyordu (tag → araç ölçümü 1 s kapalı → tag gelmez → konum kaybı). s0133 uçuşunda bu tetik 18 → 6. Parametreler `tag_gate_enable`, `tag_min_consec`, `tag_keep_s`; log `[APRILTAG] … kapi bastirdi i1=… i2=…`.
- **Tag kaydı** (17_11, serial_sender, `tag_csv`): `tag_sNNNN.csv` — her tespit mesajında (iki kamera) idler, kapı öncesi füzyon (huni merkezi, kamera ekseni, m), kapı sonrası geçerlilik ve pakete giden konum. Eskiden tag konumu yalnız video HUD'undaydı.
- **Gyroflow τ otomatik** (18_11, `gyroflow_oto_tau.hpp`, OKAD dosyasına dokunmaz): kare–gyro zaman kayması günden güne değişti (s0209 +10, s0216 −2.5, s0217 0 ms; sabit 10 ms bugün akışı 2–3× gürültülü yaptı). 9 aday (−5…+15 ms, 2.5 ms adım), kaliteli karelerde (`raw_q ≥ 30`) telafili akışın ~300 karelik EMA'sı en küçük olan seçilir (%5 histerezis); kayıtlarda kalkışta oturuyor, havada %99–100 en iyi değerin ±2.5 ms içinde. Parametreler `gyroflow_tau_oto` (true), `gyroflow_tau_min_ms/max_ms/bas_ms`, `gyroflow_tau_kalite_min`. Log `[GYROFLOW] … tau X ms (gecis, ogrenilen)`, flow CSV `tau_ms`. Test: `test_gyroflow_oto_tau`.
- **Kamera montaj (boresight) düzeltmesi** (17_11, car_detector, sadece araç yönü): OKAD .mat (RTK hedef + tutum) karşılaştırmasında gönderilen yön sabit x +2.2° / y +1.1° fazla çıktı (20–25 m'de 1–2 m, gecikme değil). `car_boresight_x_deg` / `car_boresight_y_deg` (launch; 17_11 −2.2/−1.1, **18_11 −1.8/−1.8**: montaj günler arasında ~0.6° oynadı, yerde D435i ivme − otopilot eğimiyle de görüldü) ile ofset eklenmiş yön döndürülüyor; s0209 verisinde 20–27 m hata 1.08 → 0.26 m. **Otopilot kendi boresight'ını eklerse ikisi 0.0 yapılmalı.** Doğrulama: `python3 -s -W ignore tools/analiz/rtk_bearing.py <OKAD_…mat>`.
- **Araç yönü zaman damgası** (17_11, car_detector): appsrc'ye kendi PTS'imiz yazılıp `NvDsFrameMeta.buf_pts` ile eşleniyor → `/car_detection/bearing` damgası tespit edilen KARENİN çekim zamanı (eskiden o an en son itilen karenin damgasıydı). Log `[CAR-GECIKME] EO çekim->yayın medyan … p90 … ms | damga eşleşmeyen …`, EO CSV'de `gecikme_ms` sütunu. Masa testi: hat gecikmesi medyan ~20 ms. Paketteki `car.timeStamp` alanı değişmedi (serial_sender'a varış anı).
- **İrtifa kapısı** (17_11, `cpu_apriltag_node`, iki kamerada): tag ~10 m'den okunduğu için AprilTag **≤ 11 m'de açık, > 12 m'de kapalı** (histerezis; `apriltag_alt_gate`, `apriltag_on_m`, `apriltag_off_m`). Kaynak EKF (0x4A), bayatsa (1.5 s) HRVS; **irtifa bilinmiyorsa açık**. Kapalıyken tespit yapılmaz, boş dizi yayınlanır (otopilota tag geçersiz gider). Log `[TAG-IRTIFA]`.

## IR LED üçgeni

- 3 LED, dik açılı ama **ikizkenar değil**: R (dik açı) – sağ LED 1.445 m, R – sol LED 1.31 m. Huni merkezi hipotenüs ortası değil: R'den kısa kenar boyunca 0.655 m, uzun kenar boyunca 0.617 m (`ir_center_on_short_m`, `ir_center_on_long_m`). Kenarlar görüntüde ayırt edilemezse (oran < 1.03) hipotenüs ortasına düşer.
- LED arama penceresi irtifaya göre: beklenen alan ≈ 750·(1.75/Z)^0.6 px (1.5–4 m'de ölçüldü; 5 m üstünde alt sınır yok), büyük blob'lar yuvarlak olmalı, üçgen ölçeği irtifanın ±%25'i içinde seçilir (`ir_z_tol`).
- Pakete giden IR değeri boyutsuz yön (X/Z, Y/Z). Video HUD: `IR:OK … merkez:olcu` / `IR:RED <sebep>` / `IR:YOK`; sadece geçerli üçgen çizilir.

## Gövdeye/platforma özel ayarlar (başka drone'a taşırken kontrol et)

Kod aynı kalır ama şunlar **mekanik yerleşime bağlı**; kamera, huni ya da LED yeri değişirse yeniden ölçülmeli:

| Ayar | Nerede | Ne |
|---|---|---|
| `cam_offset_infra1` / `cam_offset_infra2` | `serial_sender_node.cpp` varsayılanı (32.962/45 ve −17.962/45 mm); launch'ta serial_sender parametresi olarak ezilebilir | Kameradan platform merkezine ofset (kamera ekseninde, m) |
| Huni tag ofsetleri | `tag_fusion.hpp` `funnelTagMeta()` | Duvar ve küçük tag konumları |
| LED üçgeni + merkez | launch: `ir_leg_a_m`, `ir_leg_b_m`, `ir_center_on_short_m`, `ir_center_on_long_m` | LED'ler arası ve merkeze mesafeler |
| Seri port / radar | launch `serial_port`, `hrvs_port`, udev kuralı | Otopilot ve radar (CH340) bağlantısı |

Kamera intrinsics'i RealSense `camera_info`'dan otomatik gelir; elle girilmez.

## Modeli değiştirmek

Model yeniden eğitilirse:

```bash
src/car_detector/scripts/setup_visdrone_model.sh /yol/yeni.pt 736x1280   # boyut başına bir kez
cp /home/lapis/DeepStream-Yolo/visdrone_736x1280/test_visdrone_best.onnx \
   src/car_detector/models/visdrone_736x1280/
```

Export paketleri ilk çalıştırmada `/home/lapis/yolo_export_pkgs` altına kurulur (~1.3 GB). Sistem Python'una dokunulmaz. Script eski motoru siler, yeni motor bir sonraki açılışta oluşur.
