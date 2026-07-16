# LPS Felaket Kurtarma Rehberi

Jetson'un başına bir şey gelirse sıfır karttan uçuşa dönüş adımları.

## Sürüm tablosu (test edilen kombinasyon)

| Bileşen | Sürüm | Not |
|---|---|---|
| Jetson | Orin (NVMe'li devkit) | |
| JetPack | **6.2.1** (L4T **R36.4.7**) | Yeni karta MUTLAKA R36.4.x bas — imaj host sürücüleriyle bu major sürümde eşleşir |
| Isaac ROS | 3.2.x (apt, release-3 deposu) | apriltag 3.2.5, image-proc 3.2.10 |
| VPI | 3.2.4 | container içinde vpi3-dev; OFA motoru host'tan |
| İmaj | `lps-jetson:v1` | kurulu sürümler imaj içinde `/ws/VERSIONS.txt` |

## Yedekler nerede?

1. **Kaynak kod**: git remote → `<REPO_URL_BURAYA>` (ros2_ws tamamı)
2. **Docker imajı — registry**: `<REGISTRY_IMAGE_BURAYA>`
3. **Docker imajı — tar**: Google Drive + saha USB belleği (`lps-jetson-v1.tar.gz`)

İmaj tar'ı üretme (yedek tazelerken):

```bash
docker save lps-jetson:v1 | gzip > lps-jetson-v1.tar.gz
# Drive'a: tarayıcıdan yükle veya rclone copy lps-jetson-v1.tar.gz gdrive:lps/
```

## Kurtarma adımları (sıfır Jetson)

1. **Flash**: NVIDIA SDK Manager ile **JetPack 6.2.x** (L4T R36.4) bas.
   Kullanıcı adını `lapis` yap (yollar `/home/lapis/...` sabit).
2. **Kaynağı al**:
   ```bash
   git clone <REPO_URL_BURAYA> ~/ros2_ws
   ```
3. **Bootstrap** (docker, nvidia runtime, udev, imaj, otomatik başlatma):
   ```bash
   cd ~/ros2_ws/docker
   ./bootstrap.sh                    # internet varsa registry'den
   ./bootstrap.sh lps-jetson-v1.tar.gz   # yoksa USB'deki tar'dan
   ```
4. **Donanımı tak**: RealSense USB3 porta, otopilot UART `/dev/ttyTHS1`
   (pin bağlantısı değişmediyse ekstra ayar yok).
5. **Doğrulama** (uçuş öncesi zorunlu):
   ```bash
   docker compose -f docker/docker-compose.yml logs -f
   # Beklenen: RealSense stream açıldı, [CAM-WD] TUM AYARLAR AKTIF,
   #           [OPTFLOW] satırları akıyor, lps_records'a video düşüyor
   ```

## Sık durumlar

- **`permission denied ... docker.sock`** → kullanıcı docker grubunda değil;
  `sudo usermod -aG docker $USER` sonrası oturumu kapat-aç.
- **VPI/CUDA hatası container'da** → host L4T sürümü R36.4 değil; sürüm
  tablosuna dön. `cat /etc/nv_tegra_release` ile kontrol.
- **RealSense görünmüyor** → `lsusb | grep 8086` (host'ta); udev kuralları
  bootstrap ile kurulmuş mu, kamera USB3 (mavi) portta mı?
- **Kod değiştirdim, imajı tazelemek istiyorum** →
  ```bash
  docker compose -f docker/docker-compose.yml build
  docker compose -f docker/docker-compose.yml up -d
  # sonra yeni tar + registry push ile yedekleri tazele
  ```
