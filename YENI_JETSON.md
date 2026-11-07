# Yeni Jetson'a kurarken dikkat edilecekler

Bu not, JetPack 6.2 kurulu yeni bir Jetson Orin Nano Super'e bu repoyu kuracak kişi ya da yapay zekâ ajanı içindir. Kurulum komutları [README.md](README.md)'de. Burada şunlar var:

- repo'nun kendisinin yaptığı sistem ayarları (saatler, tamponlar, öncelikler)
- repo dışında, eski Jetson'da elle yapılmış ayarlar
- yapılmaması gerekenler ve bilinen sorunlar

Durum: 2026-11-07, L4T 36.4.7 üzerindeki çalışan Jetson incelenerek yazıldı.

## Kısa özet

1. Kullanıcı adı `lapis`, repo `/home/lapis/ros2_ws` olmalı. Yollar buna sabit.
2. Servis **root** olarak ve **drop-in'leriyle birlikte** kurulmalı (README adım 6). Saat yükseltme, gerçek zamanlı öncelik ve radar reset pini root ister, drop-in'ler olmadan devreye girmez.
3. Güç modu MAXN_SUPER olmalı (`sudo nvpmodel -m 2`).
4. CH340 radar sürücüsü kernel sürümüne özeldir. Kernel değişirse yeniden derlenir.
5. Eski `GDNS.service`'i **kurma ve açma**. Aşağıda neden.

## Repo'nun kendisinin yaptığı sistem ayarları

Bunlar için elle bir şey yapılmaz, servis her açıldığında uygulanır. Ama servis drop-in'leriyle kurulmamışsa ya da yığın elle `lapis` kullanıcısıyla çalıştırılırsa bir kısmı uygulanmaz.

| Ne | Nerede | Neden | Root/drop-in gerekir mi |
|---|---|---|---|
| **Saat yükseltme:** `jetson_clocks` + OFA ve VIC donanımı `performance` | `scripts/set_vision_clocks.sh`, `clocks.conf` ile her servis başlangıcında | Varsayılan DVFS optik akış donanımını minimumda tutuyordu, iki kamerada karelerin %33'ü atlanıyordu. Ayarla %0.5. `jetson_clocks` bu L4T'de OFA/VIC'e dokunmadığı için ayrıca yapılıyor. Ayarlar her açılışta sıfırlandığı için her seferinde uygulanır. | Evet, ikisi de |
| **DDS paylaşımlı bellek tamponu** 32 MB | `config/fastdds_shm.xml`, launch `FASTRTPS_DEFAULT_PROFILES_FILE` ortam değişkeniyle | FastDDS'in varsayılan ~512 KB segmentine 921 KB'lik kamera karesi sığmıyor, kareler UDP'ye düşüp tampon taşmasıyla kayboluyordu (30 Hz kameradan 18–24 Hz). Profille kayıp 0. | Hayır, launch yapar |
| **Ağ izolasyonu** `PrivateNetwork=yes` | `dds_yerel.conf` | Node'lar farklı IP setleriyle açılırsa (WiFi bağlanırken) DDS paylaşımlı belleği kapatıyor, kareler kayboluyordu. Bedeli: servisin topic'leri dışarıdan `ros2 topic` ile **görülmez**. İzleme `scripts/servis_izle.sh` ile yapılır. | Drop-in |
| **Bellek dolarsa tüm servis durmasın** `OOMPolicy=continue`, `TimeoutStopSec=15` | `robust.conf` | 19_10'da araç tespitinin bellek taşması radar ve tag dahil her şeyi ~100 s düşürmüştü. Şimdi sadece ölen node yeniden açılır. | Drop-in |
| **Seri gönderim gerçek zamanlı öncelik** (SCHED_FIFO 50) | `serial_sender_node.cpp` TX thread'i | 50 Hz paket zamanlaması kaymasın diye | Root (yoksa logda "RT oncelik alinamadi" yazar, normal devam eder) |
| **Radar reset pini** PH.07, `/dev/mem` üzerinden | `serial_sender_node.cpp` | Radar açılışta takılmasın diye 500 ms reset darbesi | Root |
| **infra2 AprilTag önceliği** | launch `infra2_equal_priority` (varsayılan `true`: infra1 ile eşit; `false`: `nice 10` + 2 thread) | Ölçümle seçildi: eşit öncelikte iki kamera da 15 Hz tag | Hayır |
| **RAM bekçisi** `max_rss_mb` 1500 | launch → car_detector | Bellek sızıntısında node kendini yeniden başlatır. Motor derlenirken (ilk açılış) otomatik kapalıdır. | Hayır |
| **AprilTag her 2. karede** | launch `apriltag_every_n` 2 | Her karede CPU %92 doyuyor ve tag hızı oynuyordu; 2 ile iki kamerada sabit 15 Hz, akış 30 Hz | Hayır |

## Repo dışında, eski Jetson'da elle yapılmış ayarlar

Yeni Jetson'da bunlar **yok**, elle yapılmalı. Komutlar README'de.

| Ne | Durum (eski Jetson) | Yeni Jetson'da |
|---|---|---|
| Güç modu | `MAXN_SUPER` (mod 2) | `sudo nvpmodel -m 2` (kalıcıdır) |
| CH340 sürücüsü (`ch341.ko`) | Elle derlenip kurulmuş | `src/lps_bringup/drivers/ch341/install_ch341.sh`. **Kernel değişince (`uname -r`) tekrar.** |
| udev kuralı `/dev/ttyCH340` | `/etc/udev/rules.d/99-usb-serial.rules` | README adım 5 |
| Kullanıcı grupları | `lapis`: `dialout video gpio i2c render plugdev` | Servis root olduğu için şart değil. Yığını elle `lapis` ile çalıştıracaksan: `sudo usermod -aG dialout,video,gpio,i2c,render,plugdev lapis` |
| RealSense sürümü sabit | `apt-mark hold` (2.57.7 / 4.57.7) | Tavsiye, README "Tavsiyeler" |
| Otomatik güncelleme kapalı | packagekit, apt-daily, fwupd maskeli | Tavsiye: `scripts/otomatik_guncelleme_kapat.sh` |
| jtop (jetson-stats) | Kurulu, servis açık | İsteğe bağlı. İzleme betikleri `tegrastats` kullanır, jtop'a gerek yok. |

## Bilerek değiştirilmemiş varsayılanlar

Eski Jetson'da bunlar **JetPack varsayılanı**. Yeni kurulumda da dokunmaya gerek yok. Bir ajan "optimizasyon" diye değiştirmesin, sorun çıkarsa önce burayı oku.

- **sysctl:** Hiçbir ağ/bellek ayarı değiştirilmedi. DDS tampon sorunu sysctl ile değil paylaşımlı bellek profiliyle çözüldü.
- **`usbcore.usbfs_memory_mb`:** 16 (varsayılan). RealSense bununla iki IR + renkli akışta çalışıyor.
- **Kernel boot parametreleri** (`/boot/extlinux/extlinux.conf`): Varsayılan.
- **Swap:** JetPack'in zram'ı (6 × 635 MB). Disk swap yok.
- **journald:** Varsayılan.
- **Fan:** `nvfancontrol` varsayılan (`quiet`, kapalı döngü).
- **Seri konsollar:** `ttyTCU0` ve `ttyGS0`'da getty var. Otopilot portu `ttyTHS1` boş; ona getty bağlanmamalı.

## Yapılmaması gerekenler

- **`GDNS.service`'i kurma ve açma.** Eski Jetson'da `/etc/systemd/system/GDNS.service` var (devre dışı). HRVS SDK'nın örnek programını (`/home/lapis/Desktop/GDNS/.../example.out`) doğrudan çalıştıran eski sistem. Açılırsa radar portunu bizim servisle aynı anda kullanır, ikisi de bozulur. Yeni Jetson'a taşınmamalı.
- **Servis çalışırken yığının ikinci kopyasını açma.** Radar UART'ı çakışır. Elle test için önce `sudo systemctl stop apriltag_bringup`. Radarsız masa testinde `hrvs_enabled:=false`, otopilota bir şey gitmesin diye `serial_port:=<pty>`.
- **Sistem Python'una ya da `~/.local`'a pip ile paket kurma.**
  - Eski Jetson'da `~/.local`'a gelen numpy 2.2.6, apt'nin OpenCV'sini `lapis` kullanıcısı için bozuyor.
  - setuptools 78 ise bir keresinde colcon'u bozdu.
  - Python betiklerini `python3 -s` ile çalıştır (`~/.local`'ı yok sayar).
  - Model export paketleri `/home/lapis/yolo_export_pkgs` altına `--target` ile kurulur (`setup_visdrone_model.sh` kendisi yapar).
- **Güncellemeyi GUI'den (Yazılım Güncelleyici) yapma.** 22_10'da packagekit kendini güncellerken öldü, sistem diski onarıldı. Terminalden `sudo apt upgrade` kullan, sonra `uname -r` değiştiyse CH340 sürücüsünü yeniden kur.
- **Intel'in librealsense apt deposunu ekleme.** Gerekmez (ROS paketi yeterli). Eski Jetson'da ekli ve `apt update`'te anahtar uyarısı veriyor.
- **TensorRT motorlarını (`*.engine`) başka Jetson'dan kopyalama.** TensorRT sürümüne ve GPU'ya özeldir. İlk açılışta ~6 dk'da kendisi derler.

## Bilinen sorunlar (yeni Jetson'da da olabilir)

- **RealSense USB kopması → bellek dolması → servis yeniden başlar.** USB hatası (`-71`) kamerayı düşürüyor, librealsense saniyede çok sayıda hata yazıyor, `ros2 launch` bu çıktıyı belleğe biriktirip ~5 GB'a çıkıyor ve OOM oluyor (03_11 ve 05_11'de görüldü). Yazılım düzeltmesi henüz yok. Önlem: RealSense'i **doğrudan USB 3 portuna**, kısa ve sağlam kabloyla tak; hub kullanma.
- **Fan bazı açılışlarda dönmüyor** (tachometer overflow; konnektör şüphesi). Fan dönmezse Jetson 4–14 dk'da 104 °C termal korumayla kapanır. Uçuş öncesi sağlık mesajında `fanRpm > 0` olmalı. Kapalı kutu/kapak hava akışını keser; 05_11'de kapaklı testte 104 °C kapanma oldu.
- **Saat:** Sahada internet yok, RTC senkronu yok. Node'lar saat sıçramasına dayanıklı ama kayıt dosyası tarihleri için saat elle ayarlanmalı.
- **Kayıt klasörü büyür:** `/home/lapis/lps_records` (video, CSV). Düzenli temizlenmeli.
- **Linker uyarısı:** Derlemede `libopencv_core.so.4.5d … may conflict with libopencv_core.so.408` uyarısı çıkar (ROS cv_bridge 4.5, JetPack 4.8). Bilinen, zararsız.

## Gövdeye özel ayarlar

Kamera, huni ya da IR LED yerleşimi eski drone'dan farklıysa kamera→platform ofseti, huni tag ofsetleri ve LED ölçüleri yeniden ölçülmeli. Liste README'de "Gövdeye/platforma özel ayarlar" bölümünde.

## Kurulum sonrası kontrol listesi

```bash
nvpmodel -q | head -1                                   # NV Power Mode: MAXN_SUPER
ls -l /dev/ttyCH340                                     # radar portu var (CH340 + udev)
systemctl is-enabled apriltag_bringup                   # enabled
ls /etc/systemd/system/apriltag_bringup.service.d/      # clocks.conf dds_yerel.conf robust.conf
cat /sys/class/devfreq/15a50000.ofa/governor            # performance (servis açıldıktan sonra)
cat /sys/class/devfreq/15340000.vic/governor            # performance
systemctl is-enabled GDNS.service 2>&1                  # "not-found" ya da "disabled" olmalı
journalctl -u apriltag_bringup -b | grep -m1 "RT oncelik alinamadi"   # BOŞ olmalı (root ise)
for t in test_tag_fusion test_ir_triangle test_autopilot_rx test_dual_fusion; do
  ~/ros2_ws/build/lps_serial_sender/$t | tail -1; done  # hepsi GECTI
```

Servis logunda görülmesi gerekenler README adım 7'de.
