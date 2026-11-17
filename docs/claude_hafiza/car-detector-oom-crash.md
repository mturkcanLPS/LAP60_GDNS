---
name: car-detector-oom-crash
description: 19_10 ucusunda car_detector GPU hatasi (cudaErrorIllegalAddress) -> appsrc kuyrugu sinirsiz buyudu -> OOM -> systemd TUM yigini dusurdu (havada ~100 s veri yok); dosya adi cakismasi; log adli analiz yontemi
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-10-20T08:13:56.577Z
---

2026-10-19 saha uçuşu (20_10'da analiz edildi):

- car_detector'da 4 olay (Oct 12 ×2, Oct 19 ×2) aynı desen: nvinfer `cudaErrorIllegalAddress` →
  "Buffer conversion failed" → pipeline durur. onImage yine her kareyi (3.7 MB RGBA) appsrc'ye
  iter: appsrc max-bytes/leaky ayarı yok, block=FALSE, yani kuyruk sınırsız → ~55 MB/s büyür.
  2–3 dk sonra OOM ile car_detector öldürülür (RSS 3.2 GB + swap 3.2 GB). GPU hatası koşu
  başladıktan 4–27 dk sonra rastgele geliyor; 8 dk'lık masa soak testi yakalamadı.
- systemd varsayılanı OOMPolicy=stop olduğu için BÜTÜN servis düşüyor (serial, apriltag, radar).
  Radar SDK kapanırken takıldığından TimeoutStopSec 90 s doluyor, sonra yeniden başlıyor.
  Oct 19 16:27:41'de dron ~25–30 m havadayken bu oldu: otopilota ~100 s paket gitmedi,
  radar sonrasında 1.6 dk daha geçersizdi.
- Aynı oturumun son 10 dk'sında radar sürekli reset attı (dakikada ~16), muhtemelen menzil dışı.
  O sırada IR yanlış pozitifleri valid gitti (Zopt 58 m).
- 16:10:45'te gerçek bir güç kesintisi var (kapanış kaydı yok). Kullanıcı "güç kapat aç
  yapmadık" diyor; batarya değişimi mi brownout mu, sorulacak.
- Saat geri yükleme: alan dışında NTP yok, her açılış saati timesyncd clock dosyasının
  zamanına çeker. Aynı dosya adları oluşuyor ve bugünkü açılış C'nin IR CSV'sinin üzerine yazdı.

**Why:** Uçuşta kritik olmayan bir node (car) tüm LPS yığınını öldürebiliyor.

**How to apply:** 20_10'da yapıldı (kullanıcı onayıyla):
- appsrc max-buffers=2 + leaky downstream.
- Bekçi timer: GST ERROR, 3 s takılma ya da RSS > 1500 MB olursa EO kaydını kapatır ve
  `_exit(3)` yapar.
- launch'ta car_detector `respawn=True, respawn_delay=2`.
- `copy_hw` parametresi: launch argümanı `car_copy_hw`, varsayılan 2 = VIC. Bu bir DENEME.
  Logda hata nvinfer'ın kendi dönüştürmesinde, nvvideoconvert'te değil; işe yaramazsa
  sıradaki deneme nvinfer config'te `scaling-compute-hw`.
- Test parametreleri `test_fail_after_s` / `test_fail_silent` (varsayılan kapalı).
- Drop-in `src/lps_bringup/systemd/robust.conf` (OOMPolicy=continue, TimeoutStopSec=15).
  Kullanıcı sudo ile kurar.

Masa testi sonuçları:
- Enjekte hata → 0.15 s'de exit, ~3.3 s'de yeniden kilit.
- Donma → bekçi 3.0 s'de yakaladı.
- appsrc eski ayar +1128 MB / 300 kare, yeni ayar +13 MB.

Dosya adları (20_10, kullanıcı isteğiyle):
- Format: `<önek>_sNNNN_YYYYmmdd_HHMMSS[_k].<uzantı>` (lps_infra1/2, ir, flow, eo).
- Oturum numarasını launch verir (`next_record_session`): `record_dir/.oturum_sayaci`
  dosyasında, fsync'li, klasördeki en büyük `_sNNNN` ile birleştirilir.
- Ad `O_EXCL` ile atomik sahiplenilir, yani asla üzerine yazmaz. car_detector respawn
  olursa aynı oturumda `_2` alır.
- 20_10 11:25'te ilk oturum #1 açıldı.
- Neden: 20_10 11:17'de eski kod C oturumunun lps_infra1/2_..._155845/6 dosyalarının
  üzerine yazdı. Orijinaller ucus_kayit_yedek_20261019'da.
- Kaynak: lps_serial_sender/include/lps_serial_sender/record_naming.hpp, car_detector'da
  kopyası var.

20_10 11:25–12:25 1 saatlik servis testi (VIC açık, HRVS takılı değil):
- GPU MMU hatası 0, car_detector yeniden başlaması 0, OOM 0.
- car RSS 590 → 524 MB. LPS node'larının swap'ı 0; swap'ı masaüstü uygulamaları kullanıyor.
- Hata hiç tetiklenmediği için VIC'in işe yarayıp yaramadığı belirsiz kaldı.

Sistem ölçümü (masa, masaüstü + VLC + VS Code açık):
- CPU %87 (p95 %93). LPS node'ları %447/600; masaüstü ~%72/600 (gnome-shell %37).
- Akış %85 düzenli. 19_10 uçuşunda (masaüstü yok) %99.8 → bu masaya özgü.
- GPU %31 (p95 %62); GPU 1020, VIC 435, OFA 538 MHz sabit.
- Sıcaklık max 66 °C (passive trip 70 °C). Fan ~2900 rpm.
- VDD_IN ort 13.5 W, max 13.9 W (5 sn örnekler).
- oc3 aşırı akım 49 dk'da +30 (VDD_IN crit 5032 mA @ 4.94 V ≈ 25 W; anlık tepeler).
  throt_en=1, yani her olay anlık saat kısması.

Adli analiz kaynakları:
- /var/log/syslog*: rsyslog kalıcı, servis çıktısı da burada.
- /var/log/kern.log* (OOM tablosu).
- `last -x` (açılışlar).
- Video/CSV sırası için inode kullan, dosya adına güvenme.
- Yedek: /home/lapis/ucus_kayit_yedek_20261019/

İlgili: [[car-detection-bearing]] [[ir-triangle-marker]] [[infra2-test-recording]]
