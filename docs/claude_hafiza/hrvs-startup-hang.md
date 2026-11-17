---
name: hrvs-startup-hang
description: Servis yeniden başlayınca HRVS radar konfigürasyonu (hrvsConfiguration) zaman aşımı olmadan sonsuza dek takılabiliyordu; GPIO PH.07 reset darbesi çözüyor (20_10)
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-02T12:36:56.682Z
---

**Olay (2026-10-20, oturum #14/#15):** Servis restart sonrası SDK `INTERFACES_CONFIGURED →
CONFIGURATIONS_SET`'ten sonra hiç ilerlemedi; ne hata ne zaman aşımı verdi.
- gdb (`sudo gdb -p PID -batch -ex "thread apply all bt 12"`): `hrvsThread →
  hrvsConfiguration → receiveEapUartMsg → ThreadStdlib::sleep` döngüsü.
- Elle reset sonrası (servis dur, `gpioset --mode=time --usec=500000 gpiochip0 50=1`, sonra
  50=0, servis başlat) radar 1 s'de kalktı ve 21+ dk sorunsuz ölçtü.
- Kod sebep değildi: aynı binary #13'te çalışmıştı.

**Kök:** serial_sender çıkarken radar ölçüm modunda kalıyor. SDK'nın `closeRadarSystem()`'i
yalnız log boşaltıp kaynak bırakıyor, radara komut göndermiyor. Reset'siz yeni
konfigürasyon bazen hiç cevap almıyor.

**PH.07 (gpiochip0 hat 50):** SDK'nın tek GPIO'su. Kısa darbe = ölçüm tetiği; 500 ms '1' =
HRVS güç kesip açma (`RESET_ICB_DELAY`, `reset()`). Hat tutulurken (servis açıkken) başka
süreç alamaz.

**Düzeltme (serial_sender, 20_10):**
1. Her açılışta konfigürasyondan önce reset darbesi (500 ms) + 2 s bekleme. Başarısızsa
   reset+konfigürasyon tekrar; eskiden ilk hatada thread `return` ediyordu.
2. SDK zaman kaynağı `TimeSteady` (steady_clock). SDK'nın `TimeStdlib`'i system_clock
   kullanıyordu; saat geri atlarsa 1 s'lik zaman aşımları dakikalarca dolmaz.
3. Bekçi (`hrvsWatchdogTick`, 1 Hz health timer'ı): SDK 20 s ilerlemezse iki tick'lik reset
   darbesi, en fazla 30 s'de bir.
4. Çıkışta `closeRadarSystem()`.

Yedek: `ros2_ws_yedek/serial_sender_node.pre_hrvsfix.cpp`.

**Yan bulgu (kök neden 03_11'de bulundu):** servis stop'u hep 15 s'de SIGKILL ile bitiyordu.
- Sebep: projektörü kapatan `ros2 param set` arkasında kalıcı bir ros2 daemon bırakıyor
  (python3, `-c from ros2cli.daemon.daemonize`). Daemon SIGTERM'i yutuyor.
- Düzeltme: launch'ta `--no-daemon --spin-time 3` (commit ff73be5).
- Arama tuzağı: daemon'ın args'ında `_ros2_daemon` geçmiyor. `pgrep -f` kendi kabuğunu da
  öldürür; `ros2cli.daemon.daemonize` ile ara.

**How to apply:** Doğrulama, servisi art arda birkaç kez restart edip her oturumda "radar
sistemi hazir" görmekle yapılır.

İlgili: [[car-detector-oom-crash]] [[jetson-health-thermal]]

**HRVS SDK debug logu (20_10, kullanıcı isteğiyle VARSAYILAN AÇIK)**
- Param `hrvs_debug_log`. `RadarSystem(true, <record_dir>/hrvs_sNNNN/)` her açılışta
  `hrvsSdkLogs_N/` klasörünü oluşturuyor; içinde `min_range.csv`, `eap.csv`,
  `object_info.csv`, `control_msg.csv`, `config_*.csv` ve `hrvsSdkLogs.txt` (TRACE) var.
- `min_range.csv` (21 sütun) her ölçümde her radar için çip sıcaklıkları: sütun 12–14 RX0–2,
  15–17 TX0–2, 18 PM, 19–20 DIG0–1, °C.
- İlk değerler RX 52–61, TX 55–64, DIG 56–59 °C. PM ~1690 sabit, °C değil, birimi
  bilinmiyor (gerilim olabilir).
- SDK'da sıcaklık getter'ı ve besleme gerilimi alanı yok.
- Serial_sender 10 s'de bir `[HRVS-SICAKLIK] r1 RX.. TX.. DIG.. PM.. | ... | max N C`
  satırı basıyor (oturum #28'den itibaren).
- Maliyet: dosyalar ~80 MB/saat. TRACE satırları syslog'a da akıyor (~98 satır/s;
  launch + journald + rsyslog toplam ~%6 tek çekirdek). İstenirse susturulabilir.
- Amaç: ~25 dk sonra başlayan UART hatalarının radar ısınmasıyla ilişkisini görmek
  (kullanıcı besleme tarafına da bakacak).
- **TRACE konsol susturma KALDIRILDI (22_10).**
  - Ne yapılmıştı: 20_10'da `el::Loggers::reconfigureAllLoggers(Trace, ToStandardOutput, "false")`
    eklenmişti.
  - Sorun: SDK'nın AsyncLogWriter thread'i yazarken logger'ı yeniden kurmak bir yarış
    yaratıyor ve **SIGSEGV** veriyor. Oturum #31–#35'te serial_sender açılışta 1–51 kez
    çöktü. #35'te ~2.5 dk boyunca otopilota paket gitmedi; respawn kurtardı.
  - Kanıt: aynı sırayla yazılmış test programı, ayarla 100 çalıştırmada 98 segfault, ayarsız 0.
  - Ders: 20_10'daki tek çalıştırmalık test yarışı yakalamadı. Thread'li kodu döngüyle
    (100×) test et.
  - Kural: **easylogging ayarına DOKUNMA.** TRACE syslog'u şişirirse rsyslog filtresiyle kes.

**persistMode bulgusu:** `setConfigurations(proc, radar, true /*autoReset*/, true /*persist*/)`.
- persist=true olunca `runMeasurement` ve `hrvsConfiguration` başarılı olana kadar SDK içinde
  dönüyor, hata bize hiç gelmiyor.
- Kanıt: syslog'da SDK "errors in row" 800, "resetted" 2781 kez; bizim "ardisik hata"
  yolumuz 0 kez çalışmış (ölü kod).
- Radar kesintisinde thread SDK içinde bekliyor, `hrvs_data_` donuk kalıyor. Bekçi 20 s
  kesintide ek reset atar; SDK zaten kendi reset'ini yapıyor.
- Değiştirilmedi.

**Uçuş #33 (21_10) radar bulgusu: veri hacmi.**
- 13 kez ~4.4 s veri boşluğu oldu (SDK reset döngüsü); hepsi vanın üstünde, 7–19 m'de.
- O anlarda radar başına nokta/kare ~1900–2700. Yerde ~280, 30–60 m'de 500–700.
- Hata tipi: RX yönünde "payload corrupted" (FILTERED POINT, OBJECT INFO) ve radar 3/4'ten
  0 bayt. TX "eksik bayt" hatası 0.
- Teşhis: 921600 baud + CH340 (32 B FIFO) yoğun veride bayt kaçırıyor.
- SDK'da nokta/nesne mesajlarını kapatma ayarı yok. Range filtresi sadece protokol 1.1'de,
  biz 1.0'ız.
- Öneri: FTDI/CP2102N ya da Jetson'ın yerel UART'ı.
- Radar sıcaklığı uçuşta 51–53 °C (yerde 63); bu uçuşta ısınma sebep değil.

**Uçuş #42 (2026-11-01) radar:**
- 3 kesinti: 1.8 s, 6.8 s, 6.8 s. Hepsi kalkıştan sonraki 2 dk içinde, 7–10 m'de.
- İmza #33 ile aynı: payload bozuk → 4/4 MINRANGE'de 0 bayt → 3/3 → SDK reset.
- Yerde 10 dk'da 5 tek karelik hata (~70 nokta/radar), 0.3 s'de toparlandı.
- **Veri hacmi tek açıklama değil:** inişte 4 m'de 3500 nokta/radar varken hata 0.
- Kare hızı yoğunlukla düşüyor: yerde 4.9 Hz, 60 m'de 3.3 Hz, 4 m'de ~1.4 Hz.
- Radar sıcaklığı 47–67 °C, kesintilerle ilişkisiz.
- Analiz yöntemi: SDK satırları syslog'da (TRACE hariç); `object_info.csv` pointsPerObj
  toplamı → kare/radar başına nokta; systemFrameNumber → `[HRVS] frame=` log zamanı.

**02_11 saha (#47–#53): baskın hata TX yönünde (Jetson → radar)**
- Mesaj: "radar received Control message with invalid number of bytes (18 expected,
  0/2/4/17 received)" ve "corrupted Control message". Neredeyse hepsi radar 1/4.
  #33/#42'deki RX bozulmasından farklı; 20_10'daki "~25 dk sonra TX bayt kaybı" ile aynı.
- #47 (yerde 45 dk): ilk 10 dk 0 hata, sonra 5 dk başına 11 → 35 → 109 → 170.
  Toplam 90 SDK reset ve 457 s kesinti.
- Jetson tj (~76 °C) sabitken artmaya devam etti. Radar çip sıcaklıkları
  (min_range.csv) aynı sürede düştü (60–64 °C → 52–54 °C). Yani sıcaklık sebep değil.
- **DÜZELTME (aynı gün, SDK dosyalarıyla): uçuşlarda hatalar MOTORLA bağlı.**
  - #49–#53'teki 209 TX hatasının hepsi [kalkış −60 s, iniş +20 s] içinde, dışında 0.
  - Kalkıştan 3–12 s önce (motorlar devir alırken) başlıyor, inişte kesiliyor.
  - #49'da iki uçuş arasındaki yerde geçen dakikada da 0.
  - Şüphe: EMI ya da güç/şase gürültüsü UART'ı bozuyor, özellikle radarın RX girişini.
    Kablo yolu, ortak şase ve CH340 lojik seviye jumper'ı (3.3/5 V) kontrol edilmeli.
  - #47 (yerde, uçuş yok) farklı: hatalar 9. dakikada başlayıp sürekli arttı. Motor
    çalışıp çalışmadığı kullanıcıya soruldu.
- PM sütunu neredeyse hep sabit 1690, arada 1/0/149 çöpü var; gerilim olarak kullanılamaz.
  control_msg.csv: (1, 1, 1.04, 22) sabit. eap.csv: numOfRadars hep 4, roll/pitch/yaw hep 0.
- SDK 'Sending CONTROL MESSAGE ... succeded' diyor: baytlar Jetson'dan çıktıktan sonra
  kayboluyor.
- Uçuş oturumlarında hataların 2–4. dakikada başlamasının sebebi ısı değil; kalkış o anda.
- Havada 781 s'nin 74 s'si (%10) radarsız geçti. Kesintiler 0–8 m'de, en uzunu 10.7 s @ 2.3 m
  (iniş). #49 ikinci uçuşta %25, #51'de %20; #50, #52 ve #53 temiz.
- Öneri: önce motorları çalıştırarak (pervanesiz, bağlı) yerde tekrarla, sonra kablo/şase/
  seviye düzelt. CP2102N/FTDI değişimi ikinci adım (CP210x udev kuralı var: /dev/ttyCP210x).
  HRVS_UART_PORT kodda sabit `/dev/ttyCH340`; parametre yapmayı teklif ettim.

**ÇÖZÜLDÜ (02_11 akşam): shield'lı, bükümlü (twisted) yeni kablaj.**
- #56–#61'de 0 hata, 0 reset: #58 yerde 34 dk ~27 bin ölçüm; #60 ve #61 uçuş 8.5 m.
- İlk kablaj denemesinde (#55) 17:26'da motorsuz kopma görülmüştü. Kullanıcı kablajı
  düzeltti, sonrasında temiz.
- CH340 değişimi ve port parametresi artık gerekmiyor. Radar 0 bayt veriyorsa önce
  radar beslemesine bak (#44, #54 ve #62 bataryasız masa oturumlarıydı).
