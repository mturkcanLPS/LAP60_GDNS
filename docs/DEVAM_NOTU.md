# Devam notu

Bu klasör, Claude Code ile Ağustos–Kasım 2026 arasında yürüttüğümüz çalışmanın yedeğidir. Jetson'a bir şey olursa ya da sohbet kaybolursa buradan devam edilir.

Son güncelleme: 2026-11-17 (commit `6dd818d` sonrası).

## Klasörde ne var

| Yol | İçerik |
|---|---|
| `docs/sohbet/2026-10_11_ana_sohbet.md` | Ana sohbetin okunabilir hali (356 kullanıcı mesajı). Sadece mesajlar ve cevaplar; araç çıktıları, görseller, sistem notları çıkarıldı, e-postalar maskelendi. |
| `docs/sohbet/onceki_oturum_1f95f232.md` | Daha önceki kısa bir oturum. |
| `docs/claude_hafiza/` | Claude'un kalıcı hafıza notları (19 dosya). `MEMORY.md` dizin; her dosya bir konu. Ölçümler, kararlar, tuzaklar burada; sohbetten çok daha hızlı okunur. |
| `tools/sohbet_disa_aktar.py` | Sohbet kaydını (JSONL) bu Markdown'a çeviren betik. |
| `tools/analiz/` | Uçuş kayıtlarını analiz eden betikler (eskiden yalnız `~/huni_kalib`'de duruyordu). |
| `tools/analiz/test/` | Masa test araçları: sahte seri port (`pty_tut.py`), seri hat hata sayaçları (`sayac.py`), sentetik akış+IMU yayıncısı (`sentetik.py`). |

Uçuş kayıtları (`/home/lapis/lps_records`, video/CSV) repoda **yok**; boyutları GB'larca.

## Yeni bir Claude oturumunda nasıl devam edilir

1. Hafızayı geri yükle (Claude bunları her oturumun başında otomatik okur):
   ```bash
   mkdir -p ~/.claude/projects/-home-lapis-ros2-ws/memory
   cp ~/ros2_ws/docs/claude_hafiza/*.md ~/.claude/projects/-home-lapis-ros2-ws/memory/
   ```
   Repo farklı bir yoldaysa klasör adı da değişir (yol, `/` yerine `-` ile yazılır).
2. Analiz betiklerini eski yerine koy (betiklerde yol `/home/lapis/huni_kalib` olarak geçiyor):
   ```bash
   mkdir -p ~/huni_kalib/test && cp -r ~/ros2_ws/tools/analiz/* ~/huni_kalib/
   ```
3. Claude'a "docs/DEVAM_NOTU.md'yi oku, kaldığımız yerden devam edelim" de. Ayrıntı gerekirse sohbet dosyasında konu adıyla arar.

## Şu anki durum (17_11)

**Otopilota giden veriler**
- AprilTag (`aprtiltag`): infra1 + infra2 birleşimi, huni tag'leri tag25h9, referans = küçük tag plakası. Kısa tag görüşleri otopilota gitmiyor: 3 ardışık tespit, 1.0 s kalma (`tag_gate.hpp`). 17_11: AprilTag yalnız ≤ 11 m'de çalışıyor (> 12 m kapalı, irtifa yoksa açık).
- Optik akış (`opticflow` = infra1; `opticflow2` = infra2, 17_11'den beri varsayılan KAPALI): OKAD gyroflow, RANSAC'lı EMA'sız akış (`gyroflow_input:=raw`) − RealSense gyro dönmesi (`gyroflow.hpp`). **Otopilot V1.1.3 bu formatı bilmiyor → otopilot güncellenene kadar GPS'li modda uçulmalı.** Gyroflow geçersizse pakete yeni timestamp ile `0/0, quality=0` gider (`gyroflow_invalid_q0`; OKAD dokümanından bilinçli sapma, OKAD'a bildirilmeli). NTP saat adımı gyro tamponunu donduruyordu → 17_11 düzeltildi (`test_gyroflow`).
- Araç bearing'i (`car`): kutu merkezi + huni ofseti (y +1.45 m, kamera ekseni), dron−araç başlık farkıyla döndürülüyor (0x4A'daki başlıklar).
- IR LED üçgeni (`ir`): yüksekliğe göre LED penceresi, gerçek üçgen ölçüleri (R–sağ 1.445 m, R–sol 1.31 m), huni merkezi düzeltmesi. Gündüz güneş parlamasında çalışmıyor (beklenen).

**Otopilottan gelen**
- 0x4A (19 B, OKAD V1.1.3): EKF yüksekliği + dron/araç başlığı + INS durumu. 0x49: KAPAT.

**Donanım**
- HRVS radar CH340 ile (`/dev/ttyCH340`, 921600). FTDI ve CP2102 aynı kabloda hat hatası verdi.
- HRVS her kalkışta kopuyor: kalkış titreşim darbesinde (IMU ile görüldü), inişte geri geliyor. Elle sallamada da kopuyor → gevşek konnektör/kablo şüphesi. **Donanım tarafında çözülmedi.**
- Jetson boşta 90 °C civarında çalışabiliyor; kapak/hava akışı kontrol edilmeli.

## Açık işler

| İş | Durum / not |
|---|---|
| HRVS kalkışta kopma | Radar kablosu/konnektörleri sabitlenmeli; yerde sallama testiyle doğrulanmalı. |
| Otopilot gyroflow güncellemesi | OKAD tarafında. Güncellenene kadar GPS'li mod. |
| Gyroflow uçuş doğrulaması | İlk uçuşta `flow_sNNNN.csv` / `flow2_sNNNN.csv` `durum` sütunu ve `[GYROFLOW]` logu kontrol edilecek. |
| Huni ofseti döndürme yönü | İlk başlık farklı uçuşta EO CSV `baslik_farki` + video ile doğrulanacak. |
| Tag menzili | 26 cm tag ~9–11 m'den sonra okunmuyor; hesap çözünürlük sınırını gösteriyor. 6–12 m'den ham kare kaydıyla margin filtresi doğrulanabilir. |
| Kendi araç/huni modelini eğitmek | EO videosu artık overlay'siz kaydediliyor (`eo_sNNNN.csv` kutularla). Veri birikince GPU'lu PC'de ince ayar. |
| Journal kaybı | Güç ani kesilince sistem logu kayboluyor; HRVS SDK TRACE satırları journal'ın çoğunu dolduruyor. Önerildi, yapılmadı. |
| Otopilotun 0x42 yankısı | Otopilot bizim paketimizi geri gönderiyor; sorulmadı/çözülmedi. |
