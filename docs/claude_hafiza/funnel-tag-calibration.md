---
name: funnel-tag-calibration
description: "Huni tag25h9 kalibrasyonu 05_11 KESİN — duvar 26 cm, ofset (0,+0.2942,-0.1021) canlı tam çözünürlükten; 290→280 farkı tag kavisi; yön 0→34→24→20; küçük tagler doğrulanmadı; canlı kalibrasyon tuzakları"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-08T07:09:25.498Z
---

05_11'de duvar ofsetleri belirlendi. Kod 45/24 test modundan huni moduna geçirildi (TEST_APRITLAG 0). Değişiklikler commit'lenmedi / push edilmedi.

NOT: kalib_145232/191817/194158 bag'leri 05_11'de kullanıcı isteğiyle SİLİNDİ (37 GB); huni_kalib'de sadece scriptler/özetler kaldı. Commit e354128 (pushlandı).

**Kesin değerler** (canlı kalibrasyon, bag `kalib_145232` — silindi):
- Duvar tag siyah kare **26 cm** (kullanıcı teyit etti; eskiden 25 cm'ydi).
- **SON HAL (05_11 akşam, kullanıcı onayı): Referans = küçük tag plakası merkezi.** Duvar `(0, +0.3004, -0.0959)` = yatay 280 mm (0,.1977,−.1983) + eksen boyunca 14.5 cm (plaka duvar tag merkezlerinin 14.5 cm altında, kalib_194158). Küçük: 10/12 `(±0.045, +0.0754, 0)`, 14/16 `(±0.045, −0.0646, 0)` — y'ye +5.4 mm plaka kayması düzeltmesi eklendi (12/14/16 hep aynı yöne sapıyordu).
  - Neden plaka: küçük tag eğim gürültüsü ~9° × 14.5 cm → duvar hizası referansında ~2.2 cm (p90 4 cm) yanal oynama; plakada ~0.6 cm. Kullanıcı duvar tag kavisinin de bozup bozmadığını sordu: sabit, simetrik, kalibrasyona girmiş, karşılıklı taglerde sönüyor → mm düzeyi.
  - Eski 18 mm/(0,.3070,−.0894) değeri geçersiz.
- Huni dibi çemberine göre ofset `(0, +0.2942, -0.1021)`: tam çözünürlük, gerçek intrinsics (fx = fy = 647.5, cx 646.49, cy 354.55; infra1 ve infra2 aynı, baseline 49.95 mm).
  - 570 çift kullanıldı. Yarıçap 280.3 mm; kamera ve mesafe bandı başına 279–281 (0.85–1.3 m).
  - Çift farkı medyan 7.3 mm; Fable ofsetiyle 20.3 mm.
  - Tag başına sistematik sapma ≤3 mm, ayrı düzeltme yok.
- Launch: APRILTAG_DETECT size = FUSION_NODE_SIZE = GPU size = 0.26; tag_family varsayılanı tag25h9.
- Aile doğrulandı: 25h9 ile 0/20/24/34 çıktı, 36h11 ile hiç tespit yok.

**Yön eşlemesi**: Görüntüde saat yönünde sıra 0→34→24→20. Yani 0 Kuzey, 34 Doğu, 24 Güney, 20 Batı. Eski kod yorumu 20=E, 34=W diyordu, ters. Bitler ID'ye göre: valid2 = 20 (Batı), valid4 = 34 (Doğu).

**Çizim (OKAD-MBI-P4a-101 REV02):**
- Huni: yarı açı 44.9°, küçük Ø310.4, büyük Ø849.3.
- Tag merkezleri eksenden 290.0 mm, dipten 135.2 mm yukarıda. Üstten bakışta 410×410 karenin köşelerinde.
- Tag ile alt kenar arası boşluk 60.9 mm. Kullanıcı ~6 cm ölçtü, yani tagler yerinde.
- Fable ofseti (0, .3013, −.1092), tag merkezinden huni DİBİ merkezine giden vektör.

**290'ın 280 çıkmasının sebebi:** Tag yüzeye sarılı (eğrilik ~410 mm), PnP ise düz kare varsayıyor.
- Ray-cast simülasyonu farkın yarısını açıklıyor (~1 m'de 285).
- Kalan ~5 mm açıklanamadı. Yarım çözünürlükten değil; canlı tam çözünürlük de 280 verdi.
- Sim'e göre z mesafeyle kayıyor (3 m'de ~3 cm); yanal stabil.
- Eksen boyu (~137 mm) çizimden; duvar çiftlerinden gözlenemez.
- "Tagler 13 mm aşağıda" hipotezi YANLIŞTI (cetvel ölçümüyle çürüdü).

**Stereo bulgusu:** Aynı karede infra1 − infra2 tag konumu x'te baseline'ın %96'sı çıktı. Ayrıca dy ≈ −5 mm (1 m'de).
- Bu bir ölçek hatası değil. Kameralar arasında sabit görüntü kayması var: ~1.2 px yatay, ~3.4 px dikey.
- Muhtemel kaynak: lens önündeki filtreler (infra2 32/PL, infra1 düz 32). Kama veya eğim, ~0.3° açısal sapma demek.
- Etkisi: iki kameranın avgPos'u yükseklikle büyüyen bir farkla ayrışır (5 m'de ~2.6 cm). #74'teki 24–29 mm farkla uyumlu.
- Stereo ile ölçek ölçmek bu yüzden güvenilmez.

**Tespit oranı:** infra2'de tag 34 nadir görülüyor (37, tag 20 ise 498). Parlak vinil yansıması ve/veya polarizör etkisi; mat yüzey önerisi geçerli.

**Canlı kalibrasyon tuzakları (tekrarlanacaksa):**
- Servis root + PrivateNetwork=yes; dışarıdan abone olunamaz. Kullanıcı servisi durdurur, `/home/lapis/huni_kalib/canli.sh baslat|durdur` çalıştırılır (seri pty'ye, car kapalı).
- Bag'i /tmp'ye YAZMA: reboot'ta silinir. Kayıt sürerken sqlite'ı OKUMA: "database is locked" ile recorder çöker.
- Launch'a grup SIGINT gönderme: respawn yarışı oluyor. Önce launch PID'ine SIGINT, sonra SIGTERM, sonra pkill.
- Termal: Jetson'ın kapağı hava almayı engelliyor (kullanıcı söyledi). every_n 1 + 55 MB/s görüntü kaydında ~4 dk sonra throttle, ardından 104 °C kapanma oldu (14:28).
  - every_n 2 ile 5 dk'da 85 °C'ye çıktı.
  - 95 °C bekçisi kullan.
- Yarım çözünürlüklü AVI kayıtları bu kavisli tagler için tespitte işe yaramıyor; kalibrasyon canlı veya ham bag ile yapılmalı.

**Açık noktalar**
- Küçük tagler (kullanıcı, 05_11 son hal): 4 tane, ID 10/12/14/16, tag25h9, siyah kare **65 mm**.
  - Önce 2 tag/85 mm, sonra 4 tag/85 mm dendi; en son 65 mm ve ölçüler geldi.
  - Merkeze göre konumlar (üstten, x sağa, y yukarı, cm): 10 (−4.5, +7.0), 12 (+4.5, +7.0), 14 (−4.5, −7.0), 16 (+4.5, −7.0).
  - Koddaki ofset = (−x, +y, 0), referans plaka. Eski LPS işaret kuralıyla aynı; füzyona AÇIK.
  - 05_11 canlı kayıt (`huni_kalib/kalib_191817`, `kucuk_an.py`): işaret kuralı DOĞRULANDI.
    - Yanal: 12/14 kullanıcı ölçüsüne ~0.5 cm içinde.
    - z: küçük tagler huni dibi çemberinin 1.5–2.2 cm altında; 18 mm alındı.
    - Referans plakaya taşındı: duvar ofseti 18 mm uzatıldı, küçük z = 0. Sonrasında 12/14 için eksen farkı −0.6/+0.1 cm, yanal ~0.6 cm.
  - 10 ve 16 çok nadir okunuyor (12/14 3500'er kare, 10: 90, 16: 46).
    - 16'nın üstünden ortadaki cihazın kablosu geçiyor.
    - 10'un sebebi belirsiz (2 bit düzeltmeyle de nadir).
    - Kullanıcıya kabloyu taşımak ve 10'u kontrol etmek önerildi.
  - valid5–8 = 10/12/14/16.
  - 65 mm'lik tag ~2.8 m'nin altında görülür.
- 25h9 sahte tespitleri ÇÖZÜLDÜ (05_11). cpu_apriltag_node'a `max_hamming` 1 ve `min_decision_margin` 30 eklendi, launch APRILTAG_DETECT'te.
  - Ölçüm (`huni_kalib/hamstat`): gerçek tagler %90 0 bit, %9 1 bit; margin medyan 306, p5 237. Sahtelerin %93'ü 2 bit, margin medyan 3–10.
  - Sonuç: 203 sahtenin 0'ı kaldı, gerçeklerin %98.9'u korundu.
  - Log satırı: `[TAG-FILTRE] ... en yüksek elenen margin X`. X 30'a yaklaşırsa gerçek tag eleniyor olabilir.
  - Gerçek tag margin verisi sadece ~1 m'den; yüksek irtifada uçuş logundan kontrol edilmeli. GPU (Isaac) arka ucunda bu filtre yok.

**Kendi-z testi (05_11 akşamı, `kalib_194158`, ~1.3 m, ~1100 ortak kare):**
- Kullanıcı "her tag kendi z'si" istedi. Kodda duvar ofseti yatay (0, .1977, −.1983); küçük z 0.
- Kod şu an bu TEST halinde, commit'lenmedi.
- Plaka, duvar tag MERKEZLERİNİN huni ekseni boyunca ~13.7–14.1 cm altında (kamera-z farkı ~13.6–13.9 cm). 1.0 m'deki önceki test ~15.4 cm verdi; fark tag kavisinin mesafeye bağlı yanlılığı.
- Kullanıcının tahmini 22–23 cm'ydi; veri desteklemiyor (22–23 cm ancak tag ÜST kenarından ölçülürse yaklaşık tutar, ~24.5).
- Yanal tutarlılık 5–7 mm (eksen duvar taglerinden alınınca).
- Küçük tag normal tahmini ~9° sapıyor. Bu yüzden küçük taglere büyük z ofseti vermek, yani referansı duvar hizası yapmak, 14 cm × sin 9° ≈ 2 cm yanal hata getirir. Referansın plaka olması (küçük z 0) bu açıdan daha iyi.
- Tag 10 yine nadir okunuyor (25). 16 bu sefer iyi okundu (1737); kablo kaymış olmalı.

**06_11 vinç testi (`kalib_093645`, 0.3–3.9 m, `donus_an.py`):**
- Tutum: eğim medyan 3.3° (p90 5°), yaw ~27° döndü, hız p90 7°/s. Duvar tag yaw farkları 0/−90/+89/+178° (geometri tutarlı).
- Küçük−duvar YANAL fark: 1–2 m'de 2.5–2.7 mm, 2–2.5 m'de 6 mm, 2.5–4 m'de 9 mm (p90 16). Eğime (0–10°) bağlı değil.
- Küçük−duvar EKSEN (z) farkı mesafeyle büyüyor: 1–1.5 m +1 mm, 1.5–2 m −31, 2–2.5 m −85, 2.5–4 m **−144 mm** (küçük tag daha yakın gösteriyor). 65 mm tag 3 m'de ~14 px → PnP derinliği güvenilmez. Füzyon ortalamada z, hangi tag görünüyor diye zıplar. Öneri: küçük tagleri ~2 m üstünde füzyondan çıkar (kullanıcıya soruldu).
- Hızlı yaw'da (30–90°/s) küçük tag tespiti yarıya düşüyor (bulanıklık), duvar etkilenmiyor.

**08_11 tag geçerlilik kapısı (`tag_gate.hpp`, commit 437f79d pushlandı):** Otopilot logu (LAP60_261107_1754_17, s0133 uçuşu): iki inişte de ~10 m'de tek tag görüşü → otopilot araç ölçümünü 1 s kapatıyor → tag gelmiyor → son tag'den tam 1.0 s sonra iniş ASKIYA. Bizim kayıtta o anlar 1, 1 ve 2 karelik görüş. Kural: 3 ardışık tespit → geçerli; doğrulanmış tag `tag_keep_s` içinde dönerse anında geçerli. keep 0.3 kötüydü (6–8 m'de köprü görüşleri bastırıp "tag→1 s yok" olayını 18→20 yaptı); **keep 1.0**: 18→6, 6–8 m geçerli kare kaybı yok. Otopilota ayrıca önerildi: tek tag görüşü arac ölçümünü 1 s kapatmasın / tag 0.3 s gelmezse araca dönsün.
- s0133'te tag menzili kısa: 3–5 m %74, 7–9 m %38, 9–11 m %7, 11 m üstü 0 — tag'ler karelerde net görünüyor. Şüphe: `min_decision_margin` 30 (sadece 1 m verisiyle ayarlandı). Araştırılacak.

**Kamera ofseti işareti doğrulandı:** Platform merkezi ortalanarak çekilen kayıtta kodun kuralı (`pos -= off`) eksene en yakın sonucu verdi. Ters işaret ~11 cm uzak çıkıyor.

İlgili: [[infra2-test-recording]] [[autopilot-owns-frame-and-tilt]] [[post-flight-checks-pending]] [[jetson-health-thermal]]
