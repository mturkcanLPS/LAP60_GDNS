---
name: dds-hostid-shm-race
description: "Fast DDS 2.6 host_id IP'lerden hesaplanıyor; node'lar farklı IP setiyle açılırsa SHM devre dışı → görüntüler UDP'de kaybolur (20_10 açılışında WiFi yarışı)"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-10-20T09:56:52.369Z
---

**Olay (2026-10-20, 12:31 açılışı):** WiFi'nin IP alma anı node'ların başlama aralığına denk geldi.
- realsense IP yokken başladı → host_id 7f01. Abone node'lar WiFi IP'si geldikten sonra
  başladı → host_id f829.
- Sonuç: SHM devre dışı, kareler UDP'ye düştü, RcvbufErrors ~300/s.
- infra2 30 → 20 Hz, renkli 15 → 8.5 Hz, akış %75 düzenli. Kamera kendisi tam hızdaydı
  (metadata topic 30/15 Hz).

**Neden:** Fast DDS 2.6.11'de `Host::id()` = o anki tüm IPv4 adreslerinin MD5'i, 16 bite
indirilmiş (IP yoksa 127.1 = 7f01). SHM yalnız aynı host_id arasında kullanılıyor
(`SHMLocator::is_shm_and_from_this_host`).

**Teşhis yöntemi:**
- `ros2 topic info -v <topic>` → GID'nin 3. ve 4. baytı host_id.
- Root servisi lapis'ten okumak için UDP-only FastDDS profili gerekiyor.
- `/proc/net/udp`'nin drops sütunu → hangi portta kayıp var.

**Why:** Sahada WiFi/hotspot bağlanıp koparsa, ya da car_detector respawn olursa aynı
durum uçuşta da olur.

**How to apply:** Çözüm `src/lps_bringup/systemd/dds_yerel.conf` (`PrivateNetwork=yes`,
servis yalnız lo görür, host_id hep 7f01). `unshare -rn` ile doğrulandı:
- lo-only: 15.0 Hz, kayıp 0.
- yarış kopyası: 11.8 Hz, 20 s'de 1680 RcvbufErrors.

Bedeli: servis topic'leri dışarıdan görünmez. Kurulum kullanıcı kararında (sudo), 20_10
itibarıyla kurulmadı. Masada servis "yeniden başlat" ile (ağ oturduktan sonra) açılırsa
sorun görülmez; gerçek açılışta görülür.

İlgili: [[car-detector-oom-crash]] [[infra2-test-recording]]
