---
name: new-jetson-setup-doc
description: "Yeni Jetson'a taşıma için YENI_JETSON.md + README (07_11, commit 86f2bf7); repo dışı sistem ayarları denetimi ve eski GDNS.service uyarısı"
metadata:
  node_type: memory
  type: project
  originSessionId: ae892145-43b4-4335-89c7-55834ec12cf8
  modified: 2026-11-07T07:36:02.434Z
---

07_11: Kullanıcı kodu başka bir Jetson'a (JetPack kurulu) GitHub'dan kuracak, kurulumu başka bir yapay zekâ ajanı README'den yapabilsin istedi. Commit 86f2bf7 (pushlandı):
- `YENI_JETSON.md` (repo kökü): repo'nun otomatik yaptığı sistem ayarları, eski Jetson'da elle yapılanlar, bilerek değiştirilmeyen varsayılanlar, yapılmaması gerekenler, bilinen sorunlar, kontrol listesi.
- README: sıfırdan apt kaynakları/paketler, doğrulama adımı, "Tavsiyeler" bölümü (kullanıcı isteği: otomatik güncelleme kapatma + RealSense hold ZORUNLU adım değil tavsiye), 0x4A, IR/tag, gövdeye özel ayarlar.
- `src/lps_bringup/scripts/otomatik_guncelleme_kapat.sh` (eski ~/sistem_bakim_2026-11-01/faz3'ün genel hali).

Denetim bulguları (eski Jetson): sysctl/usbfs(16)/extlinux/journald/zram/fan(quiet) hepsi varsayılan. "Tampon" çözümü sysctl değil FastDDS 32 MB SHM profili. Saat yükseltme `set_vision_clocks.sh` (clocks.conf). **`/etc/systemd/system/GDNS.service`** eski HRVS example.out servisi, disabled — açılırsa radar UART'ı çakışır, taşınmamalı. `~/.local` numpy 2.2.6 hâlâ var. Temiz klon derlemesi: 9 paket, 4 test geçti.

**How to apply:** Sistem ayarı değiştiren her yeni iş YENI_JETSON.md'ye de yazılmalı (yoksa yeni Jetson'da eksik kalır).

İlgili: [[l4t-3647-interrupted-update]] [[github-repo-lap60-gdns]] [[dds-hostid-shm-race]]
