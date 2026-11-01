#!/bin/bash
# LPS goruntu hatti icin donanim saatleri — ROOT ile calisir (servisin
# ExecStartPre'si). Ayarlar yeniden acilista sifirlandigi icin her baslangicta.
#
# Neden (28_09 olcumu, Orin Nano Super, MAXN_SUPER, 30 fps iki kamera):
#   varsayilan DVFS optik akis donanimini minimumda tutuyordu:
#     OFA 115/537 MHz, VIC 115/435 MHz, GPU 306/1020 MHz
#   -> iki kameranin akisi karelerin %33-34'unu ATLIYORDU (CPU bosken bile).
#   jetson_clocks (GPU tavan)              -> atlanan %21-22
#   + OFA/VIC 'performance' governor       -> atlanan %0.4-0.6, iki akis 30 Hz
#   jetson_clocks bu L4T surumunde OFA/VIC'e DOKUNMUYOR, o yuzden ayrica.
# Bedeli: guc tuketimi/isi artar (tam yukte birkac W).
# Geri almak: jetson_clocks --restore <yedek>; governor'lari tegra_wmark yap.

jetson_clocks || echo "jetson_clocks basarisiz" >&2

for d in 15a50000.ofa 15340000.vic; do      # OFA = optik akis, VIC = format donusumu
    g=/sys/class/devfreq/$d/governor
    if [ -w "$g" ]; then
        echo performance > "$g" && echo "$d: $(cat $g) $(( $(cat /sys/class/devfreq/$d/cur_freq) / 1000000 )) MHz"
    else
        echo "$d bulunamadi/yazilamiyor (farkli modul?)" >&2
    fi
done
exit 0   # saat ayari basarisiz olsa da servis (ucus yigini) yine baslasin
