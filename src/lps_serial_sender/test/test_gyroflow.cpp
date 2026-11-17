// test_gyroflow - gyroflow saat sicramasi testi (17_11, s0205 NTP adimi)
// Calistirma: ros2 run lps_serial_sender test_gyroflow
#include "lps_serial_sender/gyroflow.hpp"
#include <cstdio>

static int hata = 0;
#define KONTROL(c, m) do { if (!(c)) { std::printf("HATA: %s\n", m); ++hata; } else std::printf("ok  : %s\n", m); } while (0)

// t0'dan itibaren n saniye 200 Hz sabit gyro besler
static double besle(GyroFlow& g, double t0, double n, float gx = 0.1f, float gy = -0.2f) {
    double t = t0;
    for (int i = 0; i < (int)(n * 200); ++i, t += 0.005) g.gyroEkle(t, gx, gy);
    return t;
}

int main() {
    GyroFlowSonuc s;
    {   // normal calisma + bilinen donme
        GyroFlow g; besle(g, 100.0, 1.0);
        auto d = g.hesapla(100.5, 100.5 - 1.0 / 30, 0.f, 0.f, s);
        KONTROL(d == GyroFlowDurum::Tamam, "normal: Tamam");
        // dy = 0 iken comp_dy = -fx * wx * kareNom = -740*0.1/30
        KONTROL(std::fabs(s.comp_dy_kare - (-740.0 * 0.1 / 30)) < 0.05, "normal: gyro donmesi dogru cikariliyor");
    }
    {   // ileri sicrayan tek ornek, sonra saat eski yerinden devam (s0205 senaryosu)
        GyroFlow g; double t = besle(g, 100.0, 1.0);
        g.gyroEkle(t + 1400.0, 2.0f, 2.0f);             // asiri ileri ornek (farkli gyro)
        besle(g, t, 1.0);                               // gercek zaman devam
        auto d = g.hesapla(t + 0.5, t + 0.5 - 1.0 / 30, 0.f, 0.f, s);
        KONTROL(d == GyroFlowDurum::Tamam && std::fabs(s.comp_dy_kare - (-740.0 * 0.1 / 30)) < 0.05,
                "ileri sicrama sonrasi toparlaniyor (deger dogru)");
    }
    {   // kalici ileri adim (NTP saati ileri aldi), kareler de yeni saatte
        GyroFlow g; double t = besle(g, 100.0, 1.0);
        double t2 = besle(g, t + 4080.0, 1.0);
        auto d = g.hesapla(t2 - 0.3, t2 - 0.3 - 1.0 / 30, 0.f, 0.f, s);
        KONTROL(d == GyroFlowDurum::Tamam, "kalici ileri adim sonrasi Tamam");
        KONTROL(std::fabs(s.comp_dy_kare - (-740.0 * 0.1 / 30)) < 0.05, "adim sonrasi donme dogru (bosluk boyunca integral yok)");
    }
    {   // geri adim
        GyroFlow g; double t = besle(g, 5000.0, 1.0);
        double t2 = besle(g, t - 3000.0, 1.0);
        auto d = g.hesapla(t2 - 0.3, t2 - 0.3 - 1.0 / 30, 0.f, 0.f, s);
        KONTROL(d == GyroFlowDurum::Tamam, "geri adim sonrasi Tamam");
    }
    {   // kucuk geri ornek (tekrar/sira disi) hala sadece atiliyor
        GyroFlow g; double t = besle(g, 100.0, 1.0);
        g.gyroEkle(t - 0.5, 5.f, 5.f);
        auto d = g.hesapla(t - 0.2, t - 0.2 - 1.0 / 30, 0.f, 0.f, s);
        KONTROL(d == GyroFlowDurum::Tamam && std::fabs(s.comp_dy_kare - (-740.0 * 0.1 / 30)) < 0.05,
                "kucuk sira disi ornek yok sayiliyor");
    }
    std::printf(hata ? "\n%d HATA\n" : "\nHEPSI GECTI\n", hata);
    return hata ? 1 : 0;
}
