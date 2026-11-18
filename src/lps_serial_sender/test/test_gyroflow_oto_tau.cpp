// test_gyroflow_oto_tau - otomatik tau secimi sentetik testi (18_11)
// Akis = gercek tau ile gecikmis gyro donmesi + gurultu; secilen tau gercege yakin olmali.
// Calistirma: ros2 run lps_serial_sender test_gyroflow_oto_tau
#include "lps_serial_sender/gyroflow_oto_tau.hpp"
#include <cmath>
#include <cstdio>
#include <random>

static int dene(double tau_gercek) {
    GyroFlowOtoTau g;
    std::mt19937 rng(1); std::normal_distribution<double> n(0.0, 0.05);
    const double hz = 200.0, fps = 30.0;
    double ax = 0, ay = 0;   // gercek aci (titresim: 9 ve 13 Hz)
    auto w = [](double t, double f, double a) { return a * std::sin(2 * M_PI * f * t); };
    double t = 0, tk_prev = -1, ax_prev = 0, ay_prev = 0, sonraki_kare = 0.05;
    for (int i = 0; i < (int)(60 * hz); ++i, t += 1 / hz) {
        const float gx = (float)(w(t, 9, 0.35) + w(t, 2, 0.1)), gy = (float)(w(t, 13, 0.25) + w(t, 3, 0.08));
        g.gyroEkle(t, gx, gy);
        ax += gx / hz; ay += gy / hz;
        if (t >= sonraki_kare + 0.03) {   // kare: gyro biraz ilerledikten sonra islenir
            const double tk = sonraki_kare;
            // gercek donme: [tO - tau, tK - tau] araligindaki aci farki (yaklasik: t-0.03'teki aci kaydi yok -> analitik)
            auto aci = [&](double tt, bool x) {   // analitik integral
                if (x) return -0.35 / (2 * M_PI * 9) * std::cos(2 * M_PI * 9 * tt) - 0.1 / (2 * M_PI * 2) * std::cos(2 * M_PI * 2 * tt);
                return -0.25 / (2 * M_PI * 13) * std::cos(2 * M_PI * 13 * tt) - 0.08 / (2 * M_PI * 3) * std::cos(2 * M_PI * 3 * tt);
            };
            if (tk_prev > 0) {
                const double dthx = aci(tk - tau_gercek, true) - aci(tk_prev - tau_gercek, true);
                const double dthy = aci(tk - tau_gercek, false) - aci(tk_prev - tau_gercek, false);
                const float dy = (float)(740 * dthx + n(rng)), dx = (float)(-845 * dthy + n(rng));
                GyroFlowSonuc s; g.hesapla(tk, tk_prev, dx, dy, s);
                g.ogren(tk, tk_prev, dx, dy);
            }
            tk_prev = tk; sonraki_kare += 1 / fps;
        }
    }
    const double sec = g.tau();
    const bool ok = std::abs(sec - tau_gercek) <= 0.0026;
    std::printf("%s: gercek tau %+.1f ms -> secilen %+.1f ms (gecis %u, ogrenilen %ld)\n", ok ? "ok  " : "HATA",
                tau_gercek * 1000, sec * 1000, g.gecis(), g.ogrenilen());
    return ok ? 0 : 1;
}

int main() {
    int h = 0;
    for (double t : {-0.0025, 0.0, 0.005, 0.010, 0.0125}) h += dene(t);
    std::printf(h ? "\n%d HATA\n" : "\nHEPSI GECTI\n", h);
    return h ? 1 : 0;
}
