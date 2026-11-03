/**
 * test_dual_fusion.cpp — dual_camera_fusion.hpp (infra1 + infra2 tag/akis birlesimi) testi.
 *
 * Calistirma: ros2 run lps_serial_sender test_dual_fusion
 */
#include "lps_serial_sender/dual_camera_fusion.hpp"

#include <cmath>
#include <cstdio>

static int g_fail = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  HATA: %s\n", msg); ++g_fail; } \
                           else std::printf("  ok  : %s\n", msg); } while (0)
static bool near(double a, double b, double e = 1e-4) { return std::fabs(a - b) <= e; }

int main()
{
    using namespace lps;
    std::printf("dairesel aci ortalamasi\n");
    CHECK(near(circMeanDeg(359, 1), 0, 1e-3), "359 ve 1 -> 0 (180 degil)");
    CHECK(near(std::fabs(circMeanDeg(170, -170)), 180, 1e-3), "170 ve -170 -> 180");
    CHECK(near(circMeanDeg(10, 20), 15, 1e-3), "10 ve 20 -> 15");

    std::printf("tag: ofset sonrasi iki kamera (ucus kaydindaki gibi)\n");
    {
        // Ham: infra1 x, infra2'den 45 mm buyuk (kayit medyani). Ofsetler dusulunce ~ayni nokta.
        const double o1x = 0.032962, o2x = -0.017962, oy = 0.045;
        TagSample a; a.ok = true; a.x = 1.000f - o1x; a.y = 0.500f - oy; a.z = 4.0f; a.bits = 0x01;
        TagSample b; b.ok = true; b.x = 0.955f - o2x; b.y = 0.500f - oy; b.z = 4.0f; b.bits = 0x10;
        const TagPairResult r = fuseTagPair(a, b, 0.30f);
        CHECK(r.src == 3, "ikisi gecerli ve tutarli -> ikisi");
        CHECK(near(r.diff_m, 0.0059, 1e-3), "fark ~6 mm (ham 45 mm idi)");
        CHECK(near(r.out.x, 0.5 * (a.x + b.x)) && near(r.out.y, 0.455) && near(r.out.z, 4.0), "konum ortalama");
        CHECK(r.out.bits == 0x11, "huni bitleri OR");
    }
    std::printf("tag: tutarsiz / tek / yok\n");
    {
        TagSample a; a.ok = true; a.x = 1.0f; a.z = 4.0f;
        TagSample b; b.ok = true; b.x = 1.5f; b.z = 4.0f;
        TagPairResult r = fuseTagPair(a, b, 0.30f);
        CHECK(r.src == 1 && near(r.out.x, 1.0), "0.5 m fark -> infra1 (birincil)");
        b.ok = false; r = fuseTagPair(a, b, 0.30f);
        CHECK(r.src == 1 && near(r.out.x, 1.0), "sadece infra1");
        a.ok = false; b.ok = true; r = fuseTagPair(a, b, 0.30f);
        CHECK(r.src == 2 && near(r.out.x, 1.5), "sadece infra2");
        b.ok = false; r = fuseTagPair(a, b, 0.30f);
        CHECK(r.src == 0 && !r.out.ok, "ikisi de yok -> gecersiz");
        TagSample c; c.ok = true; c.yaw = 179; TagSample d; d.ok = true; d.yaw = -179;
        r = fuseTagPair(c, d, 0.30f);
        CHECK(near(std::fabs(r.out.yaw), 180, 1e-3), "yaw +-179 -> 180 (0 degil)");
    }
    std::printf("akis\n");
    {
        FlowSample a{true, 1.0f, 2.0f, 80}, b{true, 1.2f, 2.1f, 40};
        FlowPairResult r = fuseFlowPair(a, b, 1.0f, 0.3f);
        CHECK(r.src == 3 && near(r.out.dx, (80 * 1.0 + 40 * 1.2) / 120) && r.out.q == 80, "kaliteyle agirlikli ortalama, Q = buyuk");
        b = {true, 4.0f, -2.0f, 90}; r = fuseFlowPair(a, b, 1.0f, 0.3f);
        CHECK(r.src == 2 && near(r.out.dx, 4.0), "uyusmuyor -> kalitesi yuksek olan (infra2)");
        b = {true, 1.1f, 2.0f, 0}; r = fuseFlowPair(a, b, 1.0f, 0.3f);
        CHECK(r.src == 1, "Q=0 kamera disarida");
        a.ok = false; b = {true, 0.5f, 0.5f, 60}; r = fuseFlowPair(a, b, 1.0f, 0.3f);
        CHECK(r.src == 2 && near(r.out.dx, 0.5), "infra1 bayat -> infra2");
        b.ok = false; r = fuseFlowPair(a, b, 1.0f, 0.3f);
        CHECK(r.src == 0 && !r.out.ok, "ikisi de yok");
    }
    std::printf(g_fail ? "\nSONUC: %d HATA\n" : "\nSONUC: hepsi gecti\n", g_fail);
    return g_fail ? 1 : 0;
}
