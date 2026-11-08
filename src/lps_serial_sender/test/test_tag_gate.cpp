/**
 * test_tag_gate.cpp — tag_gate.hpp (AprilTag gecerlilik kapisi) testi.
 * Diziler 15 Hz tag mesajlari: '1' = tespit, '.' = tespitsiz mesaj.
 * Ilk uc dizi s0133 ucusundan (infra1 videosu), otopilotun askiya gectigi anlar.
 * Calistirma: ros2 run lps_serial_sender test_tag_gate
 */
#include "lps_serial_sender/tag_gate.hpp"

#include <cstdio>
#include <string>

static int g_fail = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  HATA: %s\n", msg); ++g_fail; } \
                           else std::printf("  ok  : %s\n", msg); } while (0)

// Diziyi kapidan gecir; gecerli cikan mesaj dizisini dondur ('V' gecerli, '-' degil).
static std::string run(const std::string &seq, int n = 3, double keep = 0.3, double dt = 1.0 / 15)
{
    lps::TagGate g(n, keep);
    std::string out;
    double t = 100.0;
    for (char c : seq) { out += g.update(c == '1', t) ? 'V' : '-'; t += dt; }
    return out;
}
static int count(const std::string &s) { int n = 0; for (char c : s) n += (c == 'V'); return n; }

int main()
{
    std::printf("s0133 ucusu: otopilotun askiya gectigi kisa gorusler (~10 m)\n");
    {
        const std::string a = ".......................1.....................";   // ApTs 571.3
        const std::string b = "..............1..............................";   // ApTs 571.9
        const std::string c = ".........................11...................";  // ApTs 654.9
        CHECK(count(run(a)) == 0, "571.3: tek kare -> otopilota gecerli GITMEZ");
        CHECK(count(run(b)) == 0, "571.9: tek kare -> gitmez");
        CHECK(count(run(c)) == 0, "654.9: iki kare -> gitmez (N=3)");
        CHECK(count(run(c, 2)) > 0, "(karsilastirma) N=2 olsaydi 654.9 gecerli giderdi");
    }

    std::printf("giris: 3 ardisik tespit\n");
    {
        CHECK(run("111111") == "--VVVV", "3. tespitte gecerli, sonra surekli");
        CHECK(run("11.11.11") == "--------", "araya tespitsiz mesaj giren 2'liler hic gecerli olmaz");
    }

    std::printf("kalma: gecerliyken kisa kesinti (<= 0.3 s)\n");
    {
        // 3 tespit -> gecerli; 1 kare bosluk; tekrar tespit hemen gecerli (3 beklemez)
        CHECK(run("111.1111") == "--V-VVVV", "1 karelik bosluktan sonra ANINDA gecerli");
        // 3 tespitsiz mesaj: son tespitten 4/15 = 0.27 s sonra yeni tespit -> kalma penceresinde
        CHECK(run("111...11") == "--V---VV", "0.27 s bosluk -> aninda gecerli");
        // 4 tespitsiz mesaj: 5/15 = 0.33 s -> pencere disi, yeniden 3 gerekir
        CHECK(run("111....111") == "--V------V", "0.33 s bosluk -> yeniden 3 ardisik");
    }

    std::printf("dusme: 0.3 s'den uzun kayip -> yeniden 3 ardisik\n");
    {
        // 6 kare bosluk = 0.4 s -> kapi kapanir; donuste 3 tespit beklenir
        CHECK(run("111......1111") == "--V--------VV", "0.47 s kayiptan sonra yeniden 3 ardisik gerekir");
        CHECK(count(run("111......1.1.1")) == 1, "uzun kayiptan sonra seyrek tek goruslerin hicbiri gecmez");
    }

    std::printf("varsayilan kalma 1.0 s (s0133 ile secildi)\n");
    {
        // dogrulanmis tag, 0.73 s kesinti (10 tespitsiz mesaj), tek kare -> gecerli (kopru)
        CHECK(run("111" + std::string(10, '.') + "1", 3, 1.0).back() == 'V', "0.73 s sonra tek kare -> gecerli");
        // 1.07 s kesinti (15 tespitsiz) -> yeniden 3 ardisik
        CHECK(run("111" + std::string(15, '.') + "111", 3, 1.0) == "--V" + std::string(15, '-') + "--V",
              "1.07 s kesintiden sonra yeniden 3 ardisik");
        lps::TagGate g;   // varsayilanlar: 3 / 1.0
        double t = 0; bool v = false;
        for (int i = 0; i < 3; ++i) { v = g.update(true, t); t += 1.0 / 15; }
        CHECK(v, "varsayilan kurucu: 3. tespitte gecerli");
    }

    std::printf("zaman araligi: mesajlar arasi > 0.25 s ise ardisik sayilmaz\n");
    {
        lps::TagGate g(3, 0.3);
        bool v1 = g.update(true, 0.0), v2 = g.update(true, 0.5), v3 = g.update(true, 1.0);
        CHECK(!v1 && !v2 && !v3, "0.5 s arali 3 tespit (arada mesaj kaybi) -> gecerli degil");
    }

    std::printf("istatistik\n");
    {
        lps::TagGate g(3, 0.3); double t = 0;
        for (char c : std::string("1.1.11.111")) { g.update(c == '1', t); t += 1.0 / 15; }
        // 1 . 1 . 1 1 . 1 1 1 -> bastirilan: 1+1+2+2 = 6, son uclunun 3.'su gecerli
        CHECK(g.suppressed() == 6, "bastirilan tespit sayaci = 6");
    }

    std::printf(g_fail ? "\nSONUC: %d HATA\n" : "\nSONUC: hepsi gecti\n", g_fail);
    return g_fail ? 1 : 0;
}
