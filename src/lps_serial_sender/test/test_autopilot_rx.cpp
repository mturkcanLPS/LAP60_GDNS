/**
 * test_autopilot_rx.cpp — autopilot_rx.hpp ayristirici testi (otopilot 0x49 KAPAT).
 *
 * Otopilotun gonderecegi cerceve: BB 49 01 FB 01 FF (kapat = true).
 * Kontrol edilenler: parca parca gelis, cop/bozuk checksum, sahte buyuk header
 * arkasinda kalan gercek cerceve (idle), arka arkaya cerceveler, ve rastgele
 * gurultuden YANLIS KAPAT cikmamasi.
 *
 * Calistirma: ros2 run lps_serial_sender test_autopilot_rx
 */
#include "lps_serial_sender/autopilot_rx.hpp"

#include <cstdio>
#include <random>
#include <vector>

static int g_fail = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  HATA: %s\n", msg); ++g_fail; } \
                           else std::printf("  ok  : %s\n", msg); } while (0)

struct Rec {
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> got;
    lps::AutopilotRxParser p{[this](uint8_t id, const std::vector<uint8_t> &pl) { got.push_back({id, pl}); }};
    int poweroffs() const {
        int n = 0;
        for (auto &g : got) if (g.first == 0x49 && g.second.size() == 1 && g.second[0] == 1) ++n;
        return n;
    }
};

static const std::vector<uint8_t> KAPAT = {0xBB, 0x49, 0x01, 0xFB, 0x01, 0xFF};

int main()
{
    std::printf("checksum kurali\n");
    {
        const uint8_t h[3] = {0xBB, 0x49, 0x01}, pl[1] = {0x01};
        CHECK(lps::apChecksum(h, 3) == 0xFB, "header BB 49 01 -> FB (otopilot tablosu)");
        CHECK(lps::apChecksum(pl, 1) == 0xFF, "yuk 01 -> FF (otopilot tablosu)");
    }

    std::printf("tam cerceve\n");
    { Rec r; r.p.feed(KAPAT.data(), KAPAT.size()); CHECK(r.poweroffs() == 1, "BB 49 01 FB 01 FF -> 1 KAPAT"); }

    std::printf("bayt bayt\n");
    { Rec r; for (uint8_t b : KAPAT) r.p.feed(&b, 1); CHECK(r.poweroffs() == 1, "tek tek baytlarla 1 KAPAT"); }

    std::printf("cop arasinda (BB iceren cop dahil)\n");
    {
        Rec r; std::vector<uint8_t> s = {0x00, 0xBB, 0x12, 0xBB, 0xBB, 0x49, 0x55};
        s.insert(s.end(), KAPAT.begin(), KAPAT.end());
        s.insert(s.end(), {0xBB, 0x49, 0x01});
        r.p.feed(s.data(), s.size());
        CHECK(r.poweroffs() == 1 && r.got.size() == 1, "copten sonra 1 KAPAT, baska cerceve yok");
    }

    std::printf("bozuk checksum\n");
    {
        Rec r; std::vector<uint8_t> a = KAPAT; a[3] = 0xFA; r.p.feed(a.data(), a.size()); r.p.idle();
        CHECK(r.got.empty(), "header checksum bozuk -> yok sayildi");
        Rec r2; std::vector<uint8_t> b = KAPAT; b[5] = 0xFE; r2.p.feed(b.data(), b.size()); r2.p.idle();
        CHECK(r2.got.empty(), "yuk checksum bozuk -> yok sayildi");
        Rec r3; std::vector<uint8_t> c = {0xBB, 0x49, 0x01, 0xFB, 0x00, 0x00}; r3.p.feed(c.data(), c.size());
        CHECK(r3.poweroffs() == 0 && r3.got.size() == 1, "kapat=false (00) -> cerceve gecerli ama KAPAT degil");
    }

    std::printf("sahte buyuk header arkasinda gercek cerceve\n");
    {
        Rec r;
        const uint8_t h[3] = {0xBB, 0x10, 200};
        std::vector<uint8_t> s = {h[0], h[1], h[2], lps::apChecksum(h, 3)};   // checksum'i TUTAN sahte header
        s.insert(s.end(), KAPAT.begin(), KAPAT.end());
        r.p.feed(s.data(), s.size());
        CHECK(r.poweroffs() == 0, "200 B yuk bekleniyor, KAPAT henuz islenmedi");
        r.p.idle();
        CHECK(r.poweroffs() == 1, "100 ms sessizlik (idle) sonrasi KAPAT bulundu");
    }

    std::printf("arka arkaya + baska mesaj\n");
    {
        Rec r; std::vector<uint8_t> s;
        const uint8_t h[3] = {0xBB, 0x50, 2}; const uint8_t pl[2] = {7, 9};
        s.insert(s.end(), {h[0], h[1], h[2], lps::apChecksum(h, 3), pl[0], pl[1], lps::apChecksum(pl, 2)});
        s.insert(s.end(), KAPAT.begin(), KAPAT.end());
        s.insert(s.end(), KAPAT.begin(), KAPAT.end());
        r.p.feed(s.data(), s.size());
        CHECK(r.got.size() == 3 && r.got[0].first == 0x50 && r.poweroffs() == 2, "0x50 + 2x KAPAT ayri ayri");
    }

    std::printf("rastgele gurultu (20 MB) -> yanlis KAPAT yok\n");
    {
        Rec r; std::mt19937 rng(12345); std::uniform_int_distribution<int> d(0, 255);
        std::vector<uint8_t> chunk(4096);
        for (int k = 0; k < 5000; ++k) {
            for (auto &b : chunk) b = (uint8_t)d(rng);
            r.p.feed(chunk.data(), chunk.size());
            if (k % 50 == 0) r.p.idle();
        }
        r.p.idle();
        std::printf("        rastgele gecerli cerceve: %zu (header bozuk %llu, yuk bozuk %llu)\n",
                    r.got.size(), (unsigned long long)r.p.badHeader(), (unsigned long long)r.p.badPayload());
        CHECK(r.poweroffs() == 0, "20 MB gurultude KAPAT cikmadi");
        r.p.feed(KAPAT.data(), KAPAT.size());
        CHECK(r.poweroffs() == 1, "gurultunun hemen ardindan gercek KAPAT yakalandi");
    }

    std::printf(g_fail ? "\nSONUC: %d HATA\n" : "\nSONUC: hepsi gecti\n", g_fail);
    return g_fail ? 1 : 0;
}
