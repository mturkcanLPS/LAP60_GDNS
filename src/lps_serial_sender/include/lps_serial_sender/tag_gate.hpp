/**
 * tag_gate.hpp — AprilTag gecerlilik kapisi (kamera basina, fuzyondan ONCE).
 *
 * Neden (08_11, otopilot logu LAP60_261107_1754_17 + kayit s0133): ~10 m'de tag 1-2 kare
 * gorulup kayboluyordu (menzil siniri). Otopilot tag gorunce arac olcumunu 1 s EKF'ye
 * almiyor; tag bir daha gelmeyince konum kaybi sayip inisi ASKIYA aliyordu (iki iniste de,
 * son tag gorusunden tam 1.0 s sonra). Bu kisa gorusler otopilota hic gecerli gitmesin.
 *
 * Kural:
 *   GIRIS : min_consec ardisik tespit (aralarinda tespitsiz mesaj yok, ve ardisik iki
 *           tespit arasi <= max_gap_s) -> gecerli.
 *   KALMA : gecerliyken tespit kesilse de son tespitten keep_s icinde yeni tespit gelirse
 *           hemen yine gecerli (kesik kesik gorulen ama DOGRULANMIS tag'in tek kareleri de gider).
 *   DUSME : son tespitten keep_s'den fazla gecti -> yeniden min_consec ardisik gerekir.
 * Tespitsiz mesaj her zaman gecersiz gider (kapi tespit UYDURMAZ, sadece kisa gorusu bastirir).
 *
 * s0133 kaydinda (infra1, 15 Hz tag): otopilotun askiya gectigi 3 olayin 3'u de 1-2 karelik
 * gorus -> N=3 hepsini bastirir (N=2 birini kaciriyor). Bedel: giriste 2 kare (~133 ms).
 * keep_s secimi (ayni ucus, "tag gecerli -> >=1 s gecersiz" olayi = otopilotun askiya alma
 * tetigi):  kapisiz 18 olay (11'i kisa gorus sonrasi) | keep 0.3: 20 (6-8 m'de kesik kesik
 * gorulen tag'lerin kopru gorusleri de bastiriliyor, gecerli kare %53 -> %44) | keep 1.0: 6
 * olay (2'si kisa gorus), 6-8 m gecerli kare %53.2 (kayip yok). 1.0 = otopilotun 1 s penceresi.
 *
 * Zaman = kamera cekim ani (header.stamp, s). ROS bagimsiz; test: test/test_tag_gate.cpp.
 */
#pragma once

#include <cstdint>

namespace lps {

class TagGate {
public:
    TagGate(int min_consec = 3, double keep_s = 1.0, double max_gap_s = 0.25)
        : n_(min_consec < 1 ? 1 : min_consec), keep_s_(keep_s), max_gap_s_(max_gap_s) {}

    /** Her tag mesajinda cagrilir. detected = fuzyon o karede gecerli sonuc uretti mi.
     *  Donus: otopilota GECERLI gidebilir mi. */
    bool update(bool detected, double t_s)
    {
        if (!detected) {
            run_ = 0;
            if (active_ && has_det_ && t_s - last_det_s_ > keep_s_) active_ = false;
            return false;
        }
        // tespit var
        if (active_ && has_det_ && t_s - last_det_s_ > keep_s_) active_ = false;   // uzun kayiptan donus
        const bool chained = has_det_ && run_ > 0 && (t_s - last_det_s_) <= max_gap_s_;
        run_ = chained ? run_ + 1 : 1;
        last_det_s_ = t_s; has_det_ = true;
        if (!active_ && run_ >= n_) active_ = true;
        if (!active_) ++suppressed_;
        return active_;
    }

    bool     active() const { return active_; }
    uint64_t suppressed() const { return suppressed_; }   // bastirilan tespit sayisi (toplam)

private:
    int    n_;
    double keep_s_, max_gap_s_;
    int    run_ = 0;
    bool   active_ = false, has_det_ = false;
    double last_det_s_ = 0.0;
    uint64_t suppressed_ = 0;
};

}  // namespace lps
