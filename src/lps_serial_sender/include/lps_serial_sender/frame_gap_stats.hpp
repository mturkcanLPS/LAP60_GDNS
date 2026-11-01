/**
 * frame_gap_stats.hpp — islenen kareler arasi KAMERA araligi istatistigi
 *
 * Hz tek basina yaniltici: 30 fps kameradan ortalama 24 Hz islemek, karelerin
 * bir kisminin atlandigi ve optik akisin bazen 33 ms'lik bazen 67 ms'lik
 * hareketi olctugu anlamina gelir (dx birimi kareden kareye degisir).
 * Bu sayac, ardisik islenen iki karenin KAMERA zaman damgasi farkini kare
 * periyoduna bolup kac kamera karesi gectigini sayar:
 *   duzenli %  = araligin tam 1 kare oldugu (hic kare atlanmayan) oran
 *   atlanan %  = kameranin urettigi karelerden ISLENMEYENLERIN orani
 * Iki kamera ayni ve sabit hizda ise ikisinde de duzenli ~%100, atlanan ~%0.
 *
 * add() isleyen thread'den, take() log thread'inden cagrilir (kilitli).
 */
#pragma once

#include <cmath>
#include <mutex>

namespace lps {

class FrameGapStats
{
public:
    explicit FrameGapStats(double camera_fps)
        : period_s_(1.0 / (camera_fps > 0.0 ? camera_fps : 30.0)) {}

    void add(double stamp_s)
    {
        std::lock_guard<std::mutex> lk(m_);
        if (have_) {
            const double dt = stamp_s - last_s_;
            if (dt > 0.0 && dt < 2.0) {            // 2 s ustu = kesinti, sayma
                long n = std::lround(dt / period_s_);
                if (n < 1) n = 1;
                ++intervals_;
                if (n == 1) ++regular_;
                frames_ += n;
            }
        }
        last_s_ = stamp_s;
        have_   = true;
    }

    /** Pencereyi okuyup sifirlar. Veri yoksa ikisi de -1. */
    void take(int &regular_pct, int &skipped_pct)
    {
        std::lock_guard<std::mutex> lk(m_);
        regular_pct = intervals_ ? (int)std::lround(100.0 * regular_ / intervals_) : -1;
        skipped_pct = frames_ ? (int)std::lround(100.0 * (frames_ - intervals_) / frames_) : -1;
        intervals_ = regular_ = frames_ = 0;
    }

private:
    std::mutex m_;
    double period_s_;
    double last_s_ = 0.0;
    bool   have_ = false;
    long   intervals_ = 0, regular_ = 0, frames_ = 0;
};

} // namespace lps
