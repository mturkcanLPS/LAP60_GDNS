// gyroflow_oto_tau.hpp - gyroflow kare-gyro zaman kaymasinin (tau) ucusta otomatik secimi (LPS, 18_11)
//
// Neden: OKAD gyroflow.hpp sabit tau = 10 ms kullaniyor. Ucus kayitlarinda en iyi tau gunden gune
// degisti: s0209 +10, s0210 +7.5 (17_11 aksam), s0216 -2.5, s0217 0 ms (18_11 gunduz). Yanlis tau ile
// gyro telafisinden sonra kalan akis gurultusu 2-3 kat (s0216: 0.80 -> 0.30 px). Muhtemel sebep IR
// pozlama suresi (kare damgasi ile pozlamanin ortasi arasindaki fark). Pozlama loglanmiyordu.
//
// Yontem: ayni IMU ile beslenen N adet GyroFlow (tau adaylari). Pakete SECILI adayin sonucu gider.
// Kaliteli her karede tum adaylarin telafili akisinin karesi (|comp|^2, tavanli) ~300 karelik EMA'ya
// eklenir; dogru tau'da donme sizintisi en az oldugu icin EMA en kucuk olur (oteleme akisi tum
// adaylarda ortak). En kucuk aday secilene gore %5'ten fazla iyiyse gecilir (histerezis).
// Kayitlarda simule edildi (tau_sim): motorlar calisinca/kalkista dogru degere oturuyor, havada
// %99-100 en iyi sabit tau'nun +-2.5 ms icinde.
// gyroflow.hpp'ye DOKUNMAZ (OKAD dosyasi).
#pragma once
#include "lps_serial_sender/gyroflow.hpp"
#include <algorithm>
#include <memory>
#include <vector>

struct GyroFlowOtoTauAyar {
    double tau_min = -0.005, tau_max = 0.015, tau_adim = 0.0025;   // [s]
    double tau_bas = 0.010;     // baslangic (OKAD varsayilani)
    double ema_n   = 300.0;     // EMA uzunlugu [kare]
    int    min_n   = 150;       // secim icin en az ogrenme karesi
    double tavan   = 25.0;      // |comp|^2 tavani [px^2] (asiri degerler istatistigi bozmasin)
    double histerezis = 0.95;   // yeni aday secilenin bu katindan kucukse gec
    bool   oto     = true;      // false: sabit tau_bas
};

class GyroFlowOtoTau {
public:
    using Ayar = GyroFlowOtoTauAyar;
    explicit GyroFlowOtoTau(const Ayar& a = Ayar(), const GyroFlowAyar& temel = GyroFlowAyar()) : a_(a) {
        for (double t = a.tau_min; t <= a.tau_max + 1e-9; t += a.tau_adim) {
            GyroFlowAyar g = temel; g.tau = t;
            tau_.push_back(t); gf_.push_back(std::make_unique<GyroFlow>(g));
        }
        ema_.assign(tau_.size(), 0.0); ni_.assign(tau_.size(), 0);
        sec_ = 0;
        for (size_t i = 1; i < tau_.size(); ++i)
            if (std::abs(tau_[i] - a.tau_bas) < std::abs(tau_[sec_] - a.tau_bas)) sec_ = i;
    }

    // IMU geri cagrisi (her aday ayni ornegi alir; GyroFlow'un kendi kilidi var)
    void gyroEkle(double t, float gx, float gy) { for (auto& g : gf_) g->gyroEkle(t, gx, gy); }

    // Pakete gidecek sonuc: secili aday. GyroGelmedi'de cagiran (eskisi gibi) bekleyip tekrar cagirir.
    GyroFlowDurum hesapla(double tK, double tO, float dx, float dy, GyroFlowSonuc& s) const {
        return gf_[sec_]->hesapla(tK, tO, dx, dy, s);
    }

    // Kaliteli kare (cagiran karar verir) icin tum adaylari ogren + gerekirse sec. Akis thread'inden.
    void ogren(double tK, double tO, float dx, float dy) {
        if (!a_.oto) return;
        const double al = 1.0 / a_.ema_n;
        for (size_t i = 0; i < gf_.size(); ++i) {
            GyroFlowSonuc s;
            if (gf_[i]->hesapla(tK, tO, dx, dy, s) != GyroFlowDurum::Tamam) continue;   // gyro henuz yok
            const double e = std::min<double>(s.comp_dx_kare * s.comp_dx_kare + s.comp_dy_kare * s.comp_dy_kare,
                                              a_.tavan);
            ema_[i] = (ni_[i] == 0) ? e : ema_[i] + al * (e - ema_[i]);
            ++ni_[i];
        }
        ++n_;
        if (n_ >= a_.min_n && n_ % 30 == 0) {
            size_t b = sec_;   // sadece yeterince ogrenmis adaylar arasinda
            for (size_t i = 0; i < ema_.size(); ++i)
                if (ni_[i] >= a_.min_n && (ni_[b] < a_.min_n || ema_[i] < ema_[b])) b = i;
            if (b != sec_ && ni_[b] >= a_.min_n && ni_[sec_] >= a_.min_n && ema_[b] < a_.histerezis * ema_[sec_]) {
                sec_ = b; ++gecis_;
            }
        }
    }

    double tau() const { return tau_[sec_]; }
    unsigned gecis() const { return gecis_; }
    long ogrenilen() const { return n_; }

private:
    Ayar a_;
    std::vector<double> tau_;
    std::vector<std::unique_ptr<GyroFlow>> gf_;
    std::vector<double> ema_;
    std::vector<long> ni_;
    size_t sec_ = 0;
    long n_ = 0;
    unsigned gecis_ = 0;
};
