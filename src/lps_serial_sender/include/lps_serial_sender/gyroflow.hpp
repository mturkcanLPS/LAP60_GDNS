// gyroflow.hpp - RealSense optic flow donme telafisi (gyroflow), tek basliklik C++17
// Kaynak: OKAD_Analiz/RealSense_Gyroflow/jetson/gyroflow.hpp (OKAD, 2026-11-16).
// LPS degisikligi (17_11): gyroEkle'de tampondan buyuk zaman sicramasinda tampon sifirlanir (asagida isaretli).
// Entegrasyon notu: OKAD_Jetson_Gyroflow.md. serial_sender'da kamera basina bir nesne (gf1 infra1,
// gf2 infra2); infra1 ve infra2 ayni D435i'nin iki gozu oldugu icin ikisi de AYNI IMU ile beslenir.
//
// Girdi : her kare cifti icin akis (dx, dy) [piksel/kare, kamera ekseni] + RealSense gyro ornekleri
// Cikti : donmesi cikarilmis akis
//   comp_dx_kare, comp_dy_kare : bugun OF paketine konan birimle ayni (piksel, nominal 1/30 s kareye
//                                normalize). OF paketinin motion.x / motion.y alanina dogrudan yazilir.
//   of_x, of_y                 : govde acisal akis [1/s]; govde hizi = yukseklik * of  (vx ileri, vy sag)
//
// Saat: kare zamani ve gyro zamani AYNI saatten gelmeli (RealSense kare/IMU metadata zaman damgasi,
// ornek: kam_t ve imu stamp_s). Saniye, double.
//
// Kullanim:
//   GyroFlow gf;                                    // varsayilan sabitler (OKAD 2026-11-16)
//   IMU geri cagrisinda : gf.gyroEkle(t_s, gx, gy);             // rad/s, RealSense IMU ekseni
//   her flow sonucunda  : GyroFlowSonuc s;
//                         auto d = gf.hesapla(t_kare, t_onceki_kare, all_dx, all_dy, s);
//                         if (d == GyroFlowDurum::Tamam) { pack.motion.x = s.comp_dx_kare; pack.motion.y = s.comp_dy_kare; }
//                         if (d == GyroFlowDurum::GyroGelmedi) -> kareyi bekletip birkac ms sonra tekrar cagir
#pragma once
#include <cmath>
#include <deque>
#include <mutex>

struct GyroFlowAyar {
    double fx       = 740.0;   // dy kanali odak / gyroflow kazanci [px/rad] (kamera x ekseni etrafinda donme)
    double fy       = -845.0;  // dx kanali [px/rad] (kamera y ekseni etrafinda donme), isaret dahil
    double olcek    = 0.85;    // oteleme olcegi: of = (akis/f - w) / olcek
    double tau      = 0.010;   // kare zamani - gyro zamani ofseti [s]
    double kareNom  = 1.0 / 30.0;  // comp_*_kare normalizasyonu [s]
    double dtMin    = 0.015;   // gecerli kare araligi [s]
    double dtMax    = 0.100;
    double tampon   = 2.0;     // gyro tampon uzunlugu [s]
};

struct GyroFlowSonuc {
    float comp_dx_kare = 0.f;  // kamera ekseni, piksel / nominal kare
    float comp_dy_kare = 0.f;
    float of_x = 0.f;          // govde ileri [1/s]
    float of_y = 0.f;          // govde sag   [1/s]
    float wx = 0.f, wy = 0.f;  // kare boyunca ortalama gyro, kamera ekseni [rad/s]
    float dt = 0.f;            // kare araligi [s]
};

enum class GyroFlowDurum { Tamam, GyroGelmedi, GyroYok, KareAraligi };

class GyroFlow {
public:
    explicit GyroFlow(const GyroFlowAyar& a = GyroFlowAyar()) : a_(a) {}

    // RealSense gyro ornegi: t [s], gx gy [rad/s] (kamera/IMU ekseni, ham; gz kullanilmaz)
    void gyroEkle(double t, float gx, float gy) {
        std::lock_guard<std::mutex> k(m_);
        // LPS 17_11: sistem saati adimi (NTP) RealSense global zamanini da sicratir. Ileri sicrayan tek bir
        // ornek yuzunden sonraki tum ornekler "geri giden" sayilip atiliyor, tampon donuyor ve dakikalarca
        // GyroYok donuyordu (s0205: +68 dk NTP adimi). Tampondan buyuk sicramada (ileri/geri) bastan baslar.
        if (!q_.empty() && std::fabs(t - q_.back().t) > a_.tampon) q_.clear();
        if (!q_.empty()) {
            const Ornek& s = q_.back();
            if (t <= s.t) return;                       // geri giden / tekrar eden ornek
            const double h = t - s.t;
            q_.push_back({t, s.ax + 0.5 * (s.gx + gx) * h, s.ay + 0.5 * (s.gy + gy) * h, gx, gy});
        } else {
            q_.push_back({t, 0.0, 0.0, gx, gy});
        }
        while (q_.size() > 2 && q_.back().t - q_.front().t > a_.tampon) q_.pop_front();
    }

    // tK: bu kare, tO: akisin hesaplandigi onceki kare [s]; dx dy [piksel/kare] (ornek: all_dx, all_dy)
    GyroFlowDurum hesapla(double tK, double tO, float dx, float dy, GyroFlowSonuc& s) const {
        const double dt = tK - tO;
        if (!(dt >= a_.dtMin && dt <= a_.dtMax)) return GyroFlowDurum::KareAraligi;
        double ax1, ay1, ax0, ay0;
        {
            std::lock_guard<std::mutex> k(m_);
            if (q_.size() < 2) return GyroFlowDurum::GyroYok;
            const double t1 = tK - a_.tau, t0 = tO - a_.tau;
            if (q_.back().t < t1) return GyroFlowDurum::GyroGelmedi;
            if (q_.front().t > t0) return GyroFlowDurum::GyroYok;
            aci(t1, ax1, ay1);
            aci(t0, ax0, ay0);
        }
        const double dthx = ax1 - ax0, dthy = ay1 - ay0;     // kare boyunca donme [rad]
        const double cdy = dy - a_.fx * dthx;                // donmesi cikarilmis akis [piksel]
        const double cdx = dx - a_.fy * dthy;
        s.dt = (float)dt;
        s.wx = (float)(dthx / dt);
        s.wy = (float)(dthy / dt);
        s.comp_dx_kare = (float)(cdx * a_.kareNom / dt);
        s.comp_dy_kare = (float)(cdy * a_.kareNom / dt);
        s.of_x = (float)(cdy / (a_.olcek * a_.fx * dt));
        s.of_y = (float)(cdx / (a_.olcek * a_.fy * dt));
        return GyroFlowDurum::Tamam;
    }

    const GyroFlowAyar& ayar() const { return a_; }

private:
    struct Ornek { double t, ax, ay; float gx, gy; };   // ax ay: gyro x/y integrali [rad]
    GyroFlowAyar a_;
    std::deque<Ornek> q_;
    mutable std::mutex m_;

    // t anindaki birikimli aci (dogrusal ara deger); cagiran kilidi tutar, t tampon icinde
    void aci(double t, double& ax, double& ay) const {
        size_t lo = 0, hi = q_.size() - 1;
        while (hi - lo > 1) {
            const size_t md = (lo + hi) / 2;
            if (q_[md].t <= t) lo = md; else hi = md;
        }
        const Ornek& a = q_[lo];
        const Ornek& b = q_[hi];
        const double u = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
        ax = a.ax + u * (b.ax - a.ax);
        ay = a.ay + u * (b.ay - a.ay);
    }
};
