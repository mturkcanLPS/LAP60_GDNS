/**
 * test_tag_fusion.cpp — tag_fusion.hpp ile ESKI serial_sender tagCb mantigi
 * arasinda BIT-BIT esdegerlik testi (iki mod da).
 *
 * Referans kod, tag_fusion.hpp'ye tasinmadan ONCEKI serial_sender_node.cpp'den
 * satir satir (script ile) kopyalandi — elle yeniden yazilmadi. Sahte mesaj
 * tipleri ROS mesajinin alan yapisini taklit eder, boylece eski govde
 * degismeden derlenir ve toTagDets sablonu da ayni anda sinanir.
 *
 * Calistirma: ros2 run lps_serial_sender test_tag_fusion
 */
#include "lps_serial_sender/tag_fusion.hpp"

#include <opencv2/core.hpp>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <vector>

namespace ref {

// --- ROS mesaj yapisini taklit eden sahte tipler ---
struct Pos  { double x = 0, y = 0, z = 0; };
struct Quat { double x = 0, y = 0, z = 0, w = 1; };
struct Pose3 { Pos position; Quat orientation; };
struct Pose2 { Pose3 pose; };
struct Pose1 { Pose2 pose; };
struct Det { int id = 0; Pose1 pose; };
struct Msg { std::vector<Det> detections; };

// --- apriltagPack_t'nin fuzyonla ilgili kismi ---
struct Validity { unsigned valid:1, valid1:1, valid2:1, valid3:1, valid4:1,
                  valid5:1, valid6:1, valid7:1, valid8:1; };
struct AvgPos { Validity validity{}; struct { float x=0,y=0,z=0; } pos;
                struct { float roll=0,pitch=0,yaw=0; } angle; };
struct Pack { AvgPos avgPos; };

struct TagMeta { cv::Vec3f offset; float physical_size_m; };
struct PerTagLanding { int id; cv::Vec3f raw; cv::Vec3f landing; bool used; };

// ===== ESKI serial_sender_node.cpp: quatToMatrix / quatToRPY (verbatim) =====
static cv::Matx33f quatToMatrix(double qx, double qy, double qz, double qw)
{
    double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n < 1e-9) return cv::Matx33f::eye();
    qx/=n; qy/=n; qz/=n; qw/=n;
    return cv::Matx33f(
        (float)(1-2*(qy*qy+qz*qz)), (float)(2*(qx*qy-qz*qw)),   (float)(2*(qx*qz+qy*qw)),
        (float)(2*(qx*qy+qz*qw)),   (float)(1-2*(qx*qx+qz*qz)), (float)(2*(qy*qz-qx*qw)),
        (float)(2*(qx*qz-qy*qw)),   (float)(2*(qy*qz+qx*qw)),   (float)(1-2*(qx*qx+qy*qy)));
}

/* ------------------------------------------------------------------ */
/* Quaternion -> Roll/Pitch/Yaw (ZYX, radyan)                         */
/* ------------------------------------------------------------------ */
[[maybe_unused]] static void quatToRPY(double qx, double qy, double qz, double qw,
                      float &roll, float &pitch, float &yaw)
{
    double sinr = 2.0 * (qw * qx + qy * qz);
    double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = (float)std::atan2(sinr, cosr);

    double sinp = 2.0 * (qw * qy - qz * qx);
    pitch = (float)(std::abs(sinp) >= 1.0
                    ? std::copysign(M_PI / 2.0, sinp)
                    : std::asin(sinp));

    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = (float)std::atan2(siny, cosy);
}

struct Old {
    // Offset tablosu tag_fusion.hpp'den kopyalanir: bu test ALGORITMA
    // esdegerligini olcer; kalibrasyon (offset/boyut) degisince bozulmasin.
    const std::map<int, TagMeta> tag_meta_ = [] {
        std::map<int, TagMeta> m;
        for (const auto &kv : lps::funnelTagMeta())
            m[kv.first] = {kv.second.offset, kv.second.physical_size_m};
        return m;
    }();
    float apriltag_node_size_;

    Pack run4524(const Msg *msg) const {
        Pack ap{};
        // ===== ESKI tagCb #if TEST_APRITLAG govdesi (verbatim) =====
        for (const auto &det : msg->detections) {
            if (det.id != 45 && det.id != 24) continue;   // hedef DEGILSE atla
            const auto &p = det.pose.pose.pose;
            ap.avgPos.validity.valid = true;
            if (det.id == 45) {
                // Buyuk tag: pozisyonu kucuk tag merkezine kaydir. Offset TAG
                // FRAME'inde (0,-0.215,0 = 21.5cm) ve R ile kamera frame'ine
                // cevrilir -> her drone acisinda dogru (funnel mantigi). Offset
                // tag'in dik/hizali yapistirildigini varsayar.
                const auto &q = p.orientation;
                cv::Matx33f R = quatToMatrix(q.x, q.y, q.z, q.w);
                cv::Vec3f off = R * cv::Vec3f(0.0f, -0.215f, 0.0f);
                ap.avgPos.pos.x = (float)p.position.x + off[0];
                ap.avgPos.pos.y = (float)p.position.y + off[1];
                ap.avgPos.pos.z = (float)p.position.z + off[2];
            } else { // det.id == 24
                // Kucuk tag: node size'a gore PnP olceklendi, gercek 0.095 -> duzelt.
                float scale = 0.095f / apriltag_node_size_;
                ap.avgPos.pos.x = (float)p.position.x * scale;
                ap.avgPos.pos.y = (float)p.position.y * scale;
                ap.avgPos.pos.z = (float)p.position.z * scale;
            }
            float roll, pitch, yaw;
            quatToRPY(p.orientation.x, p.orientation.y,
                      p.orientation.z, p.orientation.w, roll, pitch, yaw);
            constexpr float R2D = (float)(180.0 / M_PI);   // firmware'e DERECE
            ap.avgPos.angle.roll  = roll  * R2D;
            ap.avgPos.angle.pitch = pitch * R2D;
            ap.avgPos.angle.yaw   = yaw   * R2D;
            break;
        }
        return ap;
    }

    Pack runFunnel(const Msg *msg) const {
        Pack ap{};
        // ===== ESKI tagCb #else (huni) govdesi (verbatim) =====
        std::vector<cv::Vec3f> estimates, raws;
        std::vector<int>       used_ids;
        for (const auto &det : msg->detections) {
            auto it = tag_meta_.find(det.id);
            if (it == tag_meta_.end()) continue;
            const auto &meta = it->second;
            float scale = meta.physical_size_m / apriltag_node_size_;
            const auto &p = det.pose.pose.pose.position;
            const auto &q = det.pose.pose.pose.orientation;
            cv::Vec3f tag_pos((float)p.x*scale, (float)p.y*scale, (float)p.z*scale);
            cv::Matx33f R = quatToMatrix(q.x, q.y, q.z, q.w);
            cv::Vec3f   est = tag_pos + R * meta.offset;
            raws.push_back(tag_pos);
            estimates.push_back(est);
            used_ids.push_back(det.id);
        }

        // Median tabanli outlier reddi + ortalama (huni merkezi fuzyonu)
        cv::Vec3f funnel(0, 0, 0);
        int n_inliers = 0;
        std::vector<PerTagLanding> per_tag;
        per_tag.reserve(estimates.size());
        if (!estimates.empty()) {
            std::vector<float> xs, ys, zs;
            for (const auto &e : estimates) { xs.push_back(e[0]); ys.push_back(e[1]); zs.push_back(e[2]); }
            auto med = [](std::vector<float> &v){
                std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
                return v[v.size()/2]; };
            cv::Vec3f median(med(xs), med(ys), med(zs));
            constexpr float OUTLIER_M = 0.4f;
            cv::Vec3f sum(0, 0, 0);
            for (size_t i = 0; i < estimates.size(); ++i) {
                bool used = (cv::norm(estimates[i]-median) <= OUTLIER_M);
                per_tag.push_back({used_ids[i], raws[i], estimates[i], used});
                if (used) { sum += estimates[i]; n_inliers++; }
            }
            if (n_inliers > 0) funnel = sum * (1.0f/n_inliers);
        }

        // POZISYON sadece avgPos'ta gider; tek tek tag pozlari GONDERILMIYOR -
        // her tag'in yalnizca VALIDITY bit'i basiliyor.
        ap.avgPos.validity.valid = (n_inliers > 0);
        ap.avgPos.pos.x = funnel[0];
        ap.avgPos.pos.y = funnel[1];
        ap.avgPos.pos.z = funnel[2];
        // avgPos.angle: huni ortalamasinin tek dogal yonu yok -> 0 birakilir.

        // Her INLIER tag'in validity bit'i (pozisyon YOK). Sabit ID -> bit:
        //   valid1=0(N) valid2=20(E) valid3=24(S) valid4=34(W)  [duvar]
        //   valid5=10   valid6=12    valid7=14    valid8=16      [kucuk 2x2]
        // Outlier (median'dan >0.4m = muhtemelen false positive) bit basmaz.
        for (const auto &pt : per_tag) {
            if (!pt.used) continue;
            switch (pt.id) {
                case  0: ap.avgPos.validity.valid1 = 1; break;
                case 20: ap.avgPos.validity.valid2 = 1; break;
                case 24: ap.avgPos.validity.valid3 = 1; break;
                case 34: ap.avgPos.validity.valid4 = 1; break;
                case 10: ap.avgPos.validity.valid5 = 1; break;
                case 12: ap.avgPos.validity.valid6 = 1; break;
                case 14: ap.avgPos.validity.valid7 = 1; break;
                case 16: ap.avgPos.validity.valid8 = 1; break;
                default: break;   // tag_meta disi ID - bit yok
            }
        }
        return ap;
    }
};

} // namespace ref

static int failures = 0;

static bool sameFloat(float a, float b) { return std::memcmp(&a, &b, sizeof a) == 0; }

static bool compare(const char *mode, int trial, const ref::Pack &o, const lps::TagFusion &n)
{
    const auto &v = o.avgPos.validity;
    uint8_t obits = (v.valid1<<0)|(v.valid2<<1)|(v.valid3<<2)|(v.valid4<<3)|
                    (v.valid5<<4)|(v.valid6<<5)|(v.valid7<<6)|(v.valid8<<7);
    bool ok = (bool)v.valid == n.valid && obits == n.id_bits &&
              sameFloat(o.avgPos.pos.x, n.pos[0]) &&
              sameFloat(o.avgPos.pos.y, n.pos[1]) &&
              sameFloat(o.avgPos.pos.z, n.pos[2]) &&
              sameFloat(o.avgPos.angle.roll,  n.roll) &&
              sameFloat(o.avgPos.angle.pitch, n.pitch) &&
              sameFloat(o.avgPos.angle.yaw,   n.yaw);
    if (!ok && failures < 5) {
        std::printf("  FARK [%s #%d] eski: v=%d bits=%02x pos=(%.9g %.9g %.9g) rpy=(%.9g %.9g %.9g)\n"
                    "                yeni: v=%d bits=%02x pos=(%.9g %.9g %.9g) rpy=(%.9g %.9g %.9g)\n",
                    mode, trial, (int)v.valid, obits,
                    o.avgPos.pos.x, o.avgPos.pos.y, o.avgPos.pos.z,
                    o.avgPos.angle.roll, o.avgPos.angle.pitch, o.avgPos.angle.yaw,
                    (int)n.valid, n.id_bits, n.pos[0], n.pos[1], n.pos[2],
                    n.roll, n.pitch, n.yaw);
    }
    if (!ok) ++failures;
    return ok;
}

int main()
{
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    // Huni ID'leri + 45/24 + alakasiz ID'ler (atlanmali)
    const int ids[] = {0, 20, 24, 34, 10, 12, 14, 16, 45, 24, 45, 5, 99, 36};
    std::uniform_int_distribution<int> pick(0, (int)(sizeof(ids)/sizeof(ids[0])) - 1);
    std::uniform_int_distribution<int> count(0, 9);

    int eq4524 = 0, eqFun = 0, nonEmptyFun = 0, validFun = 0, valid4524 = 0;
    const int N = 20000;
    for (int t = 0; t < N; ++t) {
        ref::Msg msg;
        const int n = count(rng);
        // Ortak bir huni noktasi etrafinda + ara sira outlier (median reddini sinar)
        const double cx = U(rng) * 2, cy = U(rng) * 2, cz = 2.0 + std::abs(U(rng)) * 6;
        for (int i = 0; i < n; ++i) {
            ref::Det d;
            d.id = ids[pick(rng)];
            const bool outlier = (U(rng) > 0.8);
            d.pose.pose.pose.position.x = cx + U(rng) * (outlier ? 3.0 : 0.05);
            d.pose.pose.pose.position.y = cy + U(rng) * (outlier ? 3.0 : 0.05);
            d.pose.pose.pose.position.z = cz + U(rng) * (outlier ? 3.0 : 0.05);
            // Rastgele (normalize EDILMEMIS) quaternion — normalizasyon yolunu da sinar
            d.pose.pose.pose.orientation.x = U(rng);
            d.pose.pose.pose.orientation.y = U(rng);
            d.pose.pose.pose.orientation.z = U(rng);
            d.pose.pose.pose.orientation.w = U(rng);
            msg.detections.push_back(d);
        }
        const float node_size = (t % 2) ? 0.25f : 0.245f;
        // Aggregate-init KULLANMA: {} verilirse tag_meta_'nin varsayilan
        // (huni) haritasi bos haritayla ezilir. Varsayilan kur, sonra ata.
        ref::Old old;
        old.apriltag_node_size_ = node_size;
        const auto dets = lps::toTagDets(msg);

        const ref::Pack o1 = old.run4524(&msg);
        if (compare("45/24", t, o1, lps::fuse4524(dets, node_size))) ++eq4524;
        if (o1.avgPos.validity.valid) ++valid4524;

        const ref::Pack o2 = old.runFunnel(&msg);
        if (compare("huni", t, o2, lps::fuseFunnel(dets, node_size))) ++eqFun;
        if (!msg.detections.empty()) ++nonEmptyFun;
        if (o2.avgPos.validity.valid) ++validFun;
    }

    std::printf("\n=== tag_fusion esdegerlik (%d rastgele girdi, bit-bit) ===\n", N);
    std::printf("  45/24 modu : %d/%d ayni   (gecerli tespitli girdi: %d)\n", eq4524, N, valid4524);
    std::printf("  huni modu  : %d/%d ayni   (gecerli tespitli girdi: %d)\n", eqFun, N, validFun);
    std::printf("  aktif mod (TEST_APRITLAG) = %d\n", TEST_APRITLAG);
    std::printf("\n=== %s ===\n\n", failures == 0 ? "TUM TESTLER GECTI" : "ESDEGERLIK BOZUK");
    return failures == 0 ? 0 : 1;
}
