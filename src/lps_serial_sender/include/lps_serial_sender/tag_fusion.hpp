/**
 * tag_fusion.hpp — AprilTag tespitleri -> tek inis noktasi (avgPos)
 *
 * serial_sender_node (infra1, seriale giden) ve infra2_recorder_node (test
 * videosu, seriale GITMEZ) AYNI hesabi yapsin diye ortak basliga alindi.
 * Iki node'da ayri kopya olsaydi mod/offset degisikligi birinde unutulur,
 * iki video farkli seyi olcerdi. ROS bagimsizdir (toTagDets sablonu mesaj
 * tipine dokunmadan alan adlariyla calisir).
 *
 * Bit-bit esdegerlik: test/test_tag_fusion.cpp, serial_sender_node'daki
 * ESKI tagCb mantigini referans alip iki modu da rastgele girdilerle kiyaslar.
 */
#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

/* ------------------------------------------------------------------ */
/* Inis modu secimi (DERLEME ZAMANI) — iki node'u BIRLIKTE etkiler     */
/*   0 = HUNI modu  : avgPos = tum huni tag'lerinin outlier'siz        */
/*                    ORTALAMASI                                        */
/*   1 = 45/24 modu : iki-tag DUZ TAG test inisi                       */
/*                    (avgPos = 45 offset'li / 24 olcekli)             */
/* Modu degistirmek icin SADECE bu satirdaki 0/1'i degistir + build.  */
/* NOT: 45/24 modunda node size 0.245 olmali (tag 45=245mm);         */
/*      huni modunda 0.26 (duvar tag) — launch APRILTAG_DETECT['size']. */
/* ------------------------------------------------------------------ */
#define TEST_APRITLAG 0

namespace lps {

/** Tek tag tespiti — ROS mesajindan bagimsiz ara temsil. */
struct TagDet {
    int       id;
    cv::Vec3d p;   // pozisyon (m, kamera optik frame'i, node size ile PnP)
    cv::Vec4d q;   // oryantasyon quaternion (x, y, z, w)
};

/** Fuzyon sonucu — apriltagPack_t'ye birebir eslenir. */
struct TagFusion {
    bool      valid = false;
    cv::Vec3f pos{0.0f, 0.0f, 0.0f};             // m
    float     roll = 0, pitch = 0, yaw = 0;       // DERECE (sadece 45/24 modu)
    uint8_t   id_bits = 0;   // huni modu inlier bitleri: bit0=valid1 ... bit7=valid8
};

/** Huni tag meta: tag-frame offset + gercek boyut (PnP size scaling). */
struct TagMeta {
    cv::Vec3f offset;          // tag-frame'de huni merkezine ofset
    float     physical_size_m; // gercek tag boyutu (PnP scale)
};

/* Quaternion -> 3x3 rotasyon (R_kamera<-tag): offset_kamera = R*offset_tagframe.
   Montaj egimi + drone attitude birlikte gelir (canli olculur). */
inline cv::Matx33f quatToMatrix(double qx, double qy, double qz, double qw)
{
    double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n < 1e-9) return cv::Matx33f::eye();
    qx/=n; qy/=n; qz/=n; qw/=n;
    return cv::Matx33f(
        (float)(1-2*(qy*qy+qz*qz)), (float)(2*(qx*qy-qz*qw)),   (float)(2*(qx*qz+qy*qw)),
        (float)(2*(qx*qy+qz*qw)),   (float)(1-2*(qx*qx+qz*qz)), (float)(2*(qy*qz-qx*qw)),
        (float)(2*(qx*qz-qy*qw)),   (float)(2*(qy*qz+qx*qw)),   (float)(1-2*(qx*qx+qy*qy)));
}

/* Quaternion -> Roll/Pitch/Yaw (ZYX, radyan) */
inline void quatToRPY(double qx, double qy, double qz, double qw,
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

/** tag25h9 HUNI offset'leri. Tag frame (libapriltag): x sag, y asagi, z duvara.
 *
 *  Duvar 0/20/24/34 — siyah kare 260 mm (05_11 kalibrasyonu):
 *    Cizim OKAD-MBI-P4a-101 REV02: huni yari aci 44.9, kucuk cap 310.4,
 *    tag merkezi eksenden 290.0 mm, dip duzleminden 135.2 mm yukarida.
 *    Referans nokta = huni DIBI (kucuk cember) merkezi. Alt kenarlar dibe bakar.
 *    Ustten bakista 410x410 kare koselerinde: 0 sag-ust, 34 sag-alt,
 *    24 sol-alt, 20 sol-ust ("Kuzey/Dogu/..." sadece isim).
 *    Baslangic: cizimden (Fable 5.1) = (0, +0.3013, -0.1092).
 *    s0110/s0111 elle ortalanmis IR kayitlari (infra1+infra2, 139 es-zamanli
 *    duvar tag cifti, gercek cpu_apriltag_node):
 *      - tag merkezi-eksen yaricapi 281 mm (cizim 290); her kayit x kamera
 *        alt grubunda 280-281, fx'e duyarsiz.
 *      - koni yari acisi 44.83 (cizim 44.9) -> huni cizimdeki gibi bukulmus.
 *      - tag 26 cm kesin (kullanici).
 *    Tag'ler cizimdeki yerinde: alt kenar ortasi-huni dibi ~6 cm olculdu
 *    (cizim 60.9). Olculen yaricapin 290 degil ~280 cikmasi PnP yanliligi:
 *    tag yuzeye SARILI (~410 mm egrilik, kenarlar ~2 cm kalkik), node duz
 *    kare varsayiyor. Ray-cast simulasyonu (cizim geometrisi + gercek node)
 *    farkin yarisini aciklar (~1 m'de 285).
 *    KESIN DEGER: 05_11 canli kalibrasyon, tam cozunurluk, gercek intrinsics
 *    (fx 647.5). Veri: 570 es-zamanli duvar tag cifti, infra1+infra2,
 *    0.85-1.3 m. Sonuc: yaricap 280.3; kamera ve mesafe bandi basina
 *    279-281. Cift farki medyan 7.3 mm (Fable ofsetiyle 20.3).
 *    Tek tek tag'lerde sistematik sapma <=3 mm -> tag basi duzeltme yok.
 *    Eksen boyu (~137 mm, referans huni dibi) cizimden: duvar ciftlerinden
 *    gozlenemez. Kavis z'yi mesafeyle hafif kaydirir (sim: 3 m'de ~3 cm).
 *  Kucuk 10/12/14/16 (size 0.077): ESKI LPS kalibrasyonu (26_08_20, dip =
 *    kucuk tag centroid anchor). 05_11 kayitlarinda takili degildi;
 *    yeni duvar ofsetleriyle ayni noktayi gosterdigi DOGRULANMADI. */
inline const std::map<int, TagMeta> &funnelTagMeta()
{
    static const std::map<int, TagMeta> m = {
        { 0, {cv::Vec3f(0.0f, +0.2942f, -0.1021f), 0.26f}},   // Kuzey (cizim sag-ust)
        {34, {cv::Vec3f(0.0f, +0.2942f, -0.1021f), 0.26f}},   // Dogu  (cizim sag-alt)
        {24, {cv::Vec3f(0.0f, +0.2942f, -0.1021f), 0.26f}},   // Guney (cizim sol-alt)
        {20, {cv::Vec3f(0.0f, +0.2942f, -0.1021f), 0.26f}},   // Bati  (cizim sol-ust)
        {10, {cv::Vec3f(+0.0429f, +0.0430f, -0.0010f), 0.077f}},  // sol-ust
        {12, {cv::Vec3f(-0.0435f, +0.0432f, +0.0045f), 0.077f}},  // sag-ust
        {14, {cv::Vec3f(+0.0432f, -0.0427f, -0.0095f), 0.077f}},  // sol-alt
        {16, {cv::Vec3f(-0.0434f, -0.0428f, +0.0010f), 0.077f}},  // sag-alt
    };
    return m;
}

/** ===== 45/24 IKI-TAG DUZ TAG TEST INISI =====
 *  avgPos = 45 (buyuk, offset'li) ya da 24 (kucuk, olcekli). Huni YOK.
 *  45 = buyuk (245mm), 24 = kucuk (95mm). Node size 0.245 olmali.
 *  validity bit'leri (valid1-8) bu modda basilmaz; sadece avgPos.
 *  Mesajdaki SIRAYLA ilk 45 ya da 24 kazanir. */
inline TagFusion fuse4524(const std::vector<TagDet> &dets, float node_size)
{
    TagFusion f;
    for (const auto &d : dets) {
        if (d.id != 45 && d.id != 24) continue;   // hedef DEGILSE atla
        f.valid = true;
        if (d.id == 45) {
            // Buyuk tag: pozisyonu kucuk tag merkezine kaydir. Offset TAG
            // FRAME'inde (0,-0.215,0 = 21.5cm) ve R ile kamera frame'ine
            // cevrilir -> her drone acisinda dogru (funnel mantigi). Offset
            // tag'in dik/hizali yapistirildigini varsayar.
            cv::Matx33f R = quatToMatrix(d.q[0], d.q[1], d.q[2], d.q[3]);
            cv::Vec3f off = R * cv::Vec3f(0.0f, -0.215f, 0.0f);
            f.pos[0] = (float)d.p[0] + off[0];
            f.pos[1] = (float)d.p[1] + off[1];
            f.pos[2] = (float)d.p[2] + off[2];
        } else { // d.id == 24
            // Kucuk tag: node size'a gore PnP olceklendi, gercek 0.095 -> duzelt.
            float scale = 0.095f / node_size;
            f.pos[0] = (float)d.p[0] * scale;
            f.pos[1] = (float)d.p[1] * scale;
            f.pos[2] = (float)d.p[2] * scale;
        }
        float roll, pitch, yaw;
        quatToRPY(d.q[0], d.q[1], d.q[2], d.q[3], roll, pitch, yaw);
        constexpr float R2D = (float)(180.0 / M_PI);   // firmware'e DERECE
        f.roll  = roll  * R2D;
        f.pitch = pitch * R2D;
        f.yaw   = yaw   * R2D;
        break;
    }
    return f;
}

/** ===== HUNI MODU =====
 *  Her huni tag'i: est = tag_pos*scale + R*offset (R = tag oryantasyonu,
 *  montaj egimi + drone attitude'u tasir). tag_meta'da olmayan ID atlanir.
 *  Median tabanli outlier reddi + ortalama. Inlier tag'lerin validity
 *  bitleri id_bits'e basilir; aci alanlari 0 kalir (huni ortalamasinin
 *  tek dogal yonu yok). */
inline TagFusion fuseFunnel(const std::vector<TagDet> &dets, float node_size)
{
    TagFusion f;
    const auto &meta_map = funnelTagMeta();

    std::vector<cv::Vec3f> estimates;
    std::vector<int>       used_ids;
    for (const auto &d : dets) {
        auto it = meta_map.find(d.id);
        if (it == meta_map.end()) continue;
        const auto &meta = it->second;
        float scale = meta.physical_size_m / node_size;
        cv::Vec3f tag_pos((float)d.p[0]*scale, (float)d.p[1]*scale, (float)d.p[2]*scale);
        cv::Matx33f R = quatToMatrix(d.q[0], d.q[1], d.q[2], d.q[3]);
        cv::Vec3f   est = tag_pos + R * meta.offset;
        estimates.push_back(est);
        used_ids.push_back(d.id);
    }

    cv::Vec3f funnel(0, 0, 0);
    int n_inliers = 0;
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
            if (!used) continue;
            sum += estimates[i]; n_inliers++;
            // Sabit ID -> bit (outlier = median'dan >0.4m, bit basmaz):
            //   valid1=0(Kuzey) valid2=20(Bati) valid3=24(Guney) valid4=34(Dogu)  [duvar]
            //   valid5=10   valid6=12    valid7=14    valid8=16      [kucuk 2x2]
            switch (used_ids[i]) {
                case  0: f.id_bits |= 1u << 0; break;
                case 20: f.id_bits |= 1u << 1; break;
                case 24: f.id_bits |= 1u << 2; break;
                case 34: f.id_bits |= 1u << 3; break;
                case 10: f.id_bits |= 1u << 4; break;
                case 12: f.id_bits |= 1u << 5; break;
                case 14: f.id_bits |= 1u << 6; break;
                case 16: f.id_bits |= 1u << 7; break;
                default: break;
            }
        }
        if (n_inliers > 0) funnel = sum * (1.0f/n_inliers);
    }
    f.valid = (n_inliers > 0);
    f.pos   = funnel;
    return f;
}

/** Derleme-zamani moda gore dogru fuzyonu secer (TEST_APRITLAG). */
inline TagFusion fuseTags(const std::vector<TagDet> &dets, float node_size)
{
#if TEST_APRITLAG
    return fuse4524(dets, node_size);
#else
    return fuseFunnel(dets, node_size);
#endif
}

/** AprilTagDetectionArray -> TagDet listesi. Sablon: basligin ROS mesaj
 *  tipine bagimli olmamasi icin alan adlariyla (duck typing) calisir. */
template <class DetArrayMsg>
std::vector<TagDet> toTagDets(const DetArrayMsg &msg)
{
    std::vector<TagDet> out;
    out.reserve(msg.detections.size());
    for (const auto &det : msg.detections) {
        const auto &p = det.pose.pose.pose.position;
        const auto &q = det.pose.pose.pose.orientation;
        out.push_back({det.id, cv::Vec3d(p.x, p.y, p.z), cv::Vec4d(q.x, q.y, q.z, q.w)});
    }
    return out;
}

} // namespace lps
