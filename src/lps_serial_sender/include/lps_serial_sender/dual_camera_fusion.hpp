/**
 * dual_camera_fusion.hpp — infra1 + infra2 birlesimi (03_11): AprilTag avgPos + optik akis.
 *
 * Iki IR kamera ayni RealSense modulunde, rektifiye: eksenleri PARALEL, K ayni, aralarinda
 * sadece x'te ~50 mm. Bu yuzden:
 *   - Tag: her kameranin konumundan KENDI ofseti (kameradan platform merkezine vektor,
 *     kamera ekseninde) dusulunce iki tahmin ayni noktayi verir -> ortalama. Ofset kamera
 *     ekseninde sabit (kamera govdeye rijit): yaw/egim donusu onu BOZMAZ; dunya eksenine
 *     cevirmek otopilotun isi (bkz. autopilot-owns-frame-and-tilt).
 *     Ucus kaydi dogrulamasi (2114 es zamanli olcum): ham fark x +45 mm, ofsetler
 *     dusulunce -6 mm.
 *   - Akis: iki kamera ayni hareketi gorur (kayitta isaret uyumu %89, oran 0.93-0.99,
 *     fark medyani 0.11 px) -> kaliteyle agirlikli ortalama.
 * ROS'tan bagimsiz; test/test_dual_fusion.cpp ile sinanir.
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace lps {

struct TagSample {
    bool     ok = false;
    float    x = 0, y = 0, z = 0;           // m, platform merkezine gore (ofset dusulmus)
    float    roll = 0, pitch = 0, yaw = 0;  // derece (45/24 modu; huni modunda 0)
    uint8_t  bits = 0;                      // huni inlier bitleri (valid1..8)
};

struct TagPairResult {
    TagSample out;
    int       src = 0;        // 0 yok, 1 infra1, 2 infra2, 3 ikisi
    float     diff_m = -1;    // ikisi de gecerliyse aralarindaki fark
};

/** 359 ile 1 derecenin ortalamasi 0 olmali (180 degil). */
inline float circMeanDeg(float a, float b)
{
    const double r = M_PI / 180.0;
    return (float)(std::atan2(std::sin(a * r) + std::sin(b * r),
                              std::cos(a * r) + std::cos(b * r)) / r);
}

/** Ikisi de gecerli ve tol_m icinde tutarliysa ortalama (bitler OR); tutarsizsa infra1
 *  (birincil kamera; IR ucgeni de orada). Biri gecerliyse o. */
inline TagPairResult fuseTagPair(const TagSample &a, const TagSample &b, float tol_m)
{
    TagPairResult r;
    if (a.ok && b.ok) {
        r.diff_m = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                             (a.z - b.z) * (a.z - b.z));
        if (r.diff_m <= tol_m) {
            r.out.ok = true;
            r.out.x = 0.5f * (a.x + b.x); r.out.y = 0.5f * (a.y + b.y); r.out.z = 0.5f * (a.z + b.z);
            r.out.roll  = circMeanDeg(a.roll, b.roll);
            r.out.pitch = circMeanDeg(a.pitch, b.pitch);
            r.out.yaw   = circMeanDeg(a.yaw, b.yaw);
            r.out.bits  = a.bits | b.bits;
            r.src = 3;
        } else {
            r.out = a; r.src = 1;
        }
    } else if (a.ok) { r.out = a; r.src = 1; }
    else if (b.ok)   { r.out = b; r.src = 2; }
    return r;
}

struct FlowSample {
    bool    ok = false;
    float   dx = 0, dy = 0;   // px/kare (EMA'li)
    uint8_t q = 0;            // 0-100
};

struct FlowPairResult {
    FlowSample out;
    int        src = 0;       // 0 yok, 1 infra1, 2 infra2, 3 ikisi
};

/** Ikisi de gecerliyse kaliteyle agirlikli ortalama; fark max(agree_px, agree_frac*|v|)'yi
 *  asarsa (bir kamera dokusuz/hatali zemin gormus olabilir) kalitesi yuksek olan.
 *  Birlesik kalite = buyuk olan. */
inline FlowPairResult fuseFlowPair(const FlowSample &a, const FlowSample &b,
                                   float agree_px, float agree_frac)
{
    FlowPairResult r;
    const bool okA = a.ok && a.q > 0, okB = b.ok && b.q > 0;
    if (okA && okB) {
        const float d = std::hypot(a.dx - b.dx, a.dy - b.dy);
        const float v = std::fmax(std::hypot(a.dx, a.dy), std::hypot(b.dx, b.dy));
        if (d <= std::fmax(agree_px, agree_frac * v)) {
            const float wa = a.q, wb = b.q;
            r.out.ok = true;
            r.out.dx = (wa * a.dx + wb * b.dx) / (wa + wb);
            r.out.dy = (wa * a.dy + wb * b.dy) / (wa + wb);
            r.out.q  = a.q > b.q ? a.q : b.q;
            r.src = 3;
        } else if (a.q >= b.q) { r.out = a; r.src = 1; }
        else                   { r.out = b; r.src = 2; }
    } else if (okA) { r.out = a; r.src = 1; }
    else if (okB)   { r.out = b; r.src = 2; }
    return r;
}

}  // namespace lps
