/**
 * ir_triangle.hpp — IR LED dik ucgeni: parlak nokta tespiti + ucgen eslesmesi
 *
 * serial_sender_node ve sentetik dogrulama testi AYNI kodu kullansin diye
 * ayri basliga alindi (test/test_ir_triangle.cpp). ROS bagimsizdir.
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace lps {

/** Parlak nokta tespiti: TEK esik + connectedComponents.
 *  Kisa pozlamada LED'ler doygun (≈255), arka plan karanlik — tek esik
 *  yeterli. SimpleBlobDetector'in cok-esikli taramasi bunun ~4 kati pahali
 *  ve ucus-kritik imgCb icinde donuyor; ayrica circularity filtresi 3-5 px'lik
 *  uzak LED'lerde guvenilmez (4piA/P^2 o boyutta gurultulu) — ikisinden de
 *  kacinildi. Doygun blob'da yogunluk-agirlikli centroid ek dogruluk vermez
 *  (tum pikseller 255) -> ikili maske centroid'i zaten optimal, ~0.3 px.
 *
 *  Yuvarlaklik (06_11): round_min_area > 0 ise bu alandan BUYUK blob'larda
 *  doluluk (alan / kutu) >= min_fill ve en/boy <= max_aspect aranir. LED doygun
 *  dairesi 0.60-0.75 / ~1.0 (1.5-4 m olculdu); huni/vinil parlamasi 0.3-0.4 / 2-2.5.
 *  Kucuk blob'larda (birkac px) sekil olculemez -> filtre uygulanmaz. */
inline void detectIrBlobs(const cv::Mat &gray, double thresh,
                          double min_area, double max_area, int max_blobs,
                          cv::Mat &bin, cv::Mat &labels, cv::Mat &stats,
                          cv::Mat &centroids, std::vector<cv::Point2f> &out,
                          std::vector<int> &out_area,
                          double round_min_area = 0.0, double min_fill = 0.55,
                          double max_aspect = 1.6)
{
    out.clear();
    out_area.clear();
    cv::threshold(gray, bin, thresh, 255, cv::THRESH_BINARY);
    const int n = cv::connectedComponentsWithStats(bin, labels, stats,
                                                   centroids, 8, CV_32S);

    std::vector<std::pair<int, int>> cand;   // (alan, etiket); 0 = arka plan
    for (int i = 1; i < n; ++i) {
        const int a = stats.at<int>(i, cv::CC_STAT_AREA);
        if (a < min_area || a > max_area) continue;
        if (round_min_area > 0 && a > round_min_area) {
            const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            const double fill = (double)a / ((double)w * h);
            const double asp  = (double)std::max(w, h) / std::max(1, std::min(w, h));
            if (fill < min_fill || asp > max_aspect) continue;
        }
        cand.emplace_back(a, i);
    }
    // Ucgen aramasi O(n^3): esik dusuk/sahne parlaksa yuzlerce aday cikip
    // imgCb'yi bogar. En BUYUK max_blobs adayi tut (LED'ler en parlak/genis).
    if ((int)cand.size() > max_blobs) {
        std::nth_element(cand.begin(), cand.begin() + max_blobs, cand.end(),
                         [](const auto &l, const auto &r){ return l.first > r.first; });
        cand.resize(max_blobs);
    }
    for (const auto &c : cand) {
        out.emplace_back((float)centroids.at<double>(c.second, 0),
                         (float)centroids.at<double>(c.second, 1));
        out_area.push_back(c.first);
    }
}

/** Aday noktalar icinden bilinen dik ucgeni bulur ve MENZILI de olcer.
 *
 *  Dogrulama OLCEKTEN BAGIMSIZ: gozlenen 3 kenara tek bir olcek katsayisi k
 *  (px/m) en iyi sekilde oturtulur, sonra artik hata bakilir. Boylece Z'ye
 *  ihtiyac duymadan "bu ucgen bizimki mi" sorusu cevaplanir — ve oturan k
 *  bize Z'yi bedavaya verir: kenar_px = kenar_m * f / Z  =>  Z = f / k.
 *
 *  Bu, olcegi radardan almaktan iyi: radar 10 Hz (kamera 30 Hz -> 3 karenin
 *  2'si bayat Z kullanirdi), radar olurse IR de olurdu, ve radar DIKEY
 *  yukseklik verir — oysa pinhole geri izdusum OPTIK EKSEN derinligi ister
 *  (egimde 1/cos(theta) ayrisirlar). Optik Z tam olarak istenen buyukluk.
 *
 *  out_k   = oturan olcek (px/m); Z = f_px / out_k
 *  out_err = kenar basina ORTALAMA goreli artik -> tol_frac dogrudan
 *            "her kenar %X icinde" diye okunur.
 *
 *  k_min/k_max (06_11): irtifa biliniyorsa beklenen olcek araligi (px/m). Bunun
 *  DISINDAKI uclu SECIMDE atlanir. Eskiden en kucuk hatali uclu secilip olcek SONRA
 *  kontrol ediliyordu; parlak sahnede rastgele bir uclu gercek LED'lerden daha dusuk
 *  hata alip kazaniyor, sonra Z kontroluyle reddediliyordu -> gercek ucgen kayboluyordu. */
inline bool findIrRightTriangle(const std::vector<cv::Point2f> &pts,
                                 float leg_a_m, float leg_b_m, float tol_frac,
                                 int &idx_r, int &idx_a, int &idx_b,
                                 float &out_err, float &out_k,
                                 float k_min = 0.0f,
                                 float k_max = std::numeric_limits<float>::max())
{
    const int n = (int)pts.size();
    if (n < 3) return false;

    // Beklenen kenarlar (m), sirali — gozlemle esitlemek icin ikisi de siralanir
    float e[3] = {leg_a_m, leg_b_m,
                  std::sqrt(leg_a_m * leg_a_m + leg_b_m * leg_b_m)};
    std::sort(e, e + 3);
    const float sum_e = e[0] + e[1] + e[2];
    if (sum_e < 1e-6f) return false;

    float best_err = std::numeric_limits<float>::max();
    float best_k   = 0.0f;
    bool  found    = false;

    for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j)
    for (int k = j + 1; k < n; ++k) {
        const int ids[3] = {i, j, k};
        const cv::Point2f P[3] = {pts[i], pts[j], pts[k]};
        const float d[3] = {                  // d[m] = P[m]'nin KARSISINDAKI kenar
            (float)cv::norm(P[1] - P[2]),
            (float)cv::norm(P[0] - P[2]),
            (float)cv::norm(P[0] - P[1])
        };
        int r = 0;                            // en uzun kenarin karsisi = dik acı
        for (int m = 1; m < 3; ++m) if (d[m] > d[r]) r = m;

        float s[3] = {d[0], d[1], d[2]};
        std::sort(s, s + 3);
        const float sum_s = s[0] + s[1] + s[2];
        if (sum_s < 3.0f) continue;           // dejenere/cakisik noktalar

        const float kk = sum_s / sum_e;       // en iyi oturan olcek (px/m)
        if (kk < k_min || kk > k_max) continue;   // irtifayla tutarsiz boyut
        float err = 0.0f;
        for (int m = 0; m < 3; ++m) err += std::fabs(s[m] - kk * e[m]) / (kk * e[m]);
        err /= 3.0f;

        if (err < best_err) {
            best_err = err;
            best_k   = kk;
            idx_r = ids[r]; idx_a = ids[(r + 1) % 3]; idx_b = ids[(r + 2) % 3];
            found = true;
        }
    }
    out_err = best_err;
    out_k   = best_k;
    return found && best_err < tol_frac;
}

/** Huni merkezinin goruntudeki yeri (06_11). Ucgen ikizkenar DEGIL (R-sol 131 cm,
 *  R-sag 144.5 cm) ve hipotenus ortasi huni merkezinden ~10 cm kayik. Kullanici olcusu:
 *  merkez R'den kisa kenar boyunca on_short_m, uzun kenar boyunca on_long_m (0.655 / 0.617;
 *  R'ye ve sol LED'e 90 cm, sag LED'e 105.5 cm -> 105.56 hesap). Kenarlar goruntude kisa/uzun
 *  diye ayrilir; oran ambig_ratio'dan kucukse (egimde kisalma kenarlari karistirabilir) ayrim
 *  guvenilmez -> hipotenus ortasi (eski davranis; yanlis atama 14 cm, orta nokta 10 cm hata).
 *  Afin yaklasim: marker duzlemi kameraya gore az egik ve uzakta (>=1.5 m). */
inline cv::Point2f irMarkerCenter(const cv::Point2f &R, const cv::Point2f &a, const cv::Point2f &b,
                                  float leg_short_m, float leg_long_m,
                                  float on_short_m, float on_long_m, float ambig_ratio,
                                  bool &used_legs)
{
    const float la = (float)cv::norm(a - R), lb = (float)cv::norm(b - R);
    const float hi = std::max(la, lb), lo = std::max(1e-3f, std::min(la, lb));
    used_legs = leg_short_m > 0 && leg_long_m > leg_short_m && hi / lo >= ambig_ratio;
    if (!used_legs) return (a + b) * 0.5f;
    const cv::Point2f S = (la < lb) ? a : b;   // kisa kenarin ucu (sol LED)
    const cv::Point2f L = (la < lb) ? b : a;   // uzun kenarin ucu (sag LED)
    return R + (S - R) * (on_short_m / leg_short_m) + (L - R) * (on_long_m / leg_long_m);
}

/** Irtifaya gore beklenen LED blob alani (px, esik 240 ustu). 06_11 vincle olculdu
 *  (infra1, 1.5-4 m, optik Z): medyan 754 @1.75, 723 @2.25, 586 @2.75, 506 @3.25,
 *  478 @3.75 m -> alan ~ 750 * (1.75/Z)^0.6. Doygun halenin boyu 1/Z^2 ile DEGIL cok
 *  daha yavas kuculur (pozlama/parlaklik). 4 m ustu OLCULMEDI -> alt sinir orada
 *  uygulanmaz (cagiran taraf). */
inline double irLedAreaExpected(double z_m, double ref_area, double ref_z, double expo)
{
    if (!(z_m > 0.1)) return ref_area;
    return ref_area * std::pow(ref_z / z_m, expo);
}


} // namespace lps
