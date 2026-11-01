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
 *  (tum pikseller 255) -> ikili maske centroid'i zaten optimal, ~0.3 px. */
void detectIrBlobs(const cv::Mat &gray, double thresh,
                          double min_area, double max_area, int max_blobs,
                          cv::Mat &bin, cv::Mat &labels, cv::Mat &stats,
                          cv::Mat &centroids, std::vector<cv::Point2f> &out,
                          std::vector<int> &out_area)
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
 *            "her kenar %X icinde" diye okunur. */
bool findIrRightTriangle(const std::vector<cv::Point2f> &pts,
                                 float leg_a_m, float leg_b_m, float tol_frac,
                                 int &idx_r, int &idx_a, int &idx_b,
                                 float &out_err, float &out_k)
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


} // namespace lps
