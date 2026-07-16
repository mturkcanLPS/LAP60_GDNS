/**
 * ir_projector_detector.hpp — 3 IR LED (ikizkenar dik ucgen) P3P pozisyon kestirimi
 *
 * SU AN KULLANILMIYOR — node tarafinda cagri yorumda. Header build'e dahil
 * (bit-rot olmasin diye) ama instantiate edilmiyor.
 *
 * Geometri: P1 = 90° kose, P2 = (leg,0,0), P3 = (0,leg,0), leg = 1.32m.
 * Ikizkenar oldugundan P2/P3 pixel mesafeleriyle ayirt edilemez (2-fold
 * ambiguity). Cozum: LED'lerden biri MEKANIK olarak daha sonuk yapilir
 * (default P3). Kombinatoryel eslestirme "2 parlak + 1 sonuk" area
 * profilini sart kosar; sonuk LED 90° kosede olamaz. Boylece tek P3P
 * atamasi kalir, ambiguity matematiksel olarak yok olur.
 *
 * ROS bagimliligi yok; debug gorseli cv::Mat olarak dondurulur,
 * yayinlamak cagiranin isi.
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <vector>
#include <limits>
#include <cmath>

namespace lps {

class IRProjectorDetector
{
public:
    struct Result {
        bool        valid = false;
        cv::Vec3f   cam_pos{0, 0, 0};    // kamera pozisyonu, ucgen (world) frame
        float       roll_deg  = 0.0f;    // kamera orientasyonu (ZYX Euler, derece)
        float       pitch_deg = 0.0f;
        float       yaw_deg   = 0.0f;
        cv::Point2f p1_px{0, 0};         // secilen vertex pikselleri
        cv::Point2f p2_px{0, 0};
        cv::Point2f p3_px{0, 0};
        int         areas[3] = {0,0,0};  // P1,P2,P3 blob area (tuning icin)
        uint8_t     n_blobs_seen = 0;
        uint8_t     n_blobs_used = 0;
    };

    /* --- Tuning parametreleri (dogrudan set edilebilir) --- */
    float       leg_m            = 1.320f;  // P1→P2 = P1→P3 kenar uzunlugu
    int         thresh           = 220;     // binary threshold (yuksek = parlak cekirdek)
    int         min_area         = 30;      // blob min area (px)
    int         max_area         = 30000;   // blob max area (bloom genis olabilir)
    float       max_aspect       = 1.8f;    // blob en-boy orani limiti
    float       geom_score_thr   = 0.35f;   // ucgen fit esigi (gevsek = noise toleransi)
    int         max_candidates   = 20;      // kombinatoryel n^3 icin blob limiti
    std::string dim_led_role     = "P3";    // sonuk LED koesesi: "P2" | "P3"
    float       bright_dim_ratio = 1.3f;    // parlak/sonuk area orani min esigi
    float       max_jump_m       = 0.40f;   // frameler arasi max sicrama
    int         track_reset_frames = 30;    // bu kadar invalid frame sonra track reset

    /** Kamera intrinsics — kullanmadan once gercek camera_info ile set edilmeli. */
    void setIntrinsics(double fx, double fy, double cx, double cy)
    {
        K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    /**
     * Bir gri frame'de LED ucgenini arar.
     * Donen Result.valid=false ise pozisyon kullanilmamali.
     * debug_vis != nullptr ise annotate edilmis BGR gorsel yazilir
     * (blob'lar sari, secilen ucgen yesil/mavi).
     */
    Result detect(const cv::Mat &gray, cv::Mat *debug_vis = nullptr)
    {
        Result res;
        if (gray.empty() || K_.empty()) return res;

        // 1) Threshold + morfoloji
        cv::Mat bin;
        cv::threshold(gray, bin, thresh, 255, cv::THRESH_BINARY);
        if (morph_kernel_.empty())
            morph_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
        cv::morphologyEx(bin, bin, cv::MORPH_OPEN, morph_kernel_);

        // 2) Connected components → boyut + dairesellik filtresi
        cv::Mat labels, stats, centroids;
        int n = cv::connectedComponentsWithStats(bin, labels, stats, centroids);

        std::vector<Blob> blobs;
        blobs.reserve(n);
        for (int i = 1; i < n; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < min_area || area > max_area) continue;
            int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            float ratio = (float)std::max(w, h) / (float)std::max(1, std::min(w, h));
            if (ratio > max_aspect) continue;
            blobs.push_back({
                cv::Point2f((float)centroids.at<double>(i, 0),
                            (float)centroids.at<double>(i, 1)),
                area});
        }
        res.n_blobs_seen = (uint8_t)blobs.size();

        std::sort(blobs.begin(), blobs.end(),
                  [](const Blob &a, const Blob &b){ return a.area > b.area; });

        if ((int)blobs.size() < 3) {
            onInvalid();
            if (debug_vis) drawDebug(gray, blobs, res, *debug_vis);
            return res;
        }
        if ((int)blobs.size() > max_candidates)
            blobs.resize(max_candidates);   // n^3 patlamasin

        // 3) Kombinatoryel eslestirme — ikizkenar dik ucgen + area profili
        int best_i, best_j, best_k, best_p1;
        if (!matchTriangle(blobs, best_i, best_j, best_k, best_p1)) {
            onInvalid();
            if (debug_vis) drawDebug(gray, blobs, res, *debug_vis);
            return res;
        }

        Blob p1_blob = blobs[best_p1];
        Blob other1, other2;
        bool first = true;
        for (int idx : {best_i, best_j, best_k}) {
            if (idx == best_p1) continue;
            if (first) { other1 = blobs[idx]; first = false; }
            else       { other2 = blobs[idx]; }
        }

        // 4) Sonuk LED → P2/P3 rol atamasi (matching area profilini garantiledi)
        Blob bright = (other1.area >= other2.area) ? other1 : other2;
        Blob dim    = (other1.area >= other2.area) ? other2 : other1;
        Blob p2_blob, p3_blob;
        if (dim_led_role == "P2" || dim_led_role == "p2") {
            p2_blob = dim;    p3_blob = bright;
        } else {
            p2_blob = bright; p3_blob = dim;
        }

        // 5) Tek P3P cozumu + filtreler + refinement
        cv::Vec3f cam_pos;
        float roll, pitch, yaw;
        if (!solveP3PFiltered(p1_blob.c, p2_blob.c, p3_blob.c,
                              cam_pos, roll, pitch, yaw)) {
            onInvalid();
            if (debug_vis) drawDebug(gray, blobs, res, *debug_vis);
            return res;
        }

        // 6) Jump rejection — frameler arasi buyuk sicramayi reddet
        if (has_last_) {
            cv::Vec3f d = cam_pos - last_cam_pos_;
            if (cv::norm(d) > max_jump_m) {
                onInvalid();
                if (debug_vis) drawDebug(gray, blobs, res, *debug_vis);
                return res;
            }
        }
        last_cam_pos_ = cam_pos;
        has_last_ = true;
        frames_since_valid_ = 0;

        res.valid = true;
        res.cam_pos = cam_pos;
        res.roll_deg = roll; res.pitch_deg = pitch; res.yaw_deg = yaw;
        res.p1_px = p1_blob.c; res.p2_px = p2_blob.c; res.p3_px = p3_blob.c;
        res.areas[0] = p1_blob.area;
        res.areas[1] = p2_blob.area;
        res.areas[2] = p3_blob.area;
        res.n_blobs_used = 3;

        if (debug_vis) drawDebug(gray, blobs, res, *debug_vis);
        return res;
    }

private:
    struct Blob { cv::Point2f c; int area; };

    /** TUM fail yollari bunu cagirmali — track reset sayaci tutarli islesin. */
    void onInvalid()
    {
        frames_since_valid_++;
        if (frames_since_valid_ > track_reset_frames)
            has_last_ = false;
    }

    bool matchTriangle(const std::vector<Blob> &blobs,
                       int &bi, int &bj, int &bk, int &bp1) const
    {
        constexpr float MIN_PIXEL_DIST = 15.0f;
        int n_b = (int)blobs.size();
        bi = bj = bk = bp1 = -1;
        float best_score = std::numeric_limits<float>::max();

        for (int i = 0; i < n_b; ++i) {
            for (int j = i + 1; j < n_b; ++j) {
                for (int k = j + 1; k < n_b; ++k) {
                    float d_ij = (float)cv::norm(blobs[i].c - blobs[j].c);
                    float d_ik = (float)cv::norm(blobs[i].c - blobs[k].c);
                    float d_jk = (float)cv::norm(blobs[j].c - blobs[k].c);
                    if (std::min({d_ij, d_ik, d_jk}) < MIN_PIXEL_DIST) continue;

                    // Area profili: 2 parlak + 1 sonuk sart — yanlis blob
                    // secimine (noise + 2 LED gibi) engel
                    int a_i = blobs[i].area, a_j = blobs[j].area, a_k = blobs[k].area;
                    int a_max = std::max({a_i, a_j, a_k});
                    int a_min = std::max(1, std::min({a_i, a_j, a_k}));
                    int a_mid = a_i + a_j + a_k - a_max - a_min;
                    if ((float)a_mid / (float)a_min < bright_dim_ratio) continue;
                    int dim_idx = (a_i == a_min) ? i : ((a_j == a_min) ? j : k);

                    // Her vertex'i P1 (90°) olarak dene — sonuk LED P1 olamaz
                    int   p1_cand[3] = {i, j, k};
                    float leg_a[3]   = {d_ij, d_ij, d_ik};
                    float leg_b[3]   = {d_ik, d_jk, d_jk};
                    float hyp[3]     = {d_jk, d_ik, d_ij};

                    for (int c = 0; c < 3; ++c) {
                        if (p1_cand[c] == dim_idx) continue;
                        float a = leg_a[c], b = leg_b[c], h = hyp[c];

                        // Pythagoras + ikizkenar (leg orani 1.0) skoru
                        float pyth_err  = std::abs(h*h - a*a - b*b) / std::max(1e-3f, h*h);
                        float ratio_obs = std::max(a, b) / std::max(1e-3f, std::min(a, b));
                        float ratio_err = std::abs(ratio_obs - 1.0f);

                        float score = pyth_err + ratio_err;
                        if (score < best_score) {
                            best_score = score;
                            bi = i; bj = j; bk = k; bp1 = p1_cand[c];
                        }
                    }
                }
            }
        }
        return bp1 >= 0 && best_score <= geom_score_thr;
    }

    bool solveP3PFiltered(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3,
                          cv::Vec3f &out_pos,
                          float &roll_deg, float &pitch_deg, float &yaw_deg) const
    {
        std::vector<cv::Point3f> world = {
            {0.0f,  0.0f,  0.0f},
            {leg_m, 0.0f,  0.0f},
            {0.0f,  leg_m, 0.0f}};
        std::vector<cv::Point2f> img = {p1, p2, p3};
        cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);

        std::vector<cv::Mat> rvecs, tvecs;
        int n_sols = 0;
        try {
            n_sols = cv::solveP3P(world, img, K_, dist, rvecs, tvecs,
                                  cv::SOLVEPNP_AP3P);
        } catch (...) { n_sols = 0; }
        if (n_sols == 0) return false;

        // Filtre: noktalar kamera onunde (Z>0) + kamera duzlemin ustunde (cz>0)
        int best_s = -1;
        double best_cz = -std::numeric_limits<double>::max();
        for (int s = 0; s < n_sols; ++s) {
            cv::Mat Rs; cv::Rodrigues(rvecs[s], Rs);
            double min_z = std::numeric_limits<double>::max();
            for (const auto &wp : world) {
                cv::Mat wpm = (cv::Mat_<double>(3,1) << wp.x, wp.y, wp.z);
                cv::Mat cp = Rs * wpm + tvecs[s];
                min_z = std::min(min_z, cp.at<double>(2));
            }
            if (min_z <= 0.0) continue;
            cv::Mat cam_chk = -Rs.t() * tvecs[s];
            double cz = cam_chk.at<double>(2);
            if (cz <= 0.0) continue;
            if (cz > best_cz) { best_s = s; best_cz = cz; }
        }
        if (best_s < 0) return false;

        // ITERATIVE refinement
        cv::Mat rvec_r = rvecs[best_s].clone();
        cv::Mat tvec_r = tvecs[best_s].clone();
        try {
            cv::solvePnP(world, img, K_, dist, rvec_r, tvec_r, true,
                         cv::SOLVEPNP_ITERATIVE);
        } catch (...) {}

        cv::Mat R; cv::Rodrigues(rvec_r, R);
        cv::Mat Rcw = R.t();   // cam→world

        // ZYX Euler, derece
        roll_deg  = (float)(std::atan2(Rcw.at<double>(2,1), Rcw.at<double>(2,2))
                            * (180.0 / M_PI));
        pitch_deg = (float)(std::atan2(-Rcw.at<double>(2,0),
                            std::sqrt(Rcw.at<double>(2,1)*Rcw.at<double>(2,1) +
                                      Rcw.at<double>(2,2)*Rcw.at<double>(2,2)))
                            * (180.0 / M_PI));
        yaw_deg   = (float)(std::atan2(Rcw.at<double>(1,0), Rcw.at<double>(0,0))
                            * (180.0 / M_PI));

        cv::Mat cp = -R.t() * tvec_r;
        out_pos = cv::Vec3f((float)cp.at<double>(0),
                            (float)cp.at<double>(1),
                            (float)cp.at<double>(2));
        return true;
    }

    void drawDebug(const cv::Mat &gray, const std::vector<Blob> &blobs,
                   const Result &res, cv::Mat &vis) const
    {
        cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);

        for (const auto &b : blobs) {
            cv::circle(vis, b.c, 10, cv::Scalar(0, 255, 255), 1);
            cv::putText(vis, std::to_string(b.area), b.c + cv::Point2f(12, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 200), 1);
        }

        if (res.valid) {
            cv::line(vis, res.p1_px, res.p2_px, cv::Scalar(0, 255, 0), 2);
            cv::line(vis, res.p1_px, res.p3_px, cv::Scalar(0, 255, 0), 2);
            cv::line(vis, res.p2_px, res.p3_px, cv::Scalar(255, 100, 0), 2);

            auto drawVertex = [&](cv::Point2f p, const std::string &label,
                                  cv::Scalar color){
                cv::circle(vis, p, 18, color, 3);
                cv::putText(vis, label, p + cv::Point2f(22, 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
            };
            drawVertex(res.p1_px, "P1", cv::Scalar(0, 255, 0));
            drawVertex(res.p2_px, "P2", cv::Scalar(0, 200, 255));
            drawVertex(res.p3_px, "P3", cv::Scalar(255, 200, 0));

            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "cam=(%.2f, %.2f, %.2f) rpy=(%.1f,%.1f,%.1f) range=%.2fm",
                res.cam_pos[0], res.cam_pos[1], res.cam_pos[2],
                res.roll_deg, res.pitch_deg, res.yaw_deg,
                cv::norm(res.cam_pos));
            cv::putText(vis, buf, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 0), 2);
        } else {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "INVALID  blobs=%u", res.n_blobs_seen);
            cv::putText(vis, buf, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 0, 255), 2);
        }
    }

    cv::Mat   K_;              // setIntrinsics ile doldurulur
    cv::Mat   morph_kernel_;
    cv::Vec3f last_cam_pos_{0, 0, 0};
    bool      has_last_ = false;
    int       frames_since_valid_ = 999;
};

} // namespace lps
