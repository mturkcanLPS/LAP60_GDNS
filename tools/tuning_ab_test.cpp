/**
 * tuning_ab_test.cpp — sampleStep ve ransacThresh A/B olcumu (kamera GEREKMEZ)
 *
 * Iki soru:
 *  1) sampleStep 6 -> 4/3/2: akis izgarasindaki 57.600 hucrenin %2.8'ini
 *     kullaniyoruz. Daha cok ornek tahmin gurultusunu azaltir mi, kac ms'e?
 *  2) ransacThresh 3.0 px: olculen artiklara gore COK genis; dusuk harekette
 *     hicbir seyi elemiyor. Daraltmak iyilestirir mi, yoksa gercek hareketi
 *     mi keser?
 *
 * Iki bagimsiz olcek:
 *  A) SENTETIK — gercek ucus karesi + bilinen kaydirma + gurultu. Kesin dogru.
 *  B) GERCEK ardisik kareler — hakem cv::phaseCorrelate (FFT tabanli, bizim
 *     piramit/RANSAC hattindan tamamen bagimsiz).
 *
 * Derle:
 *   g++ -O2 tools/tuning_ab_test.cpp -o /tmp/tuning_ab \
 *     -Isrc/lps_serial_sender/include -I/opt/nvidia/vpi3/include \
 *     -I/usr/include/opencv4 -L/opt/nvidia/vpi3/lib/aarch64-linux-gnu \
 *     -lnvvpi -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_calib3d
 */
#include "lps_serial_sender/optical_flow_vpi.hpp"
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using clk = std::chrono::steady_clock;
static const char *kVideo = "/home/lapis/lps_records/lps_20260723_151935.avi";

static std::vector<cv::Mat> loadFrames(int start, int n)
{
    cv::VideoCapture cap(kVideo);
    std::vector<cv::Mat> out;
    cv::Mat f;
    int i = 0;
    while (cap.read(f)) {
        if (i >= start && i < start + n) {
            cv::Mat g, up;
            cv::cvtColor(f, g, cv::COLOR_BGR2GRAY);
            g = g(cv::Rect(0, 48, g.cols, g.rows - 48)).clone();   // HUD'u at
            cv::resize(g, up, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
            out.push_back(up);
        }
        if (i >= start + n) break;
        i++;
    }
    return out;
}

static void addNoise(cv::Mat &img, double sigma, cv::RNG &rng)
{
    if (sigma <= 0) return;
    cv::Mat n(img.size(), CV_32F), f;
    rng.fill(n, cv::RNG::NORMAL, 0.0, sigma);
    img.convertTo(f, CV_32F);
    f += n;
    f.convertTo(img, CV_8U);
}

/* --- A) SENTETIK: kesin dogru, kontrollu gurultu --------------------- */
static void synthetic(const cv::Mat &base, float step, double thr, double sigma)
{
    const float shifts[] = {0.3f, 1.0f, 3.0f, 8.0f, 20.0f};
    double sum_err = 0.0, sum_ms = 0.0;
    int n = 0, q_sum = 0;

    for (float sh : shifts) {
        cv::RNG rng(12345);
        cv::Mat a = base.clone(), b;
        cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, sh, 0, 1, 0);
        cv::warpAffine(base, b, M, base.size(), cv::INTER_LINEAR,
                       cv::BORDER_REFLECT);
        addNoise(a, sigma, rng);
        addNoise(b, sigma, rng);

        lps::OpticalFlowVPI fl(4, 4, 1.0f, 8.0f);
        fl.setTuning(step, thr);
        fl.init(a);
        lps::OpticalFlowVPI::Result r;
        const auto t0 = clk::now();
        fl.process(b, r);
        sum_ms += std::chrono::duration<double, std::milli>(clk::now() - t0).count();
        sum_err += std::abs(r.raw_dx - sh);
        q_sum += r.raw_quality;
        n++;
    }
    printf("  step=%-4.1f thr=%-5.2f | ort.hata %7.4f px | Q %3d | %6.2f ms\n",
           step, thr, sum_err / n, q_sum / n, sum_ms / n);
}

/* --- B) GERCEK kareler, hakem: phaseCorrelate ------------------------ */
static void real(const std::vector<cv::Mat> &fr, float step, double thr,
                 const std::vector<double> &ref)
{
    lps::OpticalFlowVPI fl(4, 4, 1.0f, 8.0f);
    fl.setTuning(step, thr);
    fl.init(fr[0]);
    double sum_err = 0.0, sum_ms = 0.0;
    int n = 0, q_sum = 0;
    for (size_t k = 1; k < fr.size(); ++k) {
        lps::OpticalFlowVPI::Result r;
        const auto t0 = clk::now();
        fl.process(fr[k], r);
        sum_ms += std::chrono::duration<double, std::milli>(clk::now() - t0).count();
        // Hakemin guvenilir oldugu kareler: makul buyuklukteki hareketler
        if (std::abs(ref[k - 1]) < 40.0) {
            sum_err += std::abs(r.raw_dx - ref[k - 1]);
            q_sum += r.raw_quality;
            n++;
        }
    }
    printf("  step=%-4.1f thr=%-5.2f | hakeme ort.fark %7.4f px | Q %3d | %6.2f ms\n",
           step, thr, n ? sum_err / n : 0.0, n ? q_sum / n : 0, sum_ms / (fr.size() - 1));
}

int main()
{
    auto fr = loadFrames(3100, 45);
    if (fr.size() < 5) { printf("kare yuklenemedi\n"); return 1; }

    // Hakem degerleri bir kez hesaplanir
    std::vector<double> ref;
    cv::Mat win;
    cv::createHanningWindow(win, fr[0].size(), CV_32F);
    for (size_t k = 1; k < fr.size(); ++k) {
        cv::Mat a, b;
        fr[k - 1].convertTo(a, CV_32F);
        fr[k].convertTo(b, CV_32F);
        ref.push_back(cv::phaseCorrelate(a, b, win).x);
    }

    const float steps[] = {6.0f, 4.0f, 3.0f, 2.0f};
    const double thrs[] = {3.0, 1.5, 1.0, 0.5};

    printf("=== 1) sampleStep taramasi (thr=3.0 sabit) ===\n");
    printf("--- SENTETIK (kesin dogru, gurultu sigma=2) ---\n");
    for (float s : steps) synthetic(fr[10], s, 3.0, 2.0);
    printf("--- GERCEK kareler (hakem: faz korelasyonu) ---\n");
    for (float s : steps) real(fr, s, 3.0, ref);

    // Kombinasyon + kayma bazinda kirilim: dar esik BUYUK harekette gercek
    // artiklari (parallaks/donme) kesip zarar veriyor mu?
    printf("\n=== 3) KOMBINASYON, kayma bazinda (sentetik, sigma=2) ===\n");
    printf("%-10s", "kayma");
    printf("%-16s%-16s%-16s\n", "step6/thr3.0", "step6/thr1.5", "step3/thr1.5");
    for (float sh : {0.3f, 1.0f, 3.0f, 8.0f, 20.0f, 40.0f}) {
        printf("%-10.1f", sh);
        for (auto cfg : std::vector<std::pair<float,double>>{{6,3.0},{6,1.5},{3,1.5}}) {
            cv::RNG rng(12345);
            cv::Mat a = fr[10].clone(), b;
            cv::Mat M = (cv::Mat_<double>(2,3) << 1,0,sh, 0,1,0);
            cv::warpAffine(fr[10], b, M, a.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
            addNoise(a, 2.0, rng); addNoise(b, 2.0, rng);
            lps::OpticalFlowVPI fl(4,4,1.0f,8.0f);
            fl.setTuning(cfg.first, cfg.second);
            fl.init(a);
            lps::OpticalFlowVPI::Result r; fl.process(b, r);
            char buf[64];
            snprintf(buf, sizeof(buf), "%+.4f (Q%d)", r.raw_dx - sh, r.raw_quality);
            printf("%-16s", buf);
        }
        printf("\n");
    }

    printf("\n=== 2) ransacThresh taramasi (step=6 sabit) ===\n");
    printf("--- SENTETIK ---\n");
    for (double t : thrs) synthetic(fr[10], 6.0f, t, 2.0);
    printf("--- GERCEK kareler ---\n");
    for (double t : thrs) real(fr, 6.0f, t, ref);
    return 0;
}
