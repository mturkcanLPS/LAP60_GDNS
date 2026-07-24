/**
 * grid_ab_test.cpp — OpticalFlowVPI grid_size A/B testi (kamera GEREKMEZ)
 *
 * Soru: grid 4 -> 8 olcumu iyilestiriyor mu? Iddia: hucre basina 16 yerine
 * 64 piksel ortalanir, gurultu yariya iner, gercek doku korunur.
 *
 * Yontem: gercek ucus karesi alinir, BILINEN miktarda kaydirilir, iki kareye
 * de sensor gurultusu eklenir (olculen sigma ~2 DN). Uretim sinifinin ta
 * kendisi (optical_flow_vpi.hpp) calistirilir; olculen dx gercek dx ile
 * kiyaslanir. Farkli kaydirmalar = farkli irtifa rejimleri:
 *   0.3 px  ~ yuksek irtifa / yavas surunme
 *   1.0 px  ~ orta irtifa
 *   3.0 px  ~ normal seyir
 *   8.0 px  ~ alcak / hizli
 *
 * Derle:
 *   g++ -O2 tools/grid_ab_test.cpp -o /tmp/grid_ab \
 *     -Isrc/lps_serial_sender/include -I/opt/nvidia/vpi3/include \
 *     -I/usr/include/opencv4 -L/opt/nvidia/vpi3/lib/aarch64-linux-gnu \
 *     -lnvvpi -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_calib3d
 */
#include "lps_serial_sender/optical_flow_vpi.hpp"
#include <opencv2/videoio.hpp>
#include <cstdio>
#include <vector>
#include <string>

// Gercek kayit 0.5 olcekli; uretim hatti 1280x720 isliyor. Kareyi tam
// coznurluge geri buyutuyoruz ki hucre/doku orani gercekcilikte kalsin.
// NOT: buyutme kayip detayi geri getirmez — gercek sensordeki ince doku
// buradakinden zengindir. Bu yuzden grid 8'in dezavantaji (ince doku
// kaybi) bu testte OLDUGUNDAN AZ gorunur; sonucu ona gore yorumla.
static cv::Mat loadFlightFrame(const std::string &video, int frame_no)
{
    cv::VideoCapture cap(video);
    if (!cap.isOpened()) { printf("video acilamadi: %s\n", video.c_str()); exit(1); }
    cv::Mat f;
    for (int i = 0; i <= frame_no; ++i) if (!cap.read(f)) break;
    if (f.empty()) { printf("kare okunamadi\n"); exit(1); }
    cv::Mat g;
    cv::cvtColor(f, g, cv::COLOR_BGR2GRAY);
    g = g(cv::Rect(0, 48, g.cols, g.rows - 48)).clone();   // HUD bandini at
    cv::Mat up;
    cv::resize(g, up, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
    return up;
}

static void addNoise(cv::Mat &img, double sigma, cv::RNG &rng)
{
    if (sigma <= 0) return;
    cv::Mat n(img.size(), CV_32F);
    rng.fill(n, cv::RNG::NORMAL, 0.0, sigma);
    cv::Mat f;
    img.convertTo(f, CV_32F);
    f += n;
    f.convertTo(img, CV_8U);
}

int main(int argc, char **argv)
{
    const std::string video = argc > 1 ? argv[1]
        : "/home/lapis/lps_records/lps_20260723_151935.avi";
    const int frame_no = argc > 2 ? atoi(argv[2]) : 3150;   // keskin ucus karesi
    const double sigma = argc > 3 ? atof(argv[3]) : 2.0;    // olculen sensor gurultusu

    cv::Mat base = loadFlightFrame(video, frame_no);
    printf("Kare: %s #%d  (%dx%d)   eklenen gurultu sigma=%.1f DN\n\n",
           video.c_str(), frame_no, base.cols, base.rows, sigma);

    // 4. arguman: gurultuyu bastirmak icin akistan ONCE uygulanan Gauss
    // yumusatma sigmasi (0 = kapali). Node'a eklenecek olan da bu olurdu.
    const double blur = argc > 4 ? atof(argv[4]) : 0.0;
    if (blur > 0) printf("On-yumusatma: Gauss sigma=%.2f\n\n", blur);

    const float shifts[] = {0.3f, 1.0f, 3.0f, 8.0f};
    const int grids[] = {4, 8};

    printf("%-8s %-6s | %-9s %-9s | %-5s %-7s %-6s\n",
           "gercek", "grid", "olculen", "HATA", "Q", "consist", "tex");
    printf("---------------------------------------------------------------\n");

    for (float sh : shifts) {
        for (int grid : grids) {
            cv::RNG rng(12345);                 // ayni gurultu ikisinde de
            cv::Mat a = base.clone();
            cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, sh, 0, 1, 0);
            cv::Mat b;
            cv::warpAffine(base, b, M, base.size(), cv::INTER_LINEAR,
                           cv::BORDER_REFLECT);
            addNoise(a, sigma, rng);
            addNoise(b, sigma, rng);
            if (blur > 0) {                    // gurultu bastirma denemesi
                cv::GaussianBlur(a, a, cv::Size(0, 0), blur);
                cv::GaussianBlur(b, b, cv::Size(0, 0), blur);
            }

            try {
                // ema_alpha=1.0: yumusatma yok, tek adimda tam deger
                lps::OpticalFlowVPI flow(grid, 3, 1.0f, 8.0f);
                flow.init(a);
                lps::OpticalFlowVPI::Result r;
                flow.process(b, r);
                printf("%-8.1f %-6d | %-9.3f %-9.3f | %-5u %-7.2f %-6.2f\n",
                       sh, grid, r.dx, r.dx - sh, r.quality, r.consist,
                       r.tex_ratio);
            } catch (const std::exception &e) {
                printf("%-8.1f %-6d | HATA: %s\n", sh, grid, e.what());
            }
        }
        printf("\n");
    }
    return 0;
}
