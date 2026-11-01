/**
 * test_ir_triangle.cpp — IR ucgeni ucdan uca SENTETIK dogrulama
 *
 * Amac: "otopilota gonderdigimiz NORMALIZE YON dogru mu" sorusunu donanim
 * beklemeden cevaplamak. Bilinen bir 3B poza ucgen konur, kamera modeliyle
 * goruntuye izdusurulur, sentetik LED lekeleri cizilir — sonra node'un
 * KULLANDIGI kodun ta kendisi (ir_triangle.hpp) calistirilip cikan yon
 * ground truth ile karsilastirilir. Menzil (z_opt) ayrica raporlanir ama
 * pakete gitmez — otopilot kendi Z'siyle carpiyor.
 *
 * Kritik nokta: burada test edilen izdusum ZINCIRI degil, node'un ters
 * cozumu. Ileri yon (3B -> piksel) bagimsiz olarak cv::projectPoints ile
 * yapiliyor, ters yon (piksel -> x,y) node'un kendi kodu. Ikisi ayni
 * varsayimi paylasmadigi icin test gercekten bir sey dogruluyor.
 *
 * Calistirma:  ros2 run lps_serial_sender test_ir_triangle
 */

#include "lps_serial_sender/ir_triangle.hpp"

#include <opencv2/calib3d.hpp>
#include <cstdio>
#include <string>
#include <vector>

namespace {

// --- Test edilen marker: ikizkenar dik ucgen, iki dik kenar da 1.36 m ---
constexpr float LEG_A = 1.36f;
constexpr float LEG_B = 1.36f;
constexpr float TOL   = 0.12f;   // node varsayilani (ir_tol_frac)

// --- Sentetik kamera: RealSense D4xx infra1 1280x720 tipik degerler ---
const cv::Size  IMG(1280, 720);
const cv::Matx33d K(640.0,   0.0, 640.0,
                      0.0, 640.0, 360.0,
                      0.0,   0.0,   1.0);
const cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);   // image_rect_raw -> distorsiyon yok

struct Truth { cv::Vec3d center; double err_x, err_y, err_z; bool detected; };

/** Marker'i verilen poza koyup goruntuye cizer.
 *  Marker yerel duzlemi: R=(0,0,0), A=(a,0,0), B=(0,b,0) -> hipotenus orta
 *  noktasi M=(a/2,b/2,0) = HUNI MERKEZI (ground truth burasi).
 *  rvec/tvec: marker -> kamera donusumu (kamera optik frame'i). */
cv::Mat render(const cv::Vec3d &rvec, const cv::Vec3d &tvec,
               std::vector<cv::Point2f> &proj_out, cv::Vec3d &center_cam,
               int led_radius = 3)
{
    std::vector<cv::Point3f> obj = {
        {0.0f,  0.0f,  0.0f},          // dik acı kosesi
        {LEG_A, 0.0f,  0.0f},          // bacak ucu A
        {0.0f,  LEG_B, 0.0f},          // bacak ucu B
        {LEG_A * 0.5f, LEG_B * 0.5f, 0.0f}   // huni merkezi (sadece truth icin)
    };
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, K, D, proj);

    // Huni merkezinin GERCEK kamera-frame konumu (ground truth)
    cv::Mat R;
    cv::Rodrigues(cv::Mat(rvec), R);
    cv::Mat mc = R * (cv::Mat_<double>(3,1) << LEG_A*0.5, LEG_B*0.5, 0.0)
                 + (cv::Mat_<double>(3,1) << tvec[0], tvec[1], tvec[2]);
    center_cam = cv::Vec3d(mc.at<double>(0), mc.at<double>(1), mc.at<double>(2));

    cv::Mat img = cv::Mat::zeros(IMG, CV_8UC1);
    img.setTo(cv::Scalar(30));            // hafif arka plan (esigin cok altinda)
    proj_out.assign(proj.begin(), proj.begin() + 3);
    for (int i = 0; i < 3; ++i)
        cv::circle(img, proj[i], led_radius, cv::Scalar(255), -1, cv::LINE_8);
    return img;
}

/** Node'un processIr'daki hesabinin AYNISI (ir_triangle.hpp + pinhole).
 *  out = (nx, ny, Z_opt): nx/ny PAKETE GIDEN normalize yon (boyutsuz),
 *  Z_opt ise ucgenden olculen menzil — pakete gitmez, dogrulamada ve
 *  test raporunda metrik karsiligi gostermek icin dondurulur. */
bool solve(const cv::Mat &img, cv::Vec3d &out, float &fit_err, int &n_blobs)
{
    std::vector<cv::Point2f> pts; std::vector<int> areas;
    cv::Mat bin, labels, stats, centroids;
    lps::detectIrBlobs(img, 240.0, 1.0, 400.0, 16,
                       bin, labels, stats, centroids, pts, areas);
    n_blobs = (int)pts.size();

    int ir_, ia_, ib_; float k_px_per_m = 0.0f;
    if (!lps::findIrRightTriangle(pts, LEG_A, LEG_B, TOL,
                                  ir_, ia_, ib_, fit_err, k_px_per_m))
        return false;

    const double f_px = 0.5 * (K(0,0) + K(1,1));
    const double Z    = f_px / k_px_per_m;

    const cv::Point2f mid = (pts[ia_] + pts[ib_]) * 0.5f;   // hipotenus ortasi
    std::vector<cv::Point2f> in{mid}, un;
    cv::undistortPoints(in, un, K, D);
    out = cv::Vec3d(un[0].x, un[0].y, Z);   // <-- Z ile CARPILMIYOR
    return true;
}

int failures = 0;

void check(const std::string &name, const cv::Vec3d &rvec, const cv::Vec3d &tvec,
           double tol_xy_m, double tol_z_m, int led_r = 3)
{
    std::vector<cv::Point2f> proj; cv::Vec3d truth;
    cv::Mat img = render(rvec, tvec, proj, truth, led_r);

    // Ucgen kadraja sigmiyorsa test anlamsiz — atla (bilgi amacli bildir)
    for (const auto &p : proj)
        if (p.x < 0 || p.y < 0 || p.x >= IMG.width || p.y >= IMG.height) {
            std::printf("  ATLA  %-34s (ucgen kadraj disi)\n", name.c_str());
            return;
        }

    cv::Vec3d got; float err = 0; int nb = 0;
    if (!solve(img, got, err, nb)) {
        // tol<0 = "reddedilmesi BEKLENIYOR" (egim taramasinin ust ucu):
        // bozulma sessizce degil, RED ile sonuclanmali — kontrol edilen bu.
        if (tol_xy_m < 0) {
            std::printf("  OK    %-34s REDDEDILDI (fit=%.3f > %.2f)\n",
                        name.c_str(), err, TOL);
            return;
        }
        std::printf("  HATA  %-34s TESPIT YOK (blob=%d)\n", name.c_str(), nb);
        ++failures; return;
    }
    // Tolerans MENZILLE olceklenir: hata kaynagi centroid kuantalanmasi olup
    // Z buyudukce ucgen piksel olarak kuculdugu icin GORELI hata artar (fizik,
    // hata degil). Sabit mm toleransi uzak mesafede anlamsizca patlar.
    //   tol<=0 verilirse: xy = %0.5 menzil + 10 mm, z = %1.5 menzil
    if (tol_xy_m == 0.0) tol_xy_m = 0.005 * truth[2] + 0.010;
    if (tol_z_m  <= 0.0) tol_z_m  = 0.015 * truth[2];
    if (tol_xy_m <  0.0) tol_xy_m = 1e9;   // egim taramasi: sadece rakami gor

    // PAKETE GIDEN normalize yonun hatasi. Ground truth yon = X_true/Z_true.
    // Okunabilirlik icin GERCEK menzille carpilip mm'ye cevrilir: bu, otopilot
    // KUSURSUZ bir Z kullansaydi kalacak hata — yani BIZIM hatamiz, Z'ninki
    // degil. (Z hatasi ayri sutunda; artik otopilotun sorumlulugunda.)
    const double nx_true = truth[0] / truth[2], ny_true = truth[1] / truth[2];
    const double dnx = got[0] - nx_true, dny = got[1] - ny_true;
    const double dxy = std::sqrt(dnx*dnx + dny*dny) * truth[2];
    const double dz  = got[2] - truth[2];

    const bool ok = (dxy <= tol_xy_m) && (std::fabs(dz) <= tol_z_m);
    if (!ok) ++failures;
    std::printf("  %s  %-34s yon_hata=%6.1f mm  z_opt_hata=%7.1f mm  (Z=%5.2fm fit=%.3f)\n",
                ok ? "OK  " : "HATA", name.c_str(),
                dxy * 1000.0, dz * 1000.0, truth[2], err);
}

const double DEG = CV_PI / 180.0;

} // namespace

int main()
{
    std::printf("\n=== IR ucgeni sentetik dogrulama (ikizkenar, kenar %.2f m) ===\n",
                LEG_A);
    std::printf("Kamera: %dx%d  fx=fy=%.0f  hipotenus=%.3f m\n\n",
                IMG.width, IMG.height, K(0,0),
                std::sqrt(LEG_A*LEG_A + LEG_B*LEG_B));

    // --- 1) Duz bakis, merkezde: en temel dogruluk ---
    std::printf("[1] Duz bakis, hedef merkezde\n");
    for (double Z : {3.0, 5.0, 8.0, 12.0}) {
        // marker merkezini optik eksene oturt: tvec = -R*(a/2,b/2,0) + (0,0,Z)
        cv::Vec3d t(-LEG_A*0.5, -LEG_B*0.5, Z);
        check("Z=" + std::to_string((int)Z) + "m merkez", {0,0,0}, t, 0, 0);
    }

    // --- 2) Duz bakis, hedef kadrajin kenarinda (eksen disi) ---
    std::printf("\n[2] Duz bakis, hedef eksen disinda\n");
    check("Z=5m  sag+1.5m",  {0,0,0}, {-LEG_A*0.5 + 1.5, -LEG_B*0.5,       5.0}, 0, 0);
    check("Z=5m  asagi+1.0m",{0,0,0}, {-LEG_A*0.5,       -LEG_B*0.5 + 1.0, 5.0}, 0, 0);
    check("Z=8m  capraz",    {0,0,0}, {-LEG_A*0.5 + 2.0, -LEG_B*0.5 + 1.5, 8.0}, 0, 0);

    // --- 3) Marker duzlem-ici donmus: correspondence rotasyondan bagimsiz mi? ---
    std::printf("\n[3] Marker duzlem-ici donmus (Z ekseni etrafinda)\n");
    for (double a : {30.0, 90.0, 150.0, 210.0, 300.0}) {
        cv::Vec3d rv(0, 0, a * DEG);
        cv::Mat R; cv::Rodrigues(cv::Mat(rv), R);
        cv::Mat c = R * (cv::Mat_<double>(3,1) << LEG_A*0.5, LEG_B*0.5, 0.0);
        cv::Vec3d t(-c.at<double>(0), -c.at<double>(1), 6.0);
        check("yaw=" + std::to_string((int)a) + " derece", rv, t, 0, 0);
    }

    // --- 4) EGIK marker: perspektif sapmasi ne kadar? ---
    // Bu, "orta noktanin izdusumu != izdusumun orta noktasi" hatasini olcer.
    std::printf("\n[4] Marker egik (perspektif sapmasi olculuyor)\n");
    for (double a : {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}) {
        cv::Vec3d rv(a * DEG, 0, 0);
        cv::Mat R; cv::Rodrigues(cv::Mat(rv), R);
        cv::Mat c = R * (cv::Mat_<double>(3,1) << LEG_A*0.5, LEG_B*0.5, 0.0);
        cv::Vec3d t(-c.at<double>(0), -c.at<double>(1), 6.0);
        // Tolerans kasten gevsek: amac PASS/FAIL degil, sapmayi RAKAMLA gormek.
        // -1 = "reddedilirse de OK" — taramanin ust ucunda tespitin kesilmesi
        // BEKLENEN davranis (bozulma sessiz kalmasin diye tam da bunu istiyoruz).
        check("egim=" + std::to_string((int)a) + " derece", rv, t, -1.0, 5.00);
    }

    // --- 5) Sahte parlak noktalar arasindan dogru ucgeni secebiliyor mu? ---
    std::printf("\n[5] Parazit noktalar arasinda (yanlis pozitif reddi)\n");
    {
        std::vector<cv::Point2f> proj; cv::Vec3d truth;
        cv::Vec3d t(-LEG_A*0.5, -LEG_B*0.5, 6.0);
        cv::Mat img = render({0,0,0}, t, proj, truth);
        // gunes yansimasi/sicak piksel taklidi — ucgene ait olmayan lekeler
        const cv::Point2f junk[] = {{100,80},{1150,120},{300,650},{980,600},{60,400}};
        for (const auto &j : junk) cv::circle(img, j, 3, cv::Scalar(255), -1);

        cv::Vec3d got; float err = 0; int nb = 0;
        if (!solve(img, got, err, nb)) {
            std::printf("  HATA  %-34s TESPIT YOK (blob=%d)\n", "5 parazit + 3 LED", nb);
            ++failures;
        } else {
            const double d = cv::norm(cv::Vec2d(got[0]-truth[0]/truth[2],
                                                got[1]-truth[1]/truth[2])) * truth[2];
            const bool ok = d < 0.01;
            if (!ok) ++failures;
            std::printf("  %s  %-34s yon_hata=%6.1f mm  (blob=%d fit=%.3f)\n",
                        ok ? "OK  " : "HATA", "5 parazit + 3 LED", d*1000.0, nb, err);
        }
    }

    // --- 6) YANLIS geometri reddedilmeli (esdusey ucgen, dik degil) ---
    std::printf("\n[6] Yanlis geometri REDDEDILMELI\n");
    {
        cv::Mat img = cv::Mat::zeros(IMG, CV_8UC1); img.setTo(cv::Scalar(30));
        // eskenar ucgen: dik ucgen degil -> fit artigi buyuk olmali
        cv::circle(img, {500,300}, 3, cv::Scalar(255), -1);
        cv::circle(img, {800,300}, 3, cv::Scalar(255), -1);
        cv::circle(img, {650,560}, 3, cv::Scalar(255), -1);
        cv::Vec3d got; float err = 0; int nb = 0;
        const bool det = solve(img, got, err, nb);
        if (det) { std::printf("  HATA  %-34s KABUL EDILDI (fit=%.3f)\n",
                               "eskenar ucgen", err); ++failures; }
        else     { std::printf("  OK    %-34s reddedildi (fit=%.3f > %.2f)\n",
                               "eskenar ucgen", err, TOL); }
    }

    // --- 7) Menzil siniri: LED kucuguldukce nereye kadar? ---
    std::printf("\n[7] Menzil siniri (LED yaricapi 1 px = uzak mesafe taklidi)\n");
    for (double Z : {6.0, 10.0, 15.0, 20.0}) {
        cv::Vec3d t(-LEG_A*0.5, -LEG_B*0.5, Z);
        check("Z=" + std::to_string((int)Z) + "m (1px LED)", {0,0,0}, t, 0, 0, 1);
    }

    std::printf("\n=== %s ===\n\n",
                failures == 0 ? "TUM TESTLER GECTI"
                              : (std::to_string(failures) + " TEST BASARISIZ").c_str());
    return failures == 0 ? 0 : 1;
}
