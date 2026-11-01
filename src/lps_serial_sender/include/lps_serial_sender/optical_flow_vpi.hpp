/**
 * optical_flow_vpi.hpp — VPI OFA dense optical flow → global dx/dy
 *
 * Jetson OFA donanimini kullanir (CUDA + VIC yardimci backend'ler).
 * ROS'tan bagimsizdir; cagiran taraf thread'i ve frame kuyrugunu yonetir.
 *
 * Kullanim (tek thread'den):
 *   OpticalFlowVPI flow(grid_size, num_levels, ema_alpha);
 *   flow.init(first_frame);                  // bir kez
 *   flow.process(frame, result);             // her frame
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include <vpi/OpenCVInterop.hpp>
#include <vpi/Image.h>
#include <vpi/ImageFormat.h>
#include <vpi/Pyramid.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/OpticalFlowDense.h>
#include <vpi/algo/Rescale.h>

namespace lps {

class OpticalFlowVPI
{
public:
    struct Result {
        float   dx      = 0.0f;   // EMA'li global hareket (px, giris cozunurlugunde)
        float   dy      = 0.0f;
        uint8_t quality = 0;      // 0-100, EMA'li: dokulu hucre uyumu x kapsama
        // HAM (EMA ONCESI) o karenin kendi olcumu — zamansal yumusatma yok.
        // Gurultu/gecikme analizi icin: EMA gercek dalgalanmayi gizler.
        float   raw_dx  = 0.0f;
        float   raw_dy  = 0.0f;
        uint8_t raw_quality = 0;
        float   consist   = 0.0f; // debug: dokulu hucrelerin yumusak uyum skoru 0-1
        float   tex_ratio = 0.0f; // debug: dokulu (olculebilir) hucre orani 0-1
        uint8_t tex_p50   = 0;    // debug: hucre gradyan medyani (≈gurultu tabani)
        uint8_t tex_p90   = 0;    // debug: hucre gradyan p90'i
        float   tex_thresh = 0;   // debug: bu karede kullanilan efektif esik
    };

    /** texture_thresh: doku esiginin TABANI (floor). Efektif esik sahneye
     *  uyarlanir: max(taban, 1.5 x grad_p50). p50 sahnenin duz cogunlugunun
     *  gradyani = o anki gurultu tabani (isik/gain ile degisir: aksam los
     *  betonda ~4-7, gunduz ~10+). Sabit esik (12) aksam ucusunda gercek
     *  zemin dokusunu (derz/leke, 6-12 bandi) OLU sayip quality'yi eziyordu
     *  (26_07_13_T5).
     *
     *  TABAN 8 -> 4 (26_07_24): on-suzme gradyanlari dusurunce taban
     *  BAGLAYICI hale geldi. 24_07 beton ucusunda g50=3, g90=6 olculdu —
     *  sahnenin en dokulu %10'u bile 8'in altinda kaldi; kapsama (tex 0.06)
     *  ve onunla birlikte Q (medyan 14) coktu. AYNI yuzeydeki 21_07
     *  ucusunda (suzgec yok) g50=8/g90=16, tex 0.25, Q 31 idi.
     *  Hakemli olcum (cv::phaseCorrelate, bizim hattan bagimsiz) tabanin
     *  DOGRULUGU degistirmedigini gosterdi (hata 0.92-1.23 px, farklar
     *  gurultu icinde); degisen sey yalnizca raporlanan guven. Taban 4 ile
     *  kapsama geri geliyor (tex 0.12-0.29), Q daha bilgilendirici oluyor.
     *  DOKULU sahnede taban tamamen ETKISIZ: orada uyarlanir kural
     *  (1.5 x p50) zaten daha yuksek — 8/6/4/3/2 hepsi ayni sonucu verdi.
     *  Gurultulu sahnede de koruma surer: gurultu p50'yi yukseltir, esik
     *  onunla birlikte tabanin uzerine cikar. Taban artik yalnizca
     *  "temiz ama dokusuz" sahnede baglayici — hedeflenen durum tam bu. */
    /** prefilter_sigma: OFA'ya ve doku haritasina girmeden ONCE uygulanan
     *  Gauss yumusatmasi (0 = kapali). OFA bir korelasyon algoritmasi;
     *  piksel gurultusu dogrudan korelasyon yuzeyini bozar. Klasik optik
     *  akis uygulamalari girisi ON-SUZER, biz suzmuyorduk.
     *
     *  26_07_24 olcumu (gercek ucus karesi, bilinen kaydirma, tools/
     *  grid_ab_test.cpp) — ortalama mutlak hata:
     *      gurultu     suzgecsiz    sigma=1.5     kazanc
     *      sigma=0     0.071 px     0.011 px      6.5x
     *      sigma=2     0.109 px     0.033 px      3.3x   <- olculen calisma noktasi
     *      sigma=6     0.369 px     0.127 px      2.9x   (los/yuksek gain)
     *  Yan etki de olumlu: sigma=2'de tex 0.16->0.28, consist 0.70->0.94,
     *  Q 44-55 -> 84-93 (dogruluk artarken guven metrigi de dogru raporluyor).
     *
     *  ONCE DENENEN VE ELENEN: grid_size 4->8 (hucre basina 4 kat piksel).
     *  Hucre ici gurultuyu azaltiyor ama vektor sayisini da dorde boluyor
     *  (1620 -> 390 ornek); RANSAC'in cok ornek uzerinden yaptigi ortalama
     *  daha guclu cikti — sigma=6'da hata 0.369'dan 0.689'a CIKTI. Grid 4 kaldi.
     *
     *  Cagiranin karesi DEGISMEZ (kopya uzerinde calisilir): video kaydi
     *  orijinali gorur, AprilTag zaten ayri node'da kendi aboneligiyle calisir. */
    OpticalFlowVPI(int grid_size, int num_levels, float ema_alpha,
                   float texture_thresh = 4.0f, float prefilter_sigma = 1.5f)
        : grid_size_(grid_size), num_levels_(num_levels), ema_alpha_(ema_alpha),
          texture_thresh_(texture_thresh), prefilter_sigma_(prefilter_sigma) {}

    ~OpticalFlowVPI()
    {
        if (vpi_stream_)   vpiStreamDestroy(vpi_stream_);
        if (img_cur_load_) vpiImageDestroy(img_cur_load_);
        if (img_prev_pl_)  vpiImageDestroy(img_prev_pl_);
        if (img_cur_pl_)   vpiImageDestroy(img_cur_pl_);
        if (img_prev_tmp_) vpiImageDestroy(img_prev_tmp_);
        if (img_cur_tmp_)  vpiImageDestroy(img_cur_tmp_);
        if (pyr_prev_tmp_) vpiPyramidDestroy(pyr_prev_tmp_);
        if (pyr_cur_tmp_)  vpiPyramidDestroy(pyr_cur_tmp_);
        if (pyr_prev_bl_)  vpiPyramidDestroy(pyr_prev_bl_);
        if (pyr_cur_bl_)   vpiPyramidDestroy(pyr_cur_bl_);
        if (img_mv_bl_)    vpiImageDestroy(img_mv_bl_);
        if (img_mv_pl_)    vpiImageDestroy(img_mv_pl_);
        if (vpi_payload_)  vpiPayloadDestroy(vpi_payload_);
    }

    OpticalFlowVPI(const OpticalFlowVPI &) = delete;
    OpticalFlowVPI &operator=(const OpticalFlowVPI &) = delete;

    bool initialized() const { return initialized_; }

    /** Sadece OLCUM/AYAR icin — uretimde varsayilanlar kullanilir.
     *  init()'ten once cagrilmali. */
    void setTuning(float sample_step, double ransac_thresh)
    { sample_step_ = sample_step; ransac_thresh_ = ransac_thresh; }
    int  paddedWidth()  const { return vpi_w_; }
    int  paddedHeight() const { return vpi_h_; }

    /** Ilk frame ile VPI objelerini kurar ve prev pyramid'i hazirlar.
     *  VPI hatasinda std::runtime_error firlatir. */
    void init(const cv::Mat &frame)
    {
        check(vpiStreamCreate(
            VPI_BACKEND_OFA | VPI_BACKEND_CUDA | VPI_BACKEND_VIC, &vpi_stream_),
            "vpiStreamCreate");

        int32_t multiple = (1 << (num_levels_ - 1));
        vpi_w_ = ((frame.cols + multiple - 1) / multiple) * multiple;
        vpi_h_ = ((frame.rows + multiple - 1) / multiple) * multiple;

        VPIImageFormat fmtU8   = VPI_IMAGE_FORMAT_U8;
        VPIImageFormat fmtY8   = VPI_IMAGE_FORMAT_Y8_ER;
        VPIImageFormat fmtY8BL = VPI_IMAGE_FORMAT_Y8_ER_BL;

        check(vpiImageCreate(vpi_w_, vpi_h_, fmtU8, 0, &img_prev_pl_), "img_prev_pl");
        check(vpiImageCreate(vpi_w_, vpi_h_, fmtU8, 0, &img_cur_pl_),  "img_cur_pl");
        check(vpiImageCreate(vpi_w_, vpi_h_, fmtY8, 0, &img_prev_tmp_),"img_prev_tmp");
        check(vpiImageCreate(vpi_w_, vpi_h_, fmtY8, 0, &img_cur_tmp_), "img_cur_tmp");

        check(vpiPyramidCreate(vpi_w_, vpi_h_, fmtY8,   num_levels_, 0.5, 0, &pyr_prev_tmp_), "pyr_prev_tmp");
        check(vpiPyramidCreate(vpi_w_, vpi_h_, fmtY8,   num_levels_, 0.5, 0, &pyr_cur_tmp_),  "pyr_cur_tmp");
        check(vpiPyramidCreate(vpi_w_, vpi_h_, fmtY8BL, num_levels_, 0.5, 0, &pyr_prev_bl_),  "pyr_prev_bl");
        check(vpiPyramidCreate(vpi_w_, vpi_h_, fmtY8BL, num_levels_, 0.5, 0, &pyr_cur_bl_),   "pyr_cur_bl");

        std::vector<int32_t> pyrGrid(num_levels_, grid_size_);
        check(vpiCreateOpticalFlowDense(
            VPI_BACKEND_OFA, vpi_w_, vpi_h_, fmtY8BL,
            pyrGrid.data(), pyrGrid.size(),
            VPI_OPTICAL_FLOW_QUALITY_HIGH, &vpi_payload_), "vpiCreateOpticalFlowDense");

        int32_t mvW = (vpi_w_ + grid_size_ - 1) / grid_size_;
        int32_t mvH = (vpi_h_ + grid_size_ - 1) / grid_size_;
        check(vpiImageCreate(mvW, mvH, VPI_IMAGE_FORMAT_2S16_BL, 0, &img_mv_bl_), "img_mv_bl");
        check(vpiImageCreate(mvW, mvH, VPI_IMAGE_FORMAT_2S16,    0, &img_mv_pl_), "img_mv_pl");

        // On-suzme ILK karede de uygulanmali: prev/cur ayni islemden gecmezse
        // aralarinda sahte bir "keskinlik farki" olusur ve ilk akis bozulur.
        const cv::Mat &src = prefilter(frame);

        // Ilk frame'den prev pyramid — wrapper gecici, sync sonrasi hemen yok edilir
        // (dangling wrapper birakmamak icin; frame Mat'i cagirandan sonra olebilir)
        VPIImage img_first = nullptr;
        check(vpiImageCreateWrapperOpenCVMat(src, 0, &img_first), "wrap first frame");
        try {
            check(vpiSubmitRescale(vpi_stream_, VPI_BACKEND_CUDA,
                img_first, img_prev_pl_, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0), "init rescale");
            check(vpiSubmitConvertImageFormat(vpi_stream_, VPI_BACKEND_CUDA,
                img_prev_pl_, img_prev_tmp_, nullptr), "init convert");
            check(vpiSubmitGaussianPyramidGenerator(vpi_stream_, VPI_BACKEND_CUDA,
                img_prev_tmp_, pyr_prev_tmp_, VPI_BORDER_CLAMP), "init pyramid");
            check(vpiSubmitConvertImageFormatPyramid(vpi_stream_, VPI_BACKEND_VIC,
                pyr_prev_tmp_, pyr_prev_bl_, nullptr), "init convert pyramid");
            check(vpiStreamSync(vpi_stream_), "init sync");
        } catch (...) {
            vpiImageDestroy(img_first);
            throw;
        }
        vpiImageDestroy(img_first);

        // Sonraki frame'ler icin kalici wrapper (SetWrapped ile yeniden baglanir)
        check(vpiImageCreateWrapperOpenCVMat(src, 0, &img_cur_load_), "wrap cur");

        initialized_ = true;
    }

    /** Bir frame isler, global dx/dy + quality dondurur.
     *  frame verisi cagri suresince gecerli olmali (icerde sync var).
     *  VPI hatasinda std::runtime_error firlatir. */
    bool process(const cv::Mat &frame, Result &out)
    {
        // On-suzme: akis VE doku haritasi ayni suzulmus kareyi gorur
        // (init'teki prev pyramid de ayni islemden gecti).
        const cv::Mat &src = prefilter(frame);
        check(vpiImageSetWrappedOpenCVMat(img_cur_load_, src), "SetWrapped");
        check(vpiSubmitRescale(vpi_stream_, VPI_BACKEND_CUDA,
            img_cur_load_, img_cur_pl_, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0), "rescale");
        check(vpiSubmitConvertImageFormat(vpi_stream_, VPI_BACKEND_CUDA,
            img_cur_pl_, img_cur_tmp_, nullptr), "convert");
        check(vpiSubmitGaussianPyramidGenerator(vpi_stream_, VPI_BACKEND_CUDA,
            img_cur_tmp_, pyr_cur_tmp_, VPI_BORDER_CLAMP), "pyramid");
        check(vpiSubmitConvertImageFormatPyramid(vpi_stream_, VPI_BACKEND_VIC,
            pyr_cur_tmp_, pyr_cur_bl_, nullptr), "convert pyramid");
        check(vpiSubmitOpticalFlowDensePyramid(vpi_stream_, VPI_BACKEND_OFA,
            vpi_payload_, pyr_prev_bl_, pyr_cur_bl_, img_mv_bl_), "optical flow");
        check(vpiSubmitConvertImageFormat(vpi_stream_, VPI_BACKEND_VIC,
            img_mv_bl_, img_mv_pl_, nullptr), "convert mv");
        check(vpiStreamSync(vpi_stream_), "sync");

        std::swap(pyr_prev_bl_, pyr_cur_bl_);

        VPIImageData mvData;
        check(vpiImageLockData(img_mv_pl_, VPI_LOCK_READ,
            VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData), "lock mv");
        cv::Mat mvImage;
        VPIStatus st = vpiImageDataExportOpenCVMat(mvData, &mvImage);
        if (st != VPI_SUCCESS) {
            vpiImageUnlock(img_mv_pl_);
            check(st, "export mv");
        }
        cv::Mat flowMat(mvImage.size(), CV_32FC2);
        mvImage.convertTo(flowMat, CV_32F, 1.0f / (1 << 5));   // S10.5 → px
        vpiImageUnlock(img_mv_pl_);

        // Doku haritasi: flow hucresi basina ortalama gradyan buyuklugu.
        // Dokusuz hucreler (sature beton, duz zemin, golge ici) quality
        // paydasina girmez VE — yeterli dokulu ornek varsa — RANSAC
        // tahmininden de dislanir: OFA bu hucrelerde ~0 dondurur, tahmine
        // sokulunca gercek hareketi sifira ceker (gunesli ucuslarda
        // olculdu: %17-41 saturasyon "hareket yok" cogunlugu kuruyordu).
        cv::Mat texMap;
        if (src.type() == CV_8UC1) {
            cv::Mat gx, gy;
            cv::spatialGradient(src, gx, gy);          // CV_16S ciftler
            cv::Mat mag = cv::abs(gx) + cv::abs(gy);
            cv::Mat mag8;
            mag.convertTo(mag8, CV_8U);   // >255 doyar; esik ~30, onemsiz
            // INTER_AREA = hucre ici ortalama; texMap flow grid'iyle hizali
            cv::resize(mag8, texMap, flowMat.size(), 0, 0, cv::INTER_AREA);
        }

        // Gradyan dagilim percentileri — texture_thresh'in sahneye gore
        // kalibrasyonu icin debug bilgisi (p50 ~gurultu tabani, p90 ~doku)
        uint8_t tex_p50 = 0, tex_p90 = 0;
        if (!texMap.empty()) {
            int hist[256] = {0};
            for (int y = 0; y < texMap.rows; ++y) {
                const uint8_t *rp = texMap.ptr<uint8_t>(y);
                for (int x = 0; x < texMap.cols; ++x) hist[rp[x]]++;
            }
            const int total = texMap.rows * texMap.cols;
            auto pct = [&](float p) -> uint8_t {
                int need = (int)(p * total), acc = 0;
                for (int v = 0; v < 256; ++v) {
                    acc += hist[v];
                    if (acc >= need) return (uint8_t)v;
                }
                return 255;
            };
            tex_p50 = pct(0.50f);
            tex_p90 = pct(0.90f);
        }

        // SAHNEYE UYARLANIR ESIK: gurultu tabani isikla degistigi icin
        // sabit esik ya gunduz gurultuyu iceri alir ya aksam dokuyu disari
        // atar. p50 tabani her karede olcer: esik = max(taban, 1.5 x p50).
        // Not: tamamen dokulu sahnede p50 gurultuyu degil dokuyu olcer ve
        // esik yukari kayar — zararsiz, o sahnede esigi asan hucre bol.
        const float tex_thresh_eff =
            std::max(texture_thresh_, 1.5f * (float)tex_p50);

        // Deger bazli dead-zone YOK: DOKULU bir hucrenin sifir vektoru
        // gecerli "hareket yok" olcumudur, atilirsa hover/yavas drift
        // rejiminde (0.1-0.4 px/frame) tahmin bias'lanir. Filtre yalnizca
        // DOKU bazlidir (hucre olcum yapabiliyor mu) — vektorun degerine
        // bakilmaz, hover'da tahmin dogal olarak ~0 ve quality yuksek.
        //
        // TUM yollar EMA'dan gecer — yumusatma tutarli, uzun kesinti
        // sonrasi bayat EMA problemi olusmaz (her frame gunceller).
        cv::Vec2f gdxdy;
        float consist = 0.0f, tex_ratio = 0.0f;
        uint8_t quality;
        if (estimateGlobalDxDy(flowMat, gdxdy, texMap, tex_thresh_eff,
                               &consist, &tex_ratio,
                               sample_step_, ransac_thresh_)) {
            // Kapsama cezasi: dokulu hucre orani cok dusukse (gece, dumduz
            // sahne) "herkes ayni fikirde" onemsiz sekilde dogru olabilir —
            // az taniga yuksek guven verilmez. %25 kapsamaya kadar lineer
            // olceklenir, ustu tam guven.
            float cover = tex_ratio / kFullCoverageRatio;
            if (cover > 1.0f) cover = 1.0f;
            float q = 100.0f * consist * cover;
            if (q < 0.0f)   q = 0.0f;
            if (q > 100.0f) q = 100.0f;
            quality = static_cast<uint8_t>(q);
        } else {
            // Tutarli global model yok — mean fallback, dusuk guven
            cv::Scalar m = cv::mean(flowMat);
            gdxdy = cv::Vec2f((float)m[0], (float)m[1]);
            quality = QUALITY_UNRELIABLE;
        }
        // HAM degerler: EMA'dan gecmeden, o karenin kendi olcumu
        out.raw_dx      = gdxdy[0];
        out.raw_dy      = gdxdy[1];
        out.raw_quality = quality;

        ema_ = (1.0f - ema_alpha_) * ema_ + ema_alpha_ * gdxdy;
        // Quality de EMA'lanir: frame basi istatistik gurultulu (sabit
        // sahnede ±15 zipliyordu), otopilot agirlik olarak kullanacagi
        // icin kararli olmali. Kotu duruma gecis ~5 frame'de (<0.2 s)
        // asagi iner — tehlikeli gecikme yaratmaz.
        quality_ema_ = (1.0f - ema_alpha_) * quality_ema_
                     + ema_alpha_ * (float)quality;
        out.dx = ema_[0];
        out.dy = ema_[1];
        out.quality   = (uint8_t)(quality_ema_ + 0.5f);
        out.consist   = consist;
        out.tex_ratio = tex_ratio;
        out.tex_p50   = tex_p50;
        out.tex_p90   = tex_p90;
        out.tex_thresh = tex_thresh_eff;
        return true;
    }

private:
    static constexpr uint8_t QUALITY_UNRELIABLE = 10;
    // Bu orandan az dokulu hucre varsa quality lineer cezalanir —
    // 3-5 hucrelik "azinlik konsensusu"ne tam guven verilmesin diye.
    static constexpr float   kFullCoverageRatio = 0.25f;

    /** Girisi akis/doku hesabina hazirlar (gurultu on-suzmesi).
     *  Kapaliysa cagiranin Mat'ini oldugu gibi dondurur — kopya maliyeti yok.
     *  Aciksa KOPYA uzerinde calisir; cagiranin karesine dokunulmaz. */
    const cv::Mat &prefilter(const cv::Mat &frame)
    {
        if (prefilter_sigma_ <= 0.0f) return frame;
        cv::GaussianBlur(frame, filtered_, cv::Size(0, 0), prefilter_sigma_);
        return filtered_;
    }

    static void check(VPIStatus s, const char *what)
    {
        if (s != VPI_SUCCESS) {
            char buf[256];
            vpiGetLastStatusMessage(buf, sizeof(buf));
            throw std::runtime_error(std::string("VPI ") + what + ": " + buf);
        }
    }

    /** RANSAC global motion estimator (VPI flow → dx,dy)
     *
     *  Fit seti: yeterli dokulu ornek varsa SADECE dokulu hucreler.
     *  Dokusuz hucreler (sature/duz bolge) OFA'da hep ~0 dondurup gercek
     *  hareketi sifira ceker — gunesli ucus kayitlarinda olculdu. Deger
     *  bazli dead-zone YOK: dokulu hucrenin sifir vektoru gecerli "hareket
     *  yok" olcumudur (hover bias'lanmaz). RANSAC genis esikle (3px) model
     *  tutarliligini saglar; hover'da tum noktalar inlier olup
     *  least-squares'e dejenere olur — dogru davranis (medyan blend
     *  sifirda bias'siz).
     *
     *  Quality kaynagi RANSAC'in kendi inlier orani DEGIL (3px esik
     *  hover'daki 0.1px hareketin 30 kati, hep %100'e doyar):
     *  outConsist: fit setinin final tahmine yumusak uyum skoru 0-1.
     *  outTexRatio: dokulu ornek orani 0-1 (texMap >= texThresh).
     *  texMap bos verilirse tum ornekler dokulu sayilir. */
    static bool estimateGlobalDxDy(
        const cv::Mat &flow, cv::Vec2f &out,
        const cv::Mat &texMap, float texThresh,
        float *outConsist = nullptr, float *outTexRatio = nullptr,
        float sampleStep = 6.0f,
        double ransacThresh = 3.0, double confidence = 0.99,
        int refineIters = 10, float minInlierRatio = 0.05f)
    {
        CV_Assert(flow.type() == CV_32FC2);
        CV_Assert(texMap.empty() ||
                  (texMap.type() == CV_8UC1 && texMap.size() == flow.size()));
        if (outConsist)  *outConsist  = 0.0f;
        if (outTexRatio) *outTexRatio = 0.0f;

        std::vector<cv::Point2f> src, dst;           // fit seti (dokulu)
        std::vector<cv::Point2f> src_all, dst_all;   // fallback icin tumu
        src_all.reserve((flow.cols / (int)sampleStep + 1) *
                        (flow.rows / (int)sampleStep + 1));
        dst_all.reserve(src_all.capacity());

        for (int y = 0; y < flow.rows; y += (int)sampleStep) {
            const cv::Vec2f *row  = flow.ptr<cv::Vec2f>(y);
            const uint8_t   *trow = texMap.empty() ? nullptr
                                                   : texMap.ptr<uint8_t>(y);
            for (int x = 0; x < flow.cols; x += (int)sampleStep) {
                cv::Point2f s((float)x, (float)y);
                cv::Point2f d((float)x + row[x][0], (float)y + row[x][1]);
                src_all.push_back(s);
                dst_all.push_back(d);
                if (!trow || (float)trow[x] >= texThresh) {
                    src.push_back(s);
                    dst.push_back(d);
                }
            }
        }
        if (src_all.size() < 6) return false;   // dejenere (bos/kucuk alan)

        if (outTexRatio)
            *outTexRatio = (float)src.size() / (float)src_all.size();

        // Dokulu ornek cok azsa (gece, tam sature/dumduz sahne) fit tum
        // orneklere doner — akis zaten guvenilmez, quality kapsama
        // cezasiyla dusuk raporlanir; ama en azindan tahmin uretilir.
        constexpr size_t kMinTexSamples = 20;
        if (src.size() < kMinTexSamples) {
            src = src_all;
            dst = dst_all;
        }

        cv::Mat inlierMask;
        cv::Mat A = cv::estimateAffinePartial2D(src, dst, inlierMask,
            cv::RANSAC, ransacThresh, 2000, confidence, refineIters);
        if (A.empty()) return false;

        int inlierCnt = cv::countNonZero(inlierMask);
        double ratio = (double)inlierCnt / (double)src.size();
        if (ratio < minInlierRatio) return false;

        float tx = (float)A.at<double>(0, 2);
        float ty = (float)A.at<double>(1, 2);

        std::vector<float> vdx, vdy;
        vdx.reserve(inlierCnt); vdy.reserve(inlierCnt);
        for (size_t i = 0; i < src.size(); ++i) {
            if (!inlierMask.data[i]) continue;
            vdx.push_back(dst[i].x - src[i].x);
            vdy.push_back(dst[i].y - src[i].y);
        }
        if (!vdx.empty()) {
            auto mid = vdx.begin() + vdx.size() / 2;
            std::nth_element(vdx.begin(), mid, vdx.end());
            std::nth_element(vdy.begin(), vdy.begin() + vdy.size() / 2, vdy.end());
            constexpr float alpha = 0.5f;
            out = cv::Vec2f(alpha * tx + (1 - alpha) * (*mid),
                            alpha * ty + (1 - alpha) * vdy[vdy.size() / 2]);
        } else {
            out = cv::Vec2f(tx, ty);
        }

        // Consistency: fit setinin final tahmine uyumu (quality kaynagi).
        // Fit seti normalde dokulu hucreler; dokusuz-sahne fallback'inde
        // tum hucreler — o durumda quality zaten kapsama cezasiyla ~0.
        //
        // Esik HAREKETLE OLCEKLENIR: kamera oteleme yaparken yakin cisimler
        // uzaktakilerden daha cok piksel kayar (parallax, akis ∝ 1/derinlik).
        // Sabit esik hizli harekette quality'yi hakszca sifira cekiyordu
        // (olculdu: sabit=~%40, 20px/frame harekette ~%0-5).
        //   esik = 1.5 px + 0.15 x |tahmin|
        // Taban 1.5 px = OFA'nin dokulu hucrelerdeki OLCULEN gurultusunun
        // (~0.7 px rms, sabit kamera/dokulu sahne) ~2 kati — taban gurultu
        // seviyesine inerse saglam sahnede bile skor ~0.5'e yapisiyor.
        //
        // Ikili say/sayma yerine YUMUSAK skor: tam uyum → 1, esikte → 0,
        // karesel gecis (kucuk artiklar neredeyse cezasiz). Esik kenarina
        // yigilan orneklerin quality'yi 40↔60 ziplatmasini onler.
        const float motion_mag = std::abs(out[0]) + std::abs(out[1]);
        const float thresh_px  = 1.5f + 0.15f * motion_mag;
        float score_sum = 0.0f;
        for (size_t i = 0; i < src.size(); ++i) {
            float r = std::abs((dst[i].x - src[i].x) - out[0]) +
                      std::abs((dst[i].y - src[i].y) - out[1]);
            float rn = r / thresh_px;
            float s  = 1.0f - rn * rn;
            if (s > 0.0f) score_sum += s;
        }
        if (outConsist && !src.empty())
            *outConsist = score_sum / (float)src.size();
        return true;
    }

    int   grid_size_;
    int   num_levels_;
    float ema_alpha_;
    float texture_thresh_;
    float prefilter_sigma_;
    // Olcum icin disari acildi; VARSAYILANLAR ESKI DAVRANISLA BIREBIR AYNI
    // (estimateGlobalDxDy imzasindaki degerlerle esit) — tek basina
    // hicbir seyi degistirmez, sadece A/B olcumunu mumkun kilar.
    float sample_step_   = 6.0f;
    double ransac_thresh_ = 3.0;
    cv::Mat filtered_;      // on-suzulmus kopya (cagiranin karesi korunur)
    bool  initialized_ = false;
    int   vpi_w_ = 0, vpi_h_ = 0;
    cv::Vec2f ema_{0, 0};
    float     quality_ema_ = 0.0f;

    VPIStream   vpi_stream_    = nullptr;
    VPIImage    img_cur_load_  = nullptr;
    VPIImage    img_prev_pl_   = nullptr, img_cur_pl_  = nullptr;
    VPIImage    img_prev_tmp_  = nullptr, img_cur_tmp_ = nullptr;
    VPIPyramid  pyr_prev_tmp_  = nullptr, pyr_cur_tmp_ = nullptr;
    VPIPyramid  pyr_prev_bl_   = nullptr, pyr_cur_bl_  = nullptr;
    VPIImage    img_mv_bl_     = nullptr, img_mv_pl_   = nullptr;
    VPIPayload  vpi_payload_   = nullptr;
};

} // namespace lps
