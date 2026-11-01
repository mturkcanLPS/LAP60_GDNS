#include "optflow.h"

DenseOptFlowOFA::DenseOptFlowOFA(int width, int height,
                                 int gridSize, int numLevels,
                                 VPIOpticalFlowQuality quality)
    : w_(width), h_(height),
      gridSize_(gridSize), numLevels_(numLevels),
      quality_(quality)
{
    // Stream: OFA + CUDA + VIC (dönüşümler için)
    CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_OFA | VPI_BACKEND_CUDA | VPI_BACKEND_VIC, &stream_));

    // Giriş PL tamponu (BGR->Y8 veya doğrudan Y8 için)
    CHECK_STATUS(vpiImageCreate(w_, h_, VPI_IMAGE_FORMAT_Y8_ER, 0, &imgTmpY8PL_));

    // Piramitler (PL -> BL dönüşümleri için hem geçici hem BL)
    CHECK_STATUS(vpiPyramidCreate(w_, h_, VPI_IMAGE_FORMAT_Y8_ER,    numLevels_, 0.5, 0, &pyrPrevPL_));
    CHECK_STATUS(vpiPyramidCreate(w_, h_, VPI_IMAGE_FORMAT_Y8_ER,    numLevels_, 0.5, 0, &pyrCurPL_ ));
    CHECK_STATUS(vpiPyramidCreate(w_, h_, VPI_IMAGE_FORMAT_Y8_ER_BL, numLevels_, 0.5, 0, &pyrPrevBL_));
    CHECK_STATUS(vpiPyramidCreate(w_, h_, VPI_IMAGE_FORMAT_Y8_ER_BL, numLevels_, 0.5, 0, &pyrCurBL_ ));

    // Motion vectors: 2S16 BL + PL (görselleştirme/erişim PL’den)
    mvW_ = (w_ + gridSize_ - 1) / gridSize_;
    mvH_ = (h_ + gridSize_ - 1) / gridSize_;
    CHECK_STATUS(vpiImageCreate(mvW_, mvH_, VPI_IMAGE_FORMAT_2S16_BL, 0, &imgMV_BL_));
    CHECK_STATUS(vpiImageCreate(mvW_, mvH_, VPI_IMAGE_FORMAT_2S16,    0, &imgMV_PL_));

    // OFA payload
    std::vector<int32_t> grids(numLevels_, gridSize_);
    CHECK_STATUS(vpiCreateOpticalFlowDense(VPI_BACKEND_OFA, w_, h_,
                                           VPI_IMAGE_FORMAT_Y8_ER_BL,
                                           grids.data(), (int)grids.size(),
                                           quality_, &payload_));
}

DenseOptFlowOFA::~DenseOptFlowOFA()
{
    if (payload_)    vpiPayloadDestroy(payload_);
    if (imgMV_PL_)   vpiImageDestroy(imgMV_PL_);
    if (imgMV_BL_)   vpiImageDestroy(imgMV_BL_);
    if (pyrCurBL_)   vpiPyramidDestroy(pyrCurBL_);
    if (pyrPrevBL_)  vpiPyramidDestroy(pyrPrevBL_);
    if (pyrCurPL_)   vpiPyramidDestroy(pyrCurPL_);
    if (pyrPrevPL_)  vpiPyramidDestroy(pyrPrevPL_);
    if (imgTmpY8PL_) vpiImageDestroy(imgTmpY8PL_);
    if (imgCurWrap_) vpiImageDestroy(imgCurWrap_);
    if (stream_)     vpiStreamDestroy(stream_);
}

bool DenseOptFlowOFA::process(const cv::Mat &frame, FlowStats *stats)
{
    if (frame.empty())
        return false;

    // Wrap / set
    if (!imgCurWrap_)
        CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(frame, 0, &imgCurWrap_));
    else
        CHECK_STATUS(vpiImageSetWrappedOpenCVMat(imgCurWrap_, frame));

    // BGR8 veya Y8 -> Y8/PL
    CHECK_STATUS(vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_CUDA,
                                             imgCurWrap_, imgTmpY8PL_, nullptr));

    // PL -> pyramid(PL)
    CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA,
                                                   imgTmpY8PL_, pyrCurPL_, VPI_BORDER_CLAMP));

    // pyramid(PL) -> pyramid(BL)
    CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream_, VPI_BACKEND_VIC,
                                                    pyrCurPL_, pyrCurBL_, nullptr));

    bool produced = false;

    if (hasPrev_) {
        // OFA: prev vs cur
        CHECK_STATUS(vpiSubmitOpticalFlowDensePyramid(stream_, VPI_BACKEND_OFA,
                                                      payload_, pyrPrevBL_, pyrCurBL_, imgMV_BL_));

        // BL -> PL (host pitch-linear)
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_VIC,
                                                 imgMV_BL_, imgMV_PL_, nullptr));

        CHECK_STATUS(vpiStreamSync(stream_));

        // ---- 2S16(S10.5) -> float flow (CV_32FC2) ----
        VPIImageData mvData{};
        CHECK_STATUS(vpiImageLockData(imgMV_PL_, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData));
        cv::Mat mvS16;
        CHECK_STATUS(vpiImageDataExportOpenCVMat(mvData, &mvS16));
        cv::Mat flowFC2; // (dx,dy) piksel
        mvS16.convertTo(flowFC2, CV_32F, 1.0f / (1 << 5));
        CHECK_STATUS(vpiImageUnlock(imgMV_PL_));

        std::vector<cv::Mat> ch(2);
        cv::split(flowFC2, ch);

        // Basit istatistik: küçük titreşimleri at
        cv::Mat mag, ang;
        cv::cartToPolar(ch[0], ch[1], mag, ang, false);
        cv::Mat mask = mag > 0.1f;

        if (stats) {
            cv::Scalar mDx = cv::mean(ch[0], mask);
            cv::Scalar mDy = cv::mean(ch[1], mask);
            stats->mean_dx = static_cast<float>(mDx[0]);
            stats->mean_dy = static_cast<float>(mDy[0]);
        }

        produced = true;
    } else {
        CHECK_STATUS(vpiStreamSync(stream_));
        hasPrev_ = true;
    }

    // prev <- cur
    std::swap(pyrPrevPL_, pyrCurPL_);
    std::swap(pyrPrevBL_, pyrCurBL_);

    return produced;
}

void DenseOptFlowOFA::renderMVtoBGR(VPIImage mvPL, cv::Mat &outBGR)
{
    VPIImageData mvData{};
    CHECK_STATUS(vpiImageLockData(mvPL, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData));
    cv::Mat mvMat;
    CHECK_STATUS(vpiImageDataExportOpenCVMat(mvData, &mvMat));
    cv::Mat flow; // CV_32FC2
    mvMat.convertTo(flow, CV_32F, 1.0f / (1 << 5));
    CHECK_STATUS(vpiImageUnlock(mvPL));

    std::vector<cv::Mat> ch(2);
    cv::split(flow, ch);

    cv::Mat mag, ang;
    cv::cartToPolar(ch[0], ch[1], mag, ang, true);

    const float clip = 5.f;
    cv::threshold(mag, mag, clip, clip, cv::THRESH_TRUNC);

    std::vector<cv::Mat> hsvCh(3);
    hsvCh[0] = ang;                               // H: 0..360
    hsvCh[1] = cv::Mat::ones(ang.size(), CV_32F); // S: 1
    hsvCh[2] = mag / clip;                        // V: 0..1

    cv::Mat hsv;
    cv::merge(hsvCh, hsv);
    cv::cvtColor(hsv, outBGR, cv::COLOR_HSV2BGR);
    outBGR.convertTo(outBGR, CV_8U, 255.0);
}
