#pragma once
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Array.h>
#include <vpi/Image.h>
#include <vpi/ImageFormat.h>
#include <vpi/Pyramid.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/OpticalFlowDense.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <sstream>
#include <vector>

#ifndef CHECK_STATUS
#define CHECK_STATUS(STMT)                                    \
    do {                                                      \
        VPIStatus _st = (STMT);                               \
        if (_st != VPI_SUCCESS) {                             \
            char _buf[VPI_MAX_STATUS_MESSAGE_LENGTH];         \
            vpiGetLastStatusMessage(_buf, sizeof(_buf));      \
            std::ostringstream _ss;                           \
            _ss << #STMT << " -> " << vpiStatusGetName(_st)   \
                << " : " << _buf;                             \
            throw std::runtime_error(_ss.str());              \
        }                                                     \
    } while (0)
#endif

class DenseOptFlowOFA {
public:
    // quality: VPI_OPTICAL_FLOW_QUALITY_LOW/MEDIUM/HIGH
    // gridSize: 1,2,4,8
    // numLevels: 1..7
    DenseOptFlowOFA(int width, int height, int gridSize, int numLevels, VPIOpticalFlowQuality quality);
    ~DenseOptFlowOFA();

    struct FlowStats { float mean_dx = 0.f, mean_dy = 0.f; };

    // İlk çağrıda prev oluşturulur, ikinci ve sonrasında akış üretir.
    // Üretildiyse true döner; ilk karede false döner.
    bool process(const cv::Mat &frame, FlowStats *stats = nullptr);

    int mvWidth()  const { return mvW_; }
    int mvHeight() const { return mvH_; }

private:
    // MV (2S16, S10.5) → false-color BGR
    static void renderMVtoBGR(VPIImage mvPL, cv::Mat &outBGR);

    int w_{}, h_{};
    int gridSize_{}, numLevels_{};
    int mvW_{}, mvH_{};
    VPIOpticalFlowQuality quality_{};

    VPIStream   stream_      = nullptr;
    VPIImage    imgCurWrap_  = nullptr; // wrapped OpenCV frame (BGR or Y8)
    VPIImage    imgTmpY8PL_  = nullptr; // converted to Y8/PL

    VPIPyramid  pyrPrevPL_   = nullptr;
    VPIPyramid  pyrPrevBL_   = nullptr;
    VPIPyramid  pyrCurPL_    = nullptr;
    VPIPyramid  pyrCurBL_    = nullptr;

    VPIImage    imgMV_BL_    = nullptr;
    VPIImage    imgMV_PL_    = nullptr;

    VPIPayload  payload_     = nullptr;

    bool        hasPrev_     = false;
};
