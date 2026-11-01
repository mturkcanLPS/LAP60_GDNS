/**
 * ROS 2 VPI Optical Flow Node
 * ============================
 * /camera/camera/infra1/image_rect_raw → VPI OFA dense flow → RANSAC → UDP
 * Her frame geldiğinde hesaplar ve gönderir, rate limit yok.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <vpi/OpenCVInterop.hpp>
#include <vpi/Image.h>
#include <vpi/ImageFormat.h>
#include <vpi/Pyramid.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/OpticalFlowDense.h>
#include <vpi/algo/Rescale.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

#define CHECK_STATUS(STMT)                                    \
    do {                                                      \
        VPIStatus _s = (STMT);                                \
        if (_s != VPI_SUCCESS) {                              \
            char buf[256];                                    \
            vpiGetLastStatusMessage(buf, sizeof(buf));         \
            RCLCPP_FATAL(rclcpp::get_logger("vpi"),           \
                "%s:%d VPI: %s", __FILE__, __LINE__, buf);    \
            throw std::runtime_error(std::string("VPI: ") + buf); \
        }                                                     \
    } while (0)

static bool estimateGlobalDxDyRANSAC(
    const cv::Mat &flow, cv::Vec2f &robustDxDy,
    float sampleStep = 6.0f, float magDeadZone = 0.10f,
    double ransacThresh = 3.0, double confidence = 0.99,
    int refineIters = 10, float minInlierRatio = 0.05f)
{
    CV_Assert(flow.type() == CV_32FC2);
    std::vector<cv::Point2f> src, dst;
    src.reserve((flow.cols/(int)sampleStep+1)*(flow.rows/(int)sampleStep+1));
    dst.reserve(src.capacity());

    for (int y = 0; y < flow.rows; y += (int)sampleStep) {
        const cv::Vec2f* row = flow.ptr<cv::Vec2f>(y);
        for (int x = 0; x < flow.cols; x += (int)sampleStep) {
            float u = row[x][0], v = row[x][1];
            if (std::abs(u)+std::abs(v) < magDeadZone) continue;
            src.emplace_back((float)x,(float)y);
            dst.emplace_back((float)x+u,(float)y+v);
        }
    }
    if (src.size() < 6) return false;

    cv::Mat inlierMask;
    cv::Mat A = cv::estimateAffinePartial2D(src, dst, inlierMask,
        cv::RANSAC, ransacThresh, 2000, confidence, refineIters);
    if (A.empty()) return false;

    int inlierCnt = cv::countNonZero(inlierMask);
    if ((double)inlierCnt/(double)src.size() < minInlierRatio) return false;

    float tx = (float)A.at<double>(0,2);
    float ty = (float)A.at<double>(1,2);

    std::vector<float> vdx, vdy;
    vdx.reserve(inlierCnt); vdy.reserve(inlierCnt);
    for (size_t i = 0; i < src.size(); ++i) {
        if (!inlierMask.data[i]) continue;
        vdx.push_back(dst[i].x - src[i].x);
        vdy.push_back(dst[i].y - src[i].y);
    }
    if (!vdx.empty()) {
        auto mid = vdx.begin()+vdx.size()/2;
        std::nth_element(vdx.begin(), mid, vdx.end());
        std::nth_element(vdy.begin(), vdy.begin()+vdy.size()/2, vdy.end());
        float alpha = 0.5f;
        robustDxDy = cv::Vec2f(alpha*tx+(1-alpha)*(*mid),
                               alpha*ty+(1-alpha)*vdy[vdy.size()/2]);
    } else {
        robustDxDy = cv::Vec2f(tx, ty);
    }
    return true;
}

class VpiOpticalFlowNode : public rclcpp::Node
{
public:
    VpiOpticalFlowNode() : Node("vpi_optical_flow")
    {
        start_time_ = std::chrono::steady_clock::now();
        declare_parameter("image_topic", "/camera/camera/infra1/image_rect_raw");
        declare_parameter("udp_host", "127.0.0.1");
        declare_parameter("udp_port", 5546);
        declare_parameter("grid_size", 4);
        declare_parameter("num_levels", 3);
        declare_parameter("ema_alpha", 0.25);

        image_topic_ = get_parameter("image_topic").as_string();
        udp_host_    = get_parameter("udp_host").as_string();
        udp_port_    = get_parameter("udp_port").as_int();
        grid_size_   = get_parameter("grid_size").as_int();
        num_levels_  = get_parameter("num_levels").as_int();
        ema_alpha_   = static_cast<float>(get_parameter("ema_alpha").as_double());

        // UDP socket
        udp_sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sock_ < 0) throw std::runtime_error("UDP socket oluşturulamadı");

        std::memset(&udp_addr_, 0, sizeof(udp_addr_));
        udp_addr_.sin_family = AF_INET;
        udp_addr_.sin_port = htons(static_cast<uint16_t>(udp_port_));
        if (::inet_pton(AF_INET, udp_host_.c_str(), &udp_addr_.sin_addr) != 1)
            throw std::runtime_error("Geçersiz UDP host: " + udp_host_);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&VpiOpticalFlowNode::image_cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "VPI OpticalFlow: %s → UDP %s:%d (grid=%d levels=%d ema=%.2f)",
            image_topic_.c_str(), udp_host_.c_str(), udp_port_,
            grid_size_, num_levels_, ema_alpha_);
    }

    ~VpiOpticalFlowNode()
    {
        if (stream_)           vpiStreamDestroy(stream_);
        if (imgPrevFrameLoad_) vpiImageDestroy(imgPrevFrameLoad_);
        if (imgCurFrameLoad_)  vpiImageDestroy(imgCurFrameLoad_);
        if (imgPrevFramePL_)   vpiImageDestroy(imgPrevFramePL_);
        if (imgCurFramePL_)    vpiImageDestroy(imgCurFramePL_);
        if (imgPrevFrameTmp_)  vpiImageDestroy(imgPrevFrameTmp_);
        if (imgCurFrameTmp_)   vpiImageDestroy(imgCurFrameTmp_);
        if (prevPyrTmp_)       vpiPyramidDestroy(prevPyrTmp_);
        if (curPyrTmp_)        vpiPyramidDestroy(curPyrTmp_);
        if (prevPyrBL_)        vpiPyramidDestroy(prevPyrBL_);
        if (curPyrBL_)         vpiPyramidDestroy(curPyrBL_);
        if (imgMvBL_)          vpiImageDestroy(imgMvBL_);
        if (imgMvPL_)          vpiImageDestroy(imgMvPL_);
        if (payload_)          vpiPayloadDestroy(payload_);
        if (udp_sock_ >= 0)    ::close(udp_sock_);
    }

private:
    void vpi_init(const cv::Mat &frame)
    {
        CHECK_STATUS(vpiStreamCreate(
            VPI_BACKEND_OFA | VPI_BACKEND_CUDA | VPI_BACKEND_VIC, &stream_));

        CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(frame, 0, &imgPrevFrameLoad_));
        CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(frame, 0, &imgCurFrameLoad_));

        int32_t multiple = (1 << (num_levels_ - 1));
        w_ = ((frame.cols + multiple - 1) / multiple) * multiple;
        h_ = ((frame.rows + multiple - 1) / multiple) * multiple;

        VPIImageFormat fmtU8   = VPI_IMAGE_FORMAT_U8;
        VPIImageFormat fmtY8   = VPI_IMAGE_FORMAT_Y8_ER;
        VPIImageFormat fmtY8BL = VPI_IMAGE_FORMAT_Y8_ER_BL;

        CHECK_STATUS(vpiImageCreate(w_, h_, fmtU8, 0, &imgPrevFramePL_));
        CHECK_STATUS(vpiImageCreate(w_, h_, fmtU8, 0, &imgCurFramePL_));
        CHECK_STATUS(vpiImageCreate(w_, h_, fmtY8, 0, &imgPrevFrameTmp_));
        CHECK_STATUS(vpiImageCreate(w_, h_, fmtY8, 0, &imgCurFrameTmp_));

        CHECK_STATUS(vpiPyramidCreate(w_, h_, fmtY8, num_levels_, 0.5, 0, &prevPyrTmp_));
        CHECK_STATUS(vpiPyramidCreate(w_, h_, fmtY8, num_levels_, 0.5, 0, &curPyrTmp_));
        CHECK_STATUS(vpiPyramidCreate(w_, h_, fmtY8BL, num_levels_, 0.5, 0, &prevPyrBL_));
        CHECK_STATUS(vpiPyramidCreate(w_, h_, fmtY8BL, num_levels_, 0.5, 0, &curPyrBL_));

        std::vector<int32_t> pyrGridSize(num_levels_, grid_size_);
        CHECK_STATUS(vpiCreateOpticalFlowDense(
            VPI_BACKEND_OFA, w_, h_, fmtY8BL,
            pyrGridSize.data(), pyrGridSize.size(),
            VPI_OPTICAL_FLOW_QUALITY_HIGH, &payload_));

        int32_t mvW = (w_ + grid_size_ - 1) / grid_size_;
        int32_t mvH = (h_ + grid_size_ - 1) / grid_size_;
        CHECK_STATUS(vpiImageCreate(mvW, mvH, VPI_IMAGE_FORMAT_2S16_BL, 0, &imgMvBL_));
        CHECK_STATUS(vpiImageCreate(mvW, mvH, VPI_IMAGE_FORMAT_2S16, 0, &imgMvPL_));

        CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA,
            imgPrevFrameLoad_, imgPrevFramePL_, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_CUDA,
            imgPrevFramePL_, imgPrevFrameTmp_, nullptr));
        CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA,
            imgPrevFrameTmp_, prevPyrTmp_, VPI_BORDER_CLAMP));
        CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream_, VPI_BACKEND_VIC,
            prevPyrTmp_, prevPyrBL_, nullptr));
        CHECK_STATUS(vpiStreamSync(stream_));

        initialized_ = true;
        RCLCPP_INFO(get_logger(), "VPI OFA init: %dx%d → padded %dx%d",
                    frame.cols, frame.rows, w_, h_);
    }

    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "mono8")->image.clone();
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "cv_bridge: %s", e.what());
            return;
        }
        if (frame.empty()) return;

        if (!initialized_) {
            vpi_init(frame);
            return;
        }

        CHECK_STATUS(vpiImageSetWrappedOpenCVMat(imgCurFrameLoad_, frame));

        CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA,
            imgCurFrameLoad_, imgCurFramePL_, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_CUDA,
            imgCurFramePL_, imgCurFrameTmp_, nullptr));
        CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA,
            imgCurFrameTmp_, curPyrTmp_, VPI_BORDER_CLAMP));
        CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream_, VPI_BACKEND_VIC,
            curPyrTmp_, curPyrBL_, nullptr));

        CHECK_STATUS(vpiSubmitOpticalFlowDensePyramid(stream_, VPI_BACKEND_OFA,
            payload_, prevPyrBL_, curPyrBL_, imgMvBL_));
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_VIC,
            imgMvBL_, imgMvPL_, nullptr));
        CHECK_STATUS(vpiStreamSync(stream_));

        std::swap(prevPyrBL_, curPyrBL_);

        VPIImageData mvData;
        CHECK_STATUS(vpiImageLockData(imgMvPL_, VPI_LOCK_READ,
            VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData));
        cv::Mat mvImage;
        CHECK_STATUS(vpiImageDataExportOpenCVMat(mvData, &mvImage));
        cv::Mat flow(mvImage.size(), CV_32FC2);
        mvImage.convertTo(flow, CV_32F, 1.0f / (1 << 5));
        CHECK_STATUS(vpiImageUnlock(imgMvPL_));

        cv::Vec2f gdxdy;
        float dx = 0, dy = 0;
        uint8_t quality = 0;

        if (estimateGlobalDxDyRANSAC(flow, gdxdy)) {
            ema_ = (1.0f - ema_alpha_) * ema_ + ema_alpha_ * gdxdy;
            dx = ema_[0];
            dy = ema_[1];
            quality = 100;
        } else {
            cv::Scalar m = cv::mean(flow);
            dx = (float)m[0];
            dy = (float)m[1];
            quality = 50;
        }

        struct __attribute__((packed)) {
            uint32_t timestamp;
            uint8_t  quality;
            float    dx;
            float    dy;
        } pkt;

        auto now = std::chrono::steady_clock::now();

        pkt.timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time_
        ).count();

        pkt.quality = quality;
        pkt.dx = dx;
        pkt.dy = dy;

        ::sendto(udp_sock_, &pkt, sizeof(pkt), 0,
                 reinterpret_cast<const sockaddr*>(&udp_addr_), sizeof(udp_addr_));
    }

    std::string image_topic_, udp_host_;
    int udp_port_, grid_size_, num_levels_;
    float ema_alpha_;

    int udp_sock_ = -1;
    sockaddr_in udp_addr_{};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    bool initialized_ = false;
    int w_ = 0, h_ = 0;
    std::chrono::steady_clock::time_point start_time_;

    VPIStream   stream_           = nullptr;
    VPIImage    imgPrevFrameLoad_ = nullptr, imgCurFrameLoad_  = nullptr;
    VPIImage    imgPrevFramePL_   = nullptr, imgCurFramePL_    = nullptr;
    VPIImage    imgPrevFrameTmp_  = nullptr, imgCurFrameTmp_   = nullptr;
    VPIPyramid  prevPyrTmp_       = nullptr, curPyrTmp_        = nullptr;
    VPIPyramid  prevPyrBL_        = nullptr, curPyrBL_         = nullptr;
    VPIImage    imgMvBL_          = nullptr, imgMvPL_          = nullptr;
    VPIPayload  payload_          = nullptr;

    cv::Vec2f ema_{0, 0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VpiOpticalFlowNode>());

    rclcpp::shutdown();
    return 0;
}
