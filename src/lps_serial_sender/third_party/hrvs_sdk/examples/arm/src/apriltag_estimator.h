#pragma once
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Image.h>
#include <vpi/Array.h>
#include <vpi/Stream.h>
#include <vpi/algo/AprilTags.h>
#include <opencv2/core.hpp>
#include <optional>
#include <map>
#include <vector>
#include <stdexcept>



struct TagMeta {
    float yaw_deg;
    float offset[3];
};

class AprilTagEstimator {
public:
    struct TagResult {
        int id;
        float p_final[3];
        cv::Point2f center;
        cv::Point2f corners[4];
    };

    struct FrameResult {
        std::vector<TagResult> tags;
        std::optional<cv::Vec3f> average;
    };

    AprilTagEstimator(int w, int h,
                      float tagSizeMeters,
                      VPIAprilTagFamily family,
                      float tilt_deg,
                      const std::map<int,TagMeta>& meta,
                      int maxDetections = 64,
                      int maxBitsCorrected = 1);

    ~AprilTagEstimator();
    VPICameraIntrinsic __camInt; 
    FrameResult process(const cv::Mat &gray);

private:
    void check(VPIStatus st, const char *msg);

    // yardimci math fonksiyonları (başka dosyada da tutabilirsin)
    void camPosInTag(const float R[3][3], const float t[3], float out[3]);
    void toCanonicalYaw0(const float p[3], float tilt_deg, float yaw_ref_deg, float out[3]);
    void rotmatToYPR(const float R[3][3], float &yaw, float &pitch, float &roll);
    float rad2deg(float rad);
    void Rz_deg(float yaw_deg, float R[3][3]);
    void mat3_mul_vec(const float R[3][3], const float v[3], float out[3]);

    int width_{}, height_{};
    float tagSize_{};
    VPIAprilTagFamily family_{};
    float tilt_deg_{};
    std::map<int,TagMeta> meta_{};
    int maxDetections_{64};

    VPIStream stream_ = nullptr;
    VPIPayload payload_ = nullptr;
    VPIArray detections_ = nullptr, poses_ = nullptr;
    VPIImage vpiGray_ = nullptr;

 

};
