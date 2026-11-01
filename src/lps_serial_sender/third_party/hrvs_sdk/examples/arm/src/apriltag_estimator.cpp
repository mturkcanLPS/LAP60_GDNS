#include "apriltag_estimator.h"

// --- ctor ---
AprilTagEstimator::AprilTagEstimator(int w, int h,
                                     float tagSizeMeters,
                                     VPIAprilTagFamily family,
                                     float tilt_deg,
                                     const std::map<int,TagMeta>& meta,
                                     int maxDetections,
                                     int maxBitsCorrected)
    : width_(w), height_(h), tagSize_(tagSizeMeters), family_(family),
      tilt_deg_(tilt_deg), meta_(meta), maxDetections_(maxDetections)
{
    check(vpiStreamCreate(0, &stream_), "vpiStreamCreate");

    check(vpiArrayCreate(maxDetections_, VPI_ARRAY_TYPE_APRILTAG_DETECTION, VPI_BACKEND_CPU, &detections_),
          "vpiArrayCreate(detections)");
    check(vpiArrayCreate(maxDetections_, VPI_ARRAY_TYPE_POSE, VPI_BACKEND_CPU, &poses_),
          "vpiArrayCreate(poses)");

    VPIAprilTagDecodeParams params{};
    params.tagIdFilter      = nullptr;
    params.tagIdFilterSize  = 0;
    params.maxBitsCorrected = maxBitsCorrected;
    params.family           = family_;

    check(vpiCreateAprilTagDetector(VPI_BACKEND_CPU, width_, height_, &params, &payload_),
          "vpiCreateAprilTagDetector");
}

// --- dtor ---
AprilTagEstimator::~AprilTagEstimator() {
    if (vpiGray_)    vpiImageDestroy(vpiGray_);
    if (detections_) vpiArrayDestroy(detections_);
    if (poses_)      vpiArrayDestroy(poses_);
    if (payload_)    vpiPayloadDestroy(payload_);
    if (stream_)     vpiStreamDestroy(stream_);
}

// --- process ---
AprilTagEstimator::FrameResult AprilTagEstimator::process(const cv::Mat &gray)
{
    if (gray.empty() || gray.type() != CV_8UC1)
        throw std::runtime_error("gray must be CV_8UC1");

    if (!vpiGray_) {
        check(vpiImageCreateWrapperOpenCVMat(gray, VPI_BACKEND_CPU, &vpiGray_),
              "vpiImageCreateWrapperOpenCVMat");
    } else {
        check(vpiImageSetWrappedOpenCVMat(vpiGray_, gray), "vpiImageSetWrappedOpenCVMat");
    }

    check(vpiSubmitAprilTagDetector(stream_, VPI_BACKEND_CPU, payload_,
                                    maxDetections_, vpiGray_, detections_),
          "vpiSubmitAprilTagDetector");

    check(vpiSubmitAprilTagPoseEstimation(stream_, VPI_BACKEND_CPU,
                                          detections_, __camInt, tagSize_, poses_),
          "vpiSubmitAprilTagPoseEstimation");

    check(vpiStreamSync(stream_), "vpiStreamSync");

    VPIArrayData detData{};
    check(vpiArrayLockData(detections_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &detData), "lock det");
    auto &detAOS = detData.buffer.aos;
    int detCount = *detAOS.sizePointer;
    const VPIAprilTagDetection* detPtr = static_cast<const VPIAprilTagDetection*>(detAOS.data);

    VPIArrayData poseData{};
    check(vpiArrayLockData(poses_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &poseData), "lock pose");
    auto &poseAOS = poseData.buffer.aos;
    int poseCount = *poseAOS.sizePointer;
    const VPIPose* posePtr = static_cast<const VPIPose*>(poseAOS.data);

    FrameResult out;
    double sum[3] = {0,0,0};
    int count = 0;

    for (int i=0; i<detCount && i<poseCount; ++i) {
        const auto &d = detPtr[i];
        const auto &p = posePtr[i];
        const float (*T)[4] = p.transform;

        auto it = meta_.find(d.id);
        if (it == meta_.end())
            continue;
        const TagMeta &m = it->second;

        float Rtag2cam[3][3] = {
            {T[0][0], T[0][1], T[0][2]},
            {T[1][0], T[1][1], T[1][2]},
            {T[2][0], T[2][1], T[2][2]}
        };
        float t_tag2cam[3] = { T[0][3], T[1][3], T[2][3] };

        float p_tag[3];
        camPosInTag(Rtag2cam, t_tag2cam, p_tag);

        float p_norm[3];
        toCanonicalYaw0(p_tag, tilt_deg_, m.yaw_deg, p_norm);

        float yaw, pitch, roll;
        rotmatToYPR(Rtag2cam, yaw, pitch, roll);
        float yaw_meas_deg = rad2deg(yaw);
        float delta_yaw_deg = yaw_meas_deg - m.yaw_deg;

        float Rdelta[3][3], p_rot[3], off_rot[3];
        Rz_deg(delta_yaw_deg, Rdelta);
        mat3_mul_vec(Rdelta, p_norm,   p_rot);
        mat3_mul_vec(Rdelta, m.offset, off_rot);

        float p_final[3] = {
            p_rot[0] + off_rot[0],
            p_rot[1] + off_rot[1],
            p_rot[2] + off_rot[2]
        };

        // id bazlı özel düzeltmeler
        if (d.id == 15 || d.id == 50)
            p_final[1] = -p_final[1];
        else
            p_final[0] = -p_final[0];

        if (d.id == 0 || d.id == 24)
            p_final[1] = -p_final[1];
        else
            p_final[0] = -p_final[0];

        sum[0] += p_final[0];
        sum[1] += p_final[1];
        sum[2] += p_final[2];
        count++;

        TagResult tr;
        tr.id = d.id;
        tr.p_final[0]=p_final[0]; tr.p_final[1]=p_final[1]; tr.p_final[2]=p_final[2];
        tr.center = cv::Point2f(d.center.x, d.center.y);
        for (int k=0;k<4;k++) tr.corners[k] = cv::Point2f(d.corners[k].x, d.corners[k].y);
        out.tags.push_back(tr);
    }

    if (count>0)
        out.average = cv::Vec3f(float(sum[0]/count), float(sum[1]/count), float(sum[2]/count));

    check(vpiArrayUnlock(poses_), "unlock pose");
    check(vpiArrayUnlock(detections_), "unlock det");

    return out;
}

// --- check ---
void AprilTagEstimator::check(VPIStatus st, const char *msg) {
    if (st != VPI_SUCCESS) {
        char buf[VPI_MAX_STATUS_MESSAGE_LENGTH];
        vpiGetLastStatusMessage(buf, sizeof(buf));
        throw std::runtime_error(std::string(msg) + " : " + buf);
    }
}


void AprilTagEstimator::camPosInTag(const float R[3][3], const float t[3], float out[3]) {
    out[0] = -(R[0][0]*t[0] + R[0][1]*t[1] + R[0][2]*t[2]);
    out[1] = -(R[1][0]*t[0] + R[1][1]*t[1] + R[1][2]*t[2]);
    out[2] = -(R[2][0]*t[0] + R[2][1]*t[1] + R[2][2]*t[2]);
}

void AprilTagEstimator::toCanonicalYaw0(const float p[3], float tilt_deg, float yaw_ref_deg, float out[3]) {
    out[0] = p[0]; out[1] = p[1]; out[2] = p[2]; 
}

void AprilTagEstimator::rotmatToYPR(const float R[3][3], float &yaw, float &pitch, float &roll) {
    yaw   = atan2(R[1][0], R[0][0]);
    pitch = atan2(-R[2][0], sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2]));
    roll  = atan2(R[2][1], R[2][2]);
}

float AprilTagEstimator::rad2deg(float rad) { return rad * 180.0f / 3.14159265f; }

void AprilTagEstimator::Rz_deg(float yaw_deg, float R[3][3]) {
    float r = yaw_deg * 3.14159265f / 180.0f;
    R[0][0]=cos(r); R[0][1]=-sin(r); R[0][2]=0;
    R[1][0]=sin(r); R[1][1]= cos(r); R[1][2]=0;
    R[2][0]=0;      R[2][1]=0;       R[2][2]=1;
}

void AprilTagEstimator::mat3_mul_vec(const float R[3][3], const float v[3], float out[3]) {
    out[0] = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    out[1] = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    out[2] = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
}
