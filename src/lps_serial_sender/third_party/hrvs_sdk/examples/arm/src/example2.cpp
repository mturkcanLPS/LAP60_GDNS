/*
Description: This file contains example of how you can: use API to retrieve measured data from radar system.
*/
#include <cstdlib>      // getenv, system
#include <sys/stat.h>   // stat
#include <ctime>    

#include <dlfcn.h> 
#include <cstring>

#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>


#include "radar_system.h"
#include "pal/gpio_pin_linux.h"
#include "pal/uart_linux.h"
#include "pal/time_stdlib.h"
#include "pal/thread_stdlib.h"
#include <stdlib.h>
#include <iostream>
#include <getopt.h>
#include <chrono>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cstdint>


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
#include <vpi/algo/AprilTags.h>
#include <vpi/algo/Rescale.h>

#include <map>
#include <vector>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <sstream>
#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>

#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <fstream>

#include <iostream>
#include <fcntl.h>      // open
#include <unistd.h>     // write, close
#include <termios.h>
#include <filesystem>
#include <fstream>
#include <opencv2/calib3d.hpp>
using namespace std;


const int SYNC_BYTE = 0xBB;
const int MSG_ID = 0x42;

string serverIp = "10.20.30.101";// "192.168.2.3";
int port = 2005;
bool connected = false; 
#pragma pack(1)
typedef struct{
    float x;
    float y;
}opticFlow_2D_t;

typedef struct{
    float x;
    float y;
    float z;
}float_3D_t;

typedef struct{
    float roll;
    float pitch;
    float yaw;
}float_angle_t;

typedef struct{
    bool valid;
    float_3D_t pos;
    float_angle_t angle;
}AP_position_t;


typedef struct{

uint8_t sync_byte;
uint8_t msg_id;
uint8_t payloadSize;
uint8_t headerchk;

}header_t;

typedef struct{

    uint32_t timeStamp;
    uint16_t altitude;
    uint32_t frameNumber;
    int16_t body_x; 
    int16_t body_y; 
    int16_t body_z; 
    
}hrvsPack_t;

typedef struct{

    uint32_t timeStamp;
    AP_position_t avgPos;
    AP_position_t tag1;
    AP_position_t tag2;
    AP_position_t tag3;
    AP_position_t tag4;

}apriltagPack_t;

typedef struct 
{
    uint32_t timestamp;
    uint8_t  quality;
    opticFlow_2D_t motionPos;
}opticFlowPack_t;

typedef struct 
{
    hrvsPack_t      hrvs;
    apriltagPack_t  aprtiltag;
    opticFlowPack_t opticflow;
}jetsonVariables_t;


typedef struct 
{
    header_t        header;
    jetsonVariables_t   variables;
    uint8_t         chksum;
}jetsonOtopilotPack_t;

typedef union {
    
    jetsonOtopilotPack_t  pack;
    uint8_t data[sizeof(jetsonOtopilotPack_t)];

}jetsonOtopilotPack_ut;

#pragma pack()


jetsonOtopilotPack_ut jetsonPack; 
apriltagPack_t        tagPack;
opticFlowPack_t       optPack;
hrvsPack_t            hrvsPack;



VPICameraIntrinsic g_K; 
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
class CsvLogger {
public:
    CsvLogger(const std::string &filename)
        : filename_(filename)
    {
        file_.open(filename_, std::ios::app);
        if (!file_.is_open()) {
            throw std::runtime_error("CSV dosyasi acilamadi!");
        }

        // Eğer dosya yeni oluşturulduysa başlık yaz
        file_ << "small_x,small_y,small_z,"
                     "medium32_x,medium32_y,medium32_z,"
                     "medium32_yaw,medium32_pitch,medium32_roll,"
                     "medium50_x,medium50_y,medium50_z,"
                     "medium50_yaw,medium50_pitch,medium50_roll,"
                     "medium87_x,medium87_y,medium87_z,"
                     "medium87_yaw,medium87_pitch,medium87_roll,"
                     "medium15_x,medium15_y,medium15_z,"
                     "medium15_yaw,medium15_pitch,medium15_roll,"
                     "large_x,large_y,large_z\n";
    }

    void log(const cv::Vec3f small, const cv::Vec3f medium32,const cv::Vec3f medium50,const cv::Vec3f medium15,const cv::Vec3f medium87, const cv::Vec3f large, const cv::Vec3f medium32_rot,const cv::Vec3f medium50_rot,const cv::Vec3f medium15_rot,const cv::Vec3f medium87_rot) {
        file_ << small[0] << "," << small[1] << "," << small[2] << ","
              << medium32[0] << "," << medium32[1] << "," << medium32[2] << ","
              << medium32_rot[0] << "," << medium32_rot[1] << "," << medium32_rot[2] << ","
              << medium50[0] << "," << medium50[1] << "," << medium50[2] << ","
              << medium50_rot[0] << "," << medium50_rot[1] << "," << medium50_rot[2] << ","
              << medium87[0] << "," << medium87[1] << "," << medium87[2] << ","
              << medium87_rot[0] << "," << medium87_rot[1] << "," << medium87_rot[2] << ","
              << medium15[0] << "," << medium15[1] << "," << medium15[2] << ","
              << medium15_rot[0] << "," << medium15_rot[1] << "," << medium15_rot[2] << ","
              << large[0] << "," << large[1] << "," << large[2] << "\n";
    }

    ~CsvLogger() {
        file_.close();
    }

private:
    std::string filename_;
    std::ofstream file_;
};

bool connectClient(string serverIp, uint port, int *clientHandler)
{

    struct hostent *host = gethostbyname(serverIp.c_str());

    sockaddr_in sendSockAddr;
    sendSockAddr.sin_family = AF_INET;
    sendSockAddr.sin_addr.s_addr = inet_addr(inet_ntoa(*(struct in_addr *)*host->h_addr_list));
    sendSockAddr.sin_port = htons(port);

    int clientSd = socket(AF_INET, SOCK_STREAM, 0);
    int status = connect(clientSd, (sockaddr *)&sendSockAddr, sizeof(sendSockAddr));

    *clientHandler = clientSd;

    if (status < 0)
        return false;

    return true;
}

bool checkSocketError(int clientSd)
{

    int error = 0;
    socklen_t len = 4;
    int retval = getsockopt(clientSd, SOL_SOCKET, SO_ERROR, &error, &len);

    if (retval != 0 || error != 0)
    {
        cout << "error getting socket error code " << strerror(retval) << endl;
        return true;
    }
    else
    {
        return false;
    }
}


static void check(VPIStatus st, const char* what)
{
    if (st != VPI_SUCCESS) {
        std::cerr << "[VPI] " << what << " -> " << vpiStatusGetName(st) << std::endl;
        std::exit(1);
    }
}


// ---------- math helpers ----------
static inline void mat3_transpose(const float R[3][3], float RT[3][3]) {
    RT[0][0]=R[0][0]; RT[0][1]=R[1][0]; RT[0][2]=R[2][0];
    RT[1][0]=R[0][1]; RT[1][1]=R[1][1]; RT[1][2]=R[2][1];
    RT[2][0]=R[0][2]; RT[2][1]=R[1][2]; RT[2][2]=R[2][2];
}
static inline void mat3_mul_vec(const float M[3][3], const float v[3], float out[3]) {
    out[0] = M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2];
    out[2] = M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2];
}
static inline void Rx(float theta, float R[3][3]) {
    float c = std::cos(theta), s = std::sin(theta);
    R[0][0]=1; R[0][1]=0;  R[0][2]=0;
    R[1][0]=0; R[1][1]=c;  R[1][2]=-s;
    R[2][0]=0; R[2][1]=s;  R[2][2]= c;
}
static inline void Ry(float theta, float R[3][3]) {
    float c = std::cos(theta), s = std::sin(theta);
    R[0][0] =  c;  R[0][1] = 0;  R[0][2] =  s;
    R[1][0] =  0;  R[1][1] = 1;  R[1][2] =  0;
    R[2][0] = -s;  R[2][1] = 0;  R[2][2] =  c;
}
static inline void Rz(float theta, float R[3][3]) {
    float c = std::cos(theta), s = std::sin(theta);
    R[0][0]= c; R[0][1]=-s; R[0][2]=0;
    R[1][0]= s; R[1][1]= c; R[1][2]=0;
    R[2][0]= 0; R[2][1]= 0; R[2][2]=1;
}
static inline void Rz_deg(float theta_deg, float R[3][3]) {
    float th = theta_deg * float(M_PI/180.0f);
    Rz(th, R);
}
static inline float rad2deg(float r){ return r * 180.0f / float(M_PI); }

// Z-Y-X Euler
static inline void rotmatToYPR(const float R[3][3], float &yaw, float &pitch, float &roll)
{
    yaw   = std::atan2(R[1][0], R[0][0]);
    pitch = std::atan2(-R[2][0], std::sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2]));
    roll  = std::atan2(R[2][1], R[2][2]);
}

// p_tag = -R^T t (camera TAG frame)
static inline void camPosInTag(const float R_tag2cam[3][3], const float t_tag2cam_in_cam[3],
                               float p_tag[3])
{
    float RT[3][3];
    mat3_transpose(R_tag2cam, RT);
    float tmp[3]; mat3_mul_vec(RT, t_tag2cam_in_cam, tmp);
    p_tag[0] = -tmp[0]; p_tag[1] = -tmp[1]; p_tag[2] = -tmp[2];
}

static inline void camPosInFlattenedTag(const float p_tag[3], float tilt_deg, float p_flat[3]) {
    float Rfix[3][3];
    float th = tilt_deg * float(M_PI/180.0f);
    Rx(+th, Rfix);
    mat3_mul_vec(Rfix, p_tag, p_flat);
}

static inline void toCanonicalYaw0(const float p_tag[3], float tilt_deg, float yaw_deg, float out[3])
{
   
    float p_flat[3];
    camPosInFlattenedTag(p_tag, tilt_deg, p_flat);
   // std::cout << "Tag  : " << p_flat[0] << " " << p_flat[1] << " " << p_flat[2]  << std::endl;
    float RfixYaw[3][3];
    float yz = (yaw_deg) * float(M_PI/180.0f);
    Rz(yz, RfixYaw);
    mat3_mul_vec(RfixYaw, p_flat, out);

   // std::cout << "Yaw Fixed Tag : " << out[0]  << " " << out[1] << " " << out[2] << std::endl;
}
// C++11 uyumlu "optional" benzeri tip
struct Vec3fOptional {
    bool has;
    cv::Vec3f value;
    Vec3fOptional() : has(false), value(0.0f, 0.0f, 0.0f) {}
    explicit Vec3fOptional(const cv::Vec3f &v) : has(true), value(v) {}
    operator bool() const { return has; }
    const cv::Vec3f& operator*() const { return value; }
    cv::Vec3f& operator*() { return value; }
};

struct TagMeta {
    float yaw_deg;   
    float offset[3];  
};

template<typename T>
class BQueue {
public:
    explicit BQueue(size_t cap) : cap_(cap) {}
    void push(const T &v) {
        std::unique_lock<std::mutex> lk(m_);
        cv_full_.wait(lk, [&]{ return q_.size() < cap_ || stop_; });
        if (stop_) return;
        q_.push(v);
        cv_empty_.notify_one();
    }
    void push(T &&v) {
        std::unique_lock<std::mutex> lk(m_);
        cv_full_.wait(lk, [&]{ return q_.size() < cap_ || stop_; });
        if (stop_) return;
        q_.push(std::move(v));
        cv_empty_.notify_one();
    }
    bool pop(T &out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_empty_.wait(lk, [&]{ return !q_.empty() || stop_; });
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop();
        cv_full_.notify_one();
        return true;
    }
    bool pop_latest(T &out) {
    std::unique_lock<std::mutex> lk(m_);
    cv_empty_.wait(lk, [&]{ return !q_.empty() || stop_; });
    if (q_.empty()) return false;

    // en sona kadar eskileri at
    while (q_.size() > 1) q_.pop();

    out = std::move(q_.front());
    q_.pop();
    cv_full_.notify_one();
    return true;
    }
    bool push_latest(T &&v) {
    std::unique_lock<std::mutex> lk(m_);
    if (stop_) return false;
    while (q_.size() >= cap_) q_.pop();
    q_.push(std::move(v));
    cv_empty_.notify_one();
    return true;
    }
    void stop() {
        std::lock_guard<std::mutex> lk(m_);
        stop_ = true;
        cv_full_.notify_all();
        cv_empty_.notify_all();
    }
private:
    std::mutex m_;
    std::condition_variable cv_full_, cv_empty_;
    std::queue<T> q_;
    size_t cap_;
    bool stop_ = false;
};

class AprilTagEstimator {
public:


    struct TagResult {
        int id;
        float p_final[3];
        float p_rot[3];
        cv::Point2f center;
        cv::Point2f corners[4];
        bool validity;
    };

    struct FrameResult {
        std::vector<TagResult> tags;
        cv::Vec3f average;
        cv::Vec3f rotation;
        bool hasAverage;
        FrameResult() : average(0.0f,0.0f,0.0f), rotation(0.0f,0.0f,0.0f), hasAverage(false) {}
    };

    AprilTagEstimator(int w, int h,float tagSizeMeters,VPIAprilTagFamily family,float tilt_deg,const std::map<int,TagMeta>& meta,int maxDetections = 64, int maxBitsCorrected = 1)
        : width_(w),
          height_(h),
          tagSize_(tagSizeMeters),
          family_(family),
          tilt_deg_(tilt_deg),
          meta_(meta),
          maxDetections_(maxDetections),
          stream_(NULL),
          payload_(NULL),
          detections_(NULL),
          poses_(NULL),
          vpiGray_(NULL)
    {
        logger = new CsvLogger("/home/lapis/Desktop/Lapis/tag_positions.csv");

        check(vpiStreamCreate(0, &stream_), "vpiStreamCreate");

        check(vpiArrayCreate(maxDetections_, VPI_ARRAY_TYPE_APRILTAG_DETECTION, VPI_BACKEND_CPU, &detections_),
              "vpiArrayCreate(detections)");
        check(vpiArrayCreate(maxDetections_, VPI_ARRAY_TYPE_POSE, VPI_BACKEND_CPU, &poses_),
              "vpiArrayCreate(poses)");

        VPIAprilTagDecodeParams params;
        std::memset(&params, 0, sizeof(params));
        params.tagIdFilter      = NULL;
        params.tagIdFilterSize  = 0;
        params.maxBitsCorrected = maxBitsCorrected;
        params.family           = family_;

        check(vpiCreateAprilTagDetector(VPI_BACKEND_CPU, width_, height_, &params, &payload_),
              "vpiCreateAprilTagDetector");
    }

    ~AprilTagEstimator() {
        if (vpiGray_)    vpiImageDestroy(vpiGray_);
        if (detections_) vpiArrayDestroy(detections_);
        if (poses_)      vpiArrayDestroy(poses_);
        if (payload_)    vpiPayloadDestroy(payload_);
        if (stream_)     vpiStreamDestroy(stream_);
    }
// ---- 1) Kameranın TAG çerçevesindeki konumu ----
static inline void camPosInTag2(const float R_tag2cam[3][3], const float t_tag2cam_in_cam[3],
                               float p_tag[3])
{
    float RT[3][3];
    mat3_transpose(R_tag2cam, RT);          // R^T
    float tmp[3]; mat3_mul_vec(RT, t_tag2cam_in_cam, tmp);
    p_tag[0] = -tmp[0]; p_tag[1] = -tmp[1]; p_tag[2] = -tmp[2];  // p = -R^T t
}

// ---- 2) Sadece tilt düzelt (flatten) ----
// tilt_deg: tag’in yerle yaptığı yaklaşık açı (örn. 40–45°), tag’ın YUKARI-aşağı pitch’i.
// Not: inverse gerekir -> Rx(-tilt)
static inline void flatten_only(const float p_tag[3], float tilt_deg, float p_flat[3])
{
    float Rfix[3][3];
    float th = tilt_deg * float(M_PI/180.0f);
    Rx(+th, Rfix);                      
    mat3_mul_vec(Rfix, p_tag, p_flat);
}
static inline cv::Matx44f toMatx44(const float T[4][4])
{
    cv::Matx44f M;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            M(r,c) = T[r][c];
        }
    }
    return M;
}
// Yardımcı: ZYX (yaw-pitch-roll) euler açıları (rad)
static inline cv::Vec3f rotMatToEulerZYX(const cv::Matx33f &R)
{
    // ZYX konv.: R = Rz(yaw)*Ry(pitch)*Rx(roll)
    float pitch = std::asin(-R(2,0));
    float yaw, roll;
    if (std::cos(pitch) > 1e-6f) {
        yaw  = std::atan2(R(1,0), R(0,0));
        roll = std::atan2(R(2,1), R(2,2));
    } else {
        // gimbal lock
        yaw  = std::atan2(-R(0,1), R(1,1));
        roll = 0.0f;
    }
    return cv::Vec3f(yaw, pitch, roll);
}
void oneTagPoseCameraAxes(const float R[3][3], const float t[3],
                          float tilt_deg, const TagMeta &M,
                          float p_out[3])  // kamera-ekseni hizalı çıktı
{
    // 1) p_tag
    float p_tag[3]; camPosInTag(R, t, p_tag);

    // 2) flatten
    float Rxm[3][3]; Rx( tilt_deg * float(M_PI/180.0f), Rxm );
    float p_flat[3]; mat3_mul_vec(Rxm, p_tag, p_flat);
    float R_flat[3][3];
    // R_flat = Rx * R
    for(int r=0;r<3;++r) for(int c=0;c<3;++c)
        R_flat[r][c] = Rxm[r][0]*R[0][c] + Rxm[r][1]*R[1][c] + Rxm[r][2]*R[2][c];

    // 3) nominal yaw'ı kaldır (kanonik tag)
    float Rzy[3][3]; Rz( -M.yaw_deg * float(M_PI/180.0f), Rzy );
    float p_canon[3];  mat3_mul_vec(Rzy, p_flat, p_canon);
    float R_canon[3][3];
    for(int r=0;r<3;++r) for(int c=0;c<3;++c)
        R_canon[r][c] = Rzy[r][0]*R_flat[0][c] + Rzy[r][1]*R_flat[1][c] + Rzy[r][2]*R_flat[2][c];

    // 4) yaw ölç ve KAMERAYA geri hizala (yaw etkisi kalsın)
    float yaw_meas = std::atan2(R_canon[1][0], R_canon[0][0]);
    float Rdz[3][3]; Rz( +yaw_meas, Rdz );
    float p_cam_axes[3]; mat3_mul_vec(Rdz, p_canon, p_cam_axes);

    // 5) (opsiyonel) offset ekle (kanonikten kamera-eksene taşımak istersen):
    float off_cam[3] = {0,0,0};
    // eğer offset kullanacaksan:
    // mat3_mul_vec(Rdz, M.offset_canon, off_cam);

    p_out[0] = p_cam_axes[0] + off_cam[0];
    p_out[1] = p_cam_axes[1] + off_cam[1];
    p_out[2] = p_cam_axes[2] + off_cam[2];
}
// Ana hesap: T_cam_tag -> (camPos, camRot) yani kameranın tag'e göre pozu
static inline void cameraPoseFromTagPose(const float T_in[4][4],
                                         cv::Vec3f &camPos,
                                         cv::Matx33f &camRot)
{
    cv::Matx44f T = toMatx44(T_in);

    // R ve t (T_cam_tag)
    cv::Matx33f R(T(0,0), T(0,1), T(0,2),
                  T(1,0), T(1,1), T(1,2),
                  T(2,0), T(2,1), T(2,2));
    cv::Vec3f t(T(0,3), T(1,3), T(2,3));

    // T_tag_cam = [Rᵀ | -Rᵀ t]
    cv::Matx33f Rinv = R.t();
    camRot = Rinv;
    camPos = -(Rinv * t);
}

   
    FrameResult process(const cv::Mat &gray)
    {

    if (gray.empty() || gray.type() != CV_8UC1)
        throw std::runtime_error("gray must be CV_8UC1");

    if (!vpiGray_) {
        check(vpiImageCreateWrapperOpenCVMat(gray, VPI_BACKEND_CPU, &vpiGray_),
              "vpiImageCreateWrapperOpenCVMat");
    } else {
        check(vpiImageSetWrappedOpenCVMat(vpiGray_, gray), "vpiImageSetWrappedOpenCVMat");
    }

    // 1) Detect
    check(vpiSubmitAprilTagDetector(stream_, VPI_BACKEND_CPU, payload_,
                                    maxDetections_, vpiGray_, detections_),
          "vpiSubmitAprilTagDetector");

    check(vpiStreamSync(stream_), "vpiStreamSync after detector");

    // 2) Read detections safely
    VPIArrayData detData{};
    check(vpiArrayLockData(detections_, VPI_LOCK_READ,
                           VPI_ARRAY_BUFFER_HOST_AOS, &detData),
          "vpiArrayLockData(detections_)");

    const int n = (detData.buffer.aos.sizePointer) ? *detData.buffer.aos.sizePointer : 0;
    const auto *src = (n > 0) ? static_cast<const VPIAprilTagDetection*>(detData.buffer.aos.data) : nullptr;

    std::vector<VPIAprilTagDetection> detSmall, detMedium, detLarge;
    if (n > 0 && src) {
        detSmall.reserve(n); detMedium.reserve(n); detLarge.reserve(n);
        for (int i = 0; i < n; ++i) {
            const int id = src[i].id;
           // std::cout << "Detected ID : " << id << std::endl;
            auto it = AprilTagEstimator::kTagSizeById.find(id);
            if (it == AprilTagEstimator::kTagSizeById.end())
                continue;
            const float s = it->second;
            if (s <= 0.10f)      detSmall.push_back(src[i]);
            else if (s < 0.50f)  detMedium.push_back(src[i]);
            else                 detLarge.push_back(src[i]);
        }
    }

    check(vpiArrayUnlock(detections_), "vpiArrayUnlock(detections_)");

    // helper: vector -> VPIArray (detections)
    auto vecToArray = [&](const std::vector<VPIAprilTagDetection>& v, VPIArray *out){
        check(vpiArrayCreate((int)v.size(), VPI_ARRAY_TYPE_APRILTAG_DETECTION, 0, out),
              "vpiArrayCreate(APRILTAG_DETECTION)");

        if (!v.empty()) {
            VPIArrayData ad{};
            check(vpiArrayLockData(*out, VPI_LOCK_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &ad),
                  "vpiArrayLockData(write)");
            std::memcpy(ad.buffer.aos.data, v.data(), v.size()*sizeof(VPIAprilTagDetection));
            *ad.buffer.aos.sizePointer = (int)v.size();
            check(vpiArrayUnlock(*out), "vpiArrayUnlock(write)");
        } else {
            // boş vektör için boyutu 0’a çek
            VPIArrayData ad{};
            check(vpiArrayLockData(*out, VPI_LOCK_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &ad),
                  "vpiArrayLockData(write-empty)");
            *ad.buffer.aos.sizePointer = 0;
            check(vpiArrayUnlock(*out), "vpiArrayUnlock(write-empty)");
        }
    };
    FrameResult out;
    FrameResult out32;
    FrameResult out50;
    FrameResult out87;
    FrameResult out15;
    FrameResult small_out;
    FrameResult large_out;
    double sum[3] = {0.0, 0.0, 0.0};
    double sumSmall[3] = {0.0, 0.0, 0.0};
    int count = 0;
    int countSmall = 0;
    auto runPoseGroup = [&](const std::vector<VPIAprilTagDetection>& v, float tagSizeM){
        if (v.empty()) return;

        VPIArray detArr = nullptr;
        vecToArray(v, &detArr);

        VPIArray poseArr = nullptr;
        check(vpiArrayCreate((int)v.size(), VPI_ARRAY_TYPE_POSE, 0, &poseArr),
              "vpiArrayCreate(POSE)");

        check(vpiSubmitAprilTagPoseEstimation(stream_, VPI_BACKEND_CPU,
                                              detArr, g_K, tagSizeM, poseArr),
              "vpiSubmitAprilTagPoseEstimation");
        check(vpiStreamSync(stream_), "vpiStreamSync after pose");

        VPIArrayData pd{};
        check(vpiArrayLockData(poseArr, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &pd),
              "vpiArrayLockData(poseArr)");
        const int m = (pd.buffer.aos.sizePointer) ? *pd.buffer.aos.sizePointer : 0;
        const auto *poses = (m > 0) ? static_cast<const VPIPose*>(pd.buffer.aos.data) : nullptr;
       
       
        for (int i = 0; i < m; ++i) {
            const VPIPose &p = poses[i];
            const VPIAprilTagDetection &det = v[i];
            const float (*T)[4] = p.transform;

            if(tagSizeM > 0.08f && tagSizeM < 0.90f){
                std::cout << "**************************************" << std::endl;           
                // std::cout << "MEDIUM Tag ID : " <<  det.id  << " Pose[" << i << "] XYZ: "
                //       << T[0][3] <<  " " << T[1][3] << " " <<  T[2][3] << std::endl;

                std::map<int,TagMeta>::const_iterator it = meta_.find(det.id);
                if (it == meta_.end())
                    continue;
                const TagMeta &m = it->second;

                float Rtag2cam[3][3] = {
                    {T[0][0], T[0][1], T[0][2]},
                    {T[1][0], T[1][1], T[1][2]},
                    {T[2][0], T[2][1], T[2][2]}
                };
                float t_tag2cam[3] = { T[0][3], T[1][3], T[2][3] };
                
                // float p_final[3] {
                //     t_tag2cam[0] + m.offset[0],
                //     t_tag2cam[1] + m.offset[1],
                //     t_tag2cam[2] + m.offset[2]
                // };

                float p_tag[3];
                camPosInTag(Rtag2cam, t_tag2cam, p_tag);
                float yaw, pitch, roll;
                rotmatToYPR(Rtag2cam, yaw, roll, pitch);
                float off_rot[3];
                float Rdelta[3][3];
                Rz_deg(yaw, Rdelta);
                mat3_mul_vec(Rdelta, m.offset, off_rot);
                // p_tag[0] += off_rot[0];
                // p_tag[1] += off_rot[1];
                // p_tag[2] += off_rot[2];
//                std::cout << "TAG ID : " << det.id << " RAW : " << t_tag2cam[0] << " " << t_tag2cam[1] << " " << t_tag2cam[2] << std::endl;
                // std::cout << "TAG ID : " << det.id << " [ Raw Pos ] : " << p_tag[0] << " " << p_tag[1] << " " << p_tag[2] << std::endl;
                // std::cout << "YPR : " << rad2deg(yaw) << "pitch : " << rad2deg(pitch) << "roll : " << rad2deg(roll) << std::endl;

                // ----- TEK TAG UZERINDE GPS TESTI ICIN KAPATILDI ----- 
                // float p_flat[3];
                
                // if( det.id == 20){
                //     float Rxm[3][3]; Rx( rad2deg(pitch) * float(M_PI/180.0f), Rxm );
                //     mat3_mul_vec(Rxm, p_tag, p_flat);
                // }else if(det.id == 24 || det.id == 36){
                //     float Rym[3][3]; Ry( rad2deg(roll) * float(M_PI/180.0f), Rym );
                //     mat3_mul_vec(Rym, p_tag, p_flat);
                // }else{
                //     float Rxm[3][3]; Rx( 48 * float(M_PI/180.0f), Rxm );
                //     mat3_mul_vec(Rxm, p_tag, p_flat);
                // }

           
                // p_flat[0] += off_rot[0];
                // p_flat[1] += off_rot[1];
                // p_flat[2] += off_rot[2];
                
                
                // float p_rot[3];
                // Rz_deg(rad2deg(yaw), Rdelta);
                // mat3_mul_vec(Rdelta, p_flat, p_rot);
                
                // std::cout << "TAG ID : " << det.id << " [ FLAT Pos ] : " << p_rot[0] << " " << p_rot[1] << " " << p_rot[2] << std::endl;

                //  <->     GPS TESTI ICIN     <-> 
                float p_rot[3];
                Rz_deg(rad2deg(yaw), Rdelta);
                mat3_mul_vec(Rdelta, p_tag, p_rot);
            

//                 float p_norm[3];
//                 toCanonicalYaw0(p_tag, tilt_deg_, m.yaw_deg, p_norm);
             
             
//                 std::cout << "TAG ID : " << det.id << "YAW : " << rad2deg(yaw) << "PITCH : "  << rad2deg(pitch) << "ROLL : "  << rad2deg(roll) << std::endl;
//                 float yaw_meas_deg = rad2deg(yaw);
//                 float delta_yaw_deg = yaw_meas_deg - m.yaw_deg;
//                 //std::cout << "dYaw : " <<delta_yaw_deg << std::endl;
//                 float Rdelta[3][3], p_rot[3], off_rot[3];
//                 Rz_deg(delta_yaw_deg, Rdelta);
//                 mat3_mul_vec(Rdelta, p_norm,   p_rot);
//                 std::cout << "NORM Tag ID : " <<  p_rot[0] << " " <<  p_rot[1] << " " << p_rot[2] << std::endl;
//                 mat3_mul_vec(Rdelta, m.offset, off_rot);
//                 std::cout << "OFFSET Tag ID : " <<  off_rot[0] << " " <<  off_rot[1] << " " << off_rot[2] << std::endl;
//                 float p_final[3] = {
//                     p_rot[0] + off_rot[0],
//                     p_rot[1] + off_rot[1],
//                     p_rot[2] + off_rot[2]
//                 };
                
                // if (det.id == 15 || det.id == 50) {
                //     p_final[1] = -p_final[1];
                // } else {
                //     p_final[0] = -p_final[0];
                // }

                // if (det.id == 32 || det.id == 87) {
                //     p_final[1] = -p_final[1];
                // } else {
                //     p_final[0] = -p_final[0];
                // }

                // std::cout << "MEDIUM Tag ID : " <<  det.id  << " Pose[" << i << "] XYZ: "
                //       << p_final[0] <<  " " << p_final[1] << " " <<  p_final[2] << std::endl;

                // sum[0] += p_final[0];
                // sum[1] += p_final[1];
                // sum[2] += p_final[2];
                // ++count;

                // TagResult tr;
                // tr.id = det.id;
                // tr.p_final[0] = p_final[0];
                // tr.p_final[1] = p_final[1];
                // tr.p_final[2] = p_final[2];
                // tr.center = cv::Point2f(det.center.x, det.center.y);
                // for (int k = 0; k < 4; ++k)
                //     tr.corners[k] = cv::Point2f(det.corners[k].x, det.corners[k].y);
                // out.tags.push_back(tr);
        
                out.average[0] = p_rot[0];
                out.average[1] = p_rot[1];
                out.average[2] = p_rot[2];
                out.hasAverage = false;
                // if (count > 0) {
                //     out.average = cv::Vec3f(
                //         static_cast<float>(sum[0]/count),
                //         static_cast<float>(sum[1]/count),
                //         static_cast<float>(sum[2]/count)
                //     );
                //     out.hasAverage = true;
                // }
                // switch (det.id)
                // {
                // case 32:
                //     out32.average[0] = p_out[0];
                //     out32.average[1] = p_out[1];
                //     out32.average[2] = p_out[2];
                //     out32.rotation[0] = 0;
                //     out32.rotation[1] = 0;
                //     out32.rotation[2] = 0;
                //     break;
                // case 50:
                //     out50.average[0] = p_out[0];
                //     out50.average[1] = p_out[1];
                //     out50.average[2] = p_out[2];
                //     out50.rotation[0] = 0;
                //     out50.rotation[1] = 0;
                //     out50.rotation[2] = 0;
                //     break;
                // case 15:
                //     out15.average[0] = p_out[0];
                //     out15.average[1] = p_out[1];
                //     out15.average[2] = p_out[2];
                //     out15.rotation[0] = 0;
                //     out15.rotation[1] = 0;
                //     out15.rotation[2] = 0;
                //     break;
                // case 87:
                //     out87.average[0] = p_out[0];
                //     out87.average[1] = p_out[1];
                //     out87.average[2] = p_out[2];
                //     out87.rotation[0] = 0;
                //     out87.rotation[1] = 0;
                //     out87.rotation[2] = 0;
                //     break;
                // default:
                //     break;
                // }
                // out.average[0] = p_out[0];
                // out.average[1] = p_out[1];
                // out.average[2] = p_out[2];
                
                bool tag1_valid = false;
                bool tag2_valid = false;
                bool tag3_valid = false;
                bool tag4_valid = false;

                
                TagResult tr;
                tr.id = det.id;
                tr.p_final[0] = p_rot[0];
                tr.p_final[1] = p_rot[1];
                tr.p_final[2] = p_rot[2];

                tr.p_rot[0] = rad2deg(roll);
                tr.p_rot[1] = rad2deg(pitch);
                tr.p_rot[2] = rad2deg(yaw);
                switch(det.id){
                case 0:
                    tag1_valid = true;
                    tr.validity = tag1_valid;
                    break;
                case 20:
                    tag2_valid = true;
                    tr.validity = tag2_valid;
                    break;
                case 24:
                    tag3_valid = true;
                    tr.validity = tag3_valid;
                    break;
                case 36:
                    tag4_valid = true;
                    tr.validity = tag4_valid;
                    break;
                };
         
                
                tr.center = cv::Point2f(det.center.x, det.center.y);
                for (int k = 0; k < 4; ++k)
                     tr.corners[k] = cv::Point2f(det.corners[k].x, det.corners[k].y);
                out.tags.push_back(tr);
            

            }else if(tagSizeM < 0.27f){
                std::cout << "-----------------------------" << std::endl; 
                std::cout << "SMALL Tag ID : " <<  det.id  << "TAG Pose[" << i << "] XYZ: "
                      << T[0][3] <<  " " << T[1][3] << " " <<  T[2][3] << std::endl;
                std::map<int,TagMeta>::const_iterator it = s_meta.find(det.id);
                if (it == s_meta.end())
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
               

                //float p_norm[3];
                //toCanonicalYaw0(p_tag, 0, 0, p_norm);

                float yaw, pitch, roll;
                rotmatToYPR(Rtag2cam, yaw, pitch, roll);
                std::cout << "TAG ID : " << det.id << "YAW : " << rad2deg(yaw) << "PITCH : "  << rad2deg(pitch) << "ROLL : "  << rad2deg(roll) << std::endl;
                // float yaw_meas_deg = ;
                // float delta_yaw_deg = yaw_meas_deg - 0;

                float Rdelta[3][3], p_rot[3];
                Rz_deg(rad2deg(yaw), Rdelta);
                mat3_mul_vec(Rdelta, p_tag,   p_rot);
                
                

                //std::cout << setprecision(3) << "x: " << p_rot[0] << " y: " << p_rot[1] << " z: " <<  p_rot[2] << std::endl;
                std::cout << "SMALL Tag ID : " <<  det.id  << "CAM Pose[" << i << "] XYZ: "
                          << p_rot[0] <<  " " << p_rot[1] << " " <<  p_rot[2]  << std::endl;

                // sumSmall[0] += p_final[0];
                // sumSmall[1] += p_final[1];
                // sumSmall[2] += p_final[2];
                // ++countSmall;
                // if (countSmall > 0) {
                //     small_out.average = cv::Vec3f(
                //         static_cast<float>(sumSmall[0]/countSmall),
                //         static_cast<float>(sumSmall[1]/countSmall),
                //         static_cast<float>(sumSmall[2]/countSmall)
                //     );
                //     small_out.hasAverage = true;
                // }
                // small_out.average[0] = p_rot[0];
                // small_out.average[1] = p_rot[1];
                // small_out.average[2] = p_rot[2];

            }else{
                std::cout << "BIG Tag ID : " <<  det.id  << " Pose[" << i << "] XYZ: "
                      << T[0][3] <<  " " << T[1][3] << " " <<  T[2][3] << std::endl;

                float Rtag2cam[3][3] = {
                    {T[0][0], T[0][1], T[0][2]},
                    {T[1][0], T[1][1], T[1][2]},
                    {T[2][0], T[2][1], T[2][2]}
                };
                float t_tag2cam[3] = { T[0][3], T[1][3], T[2][3] };
               
                float p_tag[3];
                camPosInTag(Rtag2cam, t_tag2cam, p_tag);
               

                float p_norm[3];
                toCanonicalYaw0(p_tag, 0, 0, p_norm);

                float yaw, pitch, roll;
                rotmatToYPR(Rtag2cam, yaw, pitch, roll);

                float yaw_meas_deg = rad2deg(yaw);
                float delta_yaw_deg = yaw_meas_deg - 0;

                float Rdelta[3][3], p_rot[3], off_rot[3];
                Rz_deg(delta_yaw_deg, Rdelta);
                mat3_mul_vec(Rdelta, p_norm,   p_rot);
                //p_rot[1] =- 0.130f;
                std::cout << "BIG Tag ID ROT: " <<  det.id  << " Pose[" << i << "] XYZ: "
                      << p_rot[0] <<  " " << p_rot[1] << " " <<  p_rot[2] << std::endl;
                large_out.average[0] = p_rot[0];
                large_out.average[1] = p_rot[1];
                large_out.average[2] = p_rot[2];
            }
            
        }

        check(vpiArrayUnlock(poseArr), "vpiArrayUnlock(poseArr)");
        vpiArrayDestroy(poseArr);
        vpiArrayDestroy(detArr);

    };
    runPoseGroup(detSmall,  0.077f);
    runPoseGroup(detMedium, 0.28f);
    runPoseGroup(detLarge,  0.90f);
    
    //logger->log(small_out.average, out32.average, out15.average, out50.average, out87.average, large_out.average, out32.rotation, out15.rotation,out50.rotation, out87.rotation);



    return out;

        // check(vpiSubmitAprilTagPoseEstimation(stream_, VPI_BACKEND_CPU,
        //                                       detections_, g_K, tagSize_, poses_),
        //       "vpiSubmitAprilTagPoseEstimation");

        // check(vpiStreamSync(stream_), "vpiStreamSync");

        // VPIArrayData detData;
        // std::memset(&detData, 0, sizeof(detData));
        // check(vpiArrayLockData(detections_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &detData), "lock det");

        // auto &detAOS = detData.buffer.aos;
        // int detCount = detAOS.sizePointer ? *detAOS.sizePointer : 0;
        // const VPIAprilTagDetection* detPtr =
        //     static_cast<const VPIAprilTagDetection*>(detAOS.data);

        // VPIArrayData poseData;
        // std::memset(&poseData, 0, sizeof(poseData));
        // check(vpiArrayLockData(poses_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &poseData), "lock pose");

        // auto &poseAOS = poseData.buffer.aos;
        // int poseCount = poseAOS.sizePointer ? *poseAOS.sizePointer : 0;
        // const VPIPose* posePtr = static_cast<const VPIPose*>(poseAOS.data);

        // FrameResult out; // hasAverage=false default

        // double sum[3] = {0.0, 0.0, 0.0};
        // int count = 0;

        // for (int i = 0; i < detCount && i < poseCount; ++i) {
        //     const VPIAprilTagDetection &d = detPtr[i];
        //     const VPIPose &p = posePtr[i];

        //     const float (*T)[4] = p.transform;

        //     std::map<int,TagMeta>::const_iterator it = meta_.find(d.id);

        //     std::cout << "TAG ID : " << d.id  <<  " Margin : " << d.decisionMargin <<  " correct bits : " << d.correctedBits << std::endl; 
        //     std::cout << "TAG ID : " << d.id << "t : " << T[0][3] <<  " " << T[1][3] << " " <<  T[2][3] << std::endl;
        //     if (it == meta_.end())
        //         continue;
        //     const TagMeta &m = it->second;

        //     float Rtag2cam[3][3] = {
        //         {T[0][0], T[0][1], T[0][2]},
        //         {T[1][0], T[1][1], T[1][2]},
        //         {T[2][0], T[2][1], T[2][2]}
        //     };
        //     float t_tag2cam[3] = { T[0][3], T[1][3], T[2][3] };
        //     //std::cout << "TAG ID : " << d.id << "t : " << T[0][3] <<  " " << T[1][3] << " " <<  T[2][3] << std::endl;
        //     float p_tag[3];
        //     camPosInTag(Rtag2cam, t_tag2cam, p_tag);

        //     float p_norm[3];
        //     toCanonicalYaw0(p_tag, tilt_deg_, m.yaw_deg, p_norm);

        //     float yaw, pitch, roll;
        //     rotmatToYPR(Rtag2cam, yaw, pitch, roll);
        //     //std::cout << "TAG ID : " << d.id << "YAW : " << rad2deg(yaw) << "PITCH : "  << rad2deg(pitch) << "ROLL : "  << rad2deg(roll) << std::endl;
        //     float yaw_meas_deg = rad2deg(yaw);
        //     float delta_yaw_deg = yaw_meas_deg - m.yaw_deg;

        //     float Rdelta[3][3], p_rot[3], off_rot[3];
        //     Rz_deg(delta_yaw_deg, Rdelta);
        //     mat3_mul_vec(Rdelta, p_norm,   p_rot);
        //     mat3_mul_vec(Rdelta, m.offset, off_rot);

        //     float p_final[3] = {
        //         p_rot[0] + off_rot[0],
        //         p_rot[1] + off_rot[1],
        //         p_rot[2] + off_rot[2]
        //     };

        //     if (d.id == 15 || d.id == 50) {
        //         p_final[1] = -p_final[1];
        //     } else {
        //         p_final[0] = -p_final[0];
        //     }

        //     if (d.id == 0 || d.id == 24) {
        //         p_final[1] = -p_final[1];
        //     } else {
        //         p_final[0] = -p_final[0];
        //     }

        //     sum[0] += p_final[0];
        //     sum[1] += p_final[1];
        //     sum[2] += p_final[2];
        //     ++count;

        //     TagResult tr;
        //     tr.id = d.id;
        //     tr.p_final[0] = p_final[0];
        //     tr.p_final[1] = p_final[1];
        //     tr.p_final[2] = p_final[2];
        //     tr.center = cv::Point2f(d.center.x, d.center.y);
        //     for (int k = 0; k < 4; ++k)
        //         tr.corners[k] = cv::Point2f(d.corners[k].x, d.corners[k].y);
        //     out.tags.push_back(tr);
        // }

        // if (count > 0) {
        //     out.average = cv::Vec3f(
        //         static_cast<float>(sum[0]/count),
        //         static_cast<float>(sum[1]/count),
        //         static_cast<float>(sum[2]/count)
        //     );
        //     out.hasAverage = true;
        // }

        // check(vpiArrayUnlock(poses_), "unlock pose");
        // check(vpiArrayUnlock(detections_), "unlock det");

        // return out;
    }

private:
    int width_;
    int height_;
    float tagSize_;
    VPIAprilTagFamily family_;
    float tilt_deg_;
    std::map<int,TagMeta> meta_;
    int maxDetections_;

    VPIStream stream_;
    VPIPayload payload_;
    VPIArray detections_;
    VPIArray poses_;
    VPIImage vpiGray_;

    CsvLogger *logger = nullptr;

    const std::unordered_map<int, float> kTagSizeById = {
    // 80 mm
    {19, 0.08f}, {71, 0.08f},{72, 0.08f},{85, 0.08f},
    // 280 mm
    {20, 0.27f}, {24, 0.27f}, {36, 0.27f},{0, 0.27f},
    // 1050 mm
    {45, 1.05f}
    };
    std::map<int,TagMeta> s_meta = {
            {19, { 0.0f,  {  0.00f, 0.00f, 0.00f }}},
            {71, { 0.0f,  { 0.0f, 0.00f,  0.00f }}},
            {72, {0.0f,  {  0.0f, 0.00f,  0.00f }}},
            {85, {0.0f,  {  0.00f,  0.00f, 0.00f }}},
        /*
            {36, {  0.0f,  {  0.00f, -0.150f, 0.00f }}},
            {24, { 90.0f,  { -0.150f, 0.00f,  0.00f }}},
            { 0, {-90.0f,  {  0.150f, 0.00f,  0.00f }}},
            {20, {180.0f,  {  0.00f,  0.150f, 0.00f }}},
        */
    };


};


class DenseOptFlowOFA {
public:
    // quality: VPI_OPTICAL_FLOW_QUALITY_LOW / MEDIUM / HIGH
    // gridSize: 1, 2, 4, 8
    // numLevels: 1..7
    DenseOptFlowOFA(int width, int height, int gridSize, int numLevels, VPIOpticalFlowQuality quality)
        : w_(width), h_(height),
          gridSize_(gridSize), numLevels_(numLevels),
          quality_(quality) 
    {

    }

    ~DenseOptFlowOFA() {

    }

struct FlowStats {
        float mean_dx = 0.f, mean_dy = 0.f;
};



static bool estimateGlobalDxDyRANSAC(const cv::Mat &flow,
                                     cv::Vec2f &robustDxDy,
                                     float sampleStep = 6,     
                                     float magDeadZone = 0.10f,
                                     double ransacThresh = 3.0f,
                                     double confidence = 0.99,
                                     int refineIters = 10,
                                     float minInlierRatio = 0.05f)
{
    CV_Assert(flow.type() == CV_32FC2);
    std::vector<cv::Point2f> src, dst;
    src.reserve((flow.cols/ sampleStep+1)*(flow.rows/ sampleStep+1));
    dst.reserve(src.capacity());

    // Altörnekleme + ölü bölge filtresi
    for (int y = 0; y < flow.rows; y += (int)sampleStep) {
        const cv::Vec2f* row = flow.ptr<cv::Vec2f>(y);
        for (int x = 0; x < flow.cols; x += (int)sampleStep) {
            float u = row[x][0], v = row[x][1];
            if (std::abs(u) + std::abs(v) < magDeadZone) continue; 
            src.emplace_back((float)x, (float)y);
            dst.emplace_back((float)x + u, (float)y + v);
        }
    }
    if (src.size() < 6) return false;

 
    cv::Mat inlierMask;
    cv::Mat A = cv::estimateAffinePartial2D(
    src, dst, inlierMask,
    cv::RANSAC,
    ransacThresh,   // 1.5
    2000,           // maxIters
    0.99,           // confidence (0< <1)
    10              // refineIters
);
    if (A.empty()) return false;

    int inlierCnt = cv::countNonZero(inlierMask);
    double inlierRatio = (double)inlierCnt / (double)src.size();
    if (inlierRatio < minInlierRatio) return false;

    // [ a11 a12 tx ]
    // [ a21 a22 ty ]
    float tx = (float)A.at<double>(0,2);
    float ty = (float)A.at<double>(1,2);

    std::vector<float> vdx, vdy;
    vdx.reserve(inlierCnt); vdy.reserve(inlierCnt);
    for (size_t i=0;i<src.size();++i){
        if (!inlierMask.data[i]) continue;
        vdx.push_back(dst[i].x - src[i].x);
        vdy.push_back(dst[i].y - src[i].y);
    }
    if (!vdx.empty()) {
       
        auto mid = vdx.begin() + vdx.size()/2;
        std::nth_element(vdx.begin(), mid, vdx.end());
        std::nth_element(vdy.begin(), vdy.begin()+vdy.size()/2, vdy.end());
        float mdx = *mid;
        float mdy = vdy[vdy.size()/2];

        float alpha = 0.5f;
        robustDxDy = cv::Vec2f(alpha*tx + (1-alpha)*mdx,
                               alpha*ty + (1-alpha)*mdy);
    } else {
        robustDxDy = cv::Vec2f(tx, ty);
    }
    return true;
}

 static void ProcessMotionVector(VPIImage mvImg, cv::Mat &outputImage)
 {
     // Lock the input image to access it from CPU
     VPIImageData mvData;
     CHECK_STATUS(vpiImageLockData(mvImg, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData));
  
     // Create a cv::Mat that points to the input image data
     cv::Mat mvImage;
     CHECK_STATUS(vpiImageDataExportOpenCVMat(mvData, &mvImage));
  
     // Convert S10.5 format to float
     cv::Mat flow(mvImage.size(), CV_32FC2);
     mvImage.convertTo(flow, CV_32F, 1.0f / (1 << 5));

  
     // Image not needed anymore, we can unlock it.,
     CHECK_STATUS(vpiImageUnlock(mvImg));
        cv::Vec2f gdxdy;
     if(estimateGlobalDxDyRANSAC(flow, gdxdy)){
            float dx = gdxdy[0];
            float dy = gdxdy[1];
//    std::cout << "dx=" << dx << " dy=" << dy << std::endl;    
    // --- EMA ile zaman filtreleme ---
    static cv::Vec2f ema(0,0);   // ilk frame için sıfırdan başla
    float alpha = 0.25f;         // 0.1 → çok yumuşak, 0.5 → hızlı tepki

    ema = (1.0f-alpha)*ema + alpha*gdxdy;

   
    std::cout << "dx=" << ema[0] << " dy=" << ema[1] << std::endl;

    
    }else{
            cv::Scalar m = cv::mean(flow);
    float dx = (float)m[0];
    float dy = (float)m[1];
    std::cout << "[fallback] dx=" << dx << " dy=" << dy << std::endl;

        // std::vector<float> all_dx, all_dy;
        // all_dx.reserve(flow.total());
        // all_dy.reserve(flow.total());

        // for (int y=0; y<flow.rows; ++y) {
        //     const cv::Vec2f* row = flow.ptr<cv::Vec2f>(y);
        //     for (int x=0; x<flow.cols; ++x) {
        //         all_dx.push_back(row[x][0]);
        //         all_dy.push_back(row[x][1]);
        //     }
        // }
        // std::nth_element(all_dx.begin(), all_dx.begin()+all_dx.size()/2, all_dx.end());
        // std::nth_element(all_dy.begin(), all_dy.begin()+all_dy.size()/2, all_dy.end());
        // float median_dx = all_dx[all_dx.size()/2];
        // float median_dy = all_dy[all_dy.size()/2];

        // std::cout << "median flow: dx=" << median_dx << " dy=" << median_dy << "\n";
    } 



     // Create an image where the motion vector angle is
     // mapped to a color hue, and intensity is proportional
     // to vector's magnitude.
     cv::Mat magnitude, angle;
     {
         cv::Mat ch[2];
         split(flow, ch);
         cv::cartToPolar(ch[0], ch[1], magnitude, angle, true);
         cv::medianBlur(ch[0], ch[0], 3);
         cv::medianBlur(ch[1], ch[1], 3);
     }
  
     float clip = 3.0f;
     cv::threshold(magnitude, magnitude, clip, clip, cv::THRESH_TRUNC);
  
     // build hsv image
     cv::Mat _hsv[3], hsv, bgr;
     _hsv[0] = angle;
     _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
     _hsv[2] = magnitude / clip; // intensity must vary from 0 to 1
     merge(_hsv, 3, hsv);
  
     cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
     bgr.convertTo(outputImage, CV_8U, 255.0);
 }
    bool process(const cv::Mat &frame, FlowStats *stats = nullptr)
{
    if (frame.empty()) return false;

    static int ofaInit = false;

    if (!ofaInit) {
        cv::Mat cvPrevFrame(frame);

        CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_OFA | VPI_BACKEND_CUDA | VPI_BACKEND_VIC, &stream));

        CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvPrevFrame, 0, &imgPrevFrameLoad));
        CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvPrevFrame, 0, &imgCurFrameLoad));

        int32_t multiple = (1 << (numLevels_ - 1));
        int width  = ((cvPrevFrame.cols + multiple - 1) / multiple) * multiple;
        int height = ((cvPrevFrame.rows + multiple - 1) / multiple) * multiple;

        // 1) Rescale hedefleri U8 (PL)
        CHECK_STATUS(vpiImageCreate(width, height, imgFmtPL_U8,   0, &imgPrevFramePL));
        CHECK_STATUS(vpiImageCreate(width, height, imgFmtPL_U8,   0, &imgCurFramePL));

        // 2) U8 -> Y8_ER geçicileri (PL)
        CHECK_STATUS(vpiImageCreate(width, height, imgFmtPL_Y8ER, 0, &imgPrevFrameTmp));
        CHECK_STATUS(vpiImageCreate(width, height, imgFmtPL_Y8ER, 0, &imgCurFrameTmp));

        // 3) Piramitler
        CHECK_STATUS(vpiPyramidCreate(width, height, imgFmtPL_Y8ER, numLevels_, 0.5, 0, &prevPyrTmp));
        CHECK_STATUS(vpiPyramidCreate(width, height, imgFmtPL_Y8ER, numLevels_, 0.5, 0, &curPyrTmp));
        CHECK_STATUS(vpiPyramidCreate(width, height, imgFmtBL_Y8ER, numLevels_, 0.5, 0, &prevPyrBL));
        CHECK_STATUS(vpiPyramidCreate(width, height, imgFmtBL_Y8ER, numLevels_, 0.5, 0, &curPyrBL));

        // 4) Dense OF payload
        std::vector<int32_t> pyrGridSize(numLevels_, gridSize_);
        CHECK_STATUS(vpiCreateOpticalFlowDense(VPI_BACKEND_OFA, width, height, imgFmtBL_Y8ER,
                                               pyrGridSize.data(), pyrGridSize.size(),
                                               quality_, &payload));

        // 5) MV çıkışları
        int32_t mvWidth  = (width  + gridSize_ - 1) / gridSize_;
        int32_t mvHeight = (height + gridSize_ - 1) / gridSize_;
        CHECK_STATUS(vpiImageCreate(mvWidth, mvHeight, VPI_IMAGE_FORMAT_2S16_BL, 0, &imgMotionVecBL));
        CHECK_STATUS(vpiImageCreate(mvWidth, mvHeight, VPI_IMAGE_FORMAT_2S16,    0, &imgMotionVecPL));

        // ---- İlk frame hattı ----
        // Rescale: U8 -> U8 (formatlar aynı!)
        CHECK_STATUS(vpiSubmitRescale(stream, VPI_BACKEND_CUDA,
                                           imgPrevFrameLoad, imgPrevFramePL,
                                           VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
        // U8(PL) -> Y8_ER(PL)
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA,
                                                 imgPrevFramePL, imgPrevFrameTmp, nullptr));
        // Y8_ER(PL) -> pyr(PL) -> pyr(BL)
        CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream, VPI_BACKEND_CUDA,
                                                       imgPrevFrameTmp, prevPyrTmp, VPI_BORDER_CLAMP));
        // not: üstte vpiPyramidGetLevel(prevPyrTmp,0) gerekmiyorsa kaldır; bazı VPI sürümlerinde direkt tmp verilir.
        CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream, VPI_BACKEND_CUDA,
                                                       imgPrevFrameTmp, prevPyrTmp, VPI_BORDER_CLAMP));
        CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream, VPI_BACKEND_VIC,
                                                        prevPyrTmp, prevPyrBL, nullptr));

        ofaInit = true;
    }

    // ---- Sonraki frame ----
    cv::Mat cvCurFrame(frame);
    cv::Mat denoised;

    cv::GaussianBlur(cvCurFrame, denoised, cv::Size(3,3), 0.8);
    CHECK_STATUS(vpiImageSetWrappedOpenCVMat(imgCurFrameLoad, denoised));

    // Rescale: U8 -> U8
    CHECK_STATUS(vpiSubmitRescale(stream, VPI_BACKEND_CUDA,
                                       imgCurFrameLoad, imgCurFramePL,
                                       VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));

    // U8(PL) -> Y8_ER(PL)
    CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA,
                                             imgCurFramePL, imgCurFrameTmp, nullptr));

    // PL(Y8_ER) -> pyr(PL) -> pyr(BL)
    CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream, VPI_BACKEND_CUDA,
                                                   imgCurFrameTmp, curPyrTmp, VPI_BORDER_CLAMP));
    CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream, VPI_BACKEND_VIC,
                                                    curPyrTmp, curPyrBL, nullptr));

    // Dense OF
    CHECK_STATUS(vpiSubmitOpticalFlowDensePyramid(stream, VPI_BACKEND_OFA,
                                                  payload, prevPyrBL, curPyrBL, imgMotionVecBL));

    // BL -> PL
    CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC,
                                             imgMotionVecBL, imgMotionVecPL, nullptr));

    CHECK_STATUS(vpiStreamSync(stream));

  
    std::swap(prevPyrBL, curPyrBL);
   
    cv::Mat mvOutputImage;
    //ProcessMotionVector(imgMotionVecPL, mvOutputImage);


         // Lock the input image to access it from CPU
     VPIImageData mvData;
     CHECK_STATUS(vpiImageLockData(imgMotionVecPL, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData));
  
     // Create a cv::Mat that points to the input image data
     cv::Mat mvImage;
     CHECK_STATUS(vpiImageDataExportOpenCVMat(mvData, &mvImage));
  
     // Convert S10.5 format to float
     cv::Mat flow(mvImage.size(), CV_32FC2);
     mvImage.convertTo(flow, CV_32F, 1.0f / (1 << 5));

  
     // Image not needed anymore, we can unlock it.,
     CHECK_STATUS(vpiImageUnlock(imgMotionVecPL));
        cv::Vec2f gdxdy;
     if(estimateGlobalDxDyRANSAC(flow, gdxdy)){
        float dx = gdxdy[0];
        float dy = gdxdy[1];
//    std::cout << "dx=" << dx << " dy=" << dy << std::endl;    
    // --- EMA ile zaman filtreleme ---
        static cv::Vec2f ema(0,0);   // ilk frame için sıfırdan başla
        float alpha = 0.25f;         // 0.1 → çok yumuşak, 0.5 → hızlı tepki

        ema = (1.0f-alpha)*ema + alpha*gdxdy;

   
        //std::cout << "dx=" << ema[0] << " dy=" << ema[1] << std::endl;
        if(stats){
            stats->mean_dx = ema[0];
            stats->mean_dy = ema[1];
        }
    
        }else{
            cv::Scalar m = cv::mean(flow);
            float dx = (float)m[0];
            float dy = (float)m[1];
            //std::cout << "[fallback] dx=" << dx << " dy=" << dy << std::endl;

        // std::vector<float> all_dx, all_dy;
        // all_dx.reserve(flow.total());
        // all_dy.reserve(flow.total());

        // for (int y=0; y<flow.rows; ++y) {
        //     const cv::Vec2f* row = flow.ptr<cv::Vec2f>(y);
        //     for (int x=0; x<flow.cols; ++x) {
        //         all_dx.push_back(row[x][0]);
        //         all_dy.push_back(row[x][1]);
        //     }
        // }
        // std::nth_element(all_dx.begin(), all_dx.begin()+all_dx.size()/2, all_dx.end());
        // std::nth_element(all_dy.begin(), all_dy.begin()+all_dy.size()/2, all_dy.end());
        // float median_dx = all_dx[all_dx.size()/2];
        // float median_dy = all_dy[all_dy.size()/2];

        // std::cout << "median flow: dx=" << median_dx << " dy=" << median_dy << "\n";
    } 


    // cv::imshow("output", mvOutputImage);
    // cv::waitKey(1);



    return true;
}

    // bool  process(const cv::Mat &frame, FlowStats *stats = nullptr) {

    //     static int ofaInit = false;
    //     if (frame.empty())
    //         return false;

    
    //     if(!ofaInit){
    //         // OpenCV sar
    //         cv::Mat cvPrevFrame(frame);

    //         // ------- VPI Nesneleri -------
          
    //         CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_OFA | VPI_BACKEND_CUDA | VPI_BACKEND_VIC, &stream));

      
    //         CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvPrevFrame, 0, &imgPrevFrameLoad));
    //         CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvPrevFrame, 0, &imgCurFrameLoad));

          

    //         // Boyutu 2^(numLevels-1) katına hizala
    //         int32_t multiple = (1 << (numLevels_ - 1));
    //         int width        = ((cvPrevFrame.cols + multiple - 1) / multiple) * multiple;
    //         int height       = ((cvPrevFrame.rows + multiple - 1) / multiple) * multiple;

           
    //         CHECK_STATUS(vpiImageCreate(width, height, imgFmt, 0, &imgPrevFramePL));
    //         CHECK_STATUS(vpiImageCreate(width, height, imgFmt, 0, &imgCurFramePL));

    //         // Rescale (pad/hizalama)
    //         CHECK_STATUS(vpiSubmitRescale(stream, VPI_BACKEND_CUDA, imgPrevFrameLoad, imgPrevFramePL,
    //                                       VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
    //         CHECK_STATUS(vpiSubmitRescale(stream, VPI_BACKEND_CUDA, imgCurFrameLoad,  imgCurFramePL,
    //                                       VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
            
    //         // Piramitler ve geçiciler
           
    //         CHECK_STATUS(vpiPyramidCreate(width, height, imgFmt,   numLevels_, 0.5, 0, &prevPyrTmp));
    //         CHECK_STATUS(vpiPyramidCreate(width, height, imgFmt,   numLevels_, 0.5, 0, &curPyrTmp));
    //         CHECK_STATUS(vpiPyramidCreate(width, height, imgFmtBL, numLevels_, 0.5, 0, &prevPyrBL));
    //         CHECK_STATUS(vpiPyramidCreate(width, height, imgFmtBL, numLevels_, 0.5, 0, &curPyrBL));
            
    //         // Dense OF payload
    //         std::vector<int32_t> pyrGridSize(numLevels_, gridSize_);
 
    //         CHECK_STATUS(vpiCreateOpticalFlowDense(VPI_BACKEND_OFA, width, height, imgFmtBL,
    //                                                pyrGridSize.data(), pyrGridSize.size(),
    //                                                quality_, &payload));
            
    //         // MV boyutları (grid’e göre)
    //         int32_t mvWidth  = (width  + gridSize_ - 1) / gridSize_;
    //         int32_t mvHeight = (height + gridSize_ - 1) / gridSize_;
            
            
    //         CHECK_STATUS(vpiImageCreate(mvWidth, mvHeight, VPI_IMAGE_FORMAT_2S16_BL, 0, &imgMotionVecBL));
    //         CHECK_STATUS(vpiImageCreate(mvWidth, mvHeight, VPI_IMAGE_FORMAT_2S16,    0, &imgMotionVecPL));
            
    //         // İlk frame: PL -> pyr(PL) -> pyr(BL)
    //         CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream, VPI_BACKEND_CUDA, imgPrevFramePL, prevPyrTmp, VPI_BORDER_CLAMP));
    //         CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream, VPI_BACKEND_VIC,  prevPyrTmp,     prevPyrBL, nullptr));

    //          cv::Mat mvOutputImage;

            

    //         ofaInit = true;

    //     }
    //     cv::Mat cvCurFrame(frame);

    //     // VPI wrapper güncelle
    //     CHECK_STATUS(vpiImageSetWrappedOpenCVMat(imgCurFrameLoad, cvCurFrame));

    //     // Pad/hizalama için rescale
    //     CHECK_STATUS(vpiSubmitRescale(stream, VPI_BACKEND_CUDA, imgCurFrameLoad, imgCurFramePL,
    //                                   VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));

    //     // PL -> pyr(PL) -> pyr(BL)
    //     CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream, VPI_BACKEND_CUDA, imgCurFramePL, curPyrTmp, VPI_BORDER_CLAMP));
    //     CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream, VPI_BACKEND_VIC,  curPyrTmp,    curPyrBL,  nullptr));

    //     // Dense OF
    //     CHECK_STATUS(vpiSubmitOpticalFlowDensePyramid(stream, VPI_BACKEND_OFA, payload, prevPyrBL, curPyrBL, imgMotionVecBL));

    //     // BL -> PL
    //     CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_VIC, imgMotionVecBL, imgMotionVecPL, nullptr));

    //     // Senkronize et (RealSense buffer'ı serbest kalmadan önce bitirmek önemli!)
    //     CHECK_STATUS(vpiStreamSync(stream));


    //     VPIImageData mvData;
    //     CHECK_STATUS(vpiImageLockData(imgMotionVecPL, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData));

    //     cv::Mat mvImage;
    //     CHECK_STATUS(vpiImageDataExportOpenCVMat(mvData, &mvImage));

    //     // S10.5 -> float
    //     cv::Mat flow(mvImage.size(), CV_32FC2);
    //     mvImage.convertTo(flow, CV_32F, 1.0f / (1 << 5));

    //     CHECK_STATUS(vpiImageUnlock(imgMotionVecPL));

    //     // HSV görselleştirme
    //     cv::Mat magnitude, angle;
    //     cv::Mat ch[2];
    //     cv::split(flow, ch);
    //     cv::cartToPolar(ch[0], ch[1], magnitude, angle, true);

    //     const float clip = 5.0f;
    //     cv::threshold(magnitude, magnitude, clip, clip, cv::THRESH_TRUNC);

    //     cv::Mat H = angle;
    //     cv::Mat S = cv::Mat::ones(angle.size(), CV_32F);
    //     cv::Mat V = magnitude / clip;

    //     cv::Mat hsv;
    //     cv::merge(std::vector<cv::Mat>{H, S, V}, hsv);
        
    //     cv::imshow("hsv", hsv);
    //     cv::waitKey(1);

    //     // if (frame.empty())
    //     //     return false;

    //     // // OpenCV Mat wrap (ilk sefer yarat, sonra set)
    //     // if (!imgCurWrap_) {
    //     //     CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(frame, 0, &imgCurWrap_));
    //     // } else {
    //     //     CHECK_STATUS(vpiImageSetWrappedOpenCVMat(imgCurWrap_, frame));
    //     // }

    //     // // BGR8/Y8 -> Y8/PL
    //     // CHECK_STATUS(vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_CUDA, imgCurWrap_, imgTmpY8PL_, nullptr));

    //     // // PL -> pyramid(PL)
    //     // CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream_, VPI_BACKEND_CUDA, imgTmpY8PL_, pyrCurPL_, VPI_BORDER_CLAMP));

    //     // // pyramid(PL) -> pyramid(BL)
    //     // CHECK_STATUS(vpiSubmitConvertImageFormatPyramid(stream_, VPI_BACKEND_VIC, pyrCurPL_, pyrCurBL_, nullptr));

    //     // bool produced = false;

    //     // if (hasPrev_) {
    //     //     // Dense OFA
    //     //     CHECK_STATUS(vpiSubmitOpticalFlowDensePyramid(stream_, VPI_BACKEND_OFA, payload_, pyrPrevBL_, pyrCurBL_, imgMV_BL_));

    //     //     // 2S16 BL -> 2S16 PL 
    //     //     CHECK_STATUS(vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_VIC, imgMV_BL_, imgMV_PL_, nullptr));

    //     //     // Senkron
    //     //     CHECK_STATUS(vpiStreamSync(stream_));

    //     //     // ---- 2S16(S10.5) -> CV_32FC2 flow (dx,dy) ----
    //     //     VPIImageData mvData{};
    //     //     CHECK_STATUS(vpiImageLockData(imgMV_PL_, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &mvData));
    //     //     cv::Mat mvS16; CHECK_STATUS(vpiImageDataExportOpenCVMat(mvData, &mvS16));
    //     //     cv::Mat flowFC2; mvS16.convertTo(flowFC2, CV_32FC2, 1.0f / (1 << 5));
    //     //     CHECK_STATUS(vpiImageUnlock(imgMV_PL_));
            
            
    //     //     // --- Gürültü azaltma ve robust ortalama ---
    //     //     std::vector<cv::Mat> ch(2);
    //     //     cv::split(flowFC2, ch);
    //     //     cv::Mat mag, ang;
    //     //     cv::cartToPolar(ch[0], ch[1], mag, ang, false);

    //     //     cv::Mat mask;
    //     //     cv::threshold(mag, mask, 0.1f, 255, cv::THRESH_BINARY);
    //     //     mask.convertTo(mask, CV_8U);
    //     //     cv::Scalar mDx = cv::mean(ch[0], mask);
    //     //     cv::Scalar mDy = cv::mean(ch[1], mask);
    //     //     if (stats) {
    //     //         stats->mean_dx = static_cast<float>(mDx[0]);
    //     //         stats->mean_dy = static_cast<float>(mDy[0]);

    //     //     }
    //     //     produced = true;


    //     // }else{
    //     //     CHECK_STATUS(vpiStreamSync(stream_));
    //     //     hasPrev_ = true;
    //     // }
            
    //     // // prev <- cur (piramitleri takasla)
    //     // std::swap(pyrPrevPL_, pyrCurPL_);
    //     // std::swap(pyrPrevBL_, pyrCurBL_);

    //     return true;
    // }

    int mvWidth()  const { return mvW_; }
    int mvHeight() const { return mvH_; }


private:
    int w_{}, h_{};
    int gridSize_{}, numLevels_{};
    int mvW_{}, mvH_{};
    VPIOpticalFlowQuality quality_{};

  VPIStream stream = nullptr;
  VPIImage imgPrevFrameTmp = nullptr;
VPIImage imgCurFrameTmp  = nullptr;
              // Wrapper’lar
        VPIImage imgPrevFrameLoad = nullptr;
        VPIImage imgCurFrameLoad  = nullptr;
  // Formatlar: IR = Y8
           VPIImageFormat imgFmtPL_U8   = VPI_IMAGE_FORMAT_U8;       // Rescale için
VPIImageFormat imgFmtPL_Y8ER = VPI_IMAGE_FORMAT_Y8_ER;    // OFA girişi için
VPIImageFormat imgFmtBL_Y8ER = VPI_IMAGE_FORMAT_Y8_ER_BL; 


             // PL hedef tamponları (padlenmiş boyut)
            VPIImage imgPrevFramePL = nullptr;
            VPIImage imgCurFramePL  = nullptr;

            VPIPyramid prevPyrTmp = nullptr, prevPyrBL = nullptr;
            VPIPyramid curPyrTmp  = nullptr,  curPyrBL  = nullptr;

    VPIImage imgMotionVecBL = nullptr, imgMotionVecPL = nullptr;

    VPIPayload payload = nullptr;


    // VPIStream   stream_     = nullptr;
    // VPIImage    imgCurWrap_ = nullptr;   // OpenCV frame wrap (BGR veya Y8)
    // VPIImage    imgTmpY8PL_ = nullptr;   // Y8/PL geçici
    // VPIPyramid  pyrPrevPL_  = nullptr;
    // VPIPyramid  pyrPrevBL_  = nullptr;
    // VPIPyramid  pyrCurPL_   = nullptr;
    // VPIPyramid  pyrCurBL_   = nullptr;
    // VPIImage    imgMV_BL_   = nullptr;
    // VPIImage    imgMV_PL_   = nullptr;
    // VPIPayload  payload_    = nullptr;

    // bool hasPrev_ = false;
};





using namespace hrvs_sdk;

unsigned RADARS_NUMBER = 4;

// UART config parameters:
constexpr char UART_PORT[] = "/dev/ttyTHS1";
constexpr int UART_BAUDRATE = 921600;
constexpr IUART::Parity UART_PARITY = IUART::Parity::NO;
constexpr IUART::StopBits UART_STOP_BITS = IUART::StopBits::ONE;
constexpr int UART_DATABITS = 8;     

// GPIO config parameters:
constexpr unsigned PIN_ID = 398;
char PIO_ID[] = "PH.07";
//JP6 için revizyonlanmıstır  -- BURAK

static bool set_pad_output_ph07(uint32_t value = 0x00000140) {
    constexpr off_t PHYS = 0x02434088; // PH.07 PADCTL register
    const long pagesize = sysconf(_SC_PAGESIZE);
    if (pagesize <= 0) return false;

    const off_t page_base = PHYS & ~(off_t(pagesize - 1));
    const off_t page_off  = PHYS - page_base;

    int fd = ::open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        std::cerr << "padfix: open(/dev/mem) failed: " << std::strerror(errno) << "\n";
        return false;
    }

    void* map = ::mmap(nullptr, pagesize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
    if (map == MAP_FAILED) {
        std::cerr << "padfix: mmap failed: " << std::strerror(errno) << "\n";
        ::close(fd);
        return false;
    }

    volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
        static_cast<char*>(map) + page_off);

    *reg = value;                 // 0x140 = output, bias disabled (istersen 0x144 = output+pull-up)
    __sync_synchronize();         // memory barrier (ihtiyaten)

    ::munmap(map, pagesize);
    ::close(fd);
    return true;
}

void configureUart(UARTLinux *uartPtr)
{
    UARTLinux::ReturnCode uartReturnCode;

    uartReturnCode = static_cast<UARTLinux::ReturnCode>(uartPtr->initUART(UART_PORT));
    if(uartReturnCode == UARTLinux::ReturnCode::OK)
    {
        std::cout << "example: HRVS EAP UART initialization - passed" << std::endl;
    }
    else
    {
        std::cerr << "example: HRVS EAP UART initialization  - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }

    uartReturnCode = static_cast<UARTLinux::ReturnCode>(uartPtr->configureUART(UART_BAUDRATE, UART_PARITY, UART_STOP_BITS, UART_DATABITS));

    if (uartReturnCode == UARTLinux::ReturnCode::OK)
    {
        std::cout << "example: HRVS EAP UART configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "example: HRVS EAP UART configuration - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }
}

void configureGpio(GPIOPinLinux *gpioPinPtr)
{
    if (gpioPinPtr->configure(PIN_ID, PIO_ID))
    {
        std::cout << "example: GPIO SYNC_IN configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "example: GPIO SYNC_IN configuration - failed" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void parseStatusVariable(const RadarSystem& radarSystem, const ProcessingConfig & processingConfig)
{
    unsigned systemFrameNumber;
    RadarSystem::SystemState systemState;
    unsigned long long status; 
    unsigned resetCounter = 0;
    radarSystem.getRadarSystemState(systemFrameNumber, systemState, status, resetCounter);
    
    // example of how you can get more information about occurred error by applying bit masks on HRVS SDK status variable
    if(GET_STATUS_ITEM2(status) == RadarSystem::ReturnCode::ERR_RADAR_MSG)
    {
        switch(static_cast<RadarSystem::RadarMessageError>(GET_STATUS_ITEM3(status)))
        {
            case RadarSystem::RadarMessageError::MSG_TIMEOUT:
                std::cerr << "example: parseStatusVariable() - timeout when receiving '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)]  << "' from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl;
                break;
            case RadarSystem::RadarMessageError::MSG_CORRUPTED:
                std::cerr << "example: parseStatusVariable() - data of received '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)]  << "' from radar " << unsigned(GET_STATUS_ITEM8(status))  << '/' << processingConfig.numberOfRadars << " are corrupted" << std::endl;
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_MSG_TYPE:
                std::cerr << "example: parseStatusVariable() - system received radar message '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)] << "' from radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " when expected message was '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)] << "' from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl; 
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_RADAR:
                std::cerr << "example: parseStatusVariable() - system received radar message '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)] << "' from radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " when it should be received from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl;
                break;
            case RadarSystem::RadarMessageError::RADAR_ERR:
                std::cerr << "example: parseStatusVariable() - radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " failure" << std::endl;
                break;
            default:
                break;
        }
    }
    // method can be extended to handle any other error (not just RadarSystem::ReturnCode::ERR_RADAR_MSG) 
    // for more details see composition of status variable inside doxygen documentation for getRadarSystemState()
}



static void drawHUD(cv::Mat &frame, double fx, double fy, double fps, uint64_t timestamp)
{
    const int pad = 8;
    int baseLine = 0;

    // Yazı satırlarını hazırla
    std::string line1 = "OptFlow X: " + std::to_string(fx);
    std::string line2 = "OptFlow Y: " + std::to_string(fy);
    std::string line3 = "FPS: " + std::to_string(fps);
    std::string line4 = "TIMESTAMP: " + std::to_string(timestamp);

    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.6;
    int thickness = 2;

    cv::Size t1 = cv::getTextSize(line1, fontFace, fontScale, thickness, &baseLine);
    cv::Size t2 = cv::getTextSize(line2, fontFace, fontScale, thickness, &baseLine);
    cv::Size t3 = cv::getTextSize(line3, fontFace, fontScale, thickness, &baseLine);
    cv::Size t4 = cv::getTextSize(line4, fontFace, fontScale, thickness, &baseLine);

    int boxW = std::max(std::max(t1.width, t4.width), std::max(t2.width, t3.width)) + pad * 2;
    int lineH = std::max({t1.height, t2.height, t3.height, t4.height}) + 4;
    int boxH = lineH * 3 + pad * 2;

    // Yarı şeffaf arka plan
    cv::Mat roi = frame(cv::Rect(0, 0, boxW, boxH));
    cv::Mat overlay;
    roi.copyTo(overlay);
    cv::rectangle(overlay, cv::Rect(0, 0, boxW, boxH), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::addWeighted(overlay, 0.5, roi, 0.5, 0, roi);

    // Yazıları çiz
    int y = pad + lineH - 4;
    cv::putText(frame, line1, cv::Point(pad, y), fontFace, fontScale, cv::Scalar(255,255,255), thickness, cv::LINE_AA);
    y += lineH;
    cv::putText(frame, line2, cv::Point(pad, y), fontFace, fontScale, cv::Scalar(255,255,255), thickness, cv::LINE_AA);
    y += lineH;
    cv::putText(frame, line3, cv::Point(pad, y), fontFace, fontScale, cv::Scalar(255,255,255), thickness, cv::LINE_AA);
    y += lineH;
    cv::putText(frame, line4, cv::Point(pad, y), fontFace, fontScale, cv::Scalar(255,255,255), thickness, cv::LINE_AA);
}

// cx, cy: zoom merkez noktası (piksel). Çıktı boyutu frame.size() kalır.
cv::Mat pointZoom2x(const cv::Mat& frame, int cx, int cy, int interp=cv::INTER_LINEAR)
{
    const int W = frame.cols, H = frame.rows;
    int w = W / 2;
    int h = H / 2;

    // ROI'yi (cx,cy) merkezli tut, sınırları clamp et
    int x = cx - w/2;
    int y = cy - h/2;
    x = std::max(0, std::min(x, W - w));
    y = std::max(0, std::min(y, H - h));

    cv::Rect roi(x, y, w, h);
    cv::Mat out;
    cv::resize(frame(roi), out, frame.size(), 0, 0, interp);
    return out;
}

int main(int argc, char *argv[])
{
    memset(&jetsonPack.data, 0x00, sizeof(jetsonPack.data));
    void* h = dlopen("librealsense2.so", RTLD_LAZY | RTLD_LOCAL | 0x00008 /*RTLD_DEEPBIND*/);
    (void)h;
    /************   PARSE CMD ARGUMENTS   ************/

    //default settings:
    int measurementsCount = 5;

    // processing command line arguments:
    int c;
    while ((c = getopt(argc, argv, "m:")) != -1)
    {
        switch (c)
        {
        case 'm':
            measurementsCount = atoi(optarg);
            break;
        default:
            std::cout << "Usage: ./example.out [-m MEASUREMENTS_COUNT]" << std::endl;
            return EXIT_FAILURE;
        }
    }
    /************   1.CONFIGURE INTERFACES   ************/

    RadarSystem radarSystem;
    RadarSystem::ReturnCode rsReturnCode;

    UARTLinux uart;
    GPIOPinLinux gpioPin;
    TimeStdlib timeSource;
    ThreadStdlib currentThread;
 
    
     if (!set_pad_output_ph07(/*0x00000140*/)) {
        std::cerr << "padfix: PH.07 output ayarlanamadi (root ve /dev/mem gerekli)\n";
        
    }

    configureUart(&uart);
    configureGpio(&gpioPin);

    rsReturnCode = radarSystem.configureInterfaces(&uart, &gpioPin, &timeSource, &currentThread);
    
    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        std::cout << "example: PAL interfaces configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "example: PAL interfaces configuration - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        return EXIT_FAILURE;
    }
    
    /************   2.CONFIGURE RADAR SYSTEM   ************/
    
    // Configuration settings for radar system:  
    ProcessingConfig processingConfig;
    processingConfig.numberOfRadars = RADARS_NUMBER;
    RadarConfig radarConfig;
    radarConfig.radarType = RADAR_60GHZ_PROTO_1_0_CONFIG;

    processingConfig.numberOfRadars = RADARS_NUMBER;
    processingConfig.rollOff = 0.0;
    processingConfig.pitchOff = 0.0;
    processingConfig.yawOff = 0.0;
    processingConfig.slopeRadar1 = 15.0;
    processingConfig.slopeRadar2 = 15.0;
    processingConfig.slopeRadar3 = 15.0;
    processingConfig.slopeRadar4 = 15.0;
    processingConfig.psiOffsetRadar1 = 0.0;
    processingConfig.psiOffsetRadar2 = 0.0;
    processingConfig.psiOffsetRadar3 = 0.0;
    processingConfig.psiOffsetRadar4 = 0.0;


    bool autoResetMode = true;
    bool persistMode = true; 

    radarSystem.setConfigurations(processingConfig, radarConfig, autoResetMode, persistMode);

    std::cout << "example: radar system configuration - passed" << std::endl;
    
    /************   3. HRVS CONFIGURATION   ************/
    
    rsReturnCode = radarSystem.hrvsConfiguration();

    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        std::cout << "example: HRVS configuration - passed" << std::endl;
    }
    else
    {
        parseStatusVariable(radarSystem, processingConfig);
        std::cerr << "example:  HRVS configuration - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        return EXIT_FAILURE;
    }
    
    /************   4. TRIGGER MEASUREMENT AND GET RADAR DATA   ************/
    
    int performedMeasurements = 0;
    // declaration of variables for data retrieved by HRVS SDK
    unsigned systemFrameNumber;
    float bodyVelocity[3];
    float avgRange;
    float radarMinRange;
    float radarVelocity;
    float radarSnr;

   // auto startTime = std::chrono::steady_clock::now();
   // auto prevTime  = startTime;
    
    rs2::context ctx;       
    rs2::pipeline pipe(ctx);

    BQueue<ImgMsg> q_tag(4), q_flow(4);
    std::atomic<bool> run{true};
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 60);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920,1080, RS2_FORMAT_RGB8, 30);
    auto profile = pipe.start(cfg);
    rs2::device dev = profile.get_device();
    auto depth_sensor = dev.first<rs2::depth_sensor>();
    auto rgb_sensor = dev.first<rs2::color_sensor>();
    auto set_if_supported = [&](rs2::sensor& s, rs2_option opt, float val){
        if (!s.supports(opt)) return false;
        auto r = s.get_option_range(opt);
        val = std::min(std::max(val, r.min), r.max);
        s.set_option(opt, val);
        return true;
    };
    // set_if_supported(rgb_sensor, RS2_OPTION_BRIGHTNESS, -2.0f);
    // set_if_supported(rgb_sensor, RS2_OPTION_CONTRAST, 100.0f);
    // set_if_supported(rgb_sensor, RS2_OPTION_GAMMA, 100.0f );
    // set_if_supported(rgb_sensor, RS2_OPTION_HUE, 0.0f );
    // set_if_supported(rgb_sensor, RS2_OPTION_SATURATION, 0.0f );
    // set_if_supported(rgb_sensor, RS2_OPTION_SHARPNESS, 100.0f );
    // set_if_supported(rgb_sensor, RS2_OPTION_BACKLIGHT_COMPENSATION, 0.0f );


    // for (auto s : profile.get_device().query_sensors()) {
    // if (s.supports(RS2_OPTION_EMITTER_ENABLED)) {
    //     s.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    // }
    // }

    // for (auto s : profile.get_device().query_sensors()) {
    // if (s.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
    //     s.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.f);

    // // if (s.supports(RS2_OPTION_EXPOSURE))  // mikro-saniye
    // //     s.set_option(RS2_OPTION_EXPOSURE, 3000.f);  // 3000 us ≈ 3 ms

    // // if (s.supports(RS2_OPTION_GAIN))
    // //     s.set_option(RS2_OPTION_GAIN, 16.f);        // makul başlangıç
    // }
     // Sensörü al
    auto device_ = pipe.get_active_profile().get_device();
    rs2::sensor sensor;
    for (auto&& s : device_.query_sensors()) {
        if (s.supports(RS2_OPTION_EXPOSURE)) {
            sensor = s;
            break;
        }
    }

    if (sensor) {
        if(sensor.supports(RS2_OPTION_VISUAL_PRESET)){
            sensor.set_option(RS2_OPTION_VISUAL_PRESET, 5);
        }

        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
        // sensor.set_option(RS2_OPTION_EXPOSURE, 20.0f);
        // sensor.set_option(RS2_OPTION_GAIN, 16.0f);

        std::cout << "IR exposure set to 50 us" << std::endl;
    } else {
        std::cerr << "IR sensor not found!" << std::endl;
    }

    auto vsp = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_stream.get_intrinsics();
    
    const float fx = intr.fx, fy = intr.fy, cx = intr.ppx, cy = intr.ppy;

    VPICameraIntrinsic Ktmp = { { fx, 0.0f, cx }, { 0.0f, fy, cy } };
    std::memcpy(&g_K, &Ktmp, sizeof(g_K));
    std::cout << "IR1 intrinsics: fx="<<fx<<" fy="<<fy<<" cx="<<cx<<" cy="<<cy << std::endl;

    const float tagSizeMeters = 0.270f;
    const float tilt_deg      = 47.0f;
    std::map<int,TagMeta> meta = {
            // {32, {  0.0f,  {  0.00f, -0.260f, 0.00f }}},
            // {50, { 90.0f,  {  0.260f, 0.00f,  0.00f }}},
            // {15, {-90.0f,  {  -0.260f, 0.00f,  0.00f }}},
            // {87, {180.0f,  {  0.00f,  0.260f, 0.00f }}},
        
            {0, {  0.0f,  {  0.00f, -0.280f, 0.00f }}},
            {24, { 90.0f,  { 0.280f, 0.00f,  0.00f }}},
            {36, {-90.0f,  {  -0.280f, 0.00f,  0.00f }}},
            //{36, {-90.0f,  {  1.03f, 0.00f,  0.00f }}},
            {20, {180.0f,  {  0.00f,  0.280f, 0.00f }}},
        
    };
    DenseOptFlowOFA ofa(640, 480, 4, 3, VPI_OPTICAL_FLOW_QUALITY_HIGH);
    AprilTagEstimator est(1920, 1080, tagSizeMeters, VPI_APRILTAG_36H11, tilt_deg, meta, 64, 1); 

//     // ---- Top: yardımcı makro
#define LOGERR(msg) do { std::cerr << "[ERR] " << msg << std::endl; } while(0)

//thread tcpSocketThread(tcpSocketTask);


std::thread th_tcp([&]{
 int clientSd;
    std::cout << "tcp socket task executed" << std::endl;  
    const char* portName = "/dev/ttyCP210x"; 
int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Port açılamadı: " << portName << std::endl;
        return 1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr hatası" << std::endl;
        close(fd);
        return 1;
    }

    cfsetospeed(&tty, B115200); // baud rate out
    cfsetispeed(&tty, B115200); // baud rate in

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // break işlemesini kapat
    tty.c_lflag = 0;                            // canonical mode yok, echo yok
    tty.c_oflag = 0;                            // raw output
    tty.c_cc[VMIN]  = 1;                        // blok için minimum karakter
    tty.c_cc[VTIME] = 1;                        // timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // XON/XOFF yok
    tty.c_cflag |= (CLOCAL | CREAD);            // modem kontrollerini yoksay, oku aktif et
    tty.c_cflag &= ~(PARENB | PARODD);          // parity kapalı
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // donanım flow control yok

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr hatası" << std::endl;
        close(fd);
        return 1;
    }
    while(1){

                jetsonPack.pack.header.sync_byte = SYNC_BYTE;
                jetsonPack.pack.header.msg_id = MSG_ID;
                jetsonPack.pack.header.payloadSize = sizeof(jetsonPack.data) - 5;    

                uint32_t tempChk = 0;
                for (int i = 0; i < 3; i++)
                {
                    tempChk += jetsonPack.data[i];
                }
                jetsonPack.pack.header.headerchk = tempChk % 256;
                jetsonPack.pack.header.headerchk = ~(jetsonPack.pack.header.headerchk) + 1;

                jetsonPack.pack.variables.aprtiltag.timeStamp = tagPack.timeStamp;
                jetsonPack.pack.variables.aprtiltag.avgPos.pos.x = tagPack.avgPos.pos.x;
                jetsonPack.pack.variables.aprtiltag.avgPos.pos.y = tagPack.avgPos.pos.y;
                jetsonPack.pack.variables.aprtiltag.avgPos.pos.z = tagPack.avgPos.pos.z;
                jetsonPack.pack.variables.aprtiltag.avgPos.valid = tagPack.avgPos.valid;

                jetsonPack.pack.variables.aprtiltag.tag1.pos.x = tagPack.tag1.pos.x;
                jetsonPack.pack.variables.aprtiltag.tag1.pos.y = tagPack.tag1.pos.y;
                jetsonPack.pack.variables.aprtiltag.tag1.pos.z = tagPack.tag1.pos.z;
                jetsonPack.pack.variables.aprtiltag.tag1.valid = tagPack.tag1.valid;
                
                jetsonPack.pack.variables.aprtiltag.tag1.angle.roll = tagPack.tag1.angle.roll;
                jetsonPack.pack.variables.aprtiltag.tag1.angle.pitch = tagPack.tag1.angle.pitch;
                jetsonPack.pack.variables.aprtiltag.tag1.angle.yaw = tagPack.tag1.angle.yaw;

                jetsonPack.pack.variables.aprtiltag.tag2.pos.x = tagPack.tag2.pos.x;
                jetsonPack.pack.variables.aprtiltag.tag2.pos.y = tagPack.tag2.pos.y;
                jetsonPack.pack.variables.aprtiltag.tag2.pos.z = tagPack.tag2.pos.z;
                jetsonPack.pack.variables.aprtiltag.tag2.valid = tagPack.tag2.valid;

                jetsonPack.pack.variables.aprtiltag.tag2.angle.roll = tagPack.tag2.angle.roll;
                jetsonPack.pack.variables.aprtiltag.tag2.angle.pitch = tagPack.tag2.angle.pitch;
                jetsonPack.pack.variables.aprtiltag.tag2.angle.yaw = tagPack.tag2.angle.yaw;

                jetsonPack.pack.variables.aprtiltag.tag3.pos.x = tagPack.tag3.pos.x;
                jetsonPack.pack.variables.aprtiltag.tag3.pos.y = tagPack.tag3.pos.y;
                jetsonPack.pack.variables.aprtiltag.tag3.pos.z = tagPack.tag3.pos.z;
                jetsonPack.pack.variables.aprtiltag.tag3.valid = tagPack.tag3.valid;

                jetsonPack.pack.variables.aprtiltag.tag3.angle.roll = tagPack.tag3.angle.roll;
                jetsonPack.pack.variables.aprtiltag.tag3.angle.pitch = tagPack.tag3.angle.pitch;
                jetsonPack.pack.variables.aprtiltag.tag3.angle.yaw = tagPack.tag3.angle.yaw;
                
                jetsonPack.pack.variables.aprtiltag.tag4.pos.x = tagPack.tag4.pos.x;
                jetsonPack.pack.variables.aprtiltag.tag4.pos.y = tagPack.tag4.pos.y;
                jetsonPack.pack.variables.aprtiltag.tag4.pos.z = tagPack.tag4.pos.z;
                jetsonPack.pack.variables.aprtiltag.tag4.valid = tagPack.tag4.valid;

                jetsonPack.pack.variables.aprtiltag.tag4.angle.roll = tagPack.tag4.angle.roll;
                jetsonPack.pack.variables.aprtiltag.tag4.angle.pitch = tagPack.tag4.angle.pitch;
                jetsonPack.pack.variables.aprtiltag.tag4.angle.yaw = tagPack.tag4.angle.yaw;
                     
                jetsonPack.pack.variables.opticflow.timestamp = optPack.timestamp;
                jetsonPack.pack.variables.opticflow.motionPos.x = optPack.motionPos.x;
                jetsonPack.pack.variables.opticflow.motionPos.y = optPack.motionPos.y;
                jetsonPack.pack.variables.opticflow.quality = optPack.quality;

                std::cout << "ts : " << optPack.timestamp << " , " << optPack.motionPos.x << " , " << optPack.motionPos.y << " , " << optPack.quality << std::endl;
                 std::cout << "ap ts : " << tagPack.timeStamp << std::endl;    
                jetsonPack.pack.variables.hrvs.altitude = hrvsPack.altitude;
                jetsonPack.pack.variables.hrvs.body_x = hrvsPack.body_x;
                jetsonPack.pack.variables.hrvs.body_y = hrvsPack.body_y;
                jetsonPack.pack.variables.hrvs.body_z = hrvsPack.body_z;
                jetsonPack.pack.variables.hrvs.timeStamp = hrvsPack.timeStamp;

                uint32_t ptempChk = 0;
                for (int i = 4; i < jetsonPack.pack.header.payloadSize + 4; i++)
                {
                    ptempChk += jetsonPack.data[i];
                }
                jetsonPack.pack.chksum = ptempChk % 256;
                jetsonPack.pack.chksum = ~(jetsonPack.pack.chksum) + 1;
                
                //send(clientSd, jetsonPack.data, sizeof(jetsonPack.data), 0); //tcp
                int n = write(fd, jetsonPack.data, sizeof(jetsonPack.data));
                this_thread::sleep_for(std::chrono::milliseconds(33));
    }

});

/*
while(!connected) {
    std::cout << "connectionWaiting" << std::endl; 
    this_thread::sleep_for(std::chrono::milliseconds(1000));     
}
*/
std::thread th_cap([&]{
  while(run){
    rs2::frameset fs = pipe.wait_for_frames();

    // IR (fast)
    if (auto ir = fs.get_infrared_frame(1)) {
      cv::Mat gray(ir.get_height(), ir.get_width(), CV_8UC1,
                   (void*)ir.get_data(), ir.get_stride_in_bytes());
      ImgMsg msg;
      msg.img = gray.clone();
      msg.t_ms = (uint64_t)ir.get_timestamp();
      std::cout  //"ts=" << ir.get_timestamp()
          //<< " domain=" << ir.get_frame_timestamp_domain()
          << " frame=" << ir.get_frame_number()
          << std::endl;
      msg.frame_no = ir.get_frame_number();
      q_flow.push(std::move(msg));
    }

    // Color (slow)
    if (auto col = fs.get_color_frame()) {
      cv::Mat rgb(col.get_height(), col.get_width(), CV_8UC3,
                  (void*)col.get_data(), col.get_stride_in_bytes());
      cv::Mat gray;
      cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

      ImgMsg msg;
      msg.img = gray; // clone istersen
      msg.t_ms = (uint64_t)col.get_timestamp();
      msg.frame_no = col.get_frame_number();
      q_tag.push(std::move(msg));
    }
  }
  q_flow.stop();
  q_tag.stop();
});


std::thread th_tag([&]{
    try {
     
        //  cv::VideoWriter writer;
        //  std::time_t t = std::time(nullptr);
        // struct tm lt;
        // localtime_r(&t, &lt);
        // char buf[64];
        // std::strftime(buf, sizeof(buf), "%H-%M-%S_%Y-%m-%d.avi", &lt);
        // std::string out_path = "/home/lapis/Desktop/Lapis/"  + std::string(buf) + "AP" ;
        // if(!writer.open(out_path, cv::VideoWriter::fourcc('X','V','I','D'), 15, cv::Size(1920,1080), false)){
        //     std::cerr << "kayıt acılamadı!" << std::endl;
        // }
        ImgMsg m;
        constexpr int OUT_HZ = 15;
        constexpr int PERIOD_MS = 1000 / OUT_HZ; 

        auto t0 = std::chrono::steady_clock::now();
        auto nextTick = t0;

        uint64_t ts_ms = 0; 
        while (q_tag.pop(m)) {
            nextTick += std::chrono::milliseconds(PERIOD_MS);
          
            auto res = est.process(m.img);
            auto v = res.average;        
            memset(&tagPack, 0x00, sizeof(apriltagPack_t));
            //std::cout << "AVERAGE: [" << std::fixed << std::setprecision(3)
            //               << v[0] << ", " << v[1] << ", " << v[2] << "] m\n";
            std::lock_guard<std::mutex> lk(aptPackMtx);
            ts_ms += PERIOD_MS;
            tagPack.timeStamp = ts_ms;
            for(int i = 0; i < res.tags.size(); i++){
            
                switch(res.tags.at(i).id){
                case 0:
                   // std::cout << "0 : " << res.tags.at(i).validity << std::endl;
                    tagPack.tag1.pos.x = res.tags.at(i).p_final[0];
                    tagPack.tag1.pos.y = res.tags.at(i).p_final[1];
                    tagPack.tag1.pos.z = res.tags.at(i).p_final[2];
                    tagPack.tag1.valid = res.tags.at(i).validity;
                    tagPack.tag1.angle.roll  = res.tags.at(i).p_rot[0];
                    tagPack.tag1.angle.pitch = res.tags.at(i).p_rot[1];
                    tagPack.tag1.angle.yaw   = res.tags.at(i).p_rot[2]; 
                    break;
                case 20:
                 //   std::cout << "20 : " << res.tags.at(i).validity << std::endl;
                    tagPack.tag2.pos.x = res.tags.at(i).p_final[0];
                    tagPack.tag2.pos.y = res.tags.at(i).p_final[1];
                    tagPack.tag2.pos.z = res.tags.at(i).p_final[2];
                    tagPack.tag2.angle.roll = res.tags.at(i).p_rot[0];
                    tagPack.tag2.angle.pitch = res.tags.at(i).p_rot[1];
                    tagPack.tag2.angle.yaw = res.tags.at(i).p_rot[2]; 

                    tagPack.tag2.valid = res.tags.at(i).validity;
                    break;
                case 24:
                 //   std::cout << "24 : " << res.tags.at(i).validity << std::endl;
                    tagPack.tag3.pos.x = res.tags.at(i).p_final[0];
                    tagPack.tag3.pos.y = res.tags.at(i).p_final[1];
                    tagPack.tag3.pos.z = res.tags.at(i).p_final[2];

                    tagPack.tag3.angle.roll = res.tags.at(i).p_rot[0];
                    tagPack.tag3.angle.pitch = res.tags.at(i).p_rot[1];
                    tagPack.tag3.angle.yaw = res.tags.at(i).p_rot[2]; 
                    tagPack.tag3.valid = res.tags.at(i).validity;
                    break;
                case 36:
                  //  std::cout << "36 : " << res.tags.at(i).validity << std::endl;
                    tagPack.tag4.pos.x = res.tags.at(i).p_final[0];
                    tagPack.tag4.pos.y = res.tags.at(i).p_final[1];
                    tagPack.tag4.pos.z = res.tags.at(i).p_final[2];
                    tagPack.tag4.angle.roll = res.tags.at(i).p_rot[0];
                    tagPack.tag4.angle.pitch = res.tags.at(i).p_rot[1];
                    tagPack.tag4.angle.yaw = res.tags.at(i).p_rot[2]; 
                    tagPack.tag4.valid = res.tags.at(i).validity;
                    break;
                }
            	
            }
            tagPack.avgPos.pos.x = v[0];
            tagPack.avgPos.pos.y = v[1];
            tagPack.avgPos.pos.z = v[2];
            
            if(res.tags.size() > 0){

                tagPack.avgPos.valid = true;
            }
            else{

                tagPack.avgPos.valid = false;
            }
            //tagPack.timeStamp = atimestamp_ms;
            // drawAPHUD(m, tagPack.tag1.pos.x,tagPack.tag1.pos.y,
            // tagPack.tag1.pos.z,tagPack.tag2.pos.x,
            // tagPack.tag2.pos.y, tagPack.tag2.pos.z,
            // tagPack.tag3.pos.x, tagPack.tag3.pos.y,
            // tagPack.tag3.pos.z, tagPack.tag4.pos.x,
            // tagPack.tag4.pos.y, tagPack.tag4.pos.z, 
            // tagPack.timeStamp);
            // writer.write(m);
        }
    } catch (const std::exception &e) {
        LOGERR("th_tag exception: " << e.what());
        run = false; q_flow.stop();
    } catch (...) {
        LOGERR("th_tag unknown exception");
        run = false; q_flow.stop();
    }
});



std::thread th_flow([&]{
    try {
        ImgMsg m;
        DenseOptFlowOFA::FlowStats st;

        constexpr int OUT_HZ = 10;
        constexpr int PERIOD_MS = 1000 / OUT_HZ; 

        auto t0 = std::chrono::steady_clock::now();
        auto nextTick = t0;

        uint64_t ts_ms = 0; 

        while (run) {
            nextTick += std::chrono::milliseconds(PERIOD_MS);

         
            if (!q_flow.pop_latest(m)) break;

         
            bool ok = ofa.process(m.img, &st);

        
            ts_ms += PERIOD_MS;

        
            {
                std::lock_guard<std::mutex> lk(optPackMtx);
                if (ok) {
                    optPack.motionPos.x = st.mean_dx;
                    optPack.motionPos.y = st.mean_dy;
                    optPack.quality = 100;
                } else {
                    // istersen quality düşür
                    optPack.quality = 0;
                }
                optPack.timestamp = ts_ms;
            }
        
            // periyodu koru
            std::this_thread::sleep_until(nextTick);
        }
    } catch (const std::exception &e) {
        LOGERR("th_flow exception: " << e.what());
        run = false; q_tag.stop(); q_flow.stop();
    } catch (...) {
        LOGERR("th_flow unknown exception");
        run = false; q_tag.stop(); q_flow.stop();
    }
});
	



    constexpr int HRVS_OUT_HZ = 10;
    constexpr int HRVS_PERIOD_MS = 1000 / HRVS_OUT_HZ; 
        
    auto t0 = std::chrono::steady_clock::now();
    auto nextTick = t0;

    uint64_t hrvs_ts_ms = 0; 
        
    while (1)
    {
        nextTick += std::chrono::milliseconds(HRVS_PERIOD_MS);

        //std::cout << "#######################################################################" << std::endl;
        //this_thread::sleep_for(std::chrono::milliseconds(15));   
        hrvs_ts_ms += HRVS_PERIOD_MS;
 
        rsReturnCode = radarSystem.runMeasurement();
        if(rsReturnCode == RadarSystem::ReturnCode::OK)
        {   
            // Zaman ölçümleri
            //auto now = std::chrono::steady_clock::now();
            //auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count();
            //auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
            //prevTime = now;

            // std::cout 
            //     << "example: performed measurement n." << performedMeasurements + 1 
            //     << " - passed"
            //     << " | timestamp(ms): " << timestamp_ms
            //     << " | dt(ms): " << dt_ms
            //     << std::endl;

            // Radar verilerini al
            radarSystem.getBodyVelocity(systemFrameNumber, bodyVelocity);
            // std::cout << "Body velocity for frame n." << systemFrameNumber << " - x: " 
            //         << bodyVelocity[0] << " m/s, y: " << bodyVelocity[1] 
            //         << " m/s, z: " << bodyVelocity[2] << " m/s" << std::endl;

            radarSystem.getAvgRange(systemFrameNumber, avgRange);
            // std::cout << "Average range for frame n." << systemFrameNumber << " : " 
            //         << avgRange << " m" << std::endl;

            for(uint8_t i = 1; i <= processingConfig.numberOfRadars; i++)
            {
                //std::cout << "### RADAR " << unsigned(i) << " ###" << std::endl;

                radarSystem.getRadarMinrange(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarMinRange);
                //std::cout << "Min range for frame n." << systemFrameNumber << " : " << radarMinRange << " m" << std::endl;

                radarSystem.getRadarVelocity(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarVelocity);
                //std::cout << "Velocity for frame n." << systemFrameNumber << " : " << radarVelocity << " m/s" << std::endl;

                radarSystem.getRadarSnr(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarSnr);
                //std::cout << "SNR for frame n." << systemFrameNumber << " : " << radarSnr << " dB" << std::endl;
            }
            
            hrvsPack.altitude = int32_t(avgRange * 1000); 
            hrvsPack.frameNumber = systemFrameNumber;
            hrvsPack.body_x = int16_t(bodyVelocity[0] * 1000);
            hrvsPack.body_y = int16_t(bodyVelocity[1] * 1000);
            hrvsPack.body_z = int16_t(bodyVelocity[2] * 1000);
            hrvsPack.timeStamp = hrvs_ts_ms;

            
        }
        else
        {
            parseStatusVariable(radarSystem, processingConfig);
            //std::cerr << "example: performed measurement n." << performedMeasurements + 1 << " - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        }

        performedMeasurements++;

        std::this_thread::sleep_until(nextTick);

    }  
    
    

    //std::cout << "#######################################################################" << std::endl;
    
    /************   5. CLOSE RADAR SYSTEM   ************/

    // release resources of radar system:
    //radarSystem.closeRadarSystem();

    //return EXIT_SUCCESS;
}
