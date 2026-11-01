/**
 * lps_serial_sender — serial_sender_node
 *
 * AprilTag detections (ID 1-4) + VPI OFA dense optical flow → serial
 *
 * Paket formatı (packed, little-endian):
 *   header_t           (4 B)
 *   jetsonVariables_t  (72 B): hrvs(16) + apriltag(30) + opticflow(13) + ir(13)
 *   chksum             (1 B)
 *   Toplam: 77 B
 *
 * NOT: payloadSize ve checksum sinirlari sizeof()'tan turetilir; bu yorum
 * sadece okuyucu icindir.
 *
 * IR ALANI — BIRIM UYARISI: irpos METRE DEGIL, boyutsuz normalize yondur
 * (X/Z, Y/Z). Otopilot kendi menzili ile carpar. Ayrinti processIr'da.
 */

#include <rclcpp/rclcpp.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// LPS ana koddaki optik akis motoru — dokular/consistency/adaptif esik/
// on-suzme/kapsama-cezali quality/EMA hepsi burada. Bu dosyayla BIRE BIR
// ayni tutulur; VPI hattinin tamami bu sinifa devredildi.
#include "lps_serial_sender/optical_flow_vpi.hpp"
#include "lps_serial_sender/debug_recorder.hpp"   // ucus video kaydi (opsiyonel)
#include "lps_serial_sender/ir_triangle.hpp"      // IR blob tespiti + ucgen eslesmesi

#include <fstream>
#include <ctime>
#include <memory>

#include <vpi/OpenCVInterop.hpp>
#include <vpi/Image.h>
#include <vpi/ImageFormat.h>
#include <vpi/Pyramid.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/OpticalFlowDense.h>
#include <vpi/algo/Rescale.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <map>          // funnel tag_meta
#include <vector>       // funnel estimates
#include <algorithm>    // std::nth_element (median outlier)
#include <limits>       // findIrRightTriangle

// --- HRVS SDK ---
#include <sys/mman.h>
#include <cerrno>
#include <cstdint>
#include "radar_system.h"
#include "pal/gpio_pin_linux.h"
#include "pal/uart_linux.h"
#include "pal/time_stdlib.h"
#include "pal/thread_stdlib.h"

using namespace hrvs_sdk;

// --- HRVS constants ---
static constexpr char     HRVS_UART_PORT[]    = "/dev/ttyCH340";
static constexpr int      HRVS_UART_BAUDRATE  = 921600;
static constexpr unsigned HRVS_RADARS_NUMBER  = 4;
static constexpr unsigned HRVS_GPIO_PIN_ID    = 398;
static const char         HRVS_GPIO_PIO_ID[]  = "PH.07";

// JP6 Jetson Orin: PH.07 pad register fix (needs root + /dev/mem)
static bool set_pad_output_ph07()
{
    constexpr off_t PHYS = 0x02434088;
    const long pagesize = sysconf(_SC_PAGESIZE);
    if (pagesize <= 0) return false;
    const off_t page_base = PHYS & ~(off_t(pagesize - 1));
    const off_t page_off  = PHYS - page_base;
    int fd = ::open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return false;
    void *map = ::mmap(nullptr, pagesize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
    if (map == MAP_FAILED) { ::close(fd); return false; }
    volatile uint32_t *reg = reinterpret_cast<volatile uint32_t *>(
        static_cast<char *>(map) + page_off);
    *reg = 0x00000140;
    __sync_synchronize();
    ::munmap(map, pagesize);
    ::close(fd);
    return true;
}

#define SYNC_BYTE  0xBB
#define MSG_ID     0x42

#define CHECK_VPI(STMT)                                               \
    do {                                                              \
        VPIStatus _s = (STMT);                                        \
        if (_s != VPI_SUCCESS) {                                      \
            char buf[256];                                            \
            vpiGetLastStatusMessage(buf, sizeof(buf));                \
            RCLCPP_FATAL(rclcpp::get_logger("serial_sender"),        \
                "%s:%d VPI: %s", __FILE__, __LINE__, buf);            \
            throw std::runtime_error(std::string("VPI: ") + buf);    \
        }                                                             \
    } while (0)

/* ------------------------------------------------------------------ */
/* Packet layout — autopilot firmware ile birebir uyumlu              */
/* ------------------------------------------------------------------ */
#pragma pack(1)

struct opticFlow_2D_t {
    float x;
    float y;
};

struct float_3D_t {
    float x;
    float y;
    float z;
};

struct float_2D_t {
    float x;
    float y;
};


struct float_angle_t {
    float roll;
    float pitch;
    float yaw;
};

/* Tag validity bitfield (LPS packet_defs ile birebir): avgPos + 8 tag biti.
 *   valid1-4 = duvar 0/20/24/34, valid5-8 = kucuk 10/12/14/16. */
struct tag_validity_t {
    uint16_t valid    : 1;   // avgPos gecerli (n_inlier>0)
    uint16_t valid1   : 1;   // 0(N)
    uint16_t valid2   : 1;   // 20(E)
    uint16_t valid3   : 1;   // 24(S)
    uint16_t valid4   : 1;   // 34(W)
    uint16_t valid5   : 1;   // 10
    uint16_t valid6   : 1;   // 12
    uint16_t valid7   : 1;   // 14
    uint16_t valid8   : 1;   // 16
    uint16_t reserved : 7;
};

struct AP_position_t {
    tag_validity_t validity;
    float_3D_t     pos;
    float_angle_t  angle;
};

struct header_t {
    uint8_t sync_byte;
    uint8_t msg_id;
    uint8_t payloadSize;
    uint8_t headerchk;
};

struct hrvsPack_t {
    uint32_t timeStamp;
    uint16_t altitude;
    uint32_t frameNumber;
    int16_t  body_x;
    int16_t  body_y;
    int16_t  body_z;
};

struct apriltagPack_t {
    uint32_t      timeStamp;
    AP_position_t avgPos;   // tek tek tag pozu gitmiyor; validity bit'leri avgPos'ta
};

struct opticFlowPack_t {
    uint32_t       timestamp;
    uint8_t        quality;
    opticFlow_2D_t motionPos;
};

struct irPack_t {          
    uint32_t    timeStamp;
    bool        valid;
    float_2D_t  irpos;
};

struct jetsonVariables_t {
    hrvsPack_t      hrvs;
    apriltagPack_t  aprtiltag;
    opticFlowPack_t opticflow;
    irPack_t        ir;
};

struct jetsonOtopilotPack_t {
    header_t          header;
    jetsonVariables_t variables;
    uint8_t           chksum;
};

union jetsonOtopilotPack_ut {
    jetsonOtopilotPack_t pack;
    uint8_t              data[sizeof(jetsonOtopilotPack_t)];
};

#pragma pack()

/* ------------------------------------------------------------------ */
/* Serial helpers                                                      */
/* ------------------------------------------------------------------ */
static speed_t toBaudConstant(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

static int openSerial(const std::string &port, int baud)
{
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) throw std::runtime_error("Serial acilamadi: " + port +
                                         " (" + strerror(errno) + ")");
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        throw std::runtime_error("tcgetattr hatasi");
    }
    speed_t spd = toBaudConstant(baud);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);
    cfmakeraw(&tty);
    tty.c_cflag &= ~CRTSCTS;      // hardware flow control kapalı
    tty.c_cflag |=  CLOCAL;       // modem sinyali bekleme
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        throw std::runtime_error("tcsetattr hatasi");
    }
    return fd;
}

/* ------------------------------------------------------------------ */
/* RANSAC global motion estimator (VPI flow → dx,dy)                  */
/* ------------------------------------------------------------------ */
// NOT: Eski serbest estimateGlobalDxDy (basit inlier-ratio quality'li) KALDIRILDI.
// Global tahmin artik lps::OpticalFlowVPI icindeki sürüm tarafindan yapiliyor —
// dokusuz hucre gating'i, yumusak consistency skoru, sahneye uyarlanir esik ve
// kapsama-cezali quality ile (LPS ana koddaki motorun ta kendisi).

/* ------------------------------------------------------------------ */
/* Huni tag meta: tag-frame offset + gercek boyut (PnP size scaling)  */
/* ------------------------------------------------------------------ */
struct TagMeta {
    cv::Vec3f offset;          // tag-frame'de huni merkezine ofset
    float     physical_size_m; // gercek tag boyutu (PnP scale)
};

/* Quaternion -> 3x3 rotasyon (R_kamera<-tag): offset_kamera = R*offset_tagframe.
   Montaj egimi + drone attitude birlikte gelir (canli olculur). */
static cv::Matx33f quatToMatrix(double qx, double qy, double qz, double qw)
{
    double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n < 1e-9) return cv::Matx33f::eye();
    qx/=n; qy/=n; qz/=n; qw/=n;
    return cv::Matx33f(
        (float)(1-2*(qy*qy+qz*qz)), (float)(2*(qx*qy-qz*qw)),   (float)(2*(qx*qz+qy*qw)),
        (float)(2*(qx*qy+qz*qw)),   (float)(1-2*(qx*qx+qz*qz)), (float)(2*(qy*qz-qx*qw)),
        (float)(2*(qx*qz-qy*qw)),   (float)(2*(qy*qz+qx*qw)),   (float)(1-2*(qx*qx+qy*qy)));
}

/* ------------------------------------------------------------------ */
/* Quaternion → Roll/Pitch/Yaw (ZYX, radyan)                         */
/* ------------------------------------------------------------------ */
[[maybe_unused]] static void quatToRPY(double qx, double qy, double qz, double qw,
                      float &roll, float &pitch, float &yaw)
{
    double sinr = 2.0 * (qw * qx + qy * qz);
    double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = (float)std::atan2(sinr, cosr);

    double sinp = 2.0 * (qw * qy - qz * qx);
    pitch = (float)(std::abs(sinp) >= 1.0
                    ? std::copysign(M_PI / 2.0, sinp)
                    : std::asin(sinp));

    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = (float)std::atan2(siny, cosy);
}

/* ------------------------------------------------------------------ */
/* IR LED ucgeni — tespit + eslesme: ir_triangle.hpp (test ile ORTAK)  */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Node                                                                */
/* ------------------------------------------------------------------ */
class SerialSenderNode : public rclcpp::Node
{
public:
    SerialSenderNode() : Node("serial_sender")
    {
        start_time_   = std::chrono::steady_clock::now();
        fps_last_tp_  = start_time_;
        last_send_tp_ = start_time_;

        serial_port_  = declare_parameter<std::string>("serial_port",  "/dev/ttyTHS1");
        baud_rate_    = declare_parameter<int>   ("baud_rate",    115200);
        send_rate_hz_ = declare_parameter<double>("send_rate_hz", 50.0);
        tag_topic_    = declare_parameter<std::string>("tag_topic",   "/tag_detections");
        image_topic_  = declare_parameter<std::string>("image_topic", "/camera/camera/infra1/image_rect_raw");
        // AprilTag node'unda set'li size (funnel per-ID scale referansi).
        // Duvar buyuk tag siyah kare = 250mm (kucukler tag_meta'da 0.077 ile duzeltilir).
        apriltag_node_size_ = (float)declare_parameter<double>("apriltag_node_size", 0.25);
        grid_size_    = declare_parameter<int>   ("grid_size",    4);
        num_levels_   = declare_parameter<int>   ("num_levels",   3);
        ema_alpha_    = static_cast<float>(declare_parameter<double>("ema_alpha", 0.25));
        debug_        = declare_parameter<bool>  ("debug",        true);

        // --- IR LED ucgeni (dik acili, 3 nokta) ---
        // Konum SADECE dik acı kosesinden cikar (pinhole geri izdusum, Z=radar);
        // diger iki nokta yalnizca 'bu gercekten bizim ucgenimiz mi' dogrulamasi.
        // Ikizkenar dik ucgen: iki dik kenar da 1.36 m (hipotenus 1.92 m).
        // Kenar/hipotenus orani 1.414 -> 'en uzun kenarin karsisi = dik acı'
        // testi saglam marjla calisir. Iki kenar esit oldugu icin a/b eslesme
        // belirsizligi tamamen anlamsiz (konumu zaten etkilemiyordu).
        ir_enabled_    = declare_parameter<bool>  ("ir_enabled",     true);
        ir_leg_a_m_    = (float)declare_parameter<double>("ir_leg_a_m",    1.36);
        ir_leg_b_m_    = (float)declare_parameter<double>("ir_leg_b_m",    1.36);
        // 0.12: sentetik testle olculdu (test_ir_triangle). Mesru tespitlerin
        // fit artigi 30 derece egimde bile 0.074; ESKENAR ucgen ise 0.157 alir.
        // Eski 0.25 degeri eskenar ucgeni KABUL ediyordu — olcek-bagimsiz
        // dogrulama mutlak-olcek kontrolunden zayif oldugu icin daha sikı olmali.
        ir_tol_frac_   = (float)declare_parameter<double>("ir_tol_frac",   0.12);
        // Optik Z ile radar Z arasinda izin verilen goreli fark. Gevsek tutuldu:
        // amac hassas kalibrasyon degil, kabaca yanlis olcekteki sahte eslesmeyi
        // elemek (radar 10 Hz + bayat oldugu icin sikilastirmak dogru eslesmeleri
        // atmaya baslar). Radar yoksa kontrol tamamen atlanir.
        ir_z_tol_      = (float)declare_parameter<double>("ir_z_tol",      0.50);
        // min_area MENZILI BELIRLER: yukseklik arttikca LED'in piksel izi
        // 1/Z^2 ile kuculur ve tespiti sinirlayan ilk sey genelde esik degil,
        // bu alan tabani olur. 1 px'e indirildi — tek piksel gurultusunu ucgen
        // geometrisi zaten eliyor (3 nokta + dogru olcek sarti cok secici).
        ir_min_area_   = declare_parameter<double>("ir_min_area",    1.0);
        ir_max_area_   = declare_parameter<double>("ir_max_area",  400.0);
        // Yuksek esik: infra1'i AprilTag + optik akis da tuketiyor, pozlama
        // LED'ler icin kisilamaz (sahne kararir, ikisi de coker). Kameraya bakan
        // aktif LED normal pozlamada da doygun (255), sahne degil -> ayrim burada.
        ir_min_thresh_ = declare_parameter<double>("ir_min_threshold", 240.0);
        camera_info_topic_ = declare_parameter<std::string>(
            "camera_info_topic", "/camera/camera/infra1/camera_info");

        if (ir_enabled_) {
            ir_pts_.reserve(kIrMaxBlobs);
            cinfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, rclcpp::SensorDataQoS(),
                std::bind(&SerialSenderNode::cinfoCb, this, std::placeholders::_1));
        }

        // --- Optik akis motoru (LPS ana koddan bire bir) ---
        // texture_thresh tabani 4.0, prefilter sigma 1.5 = uretim varsayilanlari.
        // Sahada geri almak icin: flow_prefilter_sigma:=0.0 / flow_texture_thresh:=8.0
        float flow_tex = static_cast<float>(
            declare_parameter<double>("flow_texture_thresh", 4.0));
        float flow_pre = static_cast<float>(
            declare_parameter<double>("flow_prefilter_sigma", 1.5));
        flow_ = std::make_unique<lps::OpticalFlowVPI>(
            grid_size_, num_levels_, ema_alpha_, flow_tex, flow_pre);

        // Ham akis CSV logu (EMA ONCESI kare-basi olcum) — analiz icin, opsiyonel.
        // Varsayilan KAPALI: mevcut davranisi degistirmez, launch'tan acilir.
        if (declare_parameter<bool>("flow_csv", false)) {
            std::string dir = declare_parameter<std::string>(
                "record_dir", "/home/lapis/lps_records");
            char stamp[32];
            std::time_t now = std::time(nullptr);
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S",
                          std::localtime(&now));
            std::string path = dir + "/flow_" + stamp + ".csv";
            flow_csv_.open(path);
            if (flow_csv_) {
                flow_csv_ << "t_ms,raw_dx,raw_dy,raw_q,ema_dx,ema_dy,ema_q,"
                             "consist,tex,g50,g90\n";
                flow_csv_.flush();
                RCLCPP_INFO(get_logger(), "Ham akis CSV: %s", path.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Akis CSV acilamadi: %s", path.c_str());
            }
        }

        // IR CSV logu — ucus sonrasi analiz icin, opsiyonel (varsayilan KAPALI).
        // HER kare yazilir (tespit olmasa da): esik ayari icin asil degerli veri
        // tespitin OLMADIGI karelerdeki blob sayisi. Sutunlar saha kontrol
        // listesiyle birebir: blob/amin=menzil, zopt-zrad=olcek, fit=egim.
        if (ir_enabled_ && declare_parameter<bool>("ir_csv", false)) {
            std::string dir = has_parameter("record_dir")
                ? get_parameter("record_dir").as_string()
                : declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
            char stamp[32];
            std::time_t now = std::time(nullptr);
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S",
                          std::localtime(&now));
            std::string path = dir + "/ir_" + stamp + ".csv";
            ir_csv_.open(path);
            if (ir_csv_) {
                // nx,ny = PAKETE GIDEN normalize yon (boyutsuz). x_at_zopt /
                // y_at_zopt = nx*z_opt (metrik karsilik, SADECE analiz kolayligi
                // icin — gonderilmiyor, otopilot kendi Z'siyle carpiyor).
                ir_csv_ << "t_ms,valid,nx,ny,x_at_zopt,y_at_zopt,z_opt,z_radar,"
                           "n_blobs,amin,fit,rx,ry,ax,ay,bx,by,mx,my\n";
                ir_csv_.flush();
                RCLCPP_INFO(get_logger(), "IR CSV: %s", path.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "IR CSV acilamadi: %s", path.c_str());
            }
        }

        // Ucus video kaydi (opsiyonel): tespit cerceveleri + akis HUD'lu .avi.
        // Encode ayri DUSUK oncelikli thread'de -> ucus-kritik yolu (serial) yormaz.
        // Acmak icin record_video:=true. record_YYYYmmdd... record_dir'e duser.
        if (declare_parameter<bool>("record_video", false)) {
            std::string dir = has_parameter("record_dir")
                ? get_parameter("record_dir").as_string()
                : declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
            double fps   = declare_parameter<double>("record_fps",   15.0);
            double scale = declare_parameter<double>("record_scale", 0.5);
            recorder_ = std::make_unique<lps::DebugRecorder>(dir, fps, scale);
            RCLCPP_INFO(get_logger(), "Ucus video kaydi: %s", recorder_->path().c_str());
        }

        serial_fd_ = openSerial(serial_port_, baud_rate_);

        tag_sub_ = create_subscription<
            isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            tag_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::tagCb, this, std::placeholders::_1));

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::imgCb, this, std::placeholders::_1));

        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / send_rate_hz_));
        timer_ = create_wall_timer(period_ns,
            std::bind(&SerialSenderNode::timerCb, this));

        vpi_thread_  = std::thread(&SerialSenderNode::vpiThread,  this);
        hrvs_thread_ = std::thread(&SerialSenderNode::hrvsThread, this);

        RCLCPP_INFO(get_logger(),
            "SerialSender: tag='%s' img='%s' → %s @%d baud %.1f Hz | pkt=%zuB | debug=%s",
            tag_topic_.c_str(), image_topic_.c_str(),
            serial_port_.c_str(), baud_rate_, send_rate_hz_,
            sizeof(jetsonOtopilotPack_t), debug_ ? "ON" : "OFF");
    }

    ~SerialSenderNode()
    {
        hrvs_running_ = false;
        if (hrvs_thread_.joinable()) hrvs_thread_.join();

        vpi_running_ = false;
        frame_cv_.notify_all();
        if (vpi_thread_.joinable()) vpi_thread_.join();

        // VPI objeleri OpticalFlowVPI destructor'inda yok edilir (thread durduktan sonra)
        flow_.reset();

        recorder_.reset();   // son kareyi yazar, encode thread'ini durdurur, dosyayi kapatir

        // CSV: VPI thread durduktan SONRA kapat (yazan o thread'di)
        if (flow_csv_.is_open()) flow_csv_.close();

        // IR CSV: yazan imgCb (ROS callback) thread'iydi; buraya spin dondukten
        // SONRA gelinir, yani callback'ler artik calismiyor — kapatmak guvenli.
        if (ir_csv_.is_open()) ir_csv_.close();

        if (serial_fd_ >= 0)    ::close(serial_fd_);
    }

private:
    /* -------------------------------------------------------------- */
    /* AprilTag callback — HUNI FUZYONU (LPS ana koddan port)         */
    /*   avgPos = tum huni tag'lerinin (offset'li) outlier'siz ORTALAMASI. */
    /*   POZISYON sadece avgPos'ta gider; her tag'in sadece VALIDITY biti */
    /*   basiliyor (valid1-4=duvar, valid5-8=kucuk). hrvs/flow AYNEN korunur. */
    /* -------------------------------------------------------------- */
    void tagCb(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        tag_cb_count_++;

        apriltagPack_t ap{};
        ap.timeStamp = msSinceStart();

        // Her huni tag'i: est = tag_pos*scale + R*offset (R = tag oryantasyonu,
        // montaj egimi + drone attitude'u tasir). tag_meta'da olmayan ID atlanir.
        std::vector<cv::Vec3f> estimates, raws;
        std::vector<int>       used_ids;
        for (const auto &det : msg->detections) {
            auto it = tag_meta_.find(det.id);
            if (it == tag_meta_.end()) continue;
            const auto &meta = it->second;
            float scale = meta.physical_size_m / apriltag_node_size_;
            const auto &p = det.pose.pose.pose.position;
            const auto &q = det.pose.pose.pose.orientation;
            cv::Vec3f tag_pos((float)p.x*scale, (float)p.y*scale, (float)p.z*scale);
            cv::Matx33f R = quatToMatrix(q.x, q.y, q.z, q.w);
            cv::Vec3f   est = tag_pos + R * meta.offset;
            raws.push_back(tag_pos);
            estimates.push_back(est);
            used_ids.push_back(det.id);
        }

        // Median tabanli outlier reddi + ortalama (huni merkezi fuzyonu)
        cv::Vec3f funnel(0, 0, 0);
        int n_inliers = 0;
        std::vector<PerTagLanding> per_tag;
        per_tag.reserve(estimates.size());
        if (!estimates.empty()) {
            std::vector<float> xs, ys, zs;
            for (const auto &e : estimates) { xs.push_back(e[0]); ys.push_back(e[1]); zs.push_back(e[2]); }
            auto med = [](std::vector<float> &v){
                std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
                return v[v.size()/2]; };
            cv::Vec3f median(med(xs), med(ys), med(zs));
            constexpr float OUTLIER_M = 0.4f;
            cv::Vec3f sum(0, 0, 0);
            for (size_t i = 0; i < estimates.size(); ++i) {
                bool used = (cv::norm(estimates[i]-median) <= OUTLIER_M);
                per_tag.push_back({used_ids[i], raws[i], estimates[i], used});
                if (used) { sum += estimates[i]; n_inliers++; }
            }
            if (n_inliers > 0) funnel = sum * (1.0f/n_inliers);
        }

        // POZISYON sadece avgPos'ta gider; tek tek tag pozlari GONDERILMIYOR —
        // her tag'in yalnizca VALIDITY bit'i basiliyor.
        ap.avgPos.validity.valid = (n_inliers > 0);
        ap.avgPos.pos.x = funnel[0];
        ap.avgPos.pos.y = funnel[1];
        ap.avgPos.pos.z = funnel[2];
        // avgPos.angle: huni ortalamasinin tek dogal yonu yok -> 0 birakilir.

        // Her INLIER tag'in validity bit'i (pozisyon YOK). Sabit ID -> bit:
        //   valid1=0(N) valid2=20(E) valid3=24(S) valid4=34(W)  [duvar]
        //   valid5=10   valid6=12    valid7=14    valid8=16      [kucuk 2x2]
        // Outlier (median'dan >0.4m = muhtemelen false positive) bit basmaz.
        for (const auto &pt : per_tag) {
            if (!pt.used) continue;
            switch (pt.id) {
                case  0: ap.avgPos.validity.valid1 = 1; break;
                case 20: ap.avgPos.validity.valid2 = 1; break;
                case 24: ap.avgPos.validity.valid3 = 1; break;
                case 34: ap.avgPos.validity.valid4 = 1; break;
                case 10: ap.avgPos.validity.valid5 = 1; break;
                case 12: ap.avgPos.validity.valid6 = 1; break;
                case 14: ap.avgPos.validity.valid7 = 1; break;
                case 16: ap.avgPos.validity.valid8 = 1; break;
                default: break;   // tag_meta disi ID — bit yok
            }
        }

        // Video overlay: huni tag'lerinin (duvar+kucuk) kose koordinatlari.
        std::vector<lps::DebugRecorder::TagOverlay> overlays;
        if (recorder_) {
            for (const auto &det : msg->detections) {
                if (tag_meta_.find(det.id) == tag_meta_.end()) continue;
                lps::DebugRecorder::TagOverlay t;
                t.id = det.id;
                for (int i = 0; i < 4; ++i)
                    t.corners[i] = cv::Point2f((float)det.corners[i].x,
                                               (float)det.corners[i].y);
                overlays.push_back(t);
            }
        }

        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            apriltag_     = ap;
            tag_overlays_ = std::move(overlays);
        }
    }

    /* -------------------------------------------------------------- */
    /* CameraInfo callback — IR geri izdusumu icin intrinsics         */
    /* -------------------------------------------------------------- */
    void cinfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (msg->k[0] <= 0.0) return;
        cv::Matx33d K(msg->k[0], msg->k[1], msg->k[2],
                      msg->k[3], msg->k[4], msg->k[5],
                      msg->k[6], msg->k[7], msg->k[8]);
        cv::Mat D(1, (int)msg->d.size(), CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i) D.at<double>(0, (int)i) = msg->d[i];

        std::lock_guard<std::mutex> lk(cinfo_mutex_);
        cam_K_ = K;
        cam_D_ = D.clone();
        cinfo_ready_ = true;
    }

    /* -------------------------------------------------------------- */
    /* IR LED ucgeni → pozisyon (pinhole geri izdusum)                 */
    /*   1) blob tespiti (parlak nokta, yuksek esik)                   */
    /*   2) olcekten bagimsiz ucgen dogrulamasi + optik Z (30 Hz)      */
    /*   3) radar ile kaba capraz kontrol (varsa)                      */
    /*   4) undistort → normalize → Z ile olcekle (kamera optik frame'i,  */
    /*      AprilTag avgPos ile ayni konvansiyon)                       */
    /* -------------------------------------------------------------- */
    void processIr(const cv::Mat &frame)
    {
        cv::Matx33d K; cv::Mat D;
        {
            std::lock_guard<std::mutex> lk(cinfo_mutex_);
            if (!cinfo_ready_) return;
            K = cam_K_; D = cam_D_;
        }

        // Radar SADECE capraz kontrol icin — olcek artik ucgenin kendisinden.
        // (Radar 10 Hz, kamera 30 Hz: eskiden 3 karenin 2'si bayat Z kullaniyordu.)
        float Z_radar;
        { std::lock_guard<std::mutex> lk(hrvs_mutex_); Z_radar = hrvs_data_.altitude / 1000.0f; }

        lps::detectIrBlobs(frame, ir_min_thresh_, ir_min_area_, ir_max_area_,
                      kIrMaxBlobs, ir_bin_, ir_labels_, ir_stats_, ir_centroids_,
                      ir_pts_, ir_areas_);

        irPack_t ir{};
        ir.timeStamp = msSinceStart();
        lps::DebugRecorder::IrOverlay ov;
        ov.n_blobs = (int)ir_pts_.size();
        ov.blobs   = ir_pts_;   // <=16 nokta; eslesme olmasa da videoda gorunur
        ov.no_alt  = (Z_radar <= 0.05f);   // capraz kontrol yok (IR yine calisir)

        int i_right, i_a, i_b;
        float fit_err = 0.0f, k_px_per_m = 0.0f;
        const double f_px = 0.5 * (K(0, 0) + K(1, 1));
        if (lps::findIrRightTriangle(ir_pts_, ir_leg_a_m_, ir_leg_b_m_, ir_tol_frac_,
                                i_right, i_a, i_b, fit_err, k_px_per_m)) {
            // Z ucgenin OLCULEN buyuklugunden: Z = f / k. Kamera hiziyla (30 Hz)
            // guncellenir, bayat degildir ve OPTIK EKSEN derinligidir — pinhole
            // geri izdusumun istedigi buyuklugun ta kendisi (radar dikey verir).
            const float Z_m = (float)f_px / k_px_per_m;
            ov.z       = Z_m;
            ov.z_radar = Z_radar;


            if (Z_radar > 0.05f &&
                std::fabs(Z_m - Z_radar) / Z_radar > ir_z_tol_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "[IR] optik Z=%.2f m radar Z=%.2f m ile uyusmuyor (tol %.0f%%) "
                    "— tespit reddedildi", Z_m, Z_radar, ir_z_tol_ * 100.0f);
                std::lock_guard<std::mutex> lk(data_mutex_);
                ir_data_ = ir; ir_overlay_ = ov;   // ir.valid = false
                return;
            }

            const cv::Point2f mid = (ir_pts_[i_a] + ir_pts_[i_b]) * 0.5f;

            std::vector<cv::Point2f> in{mid}, out;
            cv::undistortPoints(in, out, K, D);

            // ---- PAKETE GIDEN: NORMALIZE YON (BOYUTSUZ), METRE DEGIL ----
            // irpos = (X/Z, Y/Z) — huni merkezinin kamera optik frame'indeki
            // YONU. Menzille CARPILMAZ; otopilot kendi (EKF-fuzyonlu) menziliyle
            // carpar: X = irpos.x * Z.
            //
            // Neden burada carpmiyoruz: kamera fiziksel olarak yon olcer, menzil
            // turetilmis buyukluktur. Bizim optik Z'mizin sessiz olcek hatalari
            // var (marker olcusu 1.36 m dogru mu, fx kalibrasyonu, egimde %5-10)
            // ve otopilot bunlari DOGRULAYAMAZ — metrik gonderseydik kaynagini
            // denetleyemedigi bir sayiya guvenmek zorunda kalirdi. Kendi
            // yuksekligi ise baro/GPS/IMU ile capraz kontrol edilebilir alaninda.
            //
            // DIKKAT (otopilot tarafi): duz yukseklik ile carpmak DIK BAKISTA
            // dogrudur; egimde menzil goruntudeki konuma gore degisir
            //   Z = h / (cos(theta) + y_n*sin(theta))
            // Ham 'h' ile carpilirsa 15 derece egimde ~19 cm hata olur.
            //
            // Optik Z atilmadi: yanlis ucgen eslesmelerini elemek icin yukarida
            // kullaniliyor (dogrulamaya %10 dogruluk yetiyor, 40 derece egimde
            // bile saglar) ve CSV/HUD'da z_opt olarak loglaniyor.
            ir.valid    = true;
            ir.irpos.x  = out[0].x;
            ir.irpos.y  = out[0].y;

            // Menzil teshisi: eslesen 3 blob'un EN KUCUGUNUN alani. Yukseklik
            // arttikca LED'in piksel izi kuculur; bu deger ir_min_area'ya
            // yaklastikca tespit sinirina geliniyor demektir (bkz. ir_min_area).
            int amin = INT32_MAX;
            for (int idx : {i_right, i_a, i_b})
                if (idx < (int)ir_areas_.size()) amin = std::min(amin, ir_areas_[idx]);
            ov.min_area = (amin == INT32_MAX) ? 0 : amin;

            ov.valid   = true;
            ov.pts[0]  = ir_pts_[i_right];   // dik acı kosesi (ucgen kimligi)
            ov.pts[1]  = ir_pts_[i_a];
            ov.pts[2]  = ir_pts_[i_b];
            ov.mid     = mid;                // huni merkezi — yon BURADAN cikti
            // ov.x/y = pakete giden NORMALIZE yon; ov.z/z_radar pakete gitmez,
            // HUD/CSV'de metrik karsiligi gostermek ve optik-radar menzilini
            // kiyaslamak icin tutulur.
            ov.x = ir.irpos.x; ov.y = ir.irpos.y;
        }
        ov.fit_err = fit_err;
        writeIrCsv(ir, ov);

        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            ir_data_    = ir;
            ir_overlay_ = ov;
        }
    }

    /** IR CSV satiri — acikken HER karede (tespit olsun olmasin) yazilir.
     *  Sadece imgCb thread'inden cagrilir (processIr icinden), kilit gerekmez.
     *  Yarim saniyede bir flush: ani kesintide en fazla ~15 satir kayip
     *  (flow_csv ile ayni strateji). */
    void writeIrCsv(const irPack_t &ir, const lps::DebugRecorder::IrOverlay &ov)
    {
        if (!ir_csv_.is_open()) return;
        ir_csv_ << ir.timeStamp << ',' << (ir.valid ? 1 : 0) << ','
                << ir.irpos.x << ',' << ir.irpos.y << ','
                << (ir.irpos.x * ov.z) << ',' << (ir.irpos.y * ov.z) << ','
                << ov.z << ',' << ov.z_radar << ','
                << ov.n_blobs << ',' << ov.min_area << ',' << ov.fit_err;
        if (ov.valid)
            ir_csv_ << ',' << ov.pts[0].x << ',' << ov.pts[0].y
                    << ',' << ov.pts[1].x << ',' << ov.pts[1].y
                    << ',' << ov.pts[2].x << ',' << ov.pts[2].y
                    << ',' << ov.mid.x    << ',' << ov.mid.y << '\n';
        else
            ir_csv_ << ",,,,,,,\n";   // eslesme yok -> kose sutunlari bos
        if (++ir_csv_lines_ % 15 == 0) ir_csv_.flush();
    }

    /* -------------------------------------------------------------- */
    /* Image callback — sadece frame'i kuyruğa koyar, anında döner   */
    /* -------------------------------------------------------------- */
    void imgCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        flow_cb_count_++;
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "cv_bridge: %s", e.what());
            return;
        }
        if (!cv_ptr || cv_ptr->image.empty()) return;

        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            latest_frame_ = cv_ptr->image.clone();   // VPI thread icin clone (once oldugu gibi)
            new_frame_    = true;
        }
        frame_cv_.notify_one();

        // IR LED ucgeni — blob tespiti ucuz (~1-2 ms @720p), ayri thread gerekmez.
        if (ir_enabled_) processIr(cv_ptr->image);

        // Ucus video kaydi: durum snapshot'i + frame (kopya YOK; cv_ptr keepalive,
        // encode ayri DUSUK oncelikli thread'de). record_fps'e gore seyreltir.
        if (recorder_) {
            lps::DebugRecorder::State st;
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                st.tags      = tag_overlays_;
                st.tag_valid = apriltag_.avgPos.validity.valid;
                st.px = apriltag_.avgPos.pos.x;
                st.py = apriltag_.avgPos.pos.y;
                st.pz = apriltag_.avgPos.pos.z;
                st.roll  = apriltag_.avgPos.angle.roll;
                st.pitch = apriltag_.avgPos.angle.pitch;
                st.yaw   = apriltag_.avgPos.angle.yaw;
                st.dx      = opticflow_.motionPos.x;
                st.dy      = opticflow_.motionPos.y;
                st.quality = opticflow_.quality;
                st.ir      = ir_overlay_;
            }
            // consist/tex/grad Lap60'ta persistent atomik degil -> 0 birakilir
            // (HUD'da o alanlar bos; tag cerceveleri + avgPos + flow ok gorunur).
            st.t_ms = msSinceStart();
            recorder_->push(cv_ptr->image, cv_ptr, st);
        }
    }

    /* -------------------------------------------------------------- */
    /* VPI thread — ayrı thread'de çalışır, spin'i bloke etmez       */
    /* -------------------------------------------------------------- */
    void vpiThread()
    {
        while (vpi_running_.load()) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lk(frame_mutex_);
                frame_cv_.wait(lk, [this]{
                    return new_frame_ || !vpi_running_.load();
                });
                if (!vpi_running_.load()) break;
                frame      = std::move(latest_frame_);
                new_frame_ = false;
            }

            // Ilk frame'de VPI objelerini kur (LPS motoru), sonra her frame isle.
            if (!flow_->initialized()) {
                try {
                    flow_->init(frame);
                    RCLCPP_INFO(get_logger(), "VPI OFA init: %dx%d → padded %dx%d",
                                frame.cols, frame.rows,
                                flow_->paddedWidth(), flow_->paddedHeight());
                } catch (const std::exception &e) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "VPI OFA init hatasi: %s", e.what());
                }
                continue;
            }

            lps::OpticalFlowVPI::Result r;
            try {
                flow_->process(frame, r);
            } catch (const std::exception &e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "VPI akis hatasi: %s", e.what());
                continue;
            }

            opticFlowPack_t of{};
            of.timestamp   = msSinceStart();
            of.motionPos.x = r.dx;        // EMA'li — pakete giden
            of.motionPos.y = r.dy;
            of.quality     = r.quality;   // kapsama-cezali, EMA'li Q

            // Ham akis CSV'si — EMA ONCESI kare-basi olcum (aciksa).
            // Yarim saniyede bir flush: ani kesintide en fazla ~15 satir kayip.
            if (flow_csv_.is_open()) {
                flow_csv_ << of.timestamp << ','
                          << r.raw_dx << ',' << r.raw_dy << ','
                          << (unsigned)r.raw_quality << ','
                          << r.dx << ',' << r.dy << ',' << (unsigned)r.quality << ','
                          << r.consist << ',' << r.tex_ratio << ','
                          << (unsigned)r.tex_p50 << ',' << (unsigned)r.tex_p90 << '\n';
                if (++csv_lines_ % 15 == 0) flow_csv_.flush();
            }

            { std::lock_guard<std::mutex> lk(data_mutex_); opticflow_ = of; }
        }
    }

    /* -------------------------------------------------------------- */
    /* Timer callback — sabit frekansta seri port'a paket gönderir    */
    /* -------------------------------------------------------------- */
    void timerCb()
    {
        // Gerçek delta T ölçümü — write öncesi
        auto now_send = std::chrono::steady_clock::now();
        auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now_send - last_send_tp_).count();
        last_send_tp_ = now_send;

        serial_cb_count_++;

        // Jitter istatistiği güncelle (ilk tick'i atla)
        if (serial_cb_count_ > 1) {
            dt_sum_us_ += dt_us;
            dt_cnt_++;
            if (dt_us < dt_min_us_) dt_min_us_ = dt_us;
            if (dt_us > dt_max_us_) dt_max_us_ = dt_us;
        }

        apriltagPack_t  at_copy;
        opticFlowPack_t of_copy;
        irPack_t        ir_copy;
        lps::DebugRecorder::IrOverlay ir_ov_copy;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            at_copy = apriltag_; of_copy = opticflow_; ir_copy = ir_data_;
            ir_ov_copy = ir_overlay_;
        }

        jetsonOtopilotPack_ut pkt;
        std::memset(pkt.data, 0, sizeof(jetsonOtopilotPack_t));

        // Header
        pkt.pack.header.sync_byte   = SYNC_BYTE;
        pkt.pack.header.msg_id      = MSG_ID;
        pkt.pack.header.payloadSize = static_cast<uint8_t>(sizeof(jetsonVariables_t));

        uint32_t hsum = 0;
        for (int i = 0; i < 3; ++i) hsum += pkt.data[i];
        pkt.pack.header.headerchk = static_cast<uint8_t>(~(hsum % 256) + 1);

        // HRVS — timestamp + frame sayacı
        pkt.pack.variables.hrvs.timeStamp   = hrvs_data_.timeStamp;//msSinceStart();
        pkt.pack.variables.hrvs.frameNumber = hrvs_data_.frameNumber;
        // HRVS — radar thread'inden gelen altitude ve body velocity
        {
            std::lock_guard<std::mutex> lk(hrvs_mutex_);
            pkt.pack.variables.hrvs.altitude = hrvs_data_.altitude;
            pkt.pack.variables.hrvs.body_x   = hrvs_data_.body_x;
            pkt.pack.variables.hrvs.body_y   = hrvs_data_.body_y;
            pkt.pack.variables.hrvs.body_z   = hrvs_data_.body_z;
        }

        // AprilTag + OpticFlow + IR
        pkt.pack.variables.aprtiltag = at_copy;
        pkt.pack.variables.opticflow = of_copy;
        pkt.pack.variables.ir        = ir_copy;

        // Checksum (variables alanı üzerinden)
        uint32_t csum = 0;
        const int payStart = (int)sizeof(header_t);
        const int payEnd   = payStart + (int)sizeof(jetsonVariables_t);
        for (int i = payStart; i < payEnd; ++i) csum += pkt.data[i];
        pkt.pack.chksum = static_cast<uint8_t>(~(csum % 256) + 1);

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[TIMER-ALIVE] frame=%u tag_cb=%u flow_cb=%u",
            serial_cb_count_, tag_cb_count_, flow_cb_count_);

        // Paketi TAMAMI cikana kadar yaz (LPS ana koddaki duzeltme).
        // O_NONBLOCK write() ya hep ya hic DEGIL: tampon dolu ise 163 baytin
        // bir kismini yazip gerisini reddeder — 'yarim paketi drop et'
        // stratejisi o baytlar telde oldugu icin kesik paket birakiyordu.
        // Cozum: yer kalmadiysa poll(POLLOUT) ile bosalmayi bekle, kalani yaz.
        {
            const uint8_t *buf = pkt.data;
            const size_t   len = sizeof(jetsonOtopilotPack_t);
            size_t off = 0;
            const auto t0 = std::chrono::steady_clock::now();
            const int budget_ms = 20;   // 77B @115200 ≈ 6.7 ms; periyot > bu
            while (off < len) {
                ssize_t n = ::write(serial_fd_, buf + off, len - off);
                if (n > 0) { off += (size_t)n; continue; }
                if (n < 0 && errno == EINTR) continue;
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    int elapsed = (int)std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0).count();
                    int remain = budget_ms - elapsed;
                    if (remain <= 0) break;
                    struct pollfd p{serial_fd_, POLLOUT, 0};
                    if (::poll(&p, 1, remain) <= 0) break;
                    continue;
                }
                break;   // gercek hata
            }
            if (off != len) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Serial write eksik (%zu/%zu): %s", off, len, strerror(errno));
            }
        }

        // FPS / debug güncellemesi — her 1 saniyede bir
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_last_tp_).count();
        if (elapsed >= 1.0) {
            tag_fps_   = tag_cb_count_    / elapsed;
            flow_fps_  = flow_cb_count_   / elapsed;
            serial_hz_ = serial_cb_count_ / elapsed;
            tag_cb_count_ = flow_cb_count_ = serial_cb_count_ = 0;
            fps_last_tp_ = now;

            double dt_avg_ms = (dt_cnt_ > 0) ? (dt_sum_us_ / dt_cnt_ / 1000.0) : 0.0;
            double dt_min_ms = dt_min_us_ / 1000.0;
            double dt_max_ms = dt_max_us_ / 1000.0;
            double jitter_ms = dt_max_ms - dt_min_ms;

            // Sayaçları sıfırla
            dt_sum_us_ = 0; dt_cnt_ = 0;
            dt_min_us_ = INT64_MAX; dt_max_us_ = 0;

            if (debug_) {
                RCLCPP_INFO(get_logger(),
                    "[SERIAL]    %.1f Hz | dt: avg=%.2fms min=%.2fms max=%.2fms jitter=%.2fms | pkt=%zuB | frame=%u",
                    serial_hz_, dt_avg_ms, dt_min_ms, dt_max_ms, jitter_ms,
                    sizeof(jetsonOtopilotPack_t),
                    pkt.pack.variables.hrvs.frameNumber);

                // Tek tek tag pozu artik gonderilmiyor; sadece validity bit'leri.
                //   duvar valid1-4 = N/E/S/W, kucuk valid5-8 = 10/12/14/16.
                RCLCPP_INFO(get_logger(),
                    "[APRILTAG]  %.1f Hz | avg: valid=%s pos(%.3f,%.3f,%.3f) rpy(%.2f,%.2f,%.2f)"
                    " | duvar[N%d E%d S%d W%d] kucuk[%d%d%d%d]",
                    tag_fps_,
                    apriltag_.avgPos.validity.valid ? "Y" : "N",
                    apriltag_.avgPos.pos.x,
                    apriltag_.avgPos.pos.y,
                    apriltag_.avgPos.pos.z,
                    apriltag_.avgPos.angle.roll,
                    apriltag_.avgPos.angle.pitch,
                    apriltag_.avgPos.angle.yaw,
                    apriltag_.avgPos.validity.valid1,
                    apriltag_.avgPos.validity.valid2,
                    apriltag_.avgPos.validity.valid3,
                    apriltag_.avgPos.validity.valid4,
                    apriltag_.avgPos.validity.valid5,
                    apriltag_.avgPos.validity.valid6,
                    apriltag_.avgPos.validity.valid7,
                    apriltag_.avgPos.validity.valid8);

                RCLCPP_INFO(get_logger(),
                    "[OPTICFLOW] %.1f Hz | dx=%.3f dy=%.3f quality=%u",
                    flow_fps_,
                    opticflow_.motionPos.x,
                    opticflow_.motionPos.y,
                    opticflow_.quality);

                if (ir_enabled_) {
                    // Pakete NORMALIZE yon gidiyor (boyutsuz) — metreye otopilot
                    // cevirir. Koseli parantezteki metrik deger sadece gozle
                    // kontrol icin, Zopt ile carpilmis hali (gonderilmiyor).
                    // Zopt = ucgenden olculen menzil (dogrulamada kullanilan),
                    // Zrad = ayni andaki radar — sapma sahada hemen gorunsun.
                    // amin: eslesen en kucuk blob alani (menzil sinir gostergesi).
                    RCLCPP_INFO(get_logger(),
                        "[IR]        valid=%s yon(%+.4f,%+.4f) [Zopt=%.2f -> "
                        "%+.2f,%+.2f m] Zrad=%.2f | blob=%d amin=%dpx fit_err=%.2f%s",
                        ir_copy.valid ? "Y" : "N",
                        ir_copy.irpos.x, ir_copy.irpos.y, ir_ov_copy.z,
                        ir_copy.irpos.x * ir_ov_copy.z,
                        ir_copy.irpos.y * ir_ov_copy.z,
                        ir_ov_copy.z_radar,
                        ir_ov_copy.n_blobs, ir_ov_copy.min_area, ir_ov_copy.fit_err,
                        ir_ov_copy.no_alt ? " [radar yok]" : "");
                }
            }
        }
    }

    /* -------------------------------------------------------------- */
    /* HRVS thread — radar ölçüm döngüsü                             */
    /* -------------------------------------------------------------- */
    void hrvsThread()
    {
        if (!set_pad_output_ph07())
            RCLCPP_WARN(get_logger(),
                "HRVS: PH.07 pad fix basarisiz (root + /dev/mem gerekli)");

        UARTLinux    uart;
        GPIOPinLinux gpioPin;
        TimeStdlib   timeSource;
        ThreadStdlib currentThread;
        RadarSystem  radarSystem;

        auto uartRet = static_cast<UARTLinux::ReturnCode>(uart.initUART(HRVS_UART_PORT));
        if (uartRet != UARTLinux::ReturnCode::OK) {
            RCLCPP_ERROR(get_logger(), "HRVS: UART init basarisiz (kod %u)", unsigned(uartRet));
            return;
        }
        uartRet = static_cast<UARTLinux::ReturnCode>(
            uart.configureUART(HRVS_UART_BAUDRATE, IUART::Parity::NO,
                               IUART::StopBits::ONE, 8));
        if (uartRet != UARTLinux::ReturnCode::OK) {
            RCLCPP_ERROR(get_logger(), "HRVS: UART configure basarisiz (kod %u)", unsigned(uartRet));
            return;
        }

        char pio_id_buf[] = "PH.07";
        if (!gpioPin.configure(HRVS_GPIO_PIN_ID, pio_id_buf)) {
            RCLCPP_ERROR(get_logger(), "HRVS: GPIO configure basarisiz");
            return;
        }

        auto rc = radarSystem.configureInterfaces(&uart, &gpioPin, &timeSource, &currentThread);
        if (rc != RadarSystem::ReturnCode::OK) {
            RCLCPP_ERROR(get_logger(), "HRVS: configureInterfaces basarisiz (kod %u)", unsigned(rc));
            return;
        }

        ProcessingConfig procCfg;
        procCfg.numberOfRadars = HRVS_RADARS_NUMBER;
        RadarConfig radarCfg;
        radarCfg.radarType = RADAR_60GHZ_PROTO_1_0_CONFIG;
        radarSystem.setConfigurations(procCfg, radarCfg, true, true);

        rc = radarSystem.hrvsConfiguration();
        if (rc != RadarSystem::ReturnCode::OK) {
            RCLCPP_ERROR(get_logger(), "HRVS: hrvsConfiguration basarisiz (kod %u)", unsigned(rc));
            return;
        }
        RCLCPP_INFO(get_logger(), "HRVS: radar sistemi hazir (%u radar)", HRVS_RADARS_NUMBER);

        auto startTime = std::chrono::steady_clock::now();
        unsigned sysFrame = 0;
        float bodyVel[3]{};
        float avgRange = 0.0f;

        int consecutive_failures = 0;
        while (hrvs_running_.load()) {
            rc = radarSystem.runMeasurement();
            if (rc == RadarSystem::ReturnCode::OK) {
                consecutive_failures = 0;
                radarSystem.getBodyVelocity(sysFrame, bodyVel);
                radarSystem.getAvgRange(sysFrame, avgRange);

                auto ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startTime).count();

                std::lock_guard<std::mutex> lk(hrvs_mutex_);
                hrvs_data_.timeStamp   = static_cast<uint32_t>(ts_ms);
                hrvs_data_.altitude    = static_cast<uint16_t>(avgRange * 1000.0f);
                hrvs_data_.frameNumber = sysFrame;
                hrvs_data_.body_x      = static_cast<int16_t>(bodyVel[0] * 1000.0f);
                hrvs_data_.body_y      = static_cast<int16_t>(bodyVel[1] * 1000.0f);
                hrvs_data_.body_z      = static_cast<int16_t>(bodyVel[2] * 1000.0f);
                ++sysFrame;

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[HRVS] frame=%u alt=%.3fm vx=%.3f vy=%.3f vz=%.3f m/s",
                    sysFrame, avgRange,
                    bodyVel[0], bodyVel[1], bodyVel[2]);
            } else {
                ++consecutive_failures;
                // SDK internal reset sonrası donanıma toparlanma süresi ver
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // SDK reset yapıp CONFIGURATIONS_SET'e dönünce hrvsConfiguration tekrar gerekli
                if (consecutive_failures % 3 == 0) {
                    RCLCPP_WARN(get_logger(),
                        "[HRVS] %d ardisik hata — hrvsConfiguration yeniden deneniyor...",
                        consecutive_failures);
                    auto rrc = radarSystem.hrvsConfiguration();
                    if (rrc != RadarSystem::ReturnCode::OK) {
                        RCLCPP_ERROR(get_logger(),
                            "[HRVS] hrvsConfiguration basarisiz (kod %u)", unsigned(rrc));
                    }
                }
            }
        }
    }

    uint32_t msSinceStart()
    {
        return static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time_).count());
    }

    /* params */
    std::string serial_port_, tag_topic_, image_topic_;
    int    baud_rate_, grid_size_, num_levels_;
    double send_rate_hz_;
    float  ema_alpha_;
    bool   debug_;

    /* serial */
    int serial_fd_ = -1;

    /* subscriptions & timer */
    rclcpp::Subscription<
        isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* sensor verisi + mutex (spin thread ↔ vpi thread) */
    std::mutex      data_mutex_;
    apriltagPack_t  apriltag_{};
    opticFlowPack_t opticflow_{};
    irPack_t        ir_data_{};
    lps::DebugRecorder::IrOverlay ir_overlay_{};

    /* ---- IR LED dik ucgeni ---- */
    static constexpr int kIrMaxBlobs = 16;   // C(16,3)=560 kombinasyon — ucuz
    bool        ir_enabled_ = true;
    float       ir_leg_a_m_ = 1.36f, ir_leg_b_m_ = 1.36f;
    float       ir_tol_frac_ = 0.12f;
    float       ir_z_tol_    = 0.50f;   // optik Z vs radar Z kaba capraz kontrol
    double      ir_min_area_ = 1.0, ir_max_area_ = 400.0, ir_min_thresh_ = 240.0;
    std::string camera_info_topic_;
    // Tespit scratch'i — sadece imgCb thread'inden kullanilir, her karede
    // yeniden tahsis olmasin diye uye (LPS'deki small_/canvas_ ile ayni desen).
    std::vector<cv::Point2f> ir_pts_;
    std::vector<int>         ir_areas_;   // ir_pts_ ile ayni sirada (piksel alan)
    cv::Mat ir_bin_, ir_labels_, ir_stats_, ir_centroids_;
    // NOT (frame konvansiyonu): IR pozisyonu KAMERA OPTIK frame'inde gonderilir
    // (X sag, Y asagi) — govde cevrimi burada YAPILMAZ, otopilot yapar. AprilTag
    // yolu da aynidir (funnel -> avgPos.pos, hicbir mount donusumu yok). Buraya
    // bir R_mount eklemek iki yolu tutarsiz hale getirir.

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_sub_;
    std::mutex  cinfo_mutex_;
    cv::Matx33d cam_K_;
    cv::Mat     cam_D_;
    bool        cinfo_ready_ = false;

    /* ---- HUNI FUZYONU (LPS ana koddan port) ---- */
    struct PerTagLanding {
        int       id;
        cv::Vec3f raw;       // tag merkez pozisyonu (scaled, offset YOK)
        cv::Vec3f landing;   // raw + R*offset (o tag'in huni merkezi kestirimi)
        bool      used;      // false = outlier reject'lendi
    };
    float apriltag_node_size_ = 0.25f;   // AprilTag node size (duvar buyuk tag = 250mm)
    // tag25h9 HUNI offset'leri — LPS kalibrasyonu (26_08_20, dip = kucuk tag
    // centroid anchor). 8 tag da AYNI dip noktayi hedefler. Uyum ~3.6mm.
    // Lap60 AYNI fiziksel huniyi kullaniyor -> ayni degerler.
    // Duvar 0/20/24/34 (size 0.25), kucuk 10/12/14/16 (size 0.077, ID 25h9<35).
    const std::map<int, TagMeta> tag_meta_ = {
        { 0, {cv::Vec3f(+0.0023f, +0.2998f, -0.0712f), 0.25f}},   // N
        {20, {cv::Vec3f(-0.0019f, +0.2986f, -0.0789f), 0.25f}},   // E
        {24, {cv::Vec3f(+0.0027f, +0.3008f, -0.0735f), 0.25f}},   // S
        {34, {cv::Vec3f(+0.0014f, +0.2993f, -0.0757f), 0.25f}},   // W
        {10, {cv::Vec3f(+0.0429f, +0.0430f, -0.0010f), 0.077f}},  // sol-ust
        {12, {cv::Vec3f(-0.0435f, +0.0432f, +0.0045f), 0.077f}},  // sag-ust
        {14, {cv::Vec3f(+0.0432f, -0.0427f, -0.0095f), 0.077f}},  // sol-alt
        {16, {cv::Vec3f(-0.0434f, -0.0428f, +0.0010f), 0.077f}},  // sag-alt
    };

    /* Ucus video kaydi (opsiyonel, record_video param) */
    std::unique_ptr<lps::DebugRecorder> recorder_;
    // Son karede gorulen huni tag'lerinin overlay'i (data_mutex_ korumali;
    // tagCb yazar, imgCb okur -> recorder State'ine gider).
    std::vector<lps::DebugRecorder::TagOverlay> tag_overlays_;

    /* VPI thread iletişimi */
    std::thread              vpi_thread_;
    std::mutex               frame_mutex_;
    std::condition_variable  frame_cv_;
    cv::Mat                  latest_frame_;
    bool                     new_frame_  = false;
    std::atomic<bool>        vpi_running_{true};

    /* HRVS thread */
    std::thread        hrvs_thread_;
    std::atomic<bool>  hrvs_running_{true};
    std::mutex         hrvs_mutex_;
    hrvsPack_t         hrvs_data_{};

    /* FPS / Hz tracking */
    uint32_t tag_cb_count_    = 0;
    uint32_t flow_cb_count_   = 0;
    uint32_t serial_cb_count_ = 0;
    double   tag_fps_   = 0.0;
    double   flow_fps_  = 0.0;
    double   serial_hz_ = 0.0;
    std::chrono::steady_clock::time_point fps_last_tp_;

    /* Serial delta T jitter ölçümü (mikrosaniye) */
    std::chrono::steady_clock::time_point last_send_tp_;
    int64_t  dt_sum_us_ = 0;
    int64_t  dt_min_us_ = INT64_MAX;
    int64_t  dt_max_us_ = 0;
    uint32_t dt_cnt_    = 0;

    /* Optik akis motoru (LPS ana kod) — tum VPI durumu bu sinifin icinde */
    std::unique_ptr<lps::OpticalFlowVPI> flow_;

    /* Ham akis CSV logu (VPI thread'inden yazilir) */
    std::ofstream flow_csv_;
    uint64_t      csv_lines_ = 0;

    /* IR CSV logu (imgCb thread'inden yazilir — processIr) */
    std::ofstream ir_csv_;
    uint64_t      ir_csv_lines_ = 0;

    std::chrono::steady_clock::time_point start_time_;
};

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Thread 1: sensor callback'leri (imgCb VPI), Thread 2: timer (serial send)
    rclcpp::spin(std::make_shared<SerialSenderNode>());
    rclcpp::shutdown();
    return 0;
}
