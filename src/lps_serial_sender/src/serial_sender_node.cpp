/**
 * lps_serial_sender — serial_sender_node
 *
 * AprilTag detections (ID 1-4) + VPI OFA dense optical flow -> serial
 *
 * Paket formati (packed, little-endian):
 *   header_t           (4 B)
 *   jetsonVariables_t  (128 B): hrvs(16) + apriltag(30) + opticflow(13) + ir(13)
 *                              + apriltag2(30) + opticflow2(13)   <- infra2
 *                              + car(13)                          <- arac bearing
 *   chksum             (1 B)
 *   Toplam: 133 B, msg_id 0x42, 50 Hz  (static_assert ile kilitli)
 *
 * AYRI MESAJ — sistem sagligi (20_10): ayni header, msg_id 0x48, ~1 Hz
 *   header_t (4 B) + healthPack_t (26 B) + chksum (1 B) = 31 B
 *   Ana paketin hemen ARKASINDAN ayni TX thread'inden yazilir (tek yazar).
 *
 * NOT: payloadSize ve checksum sinirlari sizeof()'tan turetilir; bu yorum
 * sadece okuyucu icindir.
 *
 * IR ALANI - BIRIM UYARISI: irpos METRE DEGIL, boyutsuz normalize yondur
 * (X/Z, Y/Z). Otopilot kendi menzili ile carpar. Ayrinti processIr'da.
 */

#include <rclcpp/rclcpp.hpp>
#include "lps_serial_sender/stamp_seconds.hpp"     // damga -> saniye, istisnasiz (20_10)
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/range.hpp>        // HRVS irtifasi -> car_detector boy filtresi
#include <geometry_msgs/msg/vector3_stamped.hpp>   // infra2 optik akis (x=dx y=dy z=Q)
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// LPS ana koddaki optik akis motoru - dokular/consistency/adaptif esik/
// on-suzme/kapsama-cezali quality/EMA hepsi burada. Bu dosyayla BIRE BIR
// ayni tutulur; VPI hattinin tamami bu sinifa devredildi.
#include "lps_serial_sender/optical_flow_vpi.hpp"
#include "lps_serial_sender/debug_recorder.hpp"   // ucus video kaydi (opsiyonel)
#include "lps_serial_sender/ir_triangle.hpp"      // IR blob tespiti + ucgen eslesmesi
#include "lps_serial_sender/tag_fusion.hpp"       // AprilTag -> avgPos (infra2 ile ORTAK)
#include "lps_serial_sender/tag_gate.hpp"         // 08_11: kisa tag goruslerini otopilota gonderme
#include "lps_serial_sender/frame_gap_stats.hpp"  // akis kare araligi duzenliligi
#include "lps_serial_sender/system_health.hpp"    // Jetson sicaklik/yuk/guc -> health alani
#include "lps_serial_sender/autopilot_rx.hpp"     // otopilot -> Jetson mesajlari (0x49 KAPAT)
#include "lps_serial_sender/dual_camera_fusion.hpp"  // infra1+infra2: tag avgPos + akis birlesimi
#include <cstdlib>                                // std::system (kapatma komutu)

#include <fstream>
#include <sstream>
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
#include <pthread.h>    // TX thread: RT oncelik + CPU affinity
#include <sched.h>      // SCHED_FIFO, cpu_set_t, CPU_SET

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

// HRVS SDK zaman kaynagi: MONOTONIK. SDK'nin TimeStdlib'i system_clock (duvar saati)
// kullaniyor; bu Jetson'da saat acilista geri yuklenip NTP ile ATLIYOR -> SDK'nin
// 1000 ms'lik alim zaman asimlari saat geri atlarsa dakikalarca dolmaz (20_10).
// ITime dokumani 'acilistan beri gecen sure' de kabul ediyor.
class TimeSteady : public hrvs_sdk::ITime
{
public:
    long long getTime() override
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

// HRVS reset darbesi: SDK'nin reset()'i ile ayni — PH.07'yi 500 ms '1'de tutmak
// radari kapatip acar (RESET_ICB_DELAY). Radarin acilmasi icin ardindan bekleme.
static constexpr int HRVS_RESET_HOLD_MS = 500;
static constexpr int HRVS_BOOT_WAIT_MS  = 2000;
// SDK cagrisi bu kadar ms ilerleme kaydetmezse takildi say (normalde <= ~6 s)
static constexpr int64_t HRVS_STALL_MS      = 20000;
static constexpr int64_t HRVS_WD_REPEAT_MS  = 30000;

#define SYNC_BYTE  0xBB
#define MSG_ID        0x42   // ana LPS paketi, 50 Hz
#define MSG_ID_HEALTH 0x48   // sistem sagligi paketi, ~1 Hz (yeni ornek gelince)

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
/* Packet layout - autopilot firmware ile birebir uyumlu              */
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
 *   valid1-4 = duvar 0/20/24/34, valid5-8 = kucuk 10/12/14/16 (65 mm). */
struct tag_validity_t {
    uint16_t valid    : 1;   // avgPos gecerli (n_inlier>0)
    uint16_t valid1   : 1;   // 0(N)
    uint16_t valid2   : 1;   // 20(Bati)
    uint16_t valid3   : 1;   // 24(S)
    uint16_t valid4   : 1;   // 34(Dogu)
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

/* Arac tespiti (car_detector: DeepStream YOLOv8 VisDrone + NvDCF tek-arac kilidi,
   RENKLI kamera). irPack_t ile ayni yapi. bearing = kilitli aracin kutu merkezinin
   distorsiyon-duzeltilmis NORMALIZE yonu (X/Z, Y/Z), renkli kameranin optik frame'i —
   BOYUTSUZ; metreye otopilot kendi menziliyle cevirir. valid=0: kilit yok, ya da
   car_detector car_timeout_ms'den uzun suredir ses vermiyor. */
struct carPack_t {
    uint32_t    timeStamp;   // serial_sender'a son bearing mesajinin ulastigi an
    bool        valid;
    float_2D_t  bearing;
};

/* Jetson sistem sagligi (2026-10-20) — AYRI MESAJ (msg_id 0x48), ~1 Hz: her yeni
   ornek (system_health.hpp) bir kez gonderilir; ana paket degismez. Uzaktan izleme
   icin: 19_10 ucusundaki RAM dolmasi / sicaklik / asiri akim / guc kesintisi ve
   20_10'daki fan durmasi -> termal kapanma ancak loglardan anlasilabildi.
   Okunamayan sicaklik -128. Paket gelmez olursa: serial_sender ya da executor durdu. */
struct healthPack_t {
    uint32_t timeStamp;    // ornekleme ani (ms, serial_sender baslangicindan)
    bool     valid;        // en az bir ornek alindi
    uint32_t uptime;       // Jetson acik kalma suresi (s); kucuk = yeni acildi / guc kesildi
    uint8_t  cpuLoad;      // % (tum cekirdekler)
    uint8_t  cpuCoreMax;   // % (en yuklu cekirdek; ~100 = akis kare atlar)
    uint8_t  gpuLoad;      // %
    int8_t   cpuTemp;      // C
    int8_t   gpuTemp;      // C
    int8_t   tjTemp;       // C, en sicak nokta (70 = cekirdek uyari esigi, 99 = kisma)
    uint8_t  ramUsed;      // % (MemTotal - MemAvailable)
    uint8_t  swapUsed;     // %
    uint8_t  diskUsed;     // % (kayit diski)
    uint16_t fanRpm;
    uint16_t inVoltage;    // VDD_IN modul girisi (mV, ~5000)
    uint16_t inPower;      // VDD_IN modul girisi (mW; ~25000 = asiri akim siniri)
    uint16_t ocEvents;     // asiri akim (OC) kisma olaylari, acilistan beri toplam
};

struct jetsonVariables_t {
    hrvsPack_t      hrvs;
    apriltagPack_t  aprtiltag;    // infra1 (sol IR kamera)
    opticFlowPack_t opticflow;    // infra1
    irPack_t        ir;           // infra1
    // ---- infra2 (sag IR kamera, infra1'in 50.06 mm sagi) — PAKETIN SONUNA eklendi ----
    // infra1 ile AYNI hesap: tag -> tag_fusion.hpp fuseTags, akis -> ayni
    // OpticalFlowVPI (infra2_recorder_node, launch'ta ortak parametreler).
    // infra2 kapaliysa (infra2_test:=false) bu iki alan SIFIR gider.
    apriltagPack_t  aprtiltag2;   // infra2
    opticFlowPack_t opticflow2;   // infra2
    // ---- arac bearing'i (renkli kamera, car_detector) — PAKETIN SONUNA eklendi ----
    carPack_t       car;
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

// Sistem sagligi mesaji (msg_id 0x48): ayni header + checksum kurali
struct jetsonHealthPack_t {
    header_t     header;
    healthPack_t health;
    uint8_t      chksum;
};

union jetsonHealthPack_ut {
    jetsonHealthPack_t pack;
    uint8_t            data[sizeof(jetsonHealthPack_t)];
};

#pragma pack()

// Wire formati otopilot firmware'iyle BIREBIR olmali: bir alan yanlislikla
// degisirse derleme burada durur (firmware tarafi da ayni boyutu beklemeli).
static_assert(sizeof(apriltagPack_t)       == 30,  "apriltagPack_t boyutu degisti");
static_assert(sizeof(opticFlowPack_t)      == 13,  "opticFlowPack_t boyutu degisti");
static_assert(sizeof(irPack_t)             == 13,  "irPack_t boyutu degisti");
static_assert(sizeof(carPack_t)            == 13,  "carPack_t boyutu degisti");
static_assert(sizeof(healthPack_t)         == 26,  "healthPack_t boyutu degisti");
static_assert(sizeof(jetsonVariables_t)    == 128, "payload boyutu degisti (firmware!)");
static_assert(sizeof(jetsonOtopilotPack_t) == 133, "paket boyutu degisti (firmware!)");
static_assert(sizeof(jetsonHealthPack_t)   == 31,  "saglik paketi boyutu degisti (firmware!)");
static_assert(sizeof(jetsonVariables_t)    <= 255, "payloadSize uint8_t'ye sigmali");

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
    tty.c_cflag &= ~CRTSCTS;      // hardware flow control kapali
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
/* RANSAC global motion estimator (VPI flow - dx,dy)                  */
/* ------------------------------------------------------------------ */
// NOT: Eski serbest estimateGlobalDxDy (basit inlier-ratio quality'li) KALDIRILDI.
// Global tahmin artik lps::OpticalFlowVPI icindeki surum tarafindan yapiliyor -
// dokusuz hucre gating'i, yumusak consistency skoru, sahneye uyarlanir esik ve
// kapsama-cezali quality ile (LPS ana koddaki motorun ta kendisi).

/* ------------------------------------------------------------------ */
/* Inis modu secimi (TEST_APRITLAG: huni / 45-24), TagMeta, huni      */
/* offset tablosu, quatToMatrix/quatToRPY -> tag_fusion.hpp.          */
/* Modu DEGISTIRMEK icin: include/lps_serial_sender/tag_fusion.hpp.   */
/* (infra2_recorder_node da ayni basligi kullanir -> iki video ayni   */
/*  hesabi gosterir.)                                                  */
/* ------------------------------------------------------------------ */

/** TagFusion -> wire paketi. infra1 (aprtiltag) ve infra2 (aprtiltag2) AYNI
 *  eslemeyi kullansin diye tek yerde. */
static apriltagPack_t packFromFusion(const lps::TagFusion &f, uint32_t ts)
{
    apriltagPack_t ap{};
    ap.timeStamp = ts;
    ap.avgPos.validity.valid  = f.valid;
    ap.avgPos.pos.x           = f.pos[0];
    ap.avgPos.pos.y           = f.pos[1];
    ap.avgPos.pos.z           = f.pos[2];
    ap.avgPos.angle.roll      = f.roll;    // derece; huni modunda 0
    ap.avgPos.angle.pitch     = f.pitch;
    ap.avgPos.angle.yaw       = f.yaw;
    // Huni modunda inlier tag bitleri (45/24 modunda hepsi 0):
    //   valid1=0(Kuzey) valid2=20(Bati) valid3=24(Guney) valid4=34(Dogu)  [duvar]
    //   valid5=10   valid6=12    valid7=14    valid8=16      [kucuk, 65 mm]
    ap.avgPos.validity.valid1 = (f.id_bits >> 0) & 1u;
    ap.avgPos.validity.valid2 = (f.id_bits >> 1) & 1u;
    ap.avgPos.validity.valid3 = (f.id_bits >> 2) & 1u;
    ap.avgPos.validity.valid4 = (f.id_bits >> 3) & 1u;
    ap.avgPos.validity.valid5 = (f.id_bits >> 4) & 1u;
    ap.avgPos.validity.valid6 = (f.id_bits >> 5) & 1u;
    ap.avgPos.validity.valid7 = (f.id_bits >> 6) & 1u;
    ap.avgPos.validity.valid8 = (f.id_bits >> 7) & 1u;
    return ap;
}

/* apriltagPack_t <-> lps::TagSample (iki kamera birlesimi, dual_camera_fusion.hpp) */
static lps::TagSample tagSample(const apriltagPack_t &ap, bool fresh)
{
    const auto &v = ap.avgPos.validity;
    lps::TagSample s;
    s.ok   = fresh && v.valid;
    s.x    = ap.avgPos.pos.x;      s.y = ap.avgPos.pos.y;      s.z = ap.avgPos.pos.z;
    s.roll = ap.avgPos.angle.roll; s.pitch = ap.avgPos.angle.pitch; s.yaw = ap.avgPos.angle.yaw;
    s.bits = (uint8_t)(v.valid1 | v.valid2 << 1 | v.valid3 << 2 | v.valid4 << 3 |
                       v.valid5 << 4 | v.valid6 << 5 | v.valid7 << 6 | v.valid8 << 7);
    return s;
}

static apriltagPack_t tagPack(const lps::TagSample &s, uint32_t ts)
{
    lps::TagFusion f;
    f.valid = s.ok;
    f.pos   = cv::Vec3f(s.x, s.y, s.z);
    f.roll  = s.roll; f.pitch = s.pitch; f.yaw = s.yaw;
    f.id_bits = s.bits;
    return packFromFusion(f, ts);
}

/* Kameranin platform merkezine ofsetini dus (kamera ekseninde, m). Ofset = kameradan
   platform merkezine vektor; kamera govdeye rijit oldugu icin kamera ekseninde SABIT —
   yaw/egim donusunde bozulmaz. Sadece gecerli tespitte (gecersizde pos 0 kalsin). */
static void subtractCamOffset(apriltagPack_t &ap, const double off[2])
{
    if (!ap.avgPos.validity.valid) return;
    ap.avgPos.pos.x -= (float)off[0];
    ap.avgPos.pos.y -= (float)off[1];
}

/* ------------------------------------------------------------------ */
/* IR LED ucgeni - tespit + eslesme: ir_triangle.hpp (test ile ORTAK)  */
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
        tx_cpu_       = declare_parameter<int>   ("tx_cpu",       5);   // TX thread CPU pin (-1=kapali)
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
        // Konum SADECE dik aci kosesinden cikar (pinhole geri izdusum, Z=radar);
        // diger iki nokta yalnizca 'bu gercekten bizim ucgenimiz mi' dogrulamasi.
        // Ikizkenar dik ucgen: iki dik kenar da 1.36 m (hipotenus 1.92 m).
        // Kenar/hipotenus orani 1.414 -> 'en uzun kenarin karsisi = dik aci'
        // testi saglam marjla calisir. Iki kenar esit oldugu icin a/b eslesme
        // belirsizligi tamamen anlamsiz (konumu zaten etkilemiyordu).
        ir_enabled_    = declare_parameter<bool>  ("ir_enabled",     true);
        ir_leg_a_m_    = (float)declare_parameter<double>("ir_leg_a_m",    1.445);
        ir_leg_b_m_    = (float)declare_parameter<double>("ir_leg_b_m",    1.31);
        // 0.12: sentetik testle olculdu (test_ir_triangle). Mesru tespitlerin
        // fit artigi 30 derece egimde bile 0.074; ESKENAR ucgen ise 0.157 alir.
        // Eski 0.25 degeri eskenar ucgeni KABUL ediyordu - olcek-bagimsiz
        // dogrulama mutlak-olcek kontrolunden zayif oldugu icin daha siki olmali.
        ir_tol_frac_   = (float)declare_parameter<double>("ir_tol_frac",   0.12);
        // Huni merkezi LED'lere gore (06_11 kullanici olcusu): R'den kisa kenar (sol LED)
        // boyunca 0.655 m, uzun kenar (sag LED) boyunca 0.617 m. Hipotenus ortasi merkezden
        // ~10 cm kayikti (tag'lerle olculdu); bu degerlerle 2 karede 1.4 / 1.8 cm.
        // Goruntude kenar orani < ir_leg_ambig_ratio ise hipotenus ortasina duser.
        ir_center_on_short_m_ = (float)declare_parameter<double>("ir_center_on_short_m", 0.655);
        ir_center_on_long_m_  = (float)declare_parameter<double>("ir_center_on_long_m",  0.617);
        ir_leg_ambig_ratio_   = (float)declare_parameter<double>("ir_leg_ambig_ratio",   1.03);
        // Optik Z ile radar Z arasinda izin verilen goreli fark. Gevsek tutuldu:
        // amac hassas kalibrasyon degil, kabaca yanlis olcekteki sahte eslesmeyi
        // elemek (radar 10 Hz + bayat oldugu icin sikilastirmak dogru eslesmeleri
        // atmaya baslar). Radar yoksa kontrol tamamen atlanir.
        // 06_11: 0.25 (vinc testi: dogru ucgen 0.90-0.93, LED+lamba sahtesi 1.39/0.70).
        // Irtifa tazeyken ucgen SECIMI de bu aralikla sinirli (ir_triangle k_min/k_max).
        ir_z_tol_      = (float)declare_parameter<double>("ir_z_tol",      0.25);
        // Sahte ucgen kurallari (02_11; gece LED testinde yeniden ayarlanacak).
        // A: <= bu kadar parlak nokta varsa degerlendir (gece 3 LED + birkac lamba).
        ir_max_blobs_valid_ = declare_parameter<int>("ir_max_blobs_valid", 6);
        // B: radar yoksa/bayatsa GECERSIZ (eskiden Z kontrolu sessizce atlaniyordu).
        //    Radarsiz masa testinde ir_require_radar:=false.
        ir_require_radar_   = declare_parameter<bool>("ir_require_radar", true);
        ir_radar_stale_ms_  = (uint32_t)declare_parameter<int>("ir_radar_stale_ms", 1500);
        // D: ayni yerde (kareden kareye <= ir_max_jump_px) en az ir_min_consec kare.
        ir_min_consec_      = declare_parameter<int>("ir_min_consec", 5);
        ir_max_jump_px_     = declare_parameter<double>("ir_max_jump_px", 20.0);
        // min_area MENZILI BELIRLER: yukseklik arttikca LED'in piksel izi
        // 1/Z^2 ile kuculur ve tespiti sinirlayan ilk sey genelde esik degil,
        // bu alan tabani olur. 1 px'e indirildi - tek piksel gurultusunu ucgen
        // geometrisi zaten eliyor (3 nokta + dogru olcek sarti cok secici).
        ir_min_area_   = declare_parameter<double>("ir_min_area",    1.0);
        ir_max_area_   = declare_parameter<double>("ir_max_area",  400.0);
        // Yuksek esik: infra1'i AprilTag + optik akis da tuketiyor, pozlama
        // LED'ler icin kisilamaz (sahne kararir, ikisi de coker). Kameraya bakan
        // aktif LED normal pozlamada da doygun (255), sahne degil -> ayrim burada.
        ir_min_thresh_ = declare_parameter<double>("ir_min_threshold", 240.0);

        // IRTIFAYA GORE LED ayari (06_11 vinc testi). Sabit 400 px ust sinir LED'leri
        // ~3 m altinda eliyordu (2.4 m'de 663-723 px), yerlerine huni parlamasi kiriktilari
        // ucgene giriyordu. Irtifa tazeyse:
        //   - beklenen LED alani A(Z) = ref * (ref_z/Z)^exp  (olculen: 750, 1.75 m, 0.6)
        //   - alan penceresi [A/lo_div, A*hi]; alt sinir sadece Z <= model_max_z (olculen
        //     aralik 1.5-4 m; ustunde alt sinir ir_min_area, yani eski davranis)
        //   - ucgen SECIMI beklenen olcekle (Z +- ir_z_tol) sinirli
        // Irtifa yoksa: [ir_min_area, ir_noalt_max_area] (B kurali zaten gecersiz sayar).
        // Kaynak (ir_alt_source): auto = EKF (otopilot 0x4A), yoksa HRVS | ekf | hrvs |
        // tag = AprilTag avgPos z (SADECE yer testi: radar tavani okur; ucusta KULLANMA).
        ir_alt_source_     = declare_parameter<std::string>("ir_alt_source", "auto");
        ir_area_ref_       = declare_parameter<double>("ir_led_area_ref",    750.0);
        ir_area_ref_z_     = declare_parameter<double>("ir_led_area_ref_z",  1.75);
        ir_area_exp_       = declare_parameter<double>("ir_led_area_exp",    0.6);
        ir_area_hi_        = declare_parameter<double>("ir_led_area_hi",     2.5);
        ir_area_lo_div_    = declare_parameter<double>("ir_led_area_lo_div", 6.0);
        ir_area_model_max_z_ = declare_parameter<double>("ir_led_area_model_max_z", 5.0);
        ir_noalt_max_area_ = declare_parameter<double>("ir_noalt_max_area", 2500.0);
        // Yuvarlaklik: bu alandan buyuk blob'larda doluluk/en-boy sarti (LED 0.60-0.75 / ~1.0,
        // huni parlamasi 0.3-0.4 / 2-2.5).
        ir_round_min_area_ = declare_parameter<double>("ir_round_min_area",  30.0);
        ir_round_min_fill_ = declare_parameter<double>("ir_round_min_fill",  0.55);
        ir_round_max_asp_  = declare_parameter<double>("ir_round_max_aspect", 1.6);
        if (ir_alt_source_ != "auto" && ir_alt_source_ != "ekf" &&
            ir_alt_source_ != "hrvs" && ir_alt_source_ != "tag") {
            RCLCPP_WARN(get_logger(), "ir_alt_source '%s' bilinmiyor -> auto", ir_alt_source_.c_str());
            ir_alt_source_ = "auto";
        }
        if (ir_alt_source_ == "tag")
            RCLCPP_WARN(get_logger(), "IR irtifa kaynagi = AprilTag z (YER TESTI modu, ucusta kullanma)");
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
        // Kamera fps: sadece akis kare-araligi istatistigi icin (duzenli/atlanan %).
        gap_stats_ = std::make_unique<lps::FrameGapStats>(
            declare_parameter<double>("camera_fps", 30.0));
        flow_ = std::make_unique<lps::OpticalFlowVPI>(
            grid_size_, num_levels_, ema_alpha_, flow_tex, flow_pre);

        // Kayit oturum numarasi (launch verir): tum dosya adlarinda _sNNNN —
        // saat geri yuklense de sira/tekillik bozulmaz (bkz. record_naming.hpp).
        record_session_ = declare_parameter<int>("record_session", 0);

        // Ham akis CSV logu (EMA ONCESI kare-basi olcum) - analiz icin, opsiyonel.
        // Varsayilan KAPALI: mevcut davranisi degistirmez, launch'tan acilir.
        if (declare_parameter<bool>("flow_csv", false)) {
            std::string dir = declare_parameter<std::string>(
                "record_dir", "/home/lapis/lps_records");
            std::string path = lps::claimRecordPath(dir, "flow", record_session_, "csv");
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

        // IR CSV logu - ucus sonrasi analiz icin, opsiyonel (varsayilan KAPALI).
        // HER kare yazilir (tespit olmasa da): esik ayari icin asil degerli veri
        // tespitin OLMADIGI karelerdeki blob sayisi. Sutunlar saha kontrol
        // listesiyle birebir: blob/amin=menzil, zopt-zrad=olcek, fit=egim.
        if (ir_enabled_ && declare_parameter<bool>("ir_csv", false)) {
            std::string dir = has_parameter("record_dir")
                ? get_parameter("record_dir").as_string()
                : declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
            std::string path = lps::claimRecordPath(dir, "ir", record_session_, "csv");
            ir_csv_.open(path);
            if (ir_csv_) {
                // nx,ny = PAKETE GIDEN normalize yon (boyutsuz). x_at_zopt /
                // y_at_zopt = nx*z_opt (metrik karsilik, SADECE analiz kolayligi
                // icin - gonderilmiyor, otopilot kendi Z'siyle carpiyor).
                ir_csv_ << "t_ms,valid,nx,ny,x_at_zopt,y_at_zopt,z_opt,z_radar,"
                           "n_blobs,amin,fit,rx,ry,ax,ay,bx,by,mx,my,aday,neden,dizi\n";
                ir_csv_.flush();
                RCLCPP_INFO(get_logger(), "IR CSV: %s", path.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "IR CSV acilamadi: %s", path.c_str());
            }
        }

        // Otopilot -> Jetson RX kaydi (12_11): 148-151 ucuslarinda EKF yuksekligi ~11 m'de
        // 5.5 s + 8k s'lik bloklarla kesildi, journal guc kesilince kayboldugu icin sebep
        // gorulemedi. Her 0x4A (zaman, z, gecerli) + saniyede bir ozet (mesaj sayilari, bozuk
        // cerceve). Sadece RX thread'i yazar; ~10 satir/s, ihmal edilebilir yuk.
        if (declare_parameter<bool>("rx_csv", true)) {
            const std::string dir = has_parameter("record_dir")
                ? get_parameter("record_dir").as_string()
                : declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
            const std::string path = lps::claimRecordPath(dir, "rx", record_session_, "csv");
            rx_csv_.open(path);
            if (rx_csv_) {
                // tip E: 0x4A geldi (gecerli -1 = bozuk yuk). tip S: son ~1 s ozeti.
                rx_csv_ << "t_ms,tip,ekf_z,gecerli,n_4a,n_4a_gecerli,n_42,n_49,n_diger,"
                           "bozuk_header,bozuk_yuk,atlanan_bayt,rx_bayt\n";
                rx_csv_.flush();
                RCLCPP_INFO(get_logger(), "RX CSV: %s", path.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "RX CSV acilamadi: %s", path.c_str());
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
            record_label_ = declare_parameter<std::string>(
                "record_label", "INFRA1 - UCUS (seriale giden)");
            // Dosya adi oneki: lps_infra1_YYYYmmdd_HHMMSS.avi (infra2 testi
            // lps_infra2_... yazar -> iki video adindan ayirt edilir).
            const std::string prefix = declare_parameter<std::string>(
                "record_prefix", "lps_infra1");
            recorder_ = std::make_unique<lps::DebugRecorder>(dir, fps, scale, prefix,
                                                             record_session_);
            RCLCPP_INFO(get_logger(), "Ucus video kaydi: %s", recorder_->path().c_str());
        }

        // Sistem sagligi: 1 Hz, executor thread'inde (sysfs okuma < 1 ms). TX thread'i
        // sadece son ornegi kopyalar. Disk dolulugu kayit klasorunun dosya sisteminden.
        {
            const std::string dir = has_parameter("record_dir")
                ? get_parameter("record_dir").as_string()
                : declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
            health_ = std::make_unique<lps::SystemHealth>(dir);
            RCLCPP_INFO(get_logger(), "Sistem sagligi kaynaklari: %s", health_->sources().c_str());
            // GECICI (20_10, kullanici istegi): 0x48'in cpuTemp alanina CPU yerine TUM
            // radarlarin en yuksek cip sicakligi yazilir — paket yapisi DEGISMEZ. Radar
            // UART hatalari isinmayla mi geliyor, otopilot logundan izlemek icin.
            // Radar verisi 5 s'den eskiyse -128 (protokoldeki 'okunamadi'). Geri almak:
            // health_cpu_temp_source:=cpu. Gercek CPU sicakligi [HEALTH] logunda kalir.
            cpu_temp_from_radar_ =
                declare_parameter<std::string>("health_cpu_temp_source", "radar") == "radar";
            if (cpu_temp_from_radar_)
                RCLCPP_WARN(get_logger(), "Saglik paketi cpuTemp alani = RADAR max sicakligi%s",
                            hrvs_debug_log_ ? "" : " — ama hrvs_debug_log kapali, hep -128 gidecek");
            health_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
                hrvsWatchdogTick();
                if (hrvs_debug_log_) hrvsReadTemps(++hrvs_temp_tick_ % 10 == 0);
                const lps::HealthSample h = health_->sample();
                healthPack_t p{};
                p.timeStamp  = msSinceStart();
                p.valid      = true;
                p.uptime     = h.uptime_s;
                p.cpuLoad    = h.cpu_load;   p.cpuCoreMax = h.cpu_core_max;
                p.gpuLoad    = h.gpu_load;
                p.cpuTemp    = h.cpu_temp;   p.gpuTemp    = h.gpu_temp;   p.tjTemp = h.tj_temp;
                p.ramUsed    = h.ram_used;   p.swapUsed   = h.swap_used;  p.diskUsed = h.disk_used;
                p.fanRpm     = h.fan_rpm;
                p.inVoltage  = h.vdd_in_mv;  p.inPower    = h.vdd_in_mw;
                p.ocEvents   = h.oc_events;
                if (cpu_temp_from_radar_) {
                    const int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    const bool fresh = hrvs_temp_change_ms_ > 0 && now - hrvs_temp_change_ms_ < 5000;
                    p.cpuTemp = fresh ? (int8_t)std::clamp(std::lround(hrvs_temp_max_), -127L, 127L)
                                      : std::numeric_limits<int8_t>::min();
                }
                std::lock_guard<std::mutex> lk(data_mutex_);
                health_pack_ = p;
                health_cpu_real_ = h.cpu_temp;
            });
        }

        serial_fd_ = openSerial(serial_port_, baud_rate_);

        // Otopilot -> Jetson: 0x49 KAPAT (yuk 01) gelirse Jetson'i DUZGUN kapat
        // (servis durur, video/CSV kapanir, sonra poweroff). Servis root calistigi
        // icin systemctl izin ister/bekler degil. Test: poweroff_command:=<zararsiz komut>.
        poweroff_enable_ = declare_parameter<bool>("autopilot_poweroff_enable", true);
        poweroff_cmd_    = declare_parameter<std::string>("poweroff_command", "/usr/bin/systemctl poweroff");

        // Otopilot -> Jetson: 0x4A EKF yuksekligi (05_11). Sadece gecerli=1 iken yayinlanir;
        // car_detector boy filtresi + IR/EO gecisi icin HRVS'ten once bunu kullanir (EKF baro +
        // LRF + HRVS birlestirir, HRVS kopunca da gelir). RX thread'inden ONCE olusturulmali.
        ekf_alt_pub_ = create_publisher<sensor_msgs::msg::Range>(
            declare_parameter<std::string>("ekf_altitude_topic", "/autopilot/ekf_altitude"),
            rclcpp::SensorDataQoS());

        tag_sub_ = create_subscription<
            isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            tag_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::tagCb, this, std::placeholders::_1));

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::imgCb, this, std::placeholders::_1));

        // infra2 -> paketin sonundaki aprtiltag2 / opticflow2. infra2 kapaliysa
        // bu topic'ler sessiz kalir ve alanlar SIFIR gider (valid=0, Q=0, ts=0).
        infra2_tag_topic_  = declare_parameter<std::string>("infra2_tag_topic",  "/tag_detections_infra2");
        infra2_flow_topic_ = declare_parameter<std::string>("infra2_flow_topic", "/infra2/optical_flow");

        // Iki kamera birlesimi + platform merkezi ofseti (03_11, dual_camera_fusion.hpp).
        // Ofset = kameradan platform merkezine vektor, KAMERA ekseninde (x saga, y asagi), m.
        // Kayitla dogrulandi: farklari (50.9 mm) kameralar arasi baseline; dusulunce iki
        // kameranin tag konumu ortusuyor (ham +45 mm -> -6 mm).
        {
            const auto o1 = declare_parameter<std::vector<double>>("cam_offset_infra1", {0.032962, 0.045});
            const auto o2 = declare_parameter<std::vector<double>>("cam_offset_infra2", {-0.017962, 0.045});
            if (o1.size() >= 2) { cam_off1_[0] = o1[0]; cam_off1_[1] = o1[1]; }
            if (o2.size() >= 2) { cam_off2_[0] = o2[0]; cam_off2_[1] = o2[1]; }
        }
        dual_fusion_     = declare_parameter<bool>("dual_camera_fusion", true);
        tag_pair_tol_m_  = (float)declare_parameter<double>("tag_pair_tol_m", 0.30);   // tutarsizsa infra1
        tag_fresh_ms_    = (uint32_t)declare_parameter<int>("tag_fresh_ms", 200);      // 15 Hz x3
        // Tag gecerlilik kapisi (08_11, tag_gate.hpp): kamera basina, fuzyondan ONCE.
        // ~10 m menzil sinirinda 1-2 karelik tag gorusleri otopilotu inisi askiya almaya
        // goturuyordu (tag -> arac olcumu 1 s kapali -> tag gelmez -> konum kaybi).
        // tag_min_consec ardisik tespit -> gecerli; gecerliyken tag_keep_s (1.0 s) icindeki
        // kesintiden donus aninda gecerli. s0133'te askiya alma tetigi 18 -> 6.
        // tag_gate_enable:=false -> eski davranis.
        {
            const bool en   = declare_parameter<bool>("tag_gate_enable", true);
            const int  n    = (int)declare_parameter<int>("tag_min_consec", 3);
            const double kp = declare_parameter<double>("tag_keep_s", 1.0);
            tag_gate_en_ = en;
            gate1_ = lps::TagGate(n, kp);
            gate2_ = lps::TagGate(n, kp);
            RCLCPP_INFO(get_logger(), "Tag kapisi %s: %d ardisik tespit, kalma %.2f s",
                        en ? "ACIK" : "KAPALI", n, kp);
        }
        // Iki tag sonucu ancak cekim zamanlari bu kadar yakinsa (ayni kare; 30 fps'te kare
        // 33 ms) ortalanir; degilse yeni olan gider.
        tag_sync_s_      = declare_parameter<double>("tag_pair_sync_ms", 10.0) / 1000.0;
        flow_fresh_ms_   = (uint32_t)declare_parameter<int>("flow_fresh_ms", 100);     // 30 Hz x3
        flow_agree_px_   = (float)declare_parameter<double>("flow_agree_px", 1.0);
        flow_agree_frac_ = (float)declare_parameter<double>("flow_agree_frac", 0.3);
        tag2_sub_ = create_subscription<
            isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            infra2_tag_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::tagCb2, this, std::placeholders::_1));
        flow2_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            infra2_flow_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::flowCb2, this, std::placeholders::_1));

        // Arac bearing'i -> paketin sonundaki car. car_detector kapaliysa alan SIFIR.
        // car_timeout_ms: bu sureden eski bearing valid=0 gider (node olurse/takilirsa
        // otopilot son kilitli yonu taze sanmasin; diger dronedaki tag_timeout mantigi).
        car_topic_      = declare_parameter<std::string>("car_bearing_topic", "/car_detection/bearing");
        car_timeout_ms_ = (uint32_t)declare_parameter<int>("car_timeout_ms", 500);
        car_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            car_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::carCb, this, std::placeholders::_1));

        // Serial gonderim: ROS wall timer YERINE ozel RT TX thread (LPS gibi).
        // Executor jitter'indan bagimsiz, mutlak-deadline (sleep_until) sabit periyot.
        tx_thread_ = std::thread(&SerialSenderNode::txThread, this);
        rx_thread_ = std::thread(&SerialSenderNode::rxThread, this);

        vpi_thread_  = std::thread(&SerialSenderNode::vpiThread,  this);
        // hrvs_enabled:=false -> radar thread'i HIC baslamaz (UART/GPIO'ya
        // dokunulmaz, hrvs alanlari 0 gider). Masa testi / radarsiz calisma icin;
        // canli servis radari kullanirken ikinci bir kopyanin radar UART'ina
        // konfig yazip onu bozmamasi icin de gerekli. Varsayilan: acik.
        // HRVS SDK debug logu (VARSAYILAN ACIK, 20_10 kullanici istegi): radarla gidip gelen
        // tum mesajlar CSV'ye — min_range.csv'de HER radarin cip sicakliklari (RX0-2,
        // TX0-2, PM, DIG0-1). Amac: ~25 dk sonra baslayan UART hatalari radar isinmasiyla
        // mi geliyor. SDK'da sicaklik getter'i yok, tek yol bu. Klasor oturuma ozel.
        hrvs_debug_log_ = declare_parameter<bool>("hrvs_debug_log", true);

        {
            const std::string dir = has_parameter("record_dir")
                ? get_parameter("record_dir").as_string()
                : declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
            char ses[16];
            std::snprintf(ses, sizeof(ses), "hrvs_s%04d", record_session_);
            hrvs_debug_dir_ = dir + "/" + ses + "/";
        }
        alt_pub_ = create_publisher<sensor_msgs::msg::Range>(
            declare_parameter<std::string>("altitude_topic", "/hrvs/altitude"),
            rclcpp::SensorDataQoS());
        if (declare_parameter<bool>("hrvs_enabled", true))
            hrvs_thread_ = std::thread(&SerialSenderNode::hrvsThread, this);
        else
            RCLCPP_WARN(get_logger(), "HRVS KAPALI (hrvs_enabled:=false) — hrvs alanlari 0 gider");

        RCLCPP_INFO(get_logger(),
            "SerialSender: tag='%s' img='%s' -> %s @%d baud %.1f Hz | pkt=%zuB | debug=%s",
            tag_topic_.c_str(), image_topic_.c_str(),
            serial_port_.c_str(), baud_rate_, send_rate_hz_,
            sizeof(jetsonOtopilotPack_t), debug_ ? "ON" : "OFF");
    }

    ~SerialSenderNode()
    {
        // Radara 'dur' sinyali HEMEN, ama beklemesi EN SONDA: radar cevap vermezken
        // HRVS thread'i SDK cagrisinda (UART zaman asimlari) 15 s+ takili kalabiliyor
        // ve launch/systemd SIGKILL atiyor (28_09 olculdu). Once join edilirse video
        // kaydi/CSV'ler hic kapanmiyordu. HRVS thread'i bu kaynaklara dokunmaz.
        hrvs_running_ = false;

        tx_running_ = false;                          // once TX thread dur (serial'i o yaziyor)
        if (tx_thread_.joinable()) tx_thread_.join();
        rx_running_ = false;                          // RX poll'u en gec 100 ms'de doner
        if (rx_thread_.joinable()) rx_thread_.join();
        if (rx_csv_.is_open()) rx_csv_.close();       // yazan RX thread'iydi, artik durdu

        vpi_running_ = false;
        frame_cv_.notify_all();
        if (vpi_thread_.joinable()) vpi_thread_.join();

        // VPI objeleri OpticalFlowVPI destructor'inda yok edilir (thread durduktan sonra)
        flow_.reset();

        recorder_.reset();   // son kareyi yazar, encode thread'ini durdurur, dosyayi kapatir

        // CSV: VPI thread durduktan SONRA kapat (yazan o thread'di)
        if (flow_csv_.is_open()) flow_csv_.close();

        // IR CSV: yazan imgCb (ROS callback) thread'iydi; buraya spin dondukten
        // SONRA gelinir, yani callback'ler artik calismiyor - kapatmak guvenli.
        if (ir_csv_.is_open()) ir_csv_.close();

        if (serial_fd_ >= 0)    ::close(serial_fd_);

        // Dosyalar kapandi — artik radari beklemek guvenli (SIGKILL gelse bile kayip yok)
        if (hrvs_thread_.joinable()) hrvs_thread_.join();
    }

private:
    /* -------------------------------------------------------------- */
    /* AprilTag callback - HUNI FUZYONU (LPS ana koddan port)         */
    /*   avgPos = tum huni tag'lerinin (offset'li) outlier'siz ORTALAMASI. */
    /*   POZISYON sadece avgPos'ta gider; her tag'in sadece VALIDITY biti */
    /*   basiliyor (valid1-4=duvar, valid5-8=kucuk). hrvs/flow AYNEN korunur. */
    /* -------------------------------------------------------------- */
    /* Kapi kapaliysa (kisa gorus) sonucu "tespit yok" gibi gonder: valid ve tag bitleri 0.
       Konum alanlari da sifirlanir ki otopilot gecersiz kareden deger okumasin. */
    lps::TagFusion gateFusion(const lps::TagFusion &f, lps::TagGate &g, double t_s,
                              std::atomic<uint64_t> &supp)
    {
        if (!tag_gate_en_) return f;
        const bool pass = g.update(f.valid, t_s);
        supp.store(g.suppressed());   // log TX thread'inden okur
        if (pass) return f;
        lps::TagFusion out;   // valid=false, pos 0, id_bits 0
        return out;
    }

    void tagCb(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        tag_cb_count_++;

        // avgPos fuzyonu tag_fusion.hpp'de (infra2 ile ORTAK). Mod (huni / 45-24)
        // orada TEST_APRITLAG ile.
        const lps::TagFusion f = lps::fuseTags(lps::toTagDets(*msg), apriltag_node_size_);
        const lps::TagFusion fg = gateFusion(f, gate1_, lps::stampSeconds(msg->header.stamp), gate1_supp_);
        apriltagPack_t ap = packFromFusion(fg, msSinceStart());
        subtractCamOffset(ap, cam_off1_);   // -> platform merkezine gore
        if (f.valid && f.pos[2] > 0.1f) {   // IR irtifa kaynagi 'tag' (yer testi)
            tag_z_m_.store(f.pos[2]);
            tag_z_ms_.store(msSinceStart());
        }

        // Video overlay: huni tag'lerinin (duvar+kucuk) kose koordinatlari.
        std::vector<lps::DebugRecorder::TagOverlay> overlays;
        if (recorder_) {
            for (const auto &det : msg->detections) {
		//if (tag_meta_.find(det.id) == tag_meta_.end()) continue;
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
            tag1_cap_s_   = lps::stampSeconds(msg->header.stamp);   // kamera cekim ani
            tag_overlays_ = std::move(overlays);
        }
    }

    /* -------------------------------------------------------------- */
    /* infra2 AprilTag -> aprtiltag2. infra1 ile AYNI fuzyon + esleme; */
    /* zaman damgasi da ayni kural (alindigi an, msSinceStart).        */
    /* -------------------------------------------------------------- */
    void tagCb2(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        tag2_cb_count_++;
        const lps::TagFusion f = lps::fuseTags(lps::toTagDets(*msg), apriltag_node_size_);
        const lps::TagFusion fg = gateFusion(f, gate2_, lps::stampSeconds(msg->header.stamp), gate2_supp_);
        apriltagPack_t ap = packFromFusion(fg, msSinceStart());
        subtractCamOffset(ap, cam_off2_);   // -> platform merkezine gore (aprtiltag2 de)
        std::lock_guard<std::mutex> lk(data_mutex_);
        apriltag2_  = ap;
        tag2_cap_s_ = lps::stampSeconds(msg->header.stamp);
    }

    /* -------------------------------------------------------------- */
    /* infra2 optik akis -> opticflow2. Akis infra2_recorder_node'da   */
    /* (infra1 ile ayni OpticalFlowVPI) hesaplanip Vector3Stamped ile  */
    /* gelir: x=dx, y=dy (px/kare, EMA'li), z=quality (0-100).          */
    /* -------------------------------------------------------------- */
    void flowCb2(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        flow2_cb_count_++;
        opticFlowPack_t of{};
        of.timestamp   = msSinceStart();
        of.motionPos.x = (float)msg->vector.x;
        of.motionPos.y = (float)msg->vector.y;
        of.quality     = (uint8_t)std::clamp(msg->vector.z, 0.0, 100.0);
        std::lock_guard<std::mutex> lk(data_mutex_);
        opticflow2_ = of;
    }

    /* -------------------------------------------------------------- */
    /* Arac bearing'i -> car. car_detector her islenen karede yayinlar: */
    /*   vector.x = X/Z, vector.y = Y/Z, vector.z = kilitli track_id    */
    /*   (z < 0 -> kilit yok / intrinsics yok -> valid=0).              */
    /* -------------------------------------------------------------- */
    void carCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        car_cb_count_++;
        carPack_t c{};
        c.timeStamp = msSinceStart();
        c.valid     = (msg->vector.z >= 0.0);
        if (c.valid) {
            c.bearing.x = (float)msg->vector.x;
            c.bearing.y = (float)msg->vector.y;
        }
        std::lock_guard<std::mutex> lk(data_mutex_);
        car_ = c;
    }

    /* -------------------------------------------------------------- */
    /* CameraInfo callback - IR geri izdusumu icin intrinsics         */
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
    /* IR LED ucgeni - pozisyon (pinhole geri izdusum)                 */
    /*   1) blob tespiti (parlak nokta, yuksek esik)                   */
    /*   2) olcekten bagimsiz ucgen dogrulamasi + optik Z (30 Hz)      */
    /*   3) radar ile kaba capraz kontrol (varsa)                      */
    /*   4) undistort - normalize - Z ile olcekle (kamera optik frame'i,  */
    /*      AprilTag avgPos ile ayni konvansiyon)                       */
    /* -------------------------------------------------------------- */
    /* IR icin irtifa: ir_alt_source'a gore. auto = EKF (otopilot 0x4A) tazeyse o, yoksa
       HRVS. 'tag' sadece yer testi (radar tavan okur). Tazelik ir_radar_stale_ms. */
    struct IrAlt { float z = 0.0f; bool fresh = false; const char *src = "-"; };
    IrAlt irAltitude(uint32_t now_ms)
    {
        auto fresh = [&](uint32_t t) { return t != 0 && now_ms - t <= ir_radar_stale_ms_; };
        IrAlt a;
        const std::string &s = ir_alt_source_;
        if (s == "tag") {
            if (fresh(tag_z_ms_.load())) { a.z = tag_z_m_.load(); a.fresh = true; a.src = "TAG"; }
            return a;
        }
        if (s == "auto" || s == "ekf") {
            if (fresh(ekf_alt_ms_.load())) { a.z = ekf_alt_m_.load(); a.fresh = true; a.src = "EKF"; return a; }
            if (s == "ekf") return a;
        }
        float zr;
        { std::lock_guard<std::mutex> lk(hrvs_mutex_); zr = hrvs_data_.altitude / 100.0f; }
        if (zr > 0.05f && fresh(hrvs_last_ms_.load())) { a.z = zr; a.fresh = true; a.src = "HRVS"; }
        return a;
    }

    void processIr(const cv::Mat &frame)
    {
        cv::Matx33d K; cv::Mat D;
        {
            std::lock_guard<std::mutex> lk(cinfo_mutex_);
            if (!cinfo_ready_) return;
            K = cam_K_; D = cam_D_;
        }

        // Irtifa SADECE capraz kontrol + arama penceresi icin - olcek ucgenin kendisinden.
        // (Radar 10 Hz, kamera 30 Hz: eskiden 3 karenin 2'si bayat Z kullaniyordu.)
        const uint32_t now_ms = msSinceStart();
        const IrAlt alt = irAltitude(now_ms);
        const float Z_alt = alt.fresh ? alt.z : 0.0f;

        // Irtifaya gore LED alan penceresi + beklenen olcek (06_11)
        double a_lo = ir_min_area_, a_hi = ir_noalt_max_area_;
        float  k_min = 0.0f, k_max = std::numeric_limits<float>::max();
        const double f_px = 0.5 * (K(0, 0) + K(1, 1));
        if (alt.fresh) {
            const double A = lps::irLedAreaExpected(Z_alt, ir_area_ref_, ir_area_ref_z_, ir_area_exp_);
            a_hi = std::max(ir_max_area_, A * ir_area_hi_);
            if (Z_alt <= ir_area_model_max_z_) a_lo = std::max(ir_min_area_, A / ir_area_lo_div_);
            k_min = (float)(f_px / (Z_alt * (1.0f + ir_z_tol_)));
            if (ir_z_tol_ < 0.95f) k_max = (float)(f_px / (Z_alt * (1.0f - ir_z_tol_)));
        }

        lps::detectIrBlobs(frame, ir_min_thresh_, a_lo, a_hi,
                      kIrMaxBlobs, ir_bin_, ir_labels_, ir_stats_, ir_centroids_,
                      ir_pts_, ir_areas_,
                      ir_round_min_area_, ir_round_min_fill_, ir_round_max_asp_);

        irPack_t ir{};
        ir.timeStamp = now_ms;
        lps::DebugRecorder::IrOverlay ov;
        ov.n_blobs = (int)ir_pts_.size();
        ov.no_alt  = !alt.fresh;   // capraz kontrol yok
        ov.alt_src = alt.src;
        ov.z_radar = Z_alt;
        ov.area_lo = (int)std::lround(a_lo);
        ov.area_hi = (int)std::lround(a_hi);

        int i_right, i_a, i_b;
        float fit_err = 0.0f, k_px_per_m = 0.0f;
        if (lps::findIrRightTriangle(ir_pts_, ir_leg_a_m_, ir_leg_b_m_, ir_tol_frac_,
                                i_right, i_a, i_b, fit_err, k_px_per_m, k_min, k_max)) {
            // Z ucgenin OLCULEN buyuklugunden: Z = f / k. Kamera hiziyla (30 Hz)
            // guncellenir, bayat degildir ve OPTIK EKSEN derinligidir - pinhole
            // geri izdusumun istedigi buyuklugun ta kendisi (radar dikey verir).
            const float Z_m = (float)f_px / k_px_per_m;
            ov.z       = Z_m;

            // Huni merkezi (06_11): hipotenus ortasi DEGIL, olculen yerlesim (irMarkerCenter).
            bool legs_ok = false;
            const cv::Point2f mid = lps::irMarkerCenter(
                ir_pts_[i_right], ir_pts_[i_a], ir_pts_[i_b],
                std::min(ir_leg_a_m_, ir_leg_b_m_), std::max(ir_leg_a_m_, ir_leg_b_m_),
                ir_center_on_short_m_, ir_center_on_long_m_, ir_leg_ambig_ratio_, legs_ok);
            ov.legs_ok = legs_ok;
            ov.cand   = true;
            ov.pts[0] = ir_pts_[i_right];
            ov.pts[1] = ir_pts_[i_a];
            ov.pts[2] = ir_pts_[i_b];
            ov.mid    = mid;

            // Sahte ucgen kurallari (02_11). LED yontemi GECE icin: gunduz sahnesi 16+
            // parlak nokta uretir, 560 ucluden tesadufi dik ucgen cikar (gunduz
            // ucuslarinda 4718 sahte gecerli kare). Kayitlarla tekrar oynatma:
            // A+B+D sahtelerin %99'unu eler. Esikler gece LED testinde ayarlanacak.
            //   A) kalabalik sahne   B) irtifa yok/bayat (eskiden kontrol ATLANIYORDU)
            //   Z) optik Z irtifayla uyusmuyor   D) konum ardisik karelerde kararsiz
            // 06_11: A, irtifa penceresinden GECEN blob'lari sayar (parlama kiriktilari
            // pencere disinda kalir). Z, irtifa tazeyken secimde zaten saglanir (k_min/k_max).
            const char *red = nullptr;
            if ((int)ir_pts_.size() > ir_max_blobs_valid_)       red = "kalabalik sahne";
            else if (ir_require_radar_ && !alt.fresh)            red = "irtifa yok/bayat";
            else if (Z_alt > 0.05f &&
                     std::fabs(Z_m - Z_alt) / Z_alt > ir_z_tol_) {
                red = "Z irtifayla uyusmuyor";
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "[IR] optik Z=%.2f m %s Z=%.2f m ile uyusmuyor (tol %.0f%%) "
                    "- tespit reddedildi", Z_m, alt.src, Z_alt, ir_z_tol_ * 100.0f);
            }
            if (!red) {
                ir_run_ = (ir_run_ > 0 && cv::norm(mid - ir_last_mid_) <= ir_max_jump_px_)
                          ? ir_run_ + 1 : 1;
                ir_last_mid_ = mid;
                if (ir_run_ < ir_min_consec_) red = "kararsiz";
            } else {
                ir_run_ = 0;
            }
            ov.run    = ir_run_;
            ov.reason = red;

            int amin = INT32_MAX;
            for (int idx : {i_right, i_a, i_b})
                if (idx < (int)ir_areas_.size()) amin = std::min(amin, ir_areas_[idx]);
            ov.min_area = (amin == INT32_MAX) ? 0 : amin;
        } else {
            ir_run_ = 0;
        }

        if (ov.cand && !ov.reason) {
            const cv::Point2f mid = ov.mid;

            std::vector<cv::Point2f> in{mid}, out;
            cv::undistortPoints(in, out, K, D);

            // ---- PAKETE GIDEN: NORMALIZE YON (BOYUTSUZ), METRE DEGIL ----
            // irpos = (X/Z, Y/Z) - huni merkezinin kamera optik frame'indeki
            // YONU. Menzille CARPILMAZ; otopilot kendi (EKF-fuzyonlu) menziliyle
            // carpar: X = irpos.x * Z.
            //
            // Neden burada carpmiyoruz: kamera fiziksel olarak yon olcer, menzil
            // turetilmis buyukluktur. Bizim optik Z'mizin sessiz olcek hatalari
            // var (marker olcusu 1.36 m dogru mu, fx kalibrasyonu, egimde %5-10)
            // ve otopilot bunlari DOGRULAYAMAZ - metrik gonderseydik kaynagini
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

            // ov.min_area (menzil teshisi: eslesen 3 blob'un EN KUCUGU) ve
            // pts[0]=dik aci kosesi / mid=huni merkezi yukarida aday icin dolduruldu.
            ov.valid   = true;
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

    /** IR CSV satiri - acikken HER karede (tespit olsun olmasin) yazilir.
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
        // Koseler ADAY icin de yazilir (reddedilen dahil); son 3 sutun: aday, red
        // sebebi (ok = gecerli, bos = ucgen yok), ardisik kararli kare sayisi.
        if (ov.cand)
            ir_csv_ << ',' << ov.pts[0].x << ',' << ov.pts[0].y
                    << ',' << ov.pts[1].x << ',' << ov.pts[1].y
                    << ',' << ov.pts[2].x << ',' << ov.pts[2].y
                    << ',' << ov.mid.x    << ',' << ov.mid.y
                    << ",1," << (ov.reason ? ov.reason : "ok") << ',' << ov.run << '\n';
        else
            ir_csv_ << ",,,,,,,,0,,0\n";   // eslesme yok -> kose sutunlari bos
        if (++ir_csv_lines_ % 15 == 0) ir_csv_.flush();
    }

    /* -------------------------------------------------------------- */
    /* Image callback - sadece frame'i kuyruga koyar, aninda doner    */
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
            latest_stamp_s_ = lps::stampSeconds(msg->header.stamp);   // aralik istatistigi
            new_frame_    = true;
        }
        frame_cv_.notify_one();

        // IR LED ucgeni - blob tespiti ucuz (~1-2 ms @720p), ayri thread gerekmez.
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
            st.t_ms    = msSinceStart();
            // Karenin kamera zaman damgasi: infra2 test videosuyla ortak saat
            // (iki kamera donanim-senkron) -> videolar kare kare eslesir.
            st.stamp_s = lps::stampSeconds(msg->header.stamp);
            st.label   = record_label_;
            recorder_->push(cv_ptr->image, cv_ptr, st);
        }
    }

    /* -------------------------------------------------------------- */
    /* VPI thread - ayri thread'de calisir, spin'i bloke etmez        */
    /* -------------------------------------------------------------- */
    void vpiThread()
    {
        while (vpi_running_.load()) {
            cv::Mat frame;
            double  stamp_s = 0.0;
            {
                std::unique_lock<std::mutex> lk(frame_mutex_);
                frame_cv_.wait(lk, [this]{
                    return new_frame_ || !vpi_running_.load();
                });
                if (!vpi_running_.load()) break;
                frame      = std::move(latest_frame_);
                stamp_s    = latest_stamp_s_;
                new_frame_ = false;
            }

            // Ilk frame'de VPI objelerini kur (LPS motoru), sonra her frame isle.
            if (!flow_->initialized()) {
                try {
                    flow_->init(frame);
                    RCLCPP_INFO(get_logger(), "VPI OFA init: %dx%d -> padded %dx%d",
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
            gap_stats_->add(stamp_s);   // bu akis kac kamera karesi araligini olctu

            opticFlowPack_t of{};
            of.timestamp   = msSinceStart();
            of.motionPos.x = r.dx;        // EMA'li - pakete giden
            of.motionPos.y = r.dy;
            of.quality     = r.quality;   // kapsama-cezali, EMA'li Q

            // Ham akis CSV'si - EMA ONCESI kare-basi olcum (aciksa).
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
    /* TX thread - RT oncelik + CPU pin + mutlak-deadline (sleep_until). */
    /* ROS executor'dan BAGIMSIZ -> paket zamanlamasi jitter'siz (LPS). */
    /* -------------------------------------------------------------- */
    void txThread()
    {
        {   // RT oncelik (izin yoksa normal scheduler'a duser)
            sched_param sp{};
            sp.sched_priority = 50;
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
                RCLCPP_INFO(get_logger(),
                    "TX thread RT oncelik alinamadi (izin yok) - normal scheduler");
        }
        if (tx_cpu_ >= 0) {   // CPU pin (IRQ firtinasindan izole)
            cpu_set_t cpus; CPU_ZERO(&cpus); CPU_SET(tx_cpu_, &cpus);
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus) != 0)
                RCLCPP_INFO(get_logger(),
                    "TX thread CPU%d'e sabitlenemedi - serbest calisacak", tx_cpu_);
        }
        const auto period = std::chrono::nanoseconds((int64_t)(1e9 / send_rate_hz_));
        auto next = std::chrono::steady_clock::now() + period;
        while (tx_running_.load()) {
            std::this_thread::sleep_until(next);     // mutlak deadline -> jitter yok
            if (!tx_running_.load()) break;
            sendPacket();
            next += period;
            auto now = std::chrono::steady_clock::now();
            if (next < now) next = now;              // geride kaldiysa yakala
        }
    }

    /* -------------------------------------------------------------- */
    /* sendPacket - paket olustur + yaz (eski timerCb, icerik AYNEN;   */
    /* artik ROS wall timer degil, TX thread'inden sabit periyotta cagirilir). */
    /* -------------------------------------------------------------- */
    void sendPacket()
    {
        // Gercek delta T olcumu - write oncesi
        auto now_send = std::chrono::steady_clock::now();
        auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now_send - last_send_tp_).count();
        last_send_tp_ = now_send;

        serial_cb_count_++;

        // Jitter istatistigi guncelle (ilk tick'i atla)
        if (serial_cb_count_ > 1) {
            dt_sum_us_ += dt_us;
            dt_cnt_++;
            if (dt_us < dt_min_us_) dt_min_us_ = dt_us;
            if (dt_us > dt_max_us_) dt_max_us_ = dt_us;
        }

        apriltagPack_t  at_copy, at2_copy;
        double          cap1 = 0, cap2 = 0;   // tag sonuclarinin kamera cekim anlari (s)
        carPack_t       car_copy;
        healthPack_t    health_copy;
        int8_t          cpu_real_copy;
        opticFlowPack_t of_copy, of2_copy;
        irPack_t        ir_copy;
        lps::DebugRecorder::IrOverlay ir_ov_copy;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            at_copy = apriltag_; of_copy = opticflow_; ir_copy = ir_data_;
            at2_copy = apriltag2_; of2_copy = opticflow2_;
            cap1 = tag1_cap_s_; cap2 = tag2_cap_s_;
            car_copy = car_;
            health_copy = health_pack_;
            cpu_real_copy = health_cpu_real_;
            ir_ov_copy = ir_overlay_;
        }

        // ---- infra1 + infra2 birlesimi (03_11, dual_camera_fusion.hpp) ----
        // Pakete giden aprtiltag / opticflow = iki kameranin birlesimi; aprtiltag2 /
        // opticflow2 infra2'nin kendisi olarak kalir (karsilastirma). Bayat kamera
        // (node olu / infra2 kapali) birlesime girmez.
        if (dual_fusion_) {
            const uint32_t now_ms = msSinceStart();
            const auto fresh = [now_ms](uint32_t ts, uint32_t lim) {
                return ts != 0 && now_ms - ts <= lim;
            };
            lps::TagSample s1 = tagSample(at_copy,  fresh(at_copy.timeStamp,  tag_fresh_ms_));
            lps::TagSample s2 = tagSample(at2_copy, fresh(at2_copy.timeStamp, tag_fresh_ms_));
            // Sadece AYNI kareden gelen iki sonuc ortalanir; farkli karelerse yenisi gider.
            if (s1.ok && s2.ok) {
                ++fus_tag_both_;
                if (!lps::keepSameFrame(s1, cap1, s2, cap2, tag_sync_s_)) ++fus_tag_sync_;
            }
            const lps::TagPairResult tr = lps::fuseTagPair(s1, s2, tag_pair_tol_m_);
            if (tr.src) {
                const uint32_t ts = tr.src == 1 ? at_copy.timeStamp : tr.src == 2 ? at2_copy.timeStamp
                                  : std::max(at_copy.timeStamp, at2_copy.timeStamp);
                at_copy = tagPack(tr.out, ts);
            }
            ++fus_tag_cnt_[tr.src];
            if (tr.diff_m >= 0) { fus_tag_diff_sum_ += tr.diff_m; ++fus_tag_diff_n_; }

            const auto flowSample = [&fresh, this](const opticFlowPack_t &f) {
                lps::FlowSample s;
                s.ok = fresh(f.timestamp, flow_fresh_ms_);
                s.dx = f.motionPos.x; s.dy = f.motionPos.y; s.q = f.quality;
                return s;
            };
            const lps::FlowPairResult fr = lps::fuseFlowPair(
                flowSample(of_copy), flowSample(of2_copy), flow_agree_px_, flow_agree_frac_);
            if (fr.src) {
                of_copy.timestamp   = fr.src == 1 ? of_copy.timestamp : fr.src == 2 ? of2_copy.timestamp
                                    : std::max(of_copy.timestamp, of2_copy.timestamp);
                of_copy.motionPos.x = fr.out.dx;
                of_copy.motionPos.y = fr.out.dy;
                of_copy.quality     = fr.out.q;
            }
            ++fus_flow_cnt_[fr.src];
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

        // HRVS - timestamp + frame sayaci
        pkt.pack.variables.hrvs.timeStamp   = hrvs_data_.timeStamp;//msSinceStart();
        pkt.pack.variables.hrvs.frameNumber = hrvs_data_.frameNumber;
        // HRVS - radar thread'inden gelen altitude ve body velocity
        {
            std::lock_guard<std::mutex> lk(hrvs_mutex_);
            pkt.pack.variables.hrvs.altitude = hrvs_data_.altitude;
            pkt.pack.variables.hrvs.body_x   = hrvs_data_.body_x;
            pkt.pack.variables.hrvs.body_y   = hrvs_data_.body_y;
            pkt.pack.variables.hrvs.body_z   = hrvs_data_.body_z;
        }

        // AprilTag + OpticFlow + IR
        pkt.pack.variables.aprtiltag  = at_copy;
        pkt.pack.variables.opticflow  = of_copy;
        pkt.pack.variables.ir         = ir_copy;
        // infra2 (paketin sonu)
        pkt.pack.variables.aprtiltag2 = at2_copy;
        pkt.pack.variables.opticflow2 = of2_copy;
        // Arac bearing'i (paketin sonu). Bayatsa gecersiz: yon sifirlanir, timeStamp
        // SON mesajin zamani olarak kalir (otopilot yasini gorebilsin).
        if (car_copy.valid && msSinceStart() - car_copy.timeStamp > car_timeout_ms_) {
            car_copy.valid   = false;
            car_copy.bearing = float_2D_t{0.0f, 0.0f};
        }
        pkt.pack.variables.car        = car_copy;

        // Checksum (variables alani uzerinden)
        uint32_t csum = 0;
        const int payStart = (int)sizeof(header_t);
        const int payEnd   = payStart + (int)sizeof(jetsonVariables_t);
        for (int i = payStart; i < payEnd; ++i) csum += pkt.data[i];
        pkt.pack.chksum = static_cast<uint8_t>(~(csum % 256) + 1);

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[TIMER-ALIVE] frame=%u tag_cb=%u flow_cb=%u",
            serial_cb_count_, tag_cb_count_, flow_cb_count_);

        // Paketi TAMAMI cikana kadar yaz (bkz. writeAll).
        // 133B @115200 (8N1) = 11.5 ms telde; 50 Hz periyot 20 ms -> hat dolulugu
        // %58. Saglik paketinin oldugu tick'te +31 B (2.7 ms) = 14.2 ms, yine sigar.
        // send_rate_hz ~86 Hz'i gecerse hat YETMEZ.
        writeAll(pkt.data, sizeof(jetsonOtopilotPack_t), 20, "ana paket");

        // Sistem sagligi: yeni ornek geldiyse (~1 Hz) ana paketin HEMEN ARKASINDAN,
        // ayni thread'den (seri portun tek yazari -> baytlar karismaz).
        if (health_copy.valid && health_copy.timeStamp != health_sent_ts_) {
            health_sent_ts_ = health_copy.timeStamp;
            jetsonHealthPack_ut hp;
            std::memset(hp.data, 0, sizeof(jetsonHealthPack_t));
            hp.pack.header.sync_byte   = SYNC_BYTE;
            hp.pack.header.msg_id      = MSG_ID_HEALTH;
            hp.pack.header.payloadSize = static_cast<uint8_t>(sizeof(healthPack_t));
            uint32_t hs = 0;
            for (int i = 0; i < 3; ++i) hs += hp.data[i];
            hp.pack.header.headerchk = static_cast<uint8_t>(~(hs % 256) + 1);
            hp.pack.health = health_copy;
            uint32_t cs = 0;
            for (size_t i = sizeof(header_t); i < sizeof(header_t) + sizeof(healthPack_t); ++i)
                cs += hp.data[i];
            hp.pack.chksum = static_cast<uint8_t>(~(cs % 256) + 1);
            writeAll(hp.data, sizeof(jetsonHealthPack_t), 6, "saglik paketi");
        }

        // FPS / debug guncellemesi - her 1 saniyede bir
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_last_tp_).count();
        if (elapsed >= 1.0) {
            tag_fps_   = tag_cb_count_    / elapsed;
            flow_fps_  = flow_cb_count_   / elapsed;
            serial_hz_ = serial_cb_count_ / elapsed;
            tag_cb_count_ = flow_cb_count_ = serial_cb_count_ = 0;
            const double tag2_hz  = tag2_cb_count_.exchange(0)  / elapsed;   // infra2
            const double flow2_hz = flow2_cb_count_.exchange(0) / elapsed;
            const double car_hz   = car_cb_count_.exchange(0)   / elapsed;   // arac bearing
            fps_last_tp_ = now;

            double dt_avg_ms = (dt_cnt_ > 0) ? (dt_sum_us_ / dt_cnt_ / 1000.0) : 0.0;
            double dt_min_ms = dt_min_us_ / 1000.0;
            double dt_max_ms = dt_max_us_ / 1000.0;
            double jitter_ms = dt_max_ms - dt_min_ms;

            // Sayaclari sifirla
            dt_sum_us_ = 0; dt_cnt_ = 0;
            dt_min_us_ = INT64_MAX; dt_max_us_ = 0;

            if (debug_) {
                RCLCPP_INFO(get_logger(),
                    "[SERIAL]    %.1f Hz | dt: avg=%.2fms min=%.2fms max=%.2fms jitter=%.2fms | pkt=%zuB | frame=%u | rx=%uB",
                    serial_hz_, dt_avg_ms, dt_min_ms, dt_max_ms, jitter_ms,
                    sizeof(jetsonOtopilotPack_t),
                    pkt.pack.variables.hrvs.frameNumber, rx_bytes_.exchange(0));

                // Tek tek tag pozu artik gonderilmiyor; sadece validity bit'leri.
                //   duvar valid1-4 = 0/20/24/34, kucuk valid5-8 = 10/12/14/16.
                RCLCPP_INFO(get_logger(),
                    "[APRILTAG]  %.1f Hz | avg: valid=%s pos(%.3f,%.3f,%.3f) rpy(%.2f,%.2f,%.2f)"
                    " | duvar[N%d E%d S%d W%d] kucuk[%d%d%d%d] | kapi bastirdi i1=%llu i2=%llu",
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
                    apriltag_.avgPos.validity.valid8,
                    (unsigned long long)gate1_supp_.load(), (unsigned long long)gate2_supp_.load());

                // aralik: duzenli = hic kare atlanmadan olculen akis orani,
                // atlanan = kameranin urettigi ama akisa girmeyen kare orani.
                int reg = -1, skip = -1;
                gap_stats_->take(reg, skip);
                RCLCPP_INFO(get_logger(),
                    "[OPTICFLOW] %.1f Hz | dx=%.3f dy=%.3f quality=%u | aralik duzenli %%%d atlanan %%%d",
                    flow_fps_,
                    opticflow_.motionPos.x,
                    opticflow_.motionPos.y,
                    opticflow_.quality, reg, skip);

                // infra2 — pakete GIDEN kopyalar (at2_copy/of2_copy) basilir.
                // Hz = serial_sender'a ulasan mesaj hizi (0 ise infra2 kapali/olu).
                RCLCPP_INFO(get_logger(),
                    "[APRILTAG2] %.1f Hz | avg: valid=%s pos(%.3f,%.3f,%.3f) rpy(%.2f,%.2f,%.2f)",
                    tag2_hz, at2_copy.avgPos.validity.valid ? "Y" : "N",
                    at2_copy.avgPos.pos.x, at2_copy.avgPos.pos.y, at2_copy.avgPos.pos.z,
                    at2_copy.avgPos.angle.roll, at2_copy.avgPos.angle.pitch,
                    at2_copy.avgPos.angle.yaw);
                RCLCPP_INFO(get_logger(),
                    "[OPTICFLOW2] %.1f Hz | dx=%.3f dy=%.3f quality=%u",
                    flow2_hz, of2_copy.motionPos.x, of2_copy.motionPos.y, of2_copy.quality);
                // Pakete GIDEN birlesik deger + son 1 s'de kaynak dagilimi (TX tick'i basina).
                // [APRILTAG]/[APRILTAG2] kamera basina (ofset dusulmus), [OPTICFLOW]/2 ham.
                if (dual_fusion_) {
                    const auto pct = [](const uint32_t *c, int i) {
                        const uint32_t s = c[0] + c[1] + c[2] + c[3];
                        return s ? (int)std::lround(100.0 * c[i] / s) : 0;
                    };
                    const auto &ft = pkt.pack.variables.aprtiltag.avgPos;
                    const auto &ff = pkt.pack.variables.opticflow;
                    RCLCPP_INFO(get_logger(),
                        "[FUSION]    tag: ikisi %%%d infra1 %%%d infra2 %%%d yok %%%d fark %.3f m esli kare %u/%u"
                        " | akis: ikisi %%%d infra1 %%%d infra2 %%%d yok %%%d"
                        " | pakette pos(%.3f,%.3f,%.3f) akis(%.3f,%.3f) Q%u",
                        pct(fus_tag_cnt_, 3), pct(fus_tag_cnt_, 1), pct(fus_tag_cnt_, 2), pct(fus_tag_cnt_, 0),
                        fus_tag_diff_n_ ? fus_tag_diff_sum_ / fus_tag_diff_n_ : -1.0,
                        fus_tag_sync_, fus_tag_both_,
                        pct(fus_flow_cnt_, 3), pct(fus_flow_cnt_, 1), pct(fus_flow_cnt_, 2), pct(fus_flow_cnt_, 0),
                        ft.pos.x, ft.pos.y, ft.pos.z, ff.motionPos.x, ff.motionPos.y, ff.quality);
                    for (int i = 0; i < 4; ++i) fus_tag_cnt_[i] = fus_flow_cnt_[i] = 0;
                    fus_tag_diff_sum_ = 0; fus_tag_diff_n_ = 0;
                    fus_tag_both_ = fus_tag_sync_ = 0;
                }
                // Arac: pakete GIDEN kopya (bayatlik kontrolu uygulanmis hali)
                RCLCPP_INFO(get_logger(),
                    "[CAR]       %.1f Hz | valid=%s bearing(%+.4f,%+.4f)",
                    car_hz, car_copy.valid ? "Y" : "N",
                    car_copy.bearing.x, car_copy.bearing.y);
                // Sistem sagligi: pakete GIDEN kopya
                RCLCPP_INFO(get_logger(),
                    "[HEALTH]    up=%us | cpu %u%% (max cekirdek %u%%) gpu %u%% | "
                    "sicaklik cpu %d gpu %d tj %d C | ram %u%% swap %u%% disk %u%% | "
                    "fan %u rpm | giris %.2f V %.2f W | oc %u%s",
                    health_copy.uptime, health_copy.cpuLoad, health_copy.cpuCoreMax,
                    health_copy.gpuLoad, cpu_real_copy, health_copy.gpuTemp,
                    health_copy.tjTemp, health_copy.ramUsed, health_copy.swapUsed,
                    health_copy.diskUsed, health_copy.fanRpm,
                    health_copy.inVoltage / 1000.0, health_copy.inPower / 1000.0,
                    health_copy.ocEvents,
                    cpu_temp_from_radar_
                        ? (" | pakette cpuTemp=RADAR " + std::to_string(health_copy.cpuTemp) + " C").c_str()
                        : "");

                if (ir_enabled_) {
                    // Pakete NORMALIZE yon gidiyor (boyutsuz) - metreye otopilot
                    // cevirir. Koseli parantezteki metrik deger sadece gozle
                    // kontrol icin, Zopt ile carpilmis hali (gonderilmiyor).
                    // Zopt = ucgenden olculen menzil (dogrulamada kullanilan),
                    // Zrad = ayni andaki radar - sapma sahada hemen gorunsun.
                    // amin: eslesen en kucuk blob alani (menzil sinir gostergesi).
                    RCLCPP_INFO(get_logger(),
                        "[IR]        valid=%s yon(%+.4f,%+.4f) [Zopt=%.2f -> "
                        "%+.2f,%+.2f m] Z%s=%.2f | blob=%d amin=%dpx fit_err=%.2f LED %d-%dpx%s%s",
                        ir_copy.valid ? "Y" : "N",
                        ir_copy.irpos.x, ir_copy.irpos.y, ir_ov_copy.z,
                        ir_copy.irpos.x * ir_ov_copy.z,
                        ir_copy.irpos.y * ir_ov_copy.z,
                        ir_ov_copy.alt_src, ir_ov_copy.z_radar,
                        ir_ov_copy.n_blobs, ir_ov_copy.min_area, ir_ov_copy.fit_err,
                        ir_ov_copy.area_lo, ir_ov_copy.area_hi,
                        ir_ov_copy.no_alt ? " [irtifa yok]" : "",
                        ir_ov_copy.reason ? (std::string(" RED:") + ir_ov_copy.reason).c_str() : "");
                }
            }
        }
    }

    /* -------------------------------------------------------------- */
    /* Paketi TAMAMI cikana kadar yaz (LPS ana koddaki duzeltme).      */
    /* O_NONBLOCK write() ya hep ya hic DEGIL: tampon dolu ise paketin  */
    /* bir kismini yazip gerisini reddeder - 'yarim paketi drop et'     */
    /* stratejisi o baytlar telde oldugu icin kesik paket birakiyordu.  */
    /* Cozum: yer kalmadiysa poll(POLLOUT) ile bosalmayi bekle.         */
    /* -------------------------------------------------------------- */
    bool writeAll(const uint8_t *buf, size_t len, int budget_ms, const char *what)
    {
        size_t off = 0;
        const auto t0 = std::chrono::steady_clock::now();
        while (off < len) {
            ssize_t n = ::write(serial_fd_, buf + off, len - off);
            if (n > 0) { off += (size_t)n; continue; }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
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
                "Serial write eksik, %s (%zu/%zu): %s", what, off, len, strerror(errno));
            return false;
        }
        return true;
    }

    /* -------------------------------------------------------------- */
    /* RX thread - otopilot -> Jetson mesajlari (02_11: 0x49 KAPAT).   */
    /* Ayni fd'yi TX thread'i yaziyor; okuma/yazma ayri thread'den     */
    /* tty'de guvenli. Cerceve kurali TX ile ayni (autopilot_rx.hpp).  */
    /* -------------------------------------------------------------- */
    void rxThread()
    {
        lps::AutopilotRxParser parser(
            [this](uint8_t id, const std::vector<uint8_t> &p) { onAutopilotMsg(id, p); });
        uint8_t buf[256];
        uint64_t s_hdr = 0, s_pay = 0, s_skip = 0, s_bytes = 0;   // ozet icin onceki degerler
        uint64_t bytes_total = 0;
        auto last_sum = std::chrono::steady_clock::now();
        while (rx_running_) {
            if (rx_csv_.is_open() &&
                std::chrono::steady_clock::now() - last_sum >= std::chrono::seconds(1)) {
                last_sum = std::chrono::steady_clock::now();
                rx_csv_ << msSinceStart() << ",S,,," << rx_n4a_ << ',' << rx_n4a_ok_ << ','
                        << rx_n42_ << ',' << rx_n49_ << ',' << rx_nother_ << ','
                        << parser.badHeader() - s_hdr << ',' << parser.badPayload() - s_pay << ','
                        << parser.skipped() - s_skip << ',' << bytes_total - s_bytes << '\n';
                rx_csv_.flush();   // guc kesilirse en fazla ~1 s kaybolsun
                s_hdr = parser.badHeader(); s_pay = parser.badPayload();
                s_skip = parser.skipped(); s_bytes = bytes_total;
                rx_n4a_ = rx_n4a_ok_ = rx_n42_ = rx_n49_ = rx_nother_ = 0;
            }
            struct pollfd p{serial_fd_, POLLIN, 0};
            const int r = ::poll(&p, 1, 100);
            if (r == 0) { parser.idle(); continue; }   // 100 ms sessiz: yarim cerceve gelmeyecek
            if (r < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                    "[SERIAL-RX] poll hatasi: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            const ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
            if (n > 0) {
                rx_bytes_ += (uint32_t)n;
                bytes_total += (uint64_t)n;
                parser.feed(buf, (size_t)n);
            } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                    "[SERIAL-RX] read hatasi: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void onAutopilotMsg(uint8_t id, const std::vector<uint8_t> &p)
    {
        // RX CSV ozet sayaclari (sadece RX thread'i)
        if (id == lps::AP_MSG_EKF_ALT) ++rx_n4a_;
        else if (id == 0x42) ++rx_n42_;
        else if (id == lps::AP_MSG_POWEROFF) ++rx_n49_;
        else ++rx_nother_;
        if (id == lps::AP_MSG_EKF_ALT) { onEkfAlt(p); return; }
        if (id != lps::AP_MSG_POWEROFF) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "[SERIAL-RX] otopilot mesaji id=0x%02X %zu B (islenmiyor)", id, p.size());
            return;
        }
        if (p.size() != 1 || p[0] != 1) {
            RCLCPP_WARN(get_logger(), "[SERIAL-RX] KAPAT mesaji ama yuk %zu B, ilk bayt %d -> yok sayildi",
                        p.size(), p.empty() ? -1 : (int)p[0]);
            return;
        }
        if (!poweroff_enable_) {
            RCLCPP_WARN(get_logger(), "[SERIAL-RX] KAPAT geldi ama autopilot_poweroff_enable=false -> yok sayildi");
            return;
        }
        if (poweroff_requested_.exchange(true)) return;   // tekrarlanan KAPAT: komut bir kez
        RCLCPP_FATAL(get_logger(), "[SERIAL-RX] OTOPILOT KAPAT ISTEDI -> '%s'", poweroff_cmd_.c_str());
        const int rc = std::system(poweroff_cmd_.c_str());
        if (rc != 0) {
            RCLCPP_ERROR(get_logger(), "[SERIAL-RX] kapatma komutu basarisiz (rc=%d), sonraki KAPAT'ta tekrar", rc);
            poweroff_requested_ = false;
        }
    }

    /* 0x4A: EKF yuksekligi -> /autopilot/ekf_altitude (sadece gecerli=1). Gecersizken
       yayin kesilir -> car_detector bayat sayip HRVS'e duser. */
    void onEkfAlt(const std::vector<uint8_t> &p)
    {
        float z = 0; bool valid = false;
        const bool ok = lps::decodeEkfAlt(p, z, valid);
        if (rx_csv_.is_open()) {
            rx_csv_ << msSinceStart() << ",E," << (ok ? z : 0.0f) << ',' << (ok ? (valid ? 1 : 0) : -1)
                    << ",,,,,,,,,\n";
        }
        if (ok && valid) ++rx_n4a_ok_;
        if (!ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[SERIAL-RX] EKF yukseklik mesaji bozuk (yuk %zu B, 5 bekleniyordu ya da NaN) -> yok sayildi",
                p.size());
            return;
        }
        ++ekf_alt_frames_;
        if (!valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[SERIAL-RX] EKF yuksekligi GECERSIZ (gecerli=0, z=%.2f m) -> yayinlanmiyor, car_detector HRVS kullanir", z);
            return;
        }
        if (z > 0.1f) {                    // IR irtifa kaynagi (auto/ekf)
            ekf_alt_m_.store(z);
            ekf_alt_ms_.store(msSinceStart());
        }
        sensor_msgs::msg::Range m;
        m.header.stamp    = now();
        m.header.frame_id = "ekf";
        m.min_range = 0.0f; m.max_range = 1000.0f;
        m.range     = z;
        ekf_alt_pub_->publish(m);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "[SERIAL-RX] EKF yuksekligi %.2f m (toplam %llu mesaj)", z, (unsigned long long)ekf_alt_frames_);
    }

    /* -------------------------------------------------------------- */
    /* HRVS thread - radar olcum dongusu                              */
    /* -------------------------------------------------------------- */
    void hrvsThread()
    {
        if (!set_pad_output_ph07())
            RCLCPP_WARN(get_logger(),
                "HRVS: PH.07 pad fix basarisiz (root + /dev/mem gerekli)");

        UARTLinux    uart;
        GPIOPinLinux gpioPin;
        TimeSteady   timeSource;      // monotonik (bkz. TimeSteady)
        ThreadStdlib currentThread;
        if (hrvs_debug_log_) {
            std::error_code ec;
            std::filesystem::create_directories(hrvs_debug_dir_, ec);
            RCLCPP_INFO(get_logger(), "HRVS SDK debug logu (radar sicakliklari dahil): %shrvsSdkLogs_N/",
                        hrvs_debug_dir_.c_str());
        }
        RadarSystem  radarSystem(hrvs_debug_log_, hrvs_debug_log_ ? hrvs_debug_dir_.c_str() : nullptr);
        hrvsProgress();

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
        {   // bekci (healthTick) radar takilirsa bu pinden reset darbesi atar
            std::lock_guard<std::mutex> lk(hrvs_gpio_mutex_);
            hrvs_gpio_ = &gpioPin;
        }
        // Thread hangi yoldan cikarsa ciksin bekci yok olan pine dokunmasin
        struct GpioGuard {
            SerialSenderNode *n;
            ~GpioGuard() { std::lock_guard<std::mutex> lk(n->hrvs_gpio_mutex_); n->hrvs_gpio_ = nullptr; }
        } gpio_guard{this};

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

        // Konfigurasyondan ONCE radari reset'le: onceki surec radari olcum modunda
        // birakiyor (closeRadarSystem radara komut gondermiyor) ve reset'siz acilista
        // hrvsConfiguration bazen zaman asimi OLMADAN sonsuza dek bekliyordu
        // (20_10 #14/#15; elle GPIO reset'iyle hemen duzeldi). Basarisizsa vazgecme.
        for (int attempt = 1; hrvs_running_.load(); ++attempt) {
            RCLCPP_INFO(get_logger(), "HRVS: reset darbesi + konfigurasyon (deneme %d)", attempt);
            gpioPin.writePinState(IGPIOPin::State::ONE);
            std::this_thread::sleep_for(std::chrono::milliseconds(HRVS_RESET_HOLD_MS));
            gpioPin.writePinState(IGPIOPin::State::ZERO);
            std::this_thread::sleep_for(std::chrono::milliseconds(HRVS_BOOT_WAIT_MS));
            hrvsProgress();
            rc = radarSystem.hrvsConfiguration();
            hrvsProgress();
            if (rc == RadarSystem::ReturnCode::OK) break;
            RCLCPP_ERROR(get_logger(), "HRVS: hrvsConfiguration basarisiz (kod %u), tekrar denenecek",
                         unsigned(rc));
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        if (!hrvs_running_.load()) { radarSystem.closeRadarSystem(); return; }
        RCLCPP_INFO(get_logger(), "HRVS: radar sistemi hazir (%u radar)", HRVS_RADARS_NUMBER);

        auto startTime = std::chrono::steady_clock::now();
        unsigned sysFrame = 0;
        float bodyVel[3]{};
        float avgRange = 0.0f;

        int consecutive_failures = 0;
        while (hrvs_running_.load()) {
            hrvsProgress();
            rc = radarSystem.runMeasurement();
            hrvsProgress();
            if (rc == RadarSystem::ReturnCode::OK) {
                consecutive_failures = 0;
                radarSystem.getBodyVelocity(sysFrame, bodyVel);
                radarSystem.getAvgRange(sysFrame, avgRange);

                auto ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startTime).count();

                // Irtifa -> car_detector boy filtresi. SADECE yeni radar karesinde: radar
                // donarsa mesaj kesilir ve car_detector filtreyi kendiliginden kapatir.
                sensor_msgs::msg::Range am;
                am.header.stamp    = now();
                am.header.frame_id = "hrvs";
                am.min_range = 0.0f; am.max_range = 100.0f;
                am.range     = avgRange;
                alt_pub_->publish(am);
                hrvs_last_ms_.store(msSinceStart());   // IR kurali B: radar tazeligi

                std::lock_guard<std::mutex> lk(hrvs_mutex_);
                hrvs_data_.timeStamp   = static_cast<uint32_t>(ts_ms);
                hrvs_data_.altitude    = static_cast<uint16_t>(avgRange * 100.0f);
                hrvs_data_.frameNumber = sysFrame;
                hrvs_data_.body_x      = static_cast<int16_t>(bodyVel[0] * 100.0f);
                hrvs_data_.body_y      = static_cast<int16_t>(bodyVel[1] * 100.0f);
                hrvs_data_.body_z      = static_cast<int16_t>(bodyVel[2] * 100.0f);
                ++sysFrame;

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[HRVS] frame=%u alt=%.3fm vx=%.3f vy=%.3f vz=%.3f m/s",
                    sysFrame, avgRange,
                    bodyVel[0], bodyVel[1], bodyVel[2]);
            } else {
                ++consecutive_failures;
                // SDK internal reset sonrasi donanima toparlanma suresi ver
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // Kapanis istendiyse saniyeler suren yeniden konfigurasyona GIRME
                if (!hrvs_running_.load()) break;
                // SDK reset yapip CONFIGURATIONS_SET'e donunce hrvsConfiguration tekrar gerekli
                if (consecutive_failures % 3 == 0) {
                    RCLCPP_WARN(get_logger(),
                        "[HRVS] %d ardisik hata - hrvsConfiguration yeniden deneniyor...",
                        consecutive_failures);
                    hrvsProgress();
                    auto rrc = radarSystem.hrvsConfiguration();
                    hrvsProgress();
                    if (rrc != RadarSystem::ReturnCode::OK) {
                        RCLCPP_ERROR(get_logger(),
                            "[HRVS] hrvsConfiguration basarisiz (kod %u)", unsigned(rrc));
                    }
                }
            }
        }
        radarSystem.closeRadarSystem();   // SDK loglarini bosalt + kaynaklari birak
    }

    /* Radar cip sicakliklari (10 s'de bir log satiri): SDK debug logunun en yeni
       hrvsSdkLogs_N/min_range.csv'sinin son ~4 KB'si okunur, her radarin SON satiri
       ozetlenir. Sutunlar loggingConfig.h MINRANGE_CSV_HEADER: [2]=radarID,
       [12..14]=RX0-2, [15..17]=TX0-2, [18]=PM (birimi belirsiz, ~1690), [19..20]=DIG0-1. */
    void hrvsReadTemps(bool log)
    {
        namespace fs = std::filesystem;
        std::error_code ec;
        std::string newest;
        for (const auto &e : fs::directory_iterator(hrvs_debug_dir_, ec)) {
            const std::string n = e.path().filename().string();
            if (n.rfind("hrvsSdkLogs_", 0) == 0 && n > newest) newest = n;
        }
        if (newest.empty()) return;
        std::ifstream f(hrvs_debug_dir_ + newest + "/min_range.csv", std::ios::binary);
        if (!f) return;
        f.seekg(0, std::ios::end);
        const std::streamoff size = f.tellg();
        const std::streamoff from = size > 4096 ? size - 4096 : 0;
        f.seekg(from);
        std::string line;
        if (from > 0) std::getline(f, line);              // yarim ilk satiri at
        std::map<int, std::vector<double>> last;          // radarID -> sutunlar
        while (std::getline(f, line)) {
            std::vector<double> v;
            std::stringstream ss(line);
            std::string tok;
            while (std::getline(ss, tok, ',')) {
                char *end = nullptr;
                v.push_back(std::strtod(tok.c_str(), &end));
                if (end == tok.c_str()) { v.clear(); break; }   // baslik / bozuk satir
            }
            if (v.size() == 21) last[(int)v[2]] = v;
        }
        if (last.empty()) return;
        // Tazelik: en yeni satirin sistem frame numarasi degistiyse radar olcuyor demek
        double frame = -1;
        for (const auto &kv : last) frame = std::max(frame, kv.second[0]);
        if (frame != hrvs_temp_frame_) {
            hrvs_temp_frame_ = frame;
            hrvs_temp_change_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        }
        std::string out;
        double mx = -1e9;
        for (const auto &[id, v] : last) {
            const double rx = std::max({v[12], v[13], v[14]});
            const double tx = std::max({v[15], v[16], v[17]});
            const double dg = std::max(v[19], v[20]);
            mx = std::max({mx, rx, tx, dg});
            char b[96];
            std::snprintf(b, sizeof(b), "r%d RX%.0f TX%.0f DIG%.0f PM%.0f | ", id, rx, tx, dg, v[18]);
            out += b;
        }
        hrvs_temp_max_ = mx;
        if (log) RCLCPP_INFO(get_logger(), "[HRVS-SICAKLIK] %smax %.0f C", out.c_str(), mx);
    }

    /* NOT (22_10): SDK TRACE'ini konsoldan kapatmak icin el::Loggers::reconfigureAllLoggers
       cagriliyordu (hrvsQuietTrace). SDK'nin AsyncLogWriter thread'i ayni anda yazarken
       logger'i yeniden kurmak YARIS -> SIGSEGV: oturum #31-#35'te serial_sender acilista
       1-51 kez coktu (respawn kurtardi, #35'te ~2.5 dk paket kesik). Kaldirildi; easylogging
       ayarina DOKUNMA. TRACE syslog'u sisiriyorsa rsyslog filtresiyle kes. */

    /* HRVS ilerleme damgasi: her SDK cagrisinin oncesi/sonrasi (bekci icin) */
    void hrvsProgress()
    {
        hrvs_progress_ms_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    /* Radar bekcisi (1 Hz, health zamanlayicisi): SDK cagrisi HRVS_STALL_MS'den uzun
       ilerlemezse radari reset'le. Darbe iki tick'e bolunur (1'e cek, sonraki
       saniye 0'a birak) -> executor bloklanmaz. 20_10'da hrvsConfiguration zaman
       asimi olmadan dakikalarca beklemisti; reset radari ve SDK'yi kurtardi. */
    void hrvsWatchdogTick()
    {
        std::lock_guard<std::mutex> lk(hrvs_gpio_mutex_);
        if (hrvs_wd_pulse_high_) {                    // onceki tick'te 1'e cekildi
            if (hrvs_gpio_) hrvs_gpio_->writePinState(IGPIOPin::State::ZERO);
            hrvs_wd_pulse_high_ = false;
            return;
        }
        if (!hrvs_gpio_ || !hrvs_running_.load()) return;
        const int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const int64_t stall = now - hrvs_progress_ms_.load();
        if (stall < HRVS_STALL_MS || now - hrvs_wd_last_ms_ < HRVS_WD_REPEAT_MS) return;
        RCLCPP_ERROR(get_logger(),
            "[HRVS] radar SDK %.0f s'dir ilerlemiyor -> radar reset darbesi (bekci)",
            stall / 1000.0);
        hrvs_wd_last_ms_ = now;
        hrvs_wd_count_++;
        hrvs_gpio_->writePinState(IGPIOPin::State::ONE);
        hrvs_wd_pulse_high_ = true;
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

    /* subscriptions */
    rclcpp::Subscription<
        isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    /* TX thread - sabit periyotlu, RT oncelikli seri gonderim (ROS timer YERINE) */
    std::thread        tx_thread_;
    std::atomic<bool>  tx_running_{true};
    int                tx_cpu_ = 5;   // -1 = CPU pin kapali

    /* RX thread - otopilot -> Jetson (0x49 KAPAT) */
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr alt_pub_;   // HRVS irtifasi (yeni karede)
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ekf_alt_pub_;   // otopilot 0x4A (RX thread)
    std::ofstream rx_csv_;   // otopilot RX kaydi (sadece RX thread'i yazar; dtor RX join'den sonra kapatir)
    uint32_t rx_n4a_ = 0, rx_n4a_ok_ = 0, rx_n42_ = 0, rx_n49_ = 0, rx_nother_ = 0;   // RX thread
    uint64_t ekf_alt_frames_ = 0;   // sadece RX thread'i yazar/okur

    std::thread           rx_thread_;
    std::atomic<bool>     rx_running_{true};
    std::atomic<uint32_t> rx_bytes_{0};          // [SERIAL] logunda saniyelik
    std::atomic<bool>     poweroff_requested_{false};
    bool                  poweroff_enable_ = true;
    std::string           poweroff_cmd_;

    /* sensor verisi + mutex (spin thread <-> vpi thread) */
    std::mutex      data_mutex_;
    apriltagPack_t  apriltag_{};
    opticFlowPack_t opticflow_{};
    apriltagPack_t  apriltag2_{};    // infra2 (tagCb2 yazar, TX okur)
    opticFlowPack_t opticflow2_{};   // infra2 (flowCb2 yazar, TX okur)

    /* infra2 abonelikleri (-> aprtiltag2 / opticflow2) */
    std::string infra2_tag_topic_, infra2_flow_topic_;
    rclcpp::Subscription<
        isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag2_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr flow2_sub_;
    std::atomic<uint32_t> tag2_cb_count_{0}, flow2_cb_count_{0};

    /* iki kamera birlesimi (03_11) — parametreler + TX thread'inin saniyelik sayaclari */
    double   cam_off1_[2] = {0.032962, 0.045}, cam_off2_[2] = {-0.017962, 0.045};
    bool     dual_fusion_ = true;
    float    tag_pair_tol_m_ = 0.30f, flow_agree_px_ = 1.0f, flow_agree_frac_ = 0.3f;
    uint32_t tag_fresh_ms_ = 200, flow_fresh_ms_ = 100;
    // Tag gecerlilik kapisi (08_11, tag_gate.hpp): gate1_ tagCb, gate2_ tagCb2 (ayri callback'ler)
    bool          tag_gate_en_ = true;
    lps::TagGate  gate1_, gate2_;
    std::atomic<uint64_t> gate1_supp_{0}, gate2_supp_{0};   // bastirilan tespit (toplam)
    uint32_t fus_tag_cnt_[4] = {0, 0, 0, 0}, fus_flow_cnt_[4] = {0, 0, 0, 0};   // [yok,1,2,ikisi]
    double   fus_tag_diff_sum_ = 0; uint32_t fus_tag_diff_n_ = 0;
    uint32_t fus_tag_both_ = 0, fus_tag_sync_ = 0;   // ikisi gecerli / bunlarin ayni kareden olani
    double   tag_sync_s_ = 0.010;
    double   tag1_cap_s_ = 0, tag2_cap_s_ = 0;       // son tag sonuclarinin cekim ani (data_mutex_)

    /* Arac bearing'i (car_detector -> car) */
    carPack_t   car_{};                 // carCb yazar, TX okur (data_mutex_)
    std::string car_topic_;
    uint32_t    car_timeout_ms_ = 500;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr car_sub_;
    // Sistem sagligi (msg 0x48): 1 Hz zamanlayici doldurur, TX thread'i gonderir
    std::unique_ptr<lps::SystemHealth> health_;
    rclcpp::TimerBase::SharedPtr       health_timer_;
    healthPack_t                       health_pack_{};   // data_mutex_ altinda
    uint32_t                           health_sent_ts_ = 0;   // yalniz TX thread'i
    std::atomic<uint32_t> car_cb_count_{0};
    irPack_t        ir_data_{};
    lps::DebugRecorder::IrOverlay ir_overlay_{};

    /* ---- IR LED dik ucgeni ---- */
    static constexpr int kIrMaxBlobs = 16;   // C(16,3)=560 kombinasyon - ucuz
    bool        ir_enabled_ = true;
    float       ir_leg_a_m_ = 1.445f, ir_leg_b_m_ = 1.31f;   // 06_11 olculen (R-sag / R-sol)
    float       ir_tol_frac_ = 0.12f;
    float       ir_center_on_short_m_ = 0.655f, ir_center_on_long_m_ = 0.617f, ir_leg_ambig_ratio_ = 1.03f;
    float       ir_z_tol_    = 0.25f;   // optik Z vs irtifa (06_11: 0.50 -> 0.25)
    // Sahte ucgen kurallari (02_11, processIr): A/B/D
    int         ir_max_blobs_valid_ = 6;      // A: daha kalabalik sahne -> gecersiz
    bool        ir_require_radar_   = true;   // B: radar yok/bayat -> gecersiz
    uint32_t    ir_radar_stale_ms_  = 1500;
    int         ir_min_consec_      = 5;      // D: ardisik kararli kare (30 fps ~170 ms)
    double      ir_max_jump_px_     = 20.0;
    int         ir_run_ = 0;                  // yalniz imgCb thread'i
    cv::Point2f ir_last_mid_;
    std::atomic<uint32_t> hrvs_last_ms_{0};   // son yeni radar karesi (msSinceStart); 0 = hic
    double      ir_min_area_ = 1.0, ir_max_area_ = 400.0, ir_min_thresh_ = 240.0;
    // Irtifaya gore LED penceresi (06_11) + irtifa kaynagi
    std::string ir_alt_source_ = "auto";
    double      ir_area_ref_ = 750.0, ir_area_ref_z_ = 1.75, ir_area_exp_ = 0.6;
    double      ir_area_hi_ = 2.5, ir_area_lo_div_ = 6.0, ir_area_model_max_z_ = 5.0;
    double      ir_noalt_max_area_ = 2500.0;
    double      ir_round_min_area_ = 30.0, ir_round_min_fill_ = 0.55, ir_round_max_asp_ = 1.6;
    std::atomic<float>    ekf_alt_m_{0.0f};   // otopilot 0x4A (gecerli=1), RX thread yazar
    std::atomic<uint32_t> ekf_alt_ms_{0};     // msSinceStart; 0 = hic
    std::atomic<float>    tag_z_m_{0.0f};     // infra1 huni fuzyonu z (yer testi kaynagi)
    std::atomic<uint32_t> tag_z_ms_{0};
    std::string camera_info_topic_;
    // Tespit scratch'i - sadece imgCb thread'inden kullanilir, her karede
    // yeniden tahsis olmasin diye uye (LPS'deki small_/canvas_ ile ayni desen).
    std::vector<cv::Point2f> ir_pts_;
    std::vector<int>         ir_areas_;   // ir_pts_ ile ayni sirada (piksel alan)
    cv::Mat ir_bin_, ir_labels_, ir_stats_, ir_centroids_;
    // NOT (frame konvansiyonu): IR pozisyonu KAMERA OPTIK frame'inde gonderilir
    // (X sag, Y asagi) - govde cevrimi burada YAPILMAZ, otopilot yapar. AprilTag
    // yolu da aynidir (funnel -> avgPos.pos, hicbir mount donusumu yok). Buraya
    // bir R_mount eklemek iki yolu tutarsiz hale getirir.

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_sub_;
    std::mutex  cinfo_mutex_;
    cv::Matx33d cam_K_;
    cv::Mat     cam_D_;
    bool        cinfo_ready_ = false;

    /* ---- AprilTag fuzyonu: mantik + huni tag_meta tablosu tag_fusion.hpp'de ---- */
    float apriltag_node_size_ = 0.25f;   // AprilTag node size (duvar buyuk tag = 250mm)

    /* Ucus video kaydi (opsiyonel, record_video param) */
    std::unique_ptr<lps::DebugRecorder> recorder_;
    int record_session_ = 0;
    std::string record_label_;   // video alt seridindeki kamera etiketi
    // Son karede gorulen huni tag'lerinin overlay'i (data_mutex_ korumali;
    // tagCb yazar, imgCb okur -> recorder State'ine gider).
    std::vector<lps::DebugRecorder::TagOverlay> tag_overlays_;

    /* VPI thread iletisimi */
    std::thread              vpi_thread_;
    std::mutex               frame_mutex_;
    std::condition_variable  frame_cv_;
    cv::Mat                  latest_frame_;
    double                   latest_stamp_s_ = 0.0;   // latest_frame_'in kamera zamani
    std::unique_ptr<lps::FrameGapStats> gap_stats_;   // akis kare-araligi duzenliligi
    bool                     new_frame_  = false;
    std::atomic<bool>        vpi_running_{true};

    /* HRVS thread */
    std::thread        hrvs_thread_;
    std::atomic<bool>  hrvs_running_{true};
    // Radar bekcisi: ilerleme damgasi (HRVS thread'i yazar) + reset darbesi (zamanlayici)
    std::atomic<int64_t> hrvs_progress_ms_{0};
    bool                 hrvs_debug_log_ = true;
    unsigned             hrvs_temp_tick_ = 0;       // health timer'i, 10 tick'te bir sicaklik logu
    // Radar sicakligi -> saglik paketi cpuTemp (GECICI; yalniz health timer'i yazar/okur)
    bool                 cpu_temp_from_radar_ = true;
    double               hrvs_temp_max_ = 0.0;
    double               hrvs_temp_frame_ = -1.0;
    int64_t              hrvs_temp_change_ms_ = 0;
    int8_t               health_cpu_real_ = 0;      // gercek CPU sicakligi (log), data_mutex_
    std::string          hrvs_debug_dir_;           // <record_dir>/hrvs_sNNNN/
    std::mutex           hrvs_gpio_mutex_;
    GPIOPinLinux        *hrvs_gpio_ = nullptr;     // hrvs_gpio_mutex_ altinda
    bool                 hrvs_wd_pulse_high_ = false;
    int64_t              hrvs_wd_last_ms_ = 0;
    unsigned             hrvs_wd_count_ = 0;
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

    /* Serial delta T jitter olcumu (mikrosaniye) */
    std::chrono::steady_clock::time_point last_send_tp_;
    int64_t  dt_sum_us_ = 0;
    int64_t  dt_min_us_ = INT64_MAX;
    int64_t  dt_max_us_ = 0;
    uint32_t dt_cnt_    = 0;

    /* Optik akis motoru (LPS ana kod) - tum VPI durumu bu sinifin icinde */
    std::unique_ptr<lps::OpticalFlowVPI> flow_;

    /* Ham akis CSV logu (VPI thread'inden yazilir) */
    std::ofstream flow_csv_;
    uint64_t      csv_lines_ = 0;

    /* IR CSV logu (imgCb thread'inden yazilir - processIr) */
    std::ofstream ir_csv_;
    uint64_t      ir_csv_lines_ = 0;

    std::chrono::steady_clock::time_point start_time_;
};

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Sensor callback'leri (imgCb/tagCb) executor'da; serial gonderim ayri RT
    // TX thread'inde (jitter'siz); VPI + HRVS de ayri thread'lerde.
    rclcpp::spin(std::make_shared<SerialSenderNode>());
    rclcpp::shutdown();
    return 0;
}
