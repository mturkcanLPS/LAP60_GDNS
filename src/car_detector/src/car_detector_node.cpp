// =============================================================================
// car_detector_node.cpp
//
// DeepStream (YOLOv8 VisDrone + NvDCF) -> TEK arac kilidi -> bearing -> ROS2
//
// DeepStream C sample app yapisi (test1/appsrc-test) + gomulu rclcpp.
// Girdi:  ROS2 Image (RealSense COLOR onerilir; model RGB bekliyor) -> appsrc
// Cikti:  geometry_msgs/Vector3Stamped  '/car_detection/bearing'
//           vector.x = X/Z, vector.y = Y/Z  (distortion-duzeltilmis yon, RENKLI kamera cerc.)
//           vector.z = kilitli track_id (>=0 gecerli; <0 => kilit yok)
//         Otopilot: yon(x,y) + kendi lazer irtifasi -> metrik ofset. Biz metrik URETMIYORUZ.
//
// IKI KAMERA (03_11, dual_camera:=true): yuksekte EO + buyuk model (736x1280, uzak menzil),
// alcakta infra1 + 640 model. EO'da 1280 giris ~11 m altinda araci kaciriyor ve 70x43 derece
// gorus 5 m'de araci sigdirmiyor; IR 87x58 derece + 640 giris kayitta 3-25 m'de %95-100
// buldu. Gecis HRVS irtifasiyla, histerezisli: alcalirken switch_down_m'de IR'ye, yukselirken
// switch_up_m'de EO'ya. Bant icinde IKI hat da beslenir ve kilit tutar -> gecis bir kare,
// yeni kanal araci zaten kilitlemis. IR bearing'i RealSense extrinsics'iyle RENKLI kamera
// cercevesine cevrilir -> otopilot tarafi degismez. Irtifa yoksa (alt_lost_s) eski davranis: EO.
//
// Guvenlik (tracker ustunde asil emniyet):
//   YAKALAMA merkeze en yakin araci kilitle | TAKIP kilitli ID | GATE ani sicramayi reddet
//   YENIDEN-KILIT ID kaybolursa son-pozisyona en yakin & gate-ici | KAYIP N frame -> birak
//
// Pipeline (kanal basina):
//   appsrc(RGBA) -> queue(leaky) -> nvvideoconvert -> NVMM/NV12 -> nvstreammux
//     -> nvinfer(config) -> nvtracker(NvDCF) -> fakesink   [probe: tracker src pad]
//
// Dayaniklilik (19_10 ucusu): nvinfer'da ara sira GPU MMU hatasi (cudaErrorIllegal-
// Address, NULL adres) -> pipeline durur. Eskiden appsrc kuyrugu sinirsizdi: 2 dk'da
// ~6 GB -> OOM -> systemd TUM servisi dusurdu. Simdi: appsrc en fazla 2 kare tutar
// (eskiyi atar); GStreamer hatasi / takilma / asiri RSS -> node _exit eder, launch
// respawn eder (~3 s'de geri gelir, diger node'lar etkilenmez).
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <realsense2_camera_msgs/msg/extrinsics.hpp>

#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>

#include "nvdsmeta.h"
#include "gstnvdsmeta.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <filesystem>

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

// NvDCF hazir configi (DS 7.1).
//   PERF     -> NvDCF (korelasyon-filtresi gorsel takip), model GEREKTIRMEZ, kutudan calisir.
//   ACCURACY -> derin ReID (resnet50_market1501.etlt) ISTER; o modeli indirmeden patlar.
// Tek-hedef kilidi + bizim gate/lock mantigi PERF ile yeterli. Lookalike arabalar
// sorun olursa ReID modelini indirip tracker_config'i accuracy'ye cevir.
static const char *DS_TRACKER_LIB =
    "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so";
static const char *DS_TRACKER_CFG =
    "/opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/"
    "config_tracker_NvDCF_perf.yml";

// Kayit adi: <onek>[_sNNNN]_YYYYmmdd_HHMMSS[_k].<uzanti>, O_EXCL ile ATOMIK
// sahiplenilir -> mevcut dosyanin ustune ASLA yazilmaz (respawn dahil). Saat
// acilista geri yuklendigi icin (sahada NTP yok) sira oturum numarasindan gelir.
// lps_serial_sender/record_naming.hpp'nin kopyasi (paketler bagimsiz kalsin).
static std::string claimRecordPath(const std::string &dir, const std::string &prefix,
                                   int session, const std::string &ext) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  char ts[32];
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
  char ses[16] = "";
  if (session > 0) std::snprintf(ses, sizeof(ses), "_s%04d", session);
  const std::string base = dir + "/" + prefix + ses + "_" + ts;
  for (int k = 1; k < 100; ++k) {
    const std::string p = base + (k == 1 ? "" : "_" + std::to_string(k)) + "." + ext;
    int fd = ::open(p.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0644);
    if (fd >= 0) { ::close(fd); return p; }
    if (errno != EEXIST) return p;
  }
  return base + "_p" + std::to_string(::getpid()) + "." + ext;
}

class CarDetector : public rclcpp::Node {
  // OpenCV recorder overlay tipleri — onlari kullanan fonksiyonlardan ONCE tanimli olmali
  struct RecBox { float left, top, w, h; long id; float conf; bool locked;
                  float len_m; bool rejected; };   // len_m < 0: boy hesaplanamadi
  struct RecOverlay {
    std::vector<RecBox> boxes;
    long   locked_id = -1;
    double bx = 0, by = 0, u = 0, v = 0;
    float  conf = 0;
    bool   valid = false;
    bool   have_k = false;   // intrinsics var mi (valid=false'un sebebini ayirmak icin)
    double alt = -1;         // boy filtresinin kullandigi HRVS irtifasi; <0: filtre kapali
  };

  enum { EO = 0, IR = 1 };

  // Bir tespit kanali: kamera + model + DeepStream hatti + kilit durumu.
  struct Chan {
    CarDetector *self = nullptr;
    int idx = EO;
    const char *name = "EO";
    std::string image_topic, cinfo_topic, nvinfer_cfg;
    int frame_skip = 1;                    // IR 30 fps -> her 2. kare (15 fps)
    uint64_t n_rx = 0;                     // yalniz executor thread'i
    GstElement *pipeline = nullptr, *appsrc = nullptr, *pgie = nullptr, *tracker = nullptr;
    std::atomic<bool> fed{false};          // su an besleniyor mu
    std::atomic<bool> reset_req{false};    // beslenmeye yeniden baslarken kilidi sifirla
    std::atomic<int64_t> last_in_ns{0}, last_out_ns{0};   // steady ns; out 0 = henuz cikti yok
    // intrinsics (mux boyutuna olcekli)
    std::mutex k_mtx;
    cv::Mat K, D;
    bool have_K = false;
    // son kare zaman damgasi
    std::mutex stamp_mtx;
    builtin_interfaces::msg::Time last_stamp;
    // kilit durumu (yalniz bu kanalin probe thread'inden erisilir)
    long locked_id = -1;
    double last_u = 0, last_v = 0;
    double vel_u = 0, vel_v = 0;           // piksel/kare hiz tahmini (pan takibi)
    bool have_last = false;
    int lost = 0;
    // son hesaplanan bearing (overlay/OSD icin; IR'de renkli cerceveye cevrilmis)
    double pub_bx = 0, pub_by = 0, pub_u = 0, pub_v = 0, pub_conf = 0;
    bool pub_valid = false;
    // kayit overlay'i (probe yazar, onImage okur)
    std::mutex rec_ov_mtx;
    RecOverlay rec_ov;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_ci;
  };

public:
  CarDetector() : Node("car_detector") {
    for (int i = 0; i < 2; ++i) {
      ch_[i] = std::make_unique<Chan>();
      ch_[i]->self = this; ch_[i]->idx = i; ch_[i]->name = i == EO ? "EO" : "IR";
    }
    Chan &eo = *ch_[EO], &ir = *ch_[IR];
    // ---- parametreler ----
    eo.image_topic = declare_parameter<std::string>("image_topic",
                        "/camera/camera/color/image_raw");
    eo.cinfo_topic = declare_parameter<std::string>("camera_info_topic",
                        "/camera/camera/color/camera_info");
    eo.nvinfer_cfg = declare_parameter<std::string>("nvinfer_config",
                        "/home/lapis/DeepStream-Yolo/config_infer_primary_yoloV8.txt");
    // Iki kamera (bkz. dosya basi). Kapaliyken davranis eskisiyle birebir (sadece EO).
    dual_          = declare_parameter<bool>("dual_camera", false);
    ir.image_topic = declare_parameter<std::string>("ir_image_topic",
                        "/camera/camera/infra1/image_rect_raw");
    ir.cinfo_topic = declare_parameter<std::string>("ir_camera_info_topic",
                        "/camera/camera/infra1/camera_info");
    ir.nvinfer_cfg = declare_parameter<std::string>("ir_nvinfer_config", "");
    ir.frame_skip  = std::max(1, (int)declare_parameter<int>("ir_frame_skip", 2));
    down_m_        = declare_parameter<double>("switch_down_m", 12.0);  // alcalirken IR'ye
    up_m_          = declare_parameter<double>("switch_up_m", 18.0);    // yukselirken EO'ya
    alt_lost_s_    = declare_parameter<double>("altitude_lost_s", 10.0); // bu kadar irtifa yok -> EO
    ext_topic_     = declare_parameter<std::string>("ir_to_color_extrinsics_topic",
                        "/camera/camera/extrinsics/depth_to_color");   // D4xx: depth = infra1
    if (dual_ && ir.nvinfer_cfg.empty()) {
      RCLCPP_ERROR(get_logger(), "dual_camera=true ama ir_nvinfer_config bos -> sadece EO");
      dual_ = false;
    }
    tracker_cfg_  = declare_parameter<std::string>("tracker_config", DS_TRACKER_CFG);
    // deepstream-test2 tarzi [tracker] keyfile (elle duzenlenebilir, recompile yok).
    // Bos -> paketin share/config/tracker_config.txt'si; o da yoksa hardcoded perf.
    tracker_cfg_file_ = declare_parameter<std::string>("tracker_config_file", "");
    if (tracker_cfg_file_.empty()) {
      try {
        tracker_cfg_file_ = ament_index_cpp::get_package_share_directory("car_detector")
                            + "/config/tracker_config.txt";
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "share dizini bulunamadi: %s", e.what());
      }
    }
    W_            = declare_parameter<int>("mux_width", 1280);
    H_            = declare_parameter<int>("mux_height", 720);
    conf_th_      = declare_parameter<double>("conf_threshold", 0.35);
    // track gate: kilitli id'nin TAHMINDEN izin verilen sapmasi (kosegen orani)
    track_gate_frac_ = declare_parameter<double>("track_gate_frac", 0.12);
    // reacq gate: yeni id benimsemek icin SIKI gate (kosegen orani)
    reacq_gate_frac_ = declare_parameter<double>("reacq_gate_frac", 0.05);
    // yeni id'ye gecmeden once kac kare bekle (ayni id geri gelsin diye)
    reacq_delay_  = declare_parameter<int>("reacq_delay_frames", 3);
    // hiz tahmini EMA agirligi
    vel_ema_      = declare_parameter<double>("vel_ema", 0.5);
    // kilit tamamen birakilmadan once kac frame bulunamayabilir
    lost_max_     = declare_parameter<int>("lost_frames_to_unlock", 15);
    pgie_interval_= declare_parameter<int>("pgie_interval", 0);
    display_      = declare_parameter<bool>("display", false);   // true -> ekranda OSD (nv3dsink)
    record_video_ = declare_parameter<bool>("record_video", true); // true -> EO .avi kaydi (OpenCV/MJPG, overlay'li)
    record_dir_   = declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
    record_fps_   = declare_parameter<double>("record_fps", 15.0);  // OpenCV VideoWriter fps
    // Kayit olcegi: kare ONCE kucultulur, sonra renk donusumu + overlay + MJPG
    // (maliyet ~olcek^2). 1.0 = tam 1280x720 (eski davranis). Deger launch'ta
    // olculerek secildi (bkz. lps_bringup bringup.launch.py car_record_scale).
    record_scale_ = declare_parameter<double>("record_scale", 1.0);
    if (!(record_scale_ > 0.05 && record_scale_ <= 1.0)) record_scale_ = 1.0;
    // ---- dayaniklilik (bkz. dosya basi) ----
    // nvvideoconvert yuzey kopyasi donanimi: 0 = eleman varsayilani (GPU), 1 = GPU, 2 = VIC.
    copy_hw_         = declare_parameter<int>("copy_hw", 0);
    // RSS bunu asarsa kapan (normal ~560 MB tek model). 0 = kapali.
    max_rss_mb_      = declare_parameter<int>("max_rss_mb", 1500);
    // Kareler geliyor ama pipeline bu kadar sn cikti vermiyorsa takildi say. 0 = kapali.
    // Ilk cikti gelene kadar devre disi (motor ilk derlemesi dakikalar surer).
    stall_timeout_s_ = declare_parameter<double>("stall_timeout_s", 3.0);
    // SADECE TEST: N sn sonra nvinfer girisinde hata enjekte et (0 = kapali).
    // silent=false: ucustaki gibi akis hatasi + bus ERROR; true: nvinfer hatasiz
    // DONAR (kareleri sessizce yutar, cikti yok) -> takilma bekcisini sinar.
    test_fail_after_s_ = declare_parameter<double>("test_fail_after_s", 0.0);
    test_fail_silent_  = declare_parameter<bool>("test_fail_silent", false);
    auto vcls    = declare_parameter<std::vector<int64_t>>("vehicle_classes",
                        std::vector<int64_t>{3, 4, 5, 8});  // VisDrone: car/van/truck/bus
    for (auto c : vcls) vehicle_cls_.insert(static_cast<int>(c));

    // Boy filtresi (02_11): kutunun uzun kenari + HRVS irtifasi -> gercek boy. Yerdeki
    // tag levhasi (~1 m) ve cim gibi yanlis tespitleri KILITLEMEDEN once eler.
    // Olcum (#42 Egea 4.53 m, 16-64 m): 3.7-6.3 m cikti; levha 1.0 m. Hilux 5.32 m
    // ayni oranla ~4.3-7.4 m. Uzun kenar arac donse de ~boy kalir (en-boy orani degil).
    // HRVS bayatsa (radar donmus/kopuk) ya da intrinsics yoksa filtre KAPALI (eski davranis).
    // Kanal basina KENDI kamerasinin intrinsics'iyle hesaplanir.
    size_filter_   = declare_parameter<bool>("size_filter_enable", true);
    len_min_m_     = declare_parameter<double>("car_len_min_m", 2.5);
    len_max_m_     = declare_parameter<double>("car_len_max_m", 9.0);
    alt_stale_s_   = declare_parameter<double>("altitude_stale_s", 1.5);
    edge_px_       = declare_parameter<int>("size_edge_px", 4);   // kenara degen kutu kesik sayilir
    wid_min_m_     = declare_parameter<double>("car_width_min_m", 1.2);   // kesik kutuda paralel boyut alt siniri
    alt_topic_     = declare_parameter<std::string>("altitude_topic", "/hrvs/altitude");

    // EO intrinsics: param > 0 ise onu kullan, degilse camera_info bekle
    double fx = declare_parameter<double>("fx", 0.0);
    double fy = declare_parameter<double>("fy", 0.0);
    double cx = declare_parameter<double>("cx", 0.0);
    double cy = declare_parameter<double>("cy", 0.0);
    if (fx > 0 && fy > 0) {
      std::lock_guard<std::mutex> lk(eo.k_mtx);
      eo.K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
      eo.D = cv::Mat::zeros(1, 5, CV_64F);
      eo.have_K = true;
      RCLCPP_INFO(get_logger(), "Intrinsics param: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                  fx, fy, cx, cy);
    } else {
      RCLCPP_WARN(get_logger(),
          "Intrinsics param yok -> camera_info bekleniyor (gelene kadar bearing yok).");
    }

    double diag = std::hypot((double)W_, (double)H_);
    track_gate_px_ = track_gate_frac_ * diag;
    reacq_gate_px_ = reacq_gate_frac_ * diag;

    // nvdsosd overlay'i SADECE display modunda (ekran). Kayit OpenCV'de kendi
    // overlay'ini cizer, nvdsosd'ye ihtiyac yok. Ekran modu sadece EO (tek kanal).
    osd_active_ = display_;
    if (display_ && dual_) {
      RCLCPP_WARN(get_logger(), "display=true -> dual_camera kapatildi (ekran sadece EO)");
      dual_ = false;
    }
    // Kayit oturum numarasi (launch verir) -> eo_sNNNN_YYYYmmdd_HHMMSS[_k].avi
    // Iki kamerada video AKTIF kanali gosterir (alcakta IR goruntusu, ust bantta KAYNAK).
    const int record_session = declare_parameter<int>("record_session", 0);
    if (record_video_ && !display_)   // ekran yoksa dosyaya kaydet (IR gibi .avi/MJPG)
      record_path_ = claimRecordPath(record_dir_, "eo", record_session, "avi");

    // ---- ROS pub/sub ----
    pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/car_detection/bearing", 10);
    auto qos = rclcpp::SensorDataQoS();
    const int nch = dual_ ? 2 : 1;
    for (int i = 0; i < nch; ++i) {
      Chan *c = ch_[i].get();
      c->sub_img = create_subscription<sensor_msgs::msg::Image>(
          c->image_topic, qos,
          [this, c](const sensor_msgs::msg::Image::SharedPtr m) { onImage(m, *c); });
      c->sub_ci = create_subscription<sensor_msgs::msg::CameraInfo>(
          c->cinfo_topic, qos,
          [this, c](const sensor_msgs::msg::CameraInfo::SharedPtr m) { onCameraInfo(m, *c); });
    }
    // serial_sender her YENI radar karesinde yayinlar; radar donarsa mesaj kesilir.
    sub_alt_ = create_subscription<sensor_msgs::msg::Range>(
        alt_topic_, qos, [this](const sensor_msgs::msg::Range::SharedPtr m) {
          alt_m_.store(m->range);
          alt_ns_.store(nowNs());
          updateMode();
        });
    if (dual_) {
      // RealSense extrinsics'i bir kez (latched) yayinlar -> transient_local ile al.
      sub_ext_ = create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
          ext_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          [this](const realsense2_camera_msgs::msg::Extrinsics::SharedPtr m) { onExtrinsics(m); });
    }

    gst_init(nullptr, nullptr);
    for (int i = 0; i < nch; ++i) buildPipeline(*ch_[i]);
    // bus watch icin GMainLoop (ayri thread, iki hattin bus'i da burada)
    loop_ = g_main_loop_new(nullptr, FALSE);
    gthread_ = std::thread([this]() { g_main_loop_run(loop_); });
    t_start_ = std::chrono::steady_clock::now();
    updateMode();   // irtifa gelene kadar: EO (eski davranis)

    // Bekci: onImage ile ayni executor thread'inde -> writer_'a guvenle dokunur.
    guard_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
      updateMode();   // irtifa bayatlamasi (mesaj gelmezse altitude callback'i calismaz)
      logBandDiff();
      guard();
    });

    RCLCPP_INFO(get_logger(),
        "car_detector hazir | in='%s' mux=%dx%d track_gate=%.0f reacq_gate=%.0f conf>=%.2f"
        " | copy_hw=%d max_rss=%dMB stall=%.1fs | boy filtresi %s %.1f-%.1f m (irtifa '%s', bayat>%.1fs)",
        eo.image_topic.c_str(), W_, H_, track_gate_px_, reacq_gate_px_, conf_th_,
        copy_hw_, max_rss_mb_, stall_timeout_s_,
        size_filter_ ? "ACIK" : "KAPALI", len_min_m_, len_max_m_, alt_topic_.c_str(), alt_stale_s_);
    if (dual_)
      RCLCPP_INFO(get_logger(),
          "IKI KAMERA: yuksek=EO '%s' | alcak=IR '%s' (her %d. kare) model '%s' | "
          "alcalirken <%.1f m IR, yukselirken >%.1f m EO, irtifa %.0f s yoksa EO",
          eo.nvinfer_cfg.c_str(), ir.image_topic.c_str(), ir.frame_skip, ir.nvinfer_cfg.c_str(),
          down_m_, up_m_, alt_lost_s_);
    if (test_fail_after_s_ > 0)
      RCLCPP_WARN(get_logger(), "TEST: %.0f sn sonra %s pipeline hatasi enjekte edilecek",
                  test_fail_after_s_, test_fail_silent_ ? "SESSIZ" : "bus ERROR'lu");
  }

  ~CarDetector() override {
    if (writer_open_) writer_.release();   // .avi index'ini yaz (temiz kapanis)
    for (auto &c : ch_) {
      if (c && c->pipeline) {
        gst_element_set_state(c->pipeline, GST_STATE_NULL);
        gst_object_unref(c->pipeline);
      }
    }
    if (loop_) { g_main_loop_quit(loop_); }
    if (gthread_.joinable()) gthread_.join();
  }

private:
  // ==========================================================================
  //  Pipeline kurulumu (kanal basina)
  // ==========================================================================
  void buildPipeline(Chan &c) {
    // nvinfer config GORELI yollar kullaniyor (onnx/labels) -> config klasorune chdir et.
    // Model hat PLAYING'e gecerken yuklenir; iki kanalda SIRAYLA kurulur ve her biri
    // kendi klasorunde yuklenene kadar beklenir (cwd sureç geneli).
    gchar *cfg_dir = g_path_get_dirname(c.nvinfer_cfg.c_str());
    if (chdir(cfg_dir) != 0)
      RCLCPP_WARN(get_logger(), "[%s] chdir('%s') basarisiz -> goreli yollar patlayabilir",
                  c.name, cfg_dir);
    else
      RCLCPP_INFO(get_logger(), "[%s] cwd -> %s (nvinfer goreli yollari icin)", c.name, cfg_dir);
    g_free(cfg_dir);

    const std::string p = std::string(c.name);
    c.pipeline = gst_pipeline_new(("car-det-" + p).c_str());

    c.appsrc       = mk("appsrc", ("ros-src-" + p).c_str());
    GstElement *q  = mk("queue", ("src-q-" + p).c_str());
    GstElement *cv = mk("nvvideoconvert", ("nvconv-" + p).c_str());
    GstElement *cf = mk("capsfilter", ("nvmm-caps-" + p).c_str());
    GstElement *mux = mk("nvstreammux", ("mux-" + p).c_str());
    c.pgie         = mk("nvinfer", ("pgie-" + p).c_str());
    c.tracker      = mk("nvtracker", ("tracker-" + p).c_str());

    // appsrc: sistem bellekte RGBA besliyoruz
    gchar *capstr = g_strdup_printf(
        "video/x-raw,format=RGBA,width=%d,height=%d,framerate=30/1", W_, H_);
    GstCaps *caps = gst_caps_from_string(capstr);
    g_free(capstr);
    g_object_set(c.appsrc, "caps", caps, "format", GST_FORMAT_TIME,
                 "is-live", TRUE, "do-timestamp", TRUE, "block", FALSE, nullptr);
    gst_caps_unref(caps);
    // Ic kuyruk SINIRLI: en fazla 2 kare, dolunca EN ESKIYI at. Sinirsizken pipeline
    // durunca (19_10 GPU hatasi) her kare (3.7 MB) birikip OOM'a goturdu.
    g_object_set(c.appsrc, "max-buffers", (guint64)2, "max-bytes", (guint64)0,
                 "max-time", (guint64)0, "leaky-type", GST_APP_LEAKY_TYPE_DOWNSTREAM, nullptr);

    g_object_set(q, "leaky", 2, "max-size-buffers", 3, nullptr);  // downstream leaky
    if (copy_hw_ > 0) g_object_set(cv, "copy-hw", copy_hw_, nullptr);

    GstCaps *nvmm = gst_caps_from_string("video/x-raw(memory:NVMM),format=NV12");
    g_object_set(cf, "caps", nvmm, nullptr);
    gst_caps_unref(nvmm);

    g_object_set(mux, "batch-size", 1, "width", W_, "height", H_,
                 "live-source", 1, "batched-push-timeout", 40000, nullptr);

    g_object_set(c.pgie, "config-file-path", c.nvinfer_cfg.c_str(), nullptr);
    if (pgie_interval_ > 0) g_object_set(c.pgie, "interval", pgie_interval_, nullptr);

    // Tracker ayarlari: once keyfile (deepstream-test2 tarzi), olmazsa hardcoded perf.
    bool tok = false;
    if (!tracker_cfg_file_.empty())
      tok = setTrackerProperties(c.tracker, tracker_cfg_file_);
    if (!tok) {
      RCLCPP_WARN(get_logger(),
          "tracker keyfile yok/basarisiz -> hardcoded perf defaults");
      g_object_set(c.tracker,
                   "ll-lib-file", DS_TRACKER_LIB,
                   "ll-config-file", tracker_cfg_.c_str(),
                   "tracker-width", 640, "tracker-height", 384, nullptr);
    }

    // ---- Pipeline kuyrugu ----
    // display=false (VARSAYILAN, bassiz): tracker -> fakesink. Tespitleri
    //   ROS topic'ten izlersin (ros2 topic echo /car_detection/bearing).
    // display=true: tracker -> nvvideoconvert -> nvdsosd -> nv3dsink -> EKRANDA
    //   kutular + track ID + kilitli arac vurgusu. (Jetson'a monitor/X gerekir.)
    gst_bin_add_many(GST_BIN(c.pipeline), c.appsrc, q, cv, cf, mux,
                     c.pgie, c.tracker, nullptr);

    // appsrc -> q -> cv -> cf -> mux.sink_0
    if (!gst_element_link_many(c.appsrc, q, cv, cf, nullptr))
      RCLCPP_FATAL(get_logger(), "[%s] appsrc..capsfilter link basarisiz", c.name);
    GstPad *muxpad = gst_element_request_pad_simple(mux, "sink_0");
    GstPad *srcpad = gst_element_get_static_pad(cf, "src");
    if (gst_pad_link(srcpad, muxpad) != GST_PAD_LINK_OK)
      RCLCPP_FATAL(get_logger(), "[%s] capsfilter -> mux link basarisiz", c.name);
    gst_object_unref(srcpad);
    gst_object_unref(muxpad);

    if (display_) {
      // EKRAN: tracker -> conv -> nvdsosd -> nv3dsink
      GstElement *oconv = mk("nvvideoconvert", "osd-conv");
      GstElement *osd   = mk("nvdsosd", "osd");
      GstElement *sink  = mk("nv3dsink", "disp-sink");   // Jetson yerlesik ekran
      g_object_set(sink, "sync", FALSE, nullptr);
      gst_bin_add_many(GST_BIN(c.pipeline), oconv, osd, sink, nullptr);
      if (!gst_element_link_many(mux, c.pgie, c.tracker, oconv, osd, sink, nullptr))
        RCLCPP_FATAL(get_logger(), "mux..nv3dsink link basarisiz");
    } else {
      // INFERENCE-ONLY: tracker -> fakesink. Kayit DeepStream'de DEGIL — IR gibi
      // OpenCV VideoWriter ile onImage'da yapiliyor (encode/mux zinciri yok,
      // kare kare yazar, crash'e dayanikli, serial_sender IR recorder mantigi).
      GstElement *sink = mk("fakesink", ("sink-" + p).c_str());
      g_object_set(sink, "sync", FALSE, "async", FALSE, nullptr);
      gst_bin_add(GST_BIN(c.pipeline), sink);
      if (!gst_element_link_many(mux, c.pgie, c.tracker, sink, nullptr))
        RCLCPP_FATAL(get_logger(), "[%s] mux..fakesink link basarisiz", c.name);
      if (record_video_ && c.idx == EO)
        RCLCPP_INFO(get_logger(), "EO kaydi (OpenCV/IR-tarzi) -> %s", record_path_.c_str());
    }

    // PROBE: tracker src pad -> object_id burada dolu
    GstPad *tsrc = gst_element_get_static_pad(c.tracker, "src");
    gst_pad_add_probe(tsrc, GST_PAD_PROBE_TYPE_BUFFER, probeThunk, &c, nullptr);
    gst_object_unref(tsrc);

    if (test_fail_after_s_ > 0 && c.idx == EO) {   // SADECE TEST: nvinfer girisinde akis hatasi
      GstPad *psink = gst_element_get_static_pad(c.pgie, "sink");
      gst_pad_add_probe(psink, GST_PAD_PROBE_TYPE_BUFFER, failThunk, this, nullptr);
      gst_object_unref(psink);
    }

    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(c.pipeline));
    gst_bus_add_watch(bus, busThunk, this);
    gst_object_unref(bus);

    gst_element_set_state(c.pipeline, GST_STATE_PLAYING);
    // Model yuklenene kadar bekle (cwd bir sonraki kanalda degisecek).
    GstState st;
    gst_element_get_state(c.pipeline, &st, nullptr, 120 * GST_SECOND);
  }

  // ==========================================================================
  //  Iki kamera: aktif kanal secimi (03_11)
  // ==========================================================================
  // Aktif kanal bearing YAYINLAR; beslenen ama aktif olmayan kanal sadece kilidini tutar.
  //   irtifa taze : alcalirken < down_m -> IR, yukselirken > up_m -> EO (histerezis).
  //                 [down, up] bandinda IKISI de beslenir -> gecis aninda yeni kanal kilitli.
  //   kisa bayat  : karari tut, sadece aktif beslenir (HRVS kesintisi 1-10 s goruldu).
  //   uzun yok    : (altitude_lost_s) EO — iki kamera oncesi davranis.
  void updateMode() {
    if (!dual_) { setFed(*ch_[EO], true); return; }
    std::lock_guard<std::mutex> lk(mode_mtx_);
    const int64_t now = nowNs(), t = alt_ns_.load();
    const double alt = alt_m_.load();
    const bool fresh = t != 0 && now - t <= (int64_t)(alt_stale_s_ * 1e9) && alt > 0.05;
    const bool lost  = t == 0 || now - t > (int64_t)(alt_lost_s_ * 1e9);
    int act = active_.load();
    bool feedEO, feedIR;
    if (fresh) {
      if (act == EO && alt < down_m_) act = IR;
      else if (act == IR && alt > up_m_) act = EO;
      feedEO = act == EO || alt > down_m_;
      feedIR = act == IR || alt < up_m_;
    } else if (lost) {
      act = EO; feedEO = true; feedIR = false;
    } else {
      feedEO = act == EO; feedIR = act == IR;
    }
    if (act != active_.load()) {
      RCLCPP_INFO(get_logger(), "KAYNAK %s -> %s (irtifa %s%.1f m) | yeni kanal kilit %s",
                  ch_[active_.load()]->name, ch_[act]->name, fresh ? "" : "bayat/yok ",
                  fresh ? alt : 0.0, ch_[act]->locked_id >= 0 ? "HAZIR" : "yok");
      active_.store(act);
    }
    setFed(*ch_[EO], feedEO);
    setFed(*ch_[IR], feedIR);
  }

  // Bant icinde iki kanal da kilitliyken (ikisi de renkli kamera cercevesinde) bearing
  // farki: gecisteki sicramanin ve IR->renkli extrinsics'in sahada dogrulamasi. 2 s'de bir.
  // Okumalar probe thread'lerinin yazdigi alanlar — sadece log, kucuk yaris zararsiz.
  void logBandDiff() {
    if (!dual_ || ++band_tick_ % 8 != 0) return;
    const Chan &e = *ch_[EO], &i = *ch_[IR];
    if (!(e.fed.load() && i.fed.load() && e.pub_valid && i.pub_valid)) return;
    RCLCPP_INFO(get_logger(), "[BANT] aktif %s | EO bearing (%+.4f,%+.4f) IR->renkli (%+.4f,%+.4f) "
                "fark (%+.4f,%+.4f) rad, irtifa %.1f m",
                ch_[active_.load()]->name, e.pub_bx, e.pub_by, i.pub_bx, i.pub_by,
                i.pub_bx - e.pub_bx, i.pub_by - e.pub_by, alt_m_.load());
  }

  void setFed(Chan &c, bool on) {
    if (on && !c.fed.load()) {
      c.reset_req.store(true);                 // eski kilit durumu bayat
      // bekci: yeniden beslenince takilma sayaci sifirdan. Hic cikti vermemis kanala
      // dokunma (0 = bekci kapali; ilk motor derlemesi dakikalar surebilir).
      if (c.last_out_ns.load() > 0) c.last_out_ns.store(nowNs());
    }
    c.fed.store(on);
  }

  // RealSense depth_to_color extrinsics'i (D4xx'te depth cercevesi = infra1).
  // p_color = R * p_ir + t, R sutun-oncelikli (librealsense rs2_extrinsics).
  void onExtrinsics(const realsense2_camera_msgs::msg::Extrinsics::SharedPtr m) {
    std::lock_guard<std::mutex> lk(ext_mtx_);
    for (int i = 0; i < 9; ++i) ext_R_[i] = m->rotation[i];
    for (int i = 0; i < 3; ++i) ext_t_[i] = m->translation[i];
    const bool first = !have_ext_;
    have_ext_ = true;
    if (first) {
      const double tr = ext_R_[0] + ext_R_[4] + ext_R_[8];
      const double ang = std::acos(std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0))) * 180.0 / M_PI;
      RCLCPP_INFO(get_logger(), "IR->renkli extrinsics alindi: t=(%.1f, %.1f, %.1f) mm, donme %.2f derece",
                  ext_t_[0] * 1000, ext_t_[1] * 1000, ext_t_[2] * 1000, ang);
    }
  }

  // IR (infra1) normalize yonunu renkli kamera cercevesine cevir. Yon icin derinlik
  // gerekir: Z = irtifa (yoksa 10 m); kameralar ~1.5 cm aralikli -> Z hatasinin etkisi
  // < 1 mrad. Extrinsics gelmediyse birim donusum (uyari bir kez).
  void irToColor(double &bx, double &by) {
    double R[9], t[3];
    {
      std::lock_guard<std::mutex> lk(ext_mtx_);
      if (!have_ext_) {
        if (!ext_warned_) {
          ext_warned_ = true;
          RCLCPP_WARN(get_logger(), "IR->renkli extrinsics yok ('%s') -> birim donusum "
                      "(hata ~1.5 cm / Z, kucuk donme)", ext_topic_.c_str());
        }
        return;
      }
      std::memcpy(R, ext_R_, sizeof(R)); std::memcpy(t, ext_t_, sizeof(t));
    }
    const double alt = alt_m_.load();
    const double Z = (alt_ns_.load() != 0 && alt > 0.3) ? alt : 10.0;
    const double p[3] = {bx * Z, by * Z, Z};
    double q[3];
    for (int i = 0; i < 3; ++i) q[i] = R[i] * p[0] + R[3 + i] * p[1] + R[6 + i] * p[2] + t[i];
    if (q[2] > 1e-6) { bx = q[0] / q[2]; by = q[1] / q[2]; }
  }

  // ==========================================================================
  //  Dayaniklilik: bekci + test hata enjeksiyonu
  // ==========================================================================
  static int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static long rssMb() {
    long pages_total = 0, pages_rss = 0;
    FILE *f = std::fopen("/proc/self/statm", "r");
    if (!f) return 0;
    if (std::fscanf(f, "%ld %ld", &pages_total, &pages_rss) != 2) pages_rss = 0;
    std::fclose(f);
    return pages_rss * sysconf(_SC_PAGESIZE) / (1024 * 1024);
  }

  // Pipeline bir daha calismayacaksa (GPU hatasi CUDA context'ini kalici bozar)
  // tek dogru hareket surecten cikmak: launch respawn eder. _exit: bozuk context'te
  // pipeline teardown / destructor takilabilir; once EO .avi'nin index'i yazilir.
  void guard() {
    std::string why;
    const int64_t now = nowNs();
    long rss = 0;
    if (fatal_.load()) {
      std::lock_guard<std::mutex> lk(fatal_mtx_);
      why = "GStreamer hatasi: " + fatal_reason_;
    } else if (max_rss_mb_ > 0 && (rss = rssMb()) > max_rss_mb_) {
      why = "RSS " + std::to_string(rss) + " MB > " + std::to_string(max_rss_mb_) + " MB";
    } else if (stall_timeout_s_ > 0) {
      for (auto &cp : ch_) {
        Chan &c = *cp;
        if (!c.fed.load() || c.last_out_ns.load() <= 0) continue;
        if (now - c.last_in_ns.load() < 1000000000LL &&
            now - c.last_out_ns.load() > (int64_t)(stall_timeout_s_ * 1e9)) {
          why = std::string("[") + c.name + "] pipeline takildi: kareler geliyor ama " +
                std::to_string((now - c.last_out_ns.load()) / 1000000) + " ms'dir cikti yok";
          break;
        }
      }
    }
    if (why.empty()) return;
    RCLCPP_FATAL(get_logger(), "%s -> car_detector KAPANIYOR (launch yeniden baslatacak)",
                 why.c_str());
    if (writer_open_) { writer_.release(); writer_open_ = false; }
    std::fflush(nullptr);
    _exit(3);
  }

  static GstPadProbeReturn failThunk(GstPad *, GstPadProbeInfo *info, gpointer u) {
    auto *self = static_cast<CarDetector *>(u);
    double el = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - self->t_start_).count();
    if (el < self->test_fail_after_s_) return GST_PAD_PROBE_OK;
    if (!self->test_fail_posted_.exchange(true)) {
      if (!self->test_fail_silent_)   // ucustaki gibi: nvinfer ERROR post eder
        GST_ELEMENT_ERROR(self->ch_[EO]->pgie, STREAM, FAILED,
                          ("TEST: enjekte edilen Buffer conversion failed"), (nullptr));
      fprintf(stderr, "[car_detector] TEST: pipeline %s enjekte edildi (%.1f s)\n",
              self->test_fail_silent_ ? "DONMASI" : "hatasi", el);
    }
    if (self->test_fail_silent_) return GST_PAD_PROBE_DROP;   // hatasiz don: cikti yok
    // Akis hatasi: yukari dogru yayilir, appsrc/queue task'lari durur (19_10'daki gibi)
    gst_buffer_unref(GST_PAD_PROBE_INFO_BUFFER(info));
    GST_PAD_PROBE_INFO_FLOW_RETURN(info) = GST_FLOW_ERROR;
    return GST_PAD_PROBE_HANDLED;
  }

  GstElement *mk(const char *factory, const char *name) {
    GstElement *e = gst_element_factory_make(factory, name);
    if (!e) {
      RCLCPP_FATAL(get_logger(), "Element olusturulamadi: %s (plugin eksik?)", factory);
      std::exit(1);
    }
    return e;
  }

  // deepstream-test2 get_absolute_file_path: keyfile'daki goreli yolu keyfile'in
  // bulundugu dizine gore mutlaklastirir (mutlaksa oldugu gibi dondurur).
  static gchar *absPath(const char *cfg_file, gchar *file_path) {
    if (file_path && file_path[0] == '/') return file_path;
    char abs_cfg[PATH_MAX + 1];
    if (!realpath(cfg_file, abs_cfg)) { g_free(file_path); return nullptr; }
    if (!file_path) return g_strdup(abs_cfg);
    char *delim = g_strrstr(abs_cfg, "/");
    if (delim) *(delim + 1) = '\0';
    gchar *out = g_strconcat(abs_cfg, file_path, nullptr);
    g_free(file_path);
    return out;
  }

  // deepstream-test2 set_tracker_properties: [tracker] keyfile'dan property'leri
  // okuyup nvtracker'a uygular. Basari -> true.
  bool setTrackerProperties(GstElement *tr, const std::string &cfgpath) {
    GError *error = nullptr;
    GKeyFile *kf = g_key_file_new();
    if (!g_key_file_load_from_file(kf, cfgpath.c_str(), G_KEY_FILE_NONE, &error)) {
      RCLCPP_WARN(get_logger(), "tracker keyfile yuklenemedi (%s): %s",
                  cfgpath.c_str(), error ? error->message : "?");
      if (error) g_error_free(error);
      g_key_file_free(kf);
      return false;
    }
    gchar **keys = g_key_file_get_keys(kf, "tracker", nullptr, &error);
    if (!keys) {
      RCLCPP_WARN(get_logger(), "[tracker] grubu yok: %s", cfgpath.c_str());
      if (error) g_error_free(error);
      g_key_file_free(kf);
      return false;
    }
    bool ok = true;
    for (gchar **k = keys; *k; ++k) {
      if (!g_strcmp0(*k, "tracker-width") || !g_strcmp0(*k, "tracker-height") ||
          !g_strcmp0(*k, "enable-batch-process") ||
          !g_strcmp0(*k, "enable-past-frame")) {
        gint v = g_key_file_get_integer(kf, "tracker", *k, &error);
        if (!error) g_object_set(G_OBJECT(tr), *k, v, nullptr);
      } else if (!g_strcmp0(*k, "gpu-id")) {
        guint v = g_key_file_get_integer(kf, "tracker", *k, &error);
        if (!error) g_object_set(G_OBJECT(tr), "gpu_id", v, nullptr);
      } else if (!g_strcmp0(*k, "ll-config-file") || !g_strcmp0(*k, "ll-lib-file")) {
        char *p = absPath(cfgpath.c_str(),
                          g_key_file_get_string(kf, "tracker", *k, &error));
        if (!error) { g_object_set(G_OBJECT(tr), *k, p, nullptr); }
        g_free(p);
      } else {
        RCLCPP_WARN(get_logger(), "[tracker] bilinmeyen key atlandi: %s", *k);
      }
      if (error) {
        RCLCPP_WARN(get_logger(), "tracker key '%s' parse hatasi: %s",
                    *k, error->message);
        g_error_free(error); error = nullptr; ok = false;
      }
    }
    g_strfreev(keys);
    g_key_file_free(kf);
    if (ok) RCLCPP_INFO(get_logger(), "tracker keyfile yuklendi: %s", cfgpath.c_str());
    return ok;
  }

  // ==========================================================================
  //  ROS -> appsrc
  // ==========================================================================
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg, Chan &c) {
    if (msg->width == 0 || msg->height == 0) return;
    double sx = (double)W_ / msg->width;
    double sy = (double)H_ / msg->height;
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        msg->k[0] * sx, 0,             msg->k[2] * sx,
        0,             msg->k[4] * sy, msg->k[5] * sy,
        0,             0,             1);
    cv::Mat D;
    if (msg->d.size() >= 4) D = cv::Mat(1, (int)msg->d.size(), CV_64F,
                                        (void *)msg->d.data()).clone();
    else                    D = cv::Mat::zeros(1, 5, CV_64F);
    std::lock_guard<std::mutex> lk(c.k_mtx);
    c.K = K; c.D = D; c.have_K = true;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg, Chan &c) {
    if (fatal_.load()) return;   // pipeline olu: bekci birazdan surecten cikacak
    if (!c.fed.load()) return;   // bu kanal su an kullanilmiyor (GPU/CPU harcama)
    if (c.frame_skip > 1 && (c.n_rx++ % c.frame_skip) != 0) return;
    c.last_in_ns.store(nowNs());
    {
      std::lock_guard<std::mutex> lk(c.stamp_mtx);
      c.last_stamp = msg->header.stamp;
    }
    const int h = msg->height, w = msg->width;
    cv::Mat rgba;
    if (msg->encoding == "bgr8") {
      cv::Mat m(h, w, CV_8UC3, (void *)msg->data.data());
      cv::cvtColor(m, rgba, cv::COLOR_BGR2RGBA);
    } else if (msg->encoding == "rgb8") {
      cv::Mat m(h, w, CV_8UC3, (void *)msg->data.data());
      cv::cvtColor(m, rgba, cv::COLOR_RGB2RGBA);
    } else if (msg->encoding == "mono8") {
      cv::Mat m(h, w, CV_8UC1, (void *)msg->data.data());
      cv::cvtColor(m, rgba, cv::COLOR_GRAY2RGBA);
    } else if (msg->encoding == "rgba8") {
      rgba = cv::Mat(h, w, CV_8UC4, (void *)msg->data.data()).clone();
    } else if (msg->encoding == "bgra8") {
      cv::Mat m(h, w, CV_8UC4, (void *)msg->data.data());
      cv::cvtColor(m, rgba, cv::COLOR_BGRA2RGBA);
    } else {
      RCLCPP_WARN(get_logger(), "Desteklenmeyen encoding: %s", msg->encoding.c_str());
      return;
    }
    if (rgba.cols != W_ || rgba.rows != H_)
      cv::resize(rgba, rgba, cv::Size(W_, H_));
    if (!rgba.isContinuous()) rgba = rgba.clone();

    const gsize sz = (gsize)rgba.total() * rgba.elemSize();
    GstBuffer *buf = gst_buffer_new_allocate(nullptr, sz, nullptr);
    gst_buffer_fill(buf, 0, rgba.data, sz);
    // Video meta: downstream (nvvideoconvert/streammux) stride'i dogru hesaplasin —
    // yoksa nvbufsurftransform yanlis kopyalayip cudaErrorIllegalAddress'e dusuyor.
    gst_buffer_add_video_meta(buf, GST_VIDEO_FRAME_FLAG_NONE,
                              GST_VIDEO_FORMAT_RGBA, (guint)W_, (guint)H_);
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(c.appsrc), buf); // sahiplik gecer
    if (ret != GST_FLOW_OK)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s] appsrc push: %d", c.name, ret);

    // ---- IR-tarzi OpenCV kayit: AKTIF kanalin goruntusu + overlay (kare kare) ----
    if (record_video_ && !display_ && c.idx == active_.load()) recordFrame(rgba, c);
  }

  // Gelen kareyi overlay'leyip .avi'ye yaz (serial_sender DebugRecorder mantigi).
  // DeepStream encode/mux YOK -> kare kare yazilir, crash'te bile o ana kadari saglam.
  void recordFrame(const cv::Mat &rgba, Chan &c) {
    // Once kucult (4 kanalda), sonra kucuk karede renk donusumu: tam cozunurlukte
    // cvtColor + MJPG bu node'un CPU'sunun buyuk kismiydi.
    cv::Mat src;
    if (record_scale_ < 0.999)
      cv::resize(rgba, src, cv::Size(), record_scale_, record_scale_, cv::INTER_AREA);
    else
      src = rgba;
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_RGBA2BGR);
    RecOverlay ov;
    { std::lock_guard<std::mutex> lk(c.rec_ov_mtx); ov = c.rec_ov; }
    drawRecOverlay(bgr, ov, (float)record_scale_, dual_ ? c.name : nullptr);

    if (!writer_open_) {   // ilk karede ac (gercek boyutla)
      int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
      writer_.open(record_path_, fourcc, record_fps_, bgr.size(), true);
      if (!writer_.isOpened()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "VideoWriter acilamadi: %s", record_path_.c_str());
        return;
      }
      writer_open_ = true;
      RCLCPP_INFO(get_logger(), "EO kaydi ACILDI (MJPG %.0ffps %dx%d, olcek %.2f) -> %s",
                  record_fps_, bgr.cols, bgr.rows, record_scale_, record_path_.c_str());
    }
    writer_.write(bgr);
  }

  // k: kayit olcegi — kutular mux (tam cozunurluk) koordinatinda, kareyle birlikte olceklenir
  void drawRecOverlay(cv::Mat &img, const RecOverlay &ov, float k, const char *src) {
    for (const auto &b : ov.boxes) {   // kilitli=yesil kalin, aday=turuncu, boyla elenen=kirmizi (BGR)
      cv::Scalar col = b.rejected ? cv::Scalar(0, 0, 255)
                     : b.locked   ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
      cv::rectangle(img, cv::Rect((int)(b.left * k), (int)(b.top * k),
                                  (int)(b.w * k), (int)(b.h * k)),
                    col, b.locked ? 3 : b.rejected ? 1 : 2);
      char t[48];
      if (b.len_m >= 0)
        snprintf(t, sizeof(t), "%ld:%.2f L=%.1fm%s", b.id, b.conf, b.len_m, b.rejected ? " X" : "");
      else
        snprintf(t, sizeof(t), "%ld:%.2f", b.id, b.conf);
      cv::putText(img, t, cv::Point((int)(b.left * k), (int)(b.top * k) - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 2, cv::LINE_AA);
    }
    // durum seridi (yari saydam koyu zemin + beyaz yazi)
    cv::Rect band(0, 0, img.cols, 46);
    img(band) *= 0.4;
    char l1[112], l2[96] = {0};
    char pre[16] = "";
    if (src) snprintf(pre, sizeof(pre), "[%s] ", src);   // iki kamerada aktif kaynak
    if (ov.locked_id >= 0 && ov.valid)
      snprintf(l1, sizeof(l1), "%sLOCK id=%ld  conf=%.2f", pre, ov.locked_id, ov.conf);
    else if (ov.locked_id >= 0 && ov.have_k)   // kilit tutuluyor, bu karede eslesme yok
      snprintf(l1, sizeof(l1), "%sLOCK id=%ld  HOLD (bu karede yok)", pre, ov.locked_id);
    else if (ov.locked_id >= 0)
      snprintf(l1, sizeof(l1), "%sLOCK id=%ld  (intrinsics yok)", pre, ov.locked_id);
    else
      snprintf(l1, sizeof(l1), "%sKILIT YOK", pre);
    if (ov.locked_id >= 0 && ov.valid)
      snprintf(l2, sizeof(l2), "px=(%.0f,%.0f)  bx=%.3f by=%.3f", ov.u, ov.v, ov.bx, ov.by);
    cv::putText(img, l1, cv::Point(10, 18), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    // Boy filtresi durumu (sag ust): kullanilan irtifa ya da neden kapali
    char l3[64];
    if (ov.alt >= 0) snprintf(l3, sizeof(l3), "HRVS %.1fm  boy %.1f-%.1fm", ov.alt, len_min_m_, len_max_m_);
    else             snprintf(l3, sizeof(l3), "boy filtresi KAPALI");
    int base = 0;
    const cv::Size ts = cv::getTextSize(l3, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &base);
    cv::putText(img, l3, cv::Point(img.cols - ts.width - 8, 18), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                ov.alt >= 0 ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    if (l2[0])
      cv::putText(img, l2, cv::Point(10, 38), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }

  // ==========================================================================
  //  PROBE -> kilit/gate -> bearing (kanal basina, o hattin streaming thread'inde)
  // ==========================================================================
  static GstPadProbeReturn probeThunk(GstPad *, GstPadProbeInfo *info, gpointer u) {
    auto *c = static_cast<Chan *>(u);
    return c->self->onProbe(info, *c);
  }

  GstPadProbeReturn onProbe(GstPadProbeInfo *info, Chan &c) {
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf) return GST_PAD_PROBE_OK;
    c.last_out_ns.store(nowNs());   // bekci: pipeline cikti veriyor
    if (c.reset_req.exchange(false)) {   // yeniden beslenmeye basladi: eski kilit bayat
      c.locked_id = -1; c.have_last = false; c.lost = 0; c.vel_u = 0; c.vel_v = 0;
      c.pub_valid = false;
    }
    NvDsBatchMeta *batch = gst_buffer_get_nvds_batch_meta(buf);
    if (!batch) return GST_PAD_PROBE_OK;

    for (NvDsMetaList *lf = batch->frame_meta_list; lf; lf = lf->next) {
      auto *fm = reinterpret_cast<NvDsFrameMeta *>(lf->data);
      const SizeGate g = sizeGate(c);
      std::vector<Cand> cands, rejected;
      for (NvDsMetaList *lo = fm->obj_meta_list; lo; lo = lo->next) {
        auto *o = reinterpret_cast<NvDsObjectMeta *>(lo->data);
        if (vehicle_cls_.count(o->class_id) &&
            o->confidence >= conf_th_) {
          const auto &r = o->rect_params;
          double u = r.left + r.width * 0.5;
          double v = r.top + r.height * 0.5;
          Cand cd{(long)o->object_id, u, v, (float)o->confidence, o, carLength(g, r, u, v)};
          if (g.on && !lengthOk(g, cd.len_m, r, u, v)) rejected.push_back(cd);
          else                                cands.push_back(cd);
        }
      }
      if (!rejected.empty())
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[%s] BOY: %zu aday elendi (ilki id=%ld L=%.2f m) | irtifa %.1f m, sinir %.1f-%.1f m",
            c.name, rejected.size(), rejected[0].id, rejected[0].len_m, g.alt, len_min_m_, len_max_m_);
      updateLock(c, cands);
      if (osd_active_) annotate(batch, fm, cands, c);      // sadece display (nvdsosd)
      if (record_video_ && !display_) buildRecOverlay(c, cands, rejected, g);  // OpenCV recorder icin
    }
    return GST_PAD_PROBE_OK;
  }

  struct Cand { long id; double u, v; float conf; NvDsObjectMeta *om; double len_m; };

  // ---- boy filtresi ----
  struct SizeGate { bool on = false; double alt = -1, fx = 0, fy = 0, cx = 0, cy = 0; };

  // Filtre bu kare icin acik mi: HRVS taze (alt_stale_s icinde yeni kare) ve intrinsics var.
  SizeGate sizeGate(Chan &c) {
    SizeGate g;
    if (!size_filter_) return g;
    const int64_t t = alt_ns_.load();
    const double alt = alt_m_.load();
    if (t == 0 || nowNs() - t > (int64_t)(alt_stale_s_ * 1e9) || !(alt > 0.1)) return g;
    std::lock_guard<std::mutex> lk(c.k_mtx);
    if (!c.have_K) return g;
    g.fx = c.K.at<double>(0, 0); g.fy = c.K.at<double>(1, 1);
    g.cx = c.K.at<double>(0, 2); g.cy = c.K.at<double>(1, 2);
    g.alt = alt; g.on = true;
    return g;
  }

  // Merkez disindaki hedef daha uzakta (duz zemin, asagi bakan kamera).
  static double slantZ(const SizeGate &g, double u, double v) {
    const double bx = (u - g.cx) / g.fx, by = (v - g.cy) / g.fy;
    return g.alt * std::sqrt(1.0 + bx * bx + by * by);
  }

  // Kutunun uzun kenari -> metre. Filtre kapaliysa -1.
  static double carLength(const SizeGate &g, const NvOSD_RectParams &r, double u, double v) {
    if (!g.on) return -1;
    return std::max(r.width / g.fx, r.height / g.fy) * slantZ(g, u, v);
  }

  // Kenara degen kutu kesik olabilir (arac kadraja sigmiyor): alt sinir yerine, kesilen
  // kenara PARALEL boyut (o kesilmez) en az arac genisligi kadar olmali; koseye degen
  // kutu serbest. Kenardaki kucuk cim/tag parcasi boylece gecemez (tum ucuslarin
  // kaydiyla sinandi: 3 m alti kabul 111 -> 0, arac kayipsiz).
  bool lengthOk(const SizeGate &g, double L, const NvOSD_RectParams &r, double u, double v) const {
    if (L > len_max_m_) return false;
    if (L >= len_min_m_) return true;
    const bool eL = r.left <= edge_px_, eR = r.left + r.width >= W_ - edge_px_;
    const bool eT = r.top <= edge_px_,  eB = r.top + r.height >= H_ - edge_px_;
    if (!(eL || eR || eT || eB)) return false;
    if ((eL || eR) && (eT || eB)) return true;   // kose
    const double Z = slantZ(g, u, v);
    const double par = (eL || eR) ? r.height / g.fy * Z : r.width / g.fx * Z;
    return par >= wid_min_m_;
  }

  // Probe'daki tespitleri OpenCV recorder icin snapshot'a al (onImage cizecek).
  // Boy filtresinin eledikleri de cizilir (kirmizi) — kayittan sinir ayari icin.
  void buildRecOverlay(Chan &c, const std::vector<Cand> &cands, const std::vector<Cand> &rejected,
                       const SizeGate &g) {
    RecOverlay ov;
    ov.boxes.reserve(cands.size() + rejected.size());
    for (const auto &cd : cands) {
      const auto &r = cd.om->rect_params;
      ov.boxes.push_back({(float)r.left, (float)r.top, (float)r.width, (float)r.height,
                          cd.id, cd.conf, (c.locked_id >= 0 && cd.id == c.locked_id),
                          (float)cd.len_m, false});
    }
    for (const auto &cd : rejected) {
      const auto &r = cd.om->rect_params;
      ov.boxes.push_back({(float)r.left, (float)r.top, (float)r.width, (float)r.height,
                          cd.id, cd.conf, false, (float)cd.len_m, true});
    }
    ov.alt = g.on ? g.alt : -1;
    ov.locked_id = c.locked_id;
    ov.bx = c.pub_bx; ov.by = c.pub_by; ov.u = c.pub_u; ov.v = c.pub_v;
    ov.conf = (float)c.pub_conf; ov.valid = c.pub_valid;
    { std::lock_guard<std::mutex> lk(c.k_mtx); ov.have_k = c.have_K; }
    std::lock_guard<std::mutex> lk(c.rec_ov_mtx);
    c.rec_ov = std::move(ov);
  }

  // OSD/kayit overlay'i icin tek metin etiketi doldur
  void setLabel(NvOSD_TextParams *tp, const char *s, int x, int y, int fsize) {
    size_t len = std::strlen(s);
    tp->display_text = static_cast<char *>(g_malloc0(len + 1));
    std::memcpy(tp->display_text, s, len);
    tp->x_offset = x < 0 ? 0 : x;
    tp->y_offset = y < 0 ? 0 : y;
    tp->font_params.font_name = const_cast<char *>("Serif");
    tp->font_params.font_size = fsize;
    tp->font_params.font_color = NvOSD_ColorParams{1.0, 1.0, 1.0, 1.0};
    tp->set_bg_clr = 1;
    tp->text_bg_clr = NvOSD_ColorParams{0.0, 0.0, 0.0, 0.55};
  }

  // Kutu renkleri (kilitli=yesil kalin, aday=turuncu) + metinler:
  //  satir1: LOCK id + conf | satir2: olculen piksel + bearing | her kutuda id:conf
  void annotate(NvDsBatchMeta *batch, NvDsFrameMeta *fm,
                const std::vector<Cand> &cands, Chan &c) {
    for (const auto &cd : cands) {
      bool locked = (c.locked_id >= 0 && cd.id == c.locked_id);
      cd.om->rect_params.border_color = locked
          ? NvOSD_ColorParams{0.0, 1.0, 0.0, 1.0}
          : NvOSD_ColorParams{1.0, 0.6, 0.0, 1.0};
      cd.om->rect_params.border_width = locked ? 4 : 2;
    }

    NvDsDisplayMeta *dm = nvds_acquire_display_meta_from_pool(batch);
    int n = 0;
    char buf[128];

    if (c.locked_id >= 0)
      snprintf(buf, sizeof(buf), "LOCK id=%ld  conf=%.2f", c.locked_id, c.pub_conf);
    else
      snprintf(buf, sizeof(buf), "KILIT YOK");
    setLabel(&dm->text_params[n++], buf, 12, 12, 13);

    if (c.locked_id >= 0 && c.pub_valid)
      snprintf(buf, sizeof(buf), "px=(%.0f,%.0f)  bx=%.3f by=%.3f",
               c.pub_u, c.pub_v, c.pub_bx, c.pub_by);
    else if (c.locked_id >= 0)
      snprintf(buf, sizeof(buf), "px=(%.0f,%.0f)  (intrinsics yok)", c.pub_u, c.pub_v);
    else
      snprintf(buf, sizeof(buf), " ");
    setLabel(&dm->text_params[n++], buf, 12, 36, 13);

    const int MAXL = 16;   // NvDsDisplayMeta text_params siniri
    for (const auto &cd : cands) {
      if (n >= MAXL) break;
      snprintf(buf, sizeof(buf), "%ld:%.2f", cd.id, cd.conf);
      setLabel(&dm->text_params[n++], buf, (int)(cd.u - 20), (int)(cd.v - 28), 11);
    }
    dm->num_labels = n;
    nvds_add_display_meta_to_frame(fm, dm);
  }

  // Kilit mantigi her beslenen kanalda calisir (bant icinde ikisi de kilit tutar);
  // yayini sadece AKTIF kanal yapar (publishBearing/publishInvalid icinde).
  void updateLock(Chan &c, const std::vector<Cand> &cands) {
    const double cx0 = W_ * 0.5, cy0 = H_ * 0.5;

    // ---- YAKALAMA: kilit yok -> merkeze en yakin araci kilitle ----
    if (c.locked_id < 0) {
      if (cands.empty()) { publishInvalid(c); return; }
      const Cand *best = nullptr; double bd = 1e18;
      for (auto &cd : cands) {
        double d = (cd.u - cx0) * (cd.u - cx0) + (cd.v - cy0) * (cd.v - cy0);
        if (d < bd) { bd = d; best = &cd; }
      }
      lockTo(c, best->id, best->u, best->v, true);
      c.pub_conf = best->conf;
      RCLCPP_INFO(get_logger(), "[%s] KILIT acildi -> id=%ld @(%.0f,%.0f) conf=%.2f L=%.1fm",
                  c.name, best->id, best->u, best->v, best->conf, best->len_m);
      publishBearing(c, best->id, best->u, best->v);
      return;
    }

    // ---- TAKIP: tahmini pozisyon = son + hiz (kamera pan'ini onceden takip et) ----
    const double pu = c.last_u + c.vel_u, pv = c.last_v + c.vel_v;

    const Cand *match = nullptr;
    for (auto &cd : cands) if (cd.id == c.locked_id) { match = &cd; break; }

    if (match) {
      // Ayni id: TAHMINE gore gate. Gercek harekete izin verir, isinlanmayi reddeder.
      double res = std::hypot(match->u - pu, match->v - pv);
      if (res > track_gate_px_) {
        RCLCPP_WARN(get_logger(),
            "[%s] GATE: id=%ld tahminden %.0fpx sapti (>%.0f) -> HOLD",
            c.name, c.locked_id, res, track_gate_px_);
        c.lost++; deadReckon(c, pu, pv); publishInvalid(c); maybeUnlock(c);
        return;
      }
      updateVel(c, match->u - c.last_u, match->v - c.last_v);
      c.last_u = match->u; c.last_v = match->v; c.lost = 0;
      c.pub_conf = match->conf;
      publishBearing(c, match->id, match->u, match->v);
      return;
    }

    // ---- KAYIP: kilitli id yok. ASLA hemen baska arabaya atlama. ----
    c.lost++;
    deadReckon(c, pu, pv);   // tahmini pozisyonu ilerlet (gate bununla takip eder)

    // Yeniden-kilit SADECE su kosulda: yeterince bekledik + tahmine yakin
    // TEK aday var (belirsizlik yok) + SIKI gate icinde. Yoksa dokunma.
    int near = 0; const Cand *only = nullptr;
    for (auto &cd : cands)
      if (std::hypot(cd.u - c.last_u, cd.v - c.last_v) <= reacq_gate_px_) { near++; only = &cd; }

    if (c.lost >= reacq_delay_ && near == 1) {
      RCLCPP_INFO(get_logger(),
          "[%s] YENIDEN-KILIT %ld->%ld @(%.0f,%.0f) (tek aday, siki gate)",
          c.name, c.locked_id, only->id, only->u, only->v);
      updateVel(c, only->u - c.last_u, only->v - c.last_v);
      lockTo(c, only->id, only->u, only->v, false);
      c.pub_conf = only->conf;
      publishBearing(c, only->id, only->u, only->v);
      return;
    }

    // HOLD: kilidi TUT, gecersiz yayinla; cok uzun kayipsa birak.
    publishInvalid(c);
    maybeUnlock(c);
  }

  void lockTo(Chan &c, long id, double u, double v, bool reset_vel) {
    c.locked_id = id; c.last_u = u; c.last_v = v; c.lost = 0; c.have_last = true;
    if (reset_vel) { c.vel_u = 0; c.vel_v = 0; }
  }
  void updateVel(Chan &c, double du, double dv) {
    // tek-kare sicramasini sinirla, sonra hiz EMA'sina kat
    double lim = track_gate_px_;
    du = std::max(-lim, std::min(lim, du));
    dv = std::max(-lim, std::min(lim, dv));
    c.vel_u = vel_ema_ * du + (1 - vel_ema_) * c.vel_u;
    c.vel_v = vel_ema_ * dv + (1 - vel_ema_) * c.vel_v;
  }
  void deadReckon(Chan &c, double pu, double pv) {
    // kayip sirasinda tahmine ilerle, hiz'i hafifce sonumle (belirsizlik artar)
    c.last_u = pu; c.last_v = pv;
    c.vel_u *= 0.8; c.vel_v *= 0.8;
  }

  void maybeUnlock(Chan &c) {
    if (c.locked_id >= 0 && c.lost > lost_max_) {
      RCLCPP_WARN(get_logger(), "[%s] KILIT birakildi (id=%ld, %d frame kayip)",
                  c.name, c.locked_id, c.lost);
      c.locked_id = -1; c.have_last = false; c.lost = 0; c.vel_u = 0; c.vel_v = 0;
    }
  }

  void publishBearing(Chan &c, long track_id, double u, double v) {
    c.pub_u = u; c.pub_v = v;   // olculen piksel — intrinsics olmasa da overlay icin sakla
    cv::Mat K, D;
    bool haveK;
    {
      std::lock_guard<std::mutex> lk(c.k_mtx);
      haveK = c.have_K;
      if (haveK) { K = c.K.clone(); D = c.D.clone(); }
    }
    if (!haveK) { c.pub_valid = false; return; }  // intrinsics yok -> yon uretemeyiz
    std::vector<cv::Point2d> in{cv::Point2d(u, v)}, out;
    cv::undistortPoints(in, out, K, D);   // normalize (X/Z, Y/Z), bu kameranin cercevesi
    double bx = out[0].x, by = out[0].y;
    if (c.idx == IR) irToColor(bx, by);  // paket RENKLI kamera cercevesinde kalsin
    c.pub_bx = bx; c.pub_by = by; c.pub_valid = true;  // display icin sakla
    if (c.idx != active_.load()) return;  // isinma kanali: kilit tutar, yayinlamaz
    geometry_msgs::msg::Vector3Stamped m;
    { std::lock_guard<std::mutex> lk(c.stamp_mtx); m.header.stamp = c.last_stamp; }
    m.header.frame_id = "camera";
    m.vector.x = bx;
    m.vector.y = by;
    m.vector.z = (double)track_id;   // >=0 gecerli
    pub_->publish(m);
  }

  void publishInvalid(Chan &c) {
    c.pub_valid = false;
    if (c.idx != active_.load()) return;
    geometry_msgs::msg::Vector3Stamped m;
    { std::lock_guard<std::mutex> lk(c.stamp_mtx); m.header.stamp = c.last_stamp; }
    m.header.frame_id = "camera";
    m.vector.x = 0.0; m.vector.y = 0.0; m.vector.z = -1.0;
    pub_->publish(m);
  }

  static gboolean busThunk(GstBus *, GstMessage *msg, gpointer u) {
    auto *self = static_cast<CarDetector *>(u);
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
      GError *e; gchar *d; gst_message_parse_error(msg, &e, &d);
      RCLCPP_ERROR(self->get_logger(), "GST ERROR: %s | %s", e->message, d ? d : "");
      if (!self->fatal_.load()) {   // ilk hata = kok sebep; bekci surecten cikar
        std::lock_guard<std::mutex> lk(self->fatal_mtx_);
        self->fatal_reason_ = e->message;
        self->fatal_.store(true);
      }
      g_error_free(e); g_free(d);
    } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_WARNING) {
      GError *e; gchar *d; gst_message_parse_warning(msg, &e, &d);
      RCLCPP_WARN(self->get_logger(), "GST WARN: %s | %s", e->message, d ? d : "");
      g_error_free(e); g_free(d);
    }
    return TRUE;
  }

  // ---- params ----
  std::string tracker_cfg_, tracker_cfg_file_;
  int W_, H_, lost_max_, pgie_interval_, reacq_delay_;
  double conf_th_, track_gate_frac_, reacq_gate_frac_, vel_ema_;
  double track_gate_px_, reacq_gate_px_;
  bool display_ = false;
  bool record_video_ = true, osd_active_ = false;
  std::string record_dir_, record_path_;
  double record_fps_ = 15.0;
  double record_scale_ = 1.0;
  std::set<int> vehicle_cls_;

  // ---- kanallar + iki kamera ----
  std::array<std::unique_ptr<Chan>, 2> ch_;
  bool   dual_ = false;
  double down_m_ = 12.0, up_m_ = 18.0, alt_lost_s_ = 10.0;
  std::atomic<int> active_{EO};
  std::mutex mode_mtx_;
  uint32_t band_tick_ = 0;   // logBandDiff (guard timer'i)
  std::string ext_topic_;
  std::mutex ext_mtx_;
  double ext_R_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, ext_t_[3] = {0, 0, 0};
  bool have_ext_ = false, ext_warned_ = false;

  // ---- dayaniklilik ----
  int    copy_hw_ = 0, max_rss_mb_ = 1500;
  double stall_timeout_s_ = 3.0, test_fail_after_s_ = 0.0;
  bool   test_fail_silent_ = false;
  std::atomic<bool> test_fail_posted_{false};
  std::atomic<bool> fatal_{false};
  std::mutex  fatal_mtx_;
  std::string fatal_reason_;
  std::chrono::steady_clock::time_point t_start_;
  rclcpp::TimerBase::SharedPtr guard_timer_;

  // ---- OpenCV recorder (IR-tarzi: VideoWriter MJPG .avi, kare kare) ----
  cv::VideoWriter writer_;
  bool         writer_open_ = false;

  // ---- gstreamer ----
  GMainLoop *loop_ = nullptr;
  std::thread gthread_;

  // ---- ROS ----
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_alt_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr sub_ext_;

  // ---- boy filtresi (HRVS irtifasi) ----
  bool   size_filter_ = true;
  double len_min_m_ = 2.5, len_max_m_ = 9.0, alt_stale_s_ = 1.5, wid_min_m_ = 1.2;
  int    edge_px_ = 4;
  std::string alt_topic_;
  std::atomic<double>  alt_m_{0.0};
  std::atomic<int64_t> alt_ns_{0};   // son HRVS mesaji (steady ns); 0 = hic gelmedi
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarDetector>());
  rclcpp::shutdown();
  return 0;
}
