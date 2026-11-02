// =============================================================================
// car_detector_node.cpp
//
// DeepStream (YOLOv8 VisDrone + NvDCF) -> TEK arac kilidi -> bearing -> ROS2
//
// DeepStream C sample app yapisi (test1/appsrc-test) + gomulu rclcpp.
// Girdi:  ROS2 Image (RealSense COLOR onerilir; model RGB bekliyor) -> appsrc
// Cikti:  geometry_msgs/Vector3Stamped  '/car_detection/bearing'
//           vector.x = X/Z, vector.y = Y/Z  (distortion-duzeltilmis yon, kamera cerc.)
//           vector.z = kilitli track_id (>=0 gecerli; <0 => kilit yok)
//         Otopilot: yon(x,y) + kendi lazer irtifasi -> metrik ofset. Biz metrik URETMIYORUZ.
//
// Guvenlik (tracker ustunde asil emniyet):
//   YAKALAMA merkeze en yakin araci kilitle | TAKIP kilitli ID | GATE ani sicramayi reddet
//   YENIDEN-KILIT ID kaybolursa son-pozisyona en yakin & gate-ici | KAYIP N frame -> birak
//
// Pipeline:
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

#include <atomic>
#include <cmath>
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

public:
  CarDetector() : Node("car_detector") {
    // ---- parametreler ----
    image_topic_  = declare_parameter<std::string>("image_topic",
                        "/camera/camera/color/image_raw");
    cinfo_topic_  = declare_parameter<std::string>("camera_info_topic",
                        "/camera/camera/color/camera_info");
    nvinfer_cfg_  = declare_parameter<std::string>("nvinfer_config",
                        "/home/lapis/DeepStream-Yolo/config_infer_primary_yoloV8.txt");
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
    // RSS bunu asarsa kapan (normal ~560 MB). 0 = kapali.
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
    size_filter_   = declare_parameter<bool>("size_filter_enable", true);
    len_min_m_     = declare_parameter<double>("car_len_min_m", 2.5);
    len_max_m_     = declare_parameter<double>("car_len_max_m", 9.0);
    alt_stale_s_   = declare_parameter<double>("altitude_stale_s", 1.5);
    edge_px_       = declare_parameter<int>("size_edge_px", 4);   // kenara degen kutu kesik sayilir
    wid_min_m_     = declare_parameter<double>("car_width_min_m", 1.2);   // kesik kutuda paralel boyut alt siniri
    alt_topic_     = declare_parameter<std::string>("altitude_topic", "/hrvs/altitude");

    // intrinsics: param > 0 ise onu kullan, degilse camera_info bekle
    double fx = declare_parameter<double>("fx", 0.0);
    double fy = declare_parameter<double>("fy", 0.0);
    double cx = declare_parameter<double>("cx", 0.0);
    double cy = declare_parameter<double>("cy", 0.0);
    if (fx > 0 && fy > 0) {
      std::lock_guard<std::mutex> lk(k_mtx_);
      K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
      D_ = cv::Mat::zeros(1, 5, CV_64F);
      have_K_ = true;
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
    // overlay'ini cizer, nvdsosd'ye ihtiyac yok.
    osd_active_ = display_;
    // Kayit oturum numarasi (launch verir) -> eo_sNNNN_YYYYmmdd_HHMMSS[_k].avi
    const int record_session = declare_parameter<int>("record_session", 0);
    if (record_video_ && !display_)   // ekran yoksa dosyaya kaydet (IR gibi .avi/MJPG)
      record_path_ = claimRecordPath(record_dir_, "eo", record_session, "avi");

    // ---- ROS pub/sub ----
    pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/car_detection/bearing", 10);
    auto qos = rclcpp::SensorDataQoS();
    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos,
        std::bind(&CarDetector::onImage, this, std::placeholders::_1));
    sub_ci_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cinfo_topic_, qos,
        std::bind(&CarDetector::onCameraInfo, this, std::placeholders::_1));
    // serial_sender her YENI radar karesinde yayinlar; radar donarsa mesaj kesilir.
    sub_alt_ = create_subscription<sensor_msgs::msg::Range>(
        alt_topic_, qos, [this](const sensor_msgs::msg::Range::SharedPtr m) {
          alt_m_.store(m->range);
          alt_ns_.store(nowNs());
        });

    buildPipeline();

    // Bekci: onImage ile ayni executor thread'inde -> writer_'a guvenle dokunur.
    guard_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() { guard(); });

    RCLCPP_INFO(get_logger(),
        "car_detector hazir | in='%s' mux=%dx%d track_gate=%.0f reacq_gate=%.0f conf>=%.2f"
        " | copy_hw=%d max_rss=%dMB stall=%.1fs | boy filtresi %s %.1f-%.1f m (irtifa '%s', bayat>%.1fs)",
        image_topic_.c_str(), W_, H_, track_gate_px_, reacq_gate_px_, conf_th_,
        copy_hw_, max_rss_mb_, stall_timeout_s_,
        size_filter_ ? "ACIK" : "KAPALI", len_min_m_, len_max_m_, alt_topic_.c_str(), alt_stale_s_);
    if (test_fail_after_s_ > 0)
      RCLCPP_WARN(get_logger(), "TEST: %.0f sn sonra %s pipeline hatasi enjekte edilecek",
                  test_fail_after_s_, test_fail_silent_ ? "SESSIZ" : "bus ERROR'lu");
  }

  ~CarDetector() override {
    if (writer_open_) writer_.release();   // .avi index'ini yaz (temiz kapanis)
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
    }
    if (loop_) { g_main_loop_quit(loop_); }
    if (gthread_.joinable()) gthread_.join();
  }

private:
  // ==========================================================================
  //  Pipeline kurulumu
  // ==========================================================================
  void buildPipeline() {
    // nvinfer config GORELI yollar kullaniyor (onnx/custom-lib/labels) ->
    // config klasorune chdir et ki hangi dizinden calisirsak calisalim bulunsun.
    gchar *cfg_dir = g_path_get_dirname(nvinfer_cfg_.c_str());
    if (chdir(cfg_dir) != 0)
      RCLCPP_WARN(get_logger(), "chdir('%s') basarisiz -> goreli yollar patlayabilir",
                  cfg_dir);
    else
      RCLCPP_INFO(get_logger(), "cwd -> %s (nvinfer goreli yollari icin)", cfg_dir);
    g_free(cfg_dir);

    gst_init(nullptr, nullptr);
    pipeline_ = gst_pipeline_new("car-det");

    appsrc_        = mk("appsrc", "ros-src");
    GstElement *q  = mk("queue", "src-q");
    GstElement *cv = mk("nvvideoconvert", "nvconv");
    GstElement *cf = mk("capsfilter", "nvmm-caps");
    mux_           = mk("nvstreammux", "mux");
    pgie_          = mk("nvinfer", "pgie");
    tracker_       = mk("nvtracker", "tracker");

    // appsrc: sistem bellekte RGBA besliyoruz
    GstCaps *caps = gst_caps_from_string(
        g_strdup_printf("video/x-raw,format=RGBA,width=%d,height=%d,framerate=30/1",
                        W_, H_));
    g_object_set(appsrc_, "caps", caps, "format", GST_FORMAT_TIME,
                 "is-live", TRUE, "do-timestamp", TRUE, "block", FALSE, nullptr);
    gst_caps_unref(caps);
    // Ic kuyruk SINIRLI: en fazla 2 kare, dolunca EN ESKIYI at. Sinirsizken pipeline
    // durunca (19_10 GPU hatasi) her kare (3.7 MB) birikip OOM'a goturdu.
    g_object_set(appsrc_, "max-buffers", (guint64)2, "max-bytes", (guint64)0,
                 "max-time", (guint64)0, "leaky-type", GST_APP_LEAKY_TYPE_DOWNSTREAM, nullptr);

    g_object_set(q, "leaky", 2, "max-size-buffers", 3, nullptr);  // downstream leaky
    if (copy_hw_ > 0) g_object_set(cv, "copy-hw", copy_hw_, nullptr);

    GstCaps *nvmm = gst_caps_from_string("video/x-raw(memory:NVMM),format=NV12");
    g_object_set(cf, "caps", nvmm, nullptr);
    gst_caps_unref(nvmm);

    g_object_set(mux_, "batch-size", 1, "width", W_, "height", H_,
                 "live-source", 1, "batched-push-timeout", 40000, nullptr);

    g_object_set(pgie_, "config-file-path", nvinfer_cfg_.c_str(), nullptr);
    if (pgie_interval_ > 0) g_object_set(pgie_, "interval", pgie_interval_, nullptr);

    // Tracker ayarlari: once keyfile (deepstream-test2 tarzi), olmazsa hardcoded perf.
    bool tok = false;
    if (!tracker_cfg_file_.empty())
      tok = setTrackerProperties(tracker_, tracker_cfg_file_);
    if (!tok) {
      RCLCPP_WARN(get_logger(),
          "tracker keyfile yok/basarisiz -> hardcoded perf defaults");
      g_object_set(tracker_,
                   "ll-lib-file", DS_TRACKER_LIB,
                   "ll-config-file", tracker_cfg_.c_str(),
                   "tracker-width", 640, "tracker-height", 384, nullptr);
    }

    // ---- Pipeline kuyrugu ----
    // display=false (VARSAYILAN, bassiz): tracker -> fakesink. Tespitleri
    //   ROS topic'ten izlersin (ros2 topic echo /car_detection/bearing).
    // display=true: tracker -> nvvideoconvert -> nvdsosd -> nv3dsink -> EKRANDA
    //   kutular + track ID + kilitli arac vurgusu. (Jetson'a monitor/X gerekir.)
    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, q, cv, cf, mux_,
                     pgie_, tracker_, nullptr);

    // appsrc -> q -> cv -> cf -> mux.sink_0
    if (!gst_element_link_many(appsrc_, q, cv, cf, nullptr))
      RCLCPP_FATAL(get_logger(), "appsrc..capsfilter link basarisiz");
    GstPad *muxpad = gst_element_request_pad_simple(mux_, "sink_0");
    GstPad *srcpad = gst_element_get_static_pad(cf, "src");
    if (gst_pad_link(srcpad, muxpad) != GST_PAD_LINK_OK)
      RCLCPP_FATAL(get_logger(), "capsfilter -> mux link basarisiz");
    gst_object_unref(srcpad);
    gst_object_unref(muxpad);

    if (display_) {
      // EKRAN: tracker -> conv -> nvdsosd -> nv3dsink
      GstElement *oconv = mk("nvvideoconvert", "osd-conv");
      GstElement *osd   = mk("nvdsosd", "osd");
      GstElement *sink  = mk("nv3dsink", "disp-sink");   // Jetson yerlesik ekran
      g_object_set(sink, "sync", FALSE, nullptr);
      gst_bin_add_many(GST_BIN(pipeline_), oconv, osd, sink, nullptr);
      if (!gst_element_link_many(mux_, pgie_, tracker_, oconv, osd, sink, nullptr))
        RCLCPP_FATAL(get_logger(), "mux..nv3dsink link basarisiz");
    } else {
      // INFERENCE-ONLY: tracker -> fakesink. Kayit DeepStream'de DEGIL — IR gibi
      // OpenCV VideoWriter ile onImage'da yapiliyor (encode/mux zinciri yok,
      // kare kare yazar, crash'e dayanikli, serial_sender IR recorder mantigi).
      GstElement *sink = mk("fakesink", "sink");
      g_object_set(sink, "sync", FALSE, "async", FALSE, nullptr);
      gst_bin_add(GST_BIN(pipeline_), sink);
      if (!gst_element_link_many(mux_, pgie_, tracker_, sink, nullptr))
        RCLCPP_FATAL(get_logger(), "mux..fakesink link basarisiz");
      if (record_video_)
        RCLCPP_INFO(get_logger(), "EO kaydi (OpenCV/IR-tarzi) -> %s", record_path_.c_str());
    }

    // PROBE: tracker src pad -> object_id burada dolu
    GstPad *tsrc = gst_element_get_static_pad(tracker_, "src");
    gst_pad_add_probe(tsrc, GST_PAD_PROBE_TYPE_BUFFER, probeThunk, this, nullptr);
    gst_object_unref(tsrc);

    if (test_fail_after_s_ > 0) {   // SADECE TEST: nvinfer girisinde akis hatasi
      GstPad *psink = gst_element_get_static_pad(pgie_, "sink");
      gst_pad_add_probe(psink, GST_PAD_PROBE_TYPE_BUFFER, failThunk, this, nullptr);
      gst_object_unref(psink);
    }

    // bus watch icin GMainLoop (ayri thread)
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
    gst_bus_add_watch(bus, busThunk, this);
    gst_object_unref(bus);

    loop_ = g_main_loop_new(nullptr, FALSE);
    gthread_ = std::thread([this]() { g_main_loop_run(loop_); });

    t_start_ = std::chrono::steady_clock::now();
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
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
    } else if (stall_timeout_s_ > 0 && last_out_ns_.load() > 0 &&
               now - last_in_ns_.load() < 1000000000LL &&
               now - last_out_ns_.load() > (int64_t)(stall_timeout_s_ * 1e9)) {
      why = "pipeline takildi: kareler geliyor ama " +
            std::to_string((now - last_out_ns_.load()) / 1000000) + " ms'dir cikti yok";
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
        GST_ELEMENT_ERROR(self->pgie_, STREAM, FAILED,
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
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
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
    std::lock_guard<std::mutex> lk(k_mtx_);
    K_ = K; D_ = D; have_K_ = true;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_in_ns_.store(nowNs());
    if (fatal_.load()) return;   // pipeline olu: bekci birazdan surecten cikacak
    {
      std::lock_guard<std::mutex> lk(stamp_mtx_);
      last_stamp_ = msg->header.stamp;
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
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf); // sahiplik gecer
    if (ret != GST_FLOW_OK)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "appsrc push: %d", ret);

    // ---- IR-tarzi OpenCV kayit: overlay ciz + VideoWriter'a yaz (kare kare) ----
    if (record_video_ && !display_) recordFrame(rgba);
  }

  // Gelen kareyi overlay'leyip .avi'ye yaz (serial_sender DebugRecorder mantigi).
  // DeepStream encode/mux YOK -> kare kare yazilir, crash'te bile o ana kadari saglam.
  void recordFrame(const cv::Mat &rgba) {
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
    { std::lock_guard<std::mutex> lk(rec_ov_mtx_); ov = rec_ov_; }
    drawRecOverlay(bgr, ov, (float)record_scale_);

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
  void drawRecOverlay(cv::Mat &img, const RecOverlay &ov, float k = 1.0f) {
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
    char l1[96], l2[96] = {0};
    if (ov.locked_id >= 0 && ov.valid)
      snprintf(l1, sizeof(l1), "LOCK id=%ld  conf=%.2f", ov.locked_id, ov.conf);
    else if (ov.locked_id >= 0 && ov.have_k)   // kilit tutuluyor, bu karede eslesme yok
      snprintf(l1, sizeof(l1), "LOCK id=%ld  HOLD (bu karede yok)", ov.locked_id);
    else if (ov.locked_id >= 0)
      snprintf(l1, sizeof(l1), "LOCK id=%ld  (intrinsics yok)", ov.locked_id);
    else
      snprintf(l1, sizeof(l1), "KILIT YOK");
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
  //  PROBE -> kilit/gate -> bearing
  // ==========================================================================
  static GstPadProbeReturn probeThunk(GstPad *, GstPadProbeInfo *info, gpointer u) {
    return static_cast<CarDetector *>(u)->onProbe(info);
  }

  GstPadProbeReturn onProbe(GstPadProbeInfo *info) {
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf) return GST_PAD_PROBE_OK;
    last_out_ns_.store(nowNs());   // bekci: pipeline cikti veriyor
    NvDsBatchMeta *batch = gst_buffer_get_nvds_batch_meta(buf);
    if (!batch) return GST_PAD_PROBE_OK;

    for (NvDsMetaList *lf = batch->frame_meta_list; lf; lf = lf->next) {
      auto *fm = reinterpret_cast<NvDsFrameMeta *>(lf->data);
      const SizeGate g = sizeGate();
      std::vector<Cand> cands, rejected;
      for (NvDsMetaList *lo = fm->obj_meta_list; lo; lo = lo->next) {
        auto *o = reinterpret_cast<NvDsObjectMeta *>(lo->data);
        if (vehicle_cls_.count(o->class_id) &&
            o->confidence >= conf_th_) {
          const auto &r = o->rect_params;
          double u = r.left + r.width * 0.5;
          double v = r.top + r.height * 0.5;
          Cand c{(long)o->object_id, u, v, (float)o->confidence, o, carLength(g, r, u, v)};
          if (g.on && !lengthOk(g, c.len_m, r, u, v)) rejected.push_back(c);
          else                               cands.push_back(c);
        }
      }
      if (!rejected.empty())
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "BOY: %zu aday elendi (ilki id=%ld L=%.2f m) | irtifa %.1f m, sinir %.1f-%.1f m",
            rejected.size(), rejected[0].id, rejected[0].len_m, g.alt, len_min_m_, len_max_m_);
      updateLock(cands);
      if (osd_active_) annotate(batch, fm, cands);        // sadece display (nvdsosd)
      if (record_video_ && !display_) buildRecOverlay(cands, rejected, g);  // OpenCV recorder icin
    }
    return GST_PAD_PROBE_OK;
  }

  struct Cand { long id; double u, v; float conf; NvDsObjectMeta *om; double len_m; };

  // ---- boy filtresi ----
  struct SizeGate { bool on = false; double alt = -1, fx = 0, fy = 0, cx = 0, cy = 0; };

  // Filtre bu kare icin acik mi: HRVS taze (alt_stale_s icinde yeni kare) ve intrinsics var.
  SizeGate sizeGate() {
    SizeGate g;
    if (!size_filter_) return g;
    const int64_t t = alt_ns_.load();
    const double alt = alt_m_.load();
    if (t == 0 || nowNs() - t > (int64_t)(alt_stale_s_ * 1e9) || !(alt > 0.1)) return g;
    std::lock_guard<std::mutex> lk(k_mtx_);
    if (!have_K_) return g;
    g.fx = K_.at<double>(0, 0); g.fy = K_.at<double>(1, 1);
    g.cx = K_.at<double>(0, 2); g.cy = K_.at<double>(1, 2);
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
  void buildRecOverlay(const std::vector<Cand> &cands, const std::vector<Cand> &rejected,
                       const SizeGate &g) {
    RecOverlay ov;
    ov.boxes.reserve(cands.size() + rejected.size());
    for (const auto &c : cands) {
      const auto &r = c.om->rect_params;
      ov.boxes.push_back({(float)r.left, (float)r.top, (float)r.width, (float)r.height,
                          c.id, c.conf, (locked_id_ >= 0 && c.id == locked_id_),
                          (float)c.len_m, false});
    }
    for (const auto &c : rejected) {
      const auto &r = c.om->rect_params;
      ov.boxes.push_back({(float)r.left, (float)r.top, (float)r.width, (float)r.height,
                          c.id, c.conf, false, (float)c.len_m, true});
    }
    ov.alt = g.on ? g.alt : -1;
    ov.locked_id = locked_id_;
    ov.bx = pub_bx_; ov.by = pub_by_; ov.u = pub_u_; ov.v = pub_v_;
    ov.conf = (float)pub_conf_; ov.valid = pub_valid_;
    { std::lock_guard<std::mutex> lk(k_mtx_); ov.have_k = have_K_; }
    std::lock_guard<std::mutex> lk(rec_ov_mtx_);
    rec_ov_ = std::move(ov);
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
                const std::vector<Cand> &cands) {
    for (const auto &c : cands) {
      bool locked = (locked_id_ >= 0 && c.id == locked_id_);
      c.om->rect_params.border_color = locked
          ? NvOSD_ColorParams{0.0, 1.0, 0.0, 1.0}
          : NvOSD_ColorParams{1.0, 0.6, 0.0, 1.0};
      c.om->rect_params.border_width = locked ? 4 : 2;
    }

    NvDsDisplayMeta *dm = nvds_acquire_display_meta_from_pool(batch);
    int n = 0;
    char buf[128];

    if (locked_id_ >= 0)
      snprintf(buf, sizeof(buf), "LOCK id=%ld  conf=%.2f", locked_id_, pub_conf_);
    else
      snprintf(buf, sizeof(buf), "KILIT YOK");
    setLabel(&dm->text_params[n++], buf, 12, 12, 13);

    if (locked_id_ >= 0 && pub_valid_)
      snprintf(buf, sizeof(buf), "px=(%.0f,%.0f)  bx=%.3f by=%.3f",
               pub_u_, pub_v_, pub_bx_, pub_by_);
    else if (locked_id_ >= 0)
      snprintf(buf, sizeof(buf), "px=(%.0f,%.0f)  (intrinsics yok)", pub_u_, pub_v_);
    else
      snprintf(buf, sizeof(buf), " ");
    setLabel(&dm->text_params[n++], buf, 12, 36, 13);

    const int MAXL = 16;   // NvDsDisplayMeta text_params siniri
    for (const auto &c : cands) {
      if (n >= MAXL) break;
      snprintf(buf, sizeof(buf), "%ld:%.2f", c.id, c.conf);
      setLabel(&dm->text_params[n++], buf, (int)(c.u - 20), (int)(c.v - 28), 11);
    }
    dm->num_labels = n;
    nvds_add_display_meta_to_frame(fm, dm);
  }

  void updateLock(const std::vector<Cand> &cands) {
    const double cx0 = W_ * 0.5, cy0 = H_ * 0.5;

    // ---- YAKALAMA: kilit yok -> merkeze en yakin araci kilitle ----
    if (locked_id_ < 0) {
      if (cands.empty()) { publishInvalid(); return; }
      const Cand *best = nullptr; double bd = 1e18;
      for (auto &c : cands) {
        double d = (c.u - cx0) * (c.u - cx0) + (c.v - cy0) * (c.v - cy0);
        if (d < bd) { bd = d; best = &c; }
      }
      lockTo(best->id, best->u, best->v, true);
      pub_conf_ = best->conf;
      RCLCPP_INFO(get_logger(), "KILIT acildi -> id=%ld @(%.0f,%.0f) conf=%.2f L=%.1fm",
                  best->id, best->u, best->v, best->conf, best->len_m);
      publishBearing(best->id, best->u, best->v);
      return;
    }

    // ---- TAKIP: tahmini pozisyon = son + hiz (kamera pan'ini onceden takip et) ----
    const double pu = last_u_ + vel_u_, pv = last_v_ + vel_v_;

    const Cand *match = nullptr;
    for (auto &c : cands) if (c.id == locked_id_) { match = &c; break; }

    if (match) {
      // Ayni id: TAHMINE gore gate. Gercek harekete izin verir, isinlanmayi reddeder.
      double res = std::hypot(match->u - pu, match->v - pv);
      if (res > track_gate_px_) {
        RCLCPP_WARN(get_logger(),
            "GATE: id=%ld tahminden %.0fpx sapti (>%.0f) -> HOLD",
            locked_id_, res, track_gate_px_);
        lost_++; deadReckon(pu, pv); publishInvalid(); maybeUnlock();
        return;
      }
      updateVel(match->u - last_u_, match->v - last_v_);
      last_u_ = match->u; last_v_ = match->v; lost_ = 0;
      pub_conf_ = match->conf;
      publishBearing(match->id, match->u, match->v);
      return;
    }

    // ---- KAYIP: kilitli id yok. ASLA hemen baska arabaya atlama. ----
    lost_++;
    deadReckon(pu, pv);   // tahmini pozisyonu ilerlet (gate bununla takip eder)

    // Yeniden-kilit SADECE su kosulda: yeterince bekledik + tahmine yakin
    // TEK aday var (belirsizlik yok) + SIKI gate icinde. Yoksa dokunma.
    int near = 0; const Cand *only = nullptr;
    for (auto &c : cands)
      if (std::hypot(c.u - last_u_, c.v - last_v_) <= reacq_gate_px_) { near++; only = &c; }

    if (lost_ >= reacq_delay_ && near == 1) {
      RCLCPP_INFO(get_logger(),
          "YENIDEN-KILIT %ld->%ld @(%.0f,%.0f) (tek aday, siki gate)",
          locked_id_, only->id, only->u, only->v);
      updateVel(only->u - last_u_, only->v - last_v_);
      lockTo(only->id, only->u, only->v, false);
      pub_conf_ = only->conf;
      publishBearing(only->id, only->u, only->v);
      return;
    }

    // HOLD: kilidi TUT, gecersiz yayinla; cok uzun kayipsa birak.
    publishInvalid();
    maybeUnlock();
  }

  void lockTo(long id, double u, double v, bool reset_vel) {
    locked_id_ = id; last_u_ = u; last_v_ = v; lost_ = 0; have_last_ = true;
    if (reset_vel) { vel_u_ = 0; vel_v_ = 0; }
  }
  void updateVel(double du, double dv) {
    // tek-kare sicramasini sinirla, sonra hiz EMA'sina kat
    double lim = track_gate_px_;
    du = std::max(-lim, std::min(lim, du));
    dv = std::max(-lim, std::min(lim, dv));
    vel_u_ = vel_ema_ * du + (1 - vel_ema_) * vel_u_;
    vel_v_ = vel_ema_ * dv + (1 - vel_ema_) * vel_v_;
  }
  void deadReckon(double pu, double pv) {
    // kayip sirasinda tahmine ilerle, hiz'i hafifce sonumle (belirsizlik artar)
    last_u_ = pu; last_v_ = pv;
    vel_u_ *= 0.8; vel_v_ *= 0.8;
  }

  void maybeUnlock() {
    if (locked_id_ >= 0 && lost_ > lost_max_) {
      RCLCPP_WARN(get_logger(), "KILIT birakildi (id=%ld, %d frame kayip)",
                  locked_id_, lost_);
      locked_id_ = -1; have_last_ = false; lost_ = 0; vel_u_ = 0; vel_v_ = 0;
    }
  }

  void publishBearing(long track_id, double u, double v) {
    pub_u_ = u; pub_v_ = v;   // olculen piksel — intrinsics olmasa da overlay icin sakla
    cv::Mat K, D;
    bool haveK;
    {
      std::lock_guard<std::mutex> lk(k_mtx_);
      haveK = have_K_;
      if (haveK) { K = K_.clone(); D = D_.clone(); }
    }
    if (!haveK) { pub_valid_ = false; return; }  // intrinsics yok -> yon uretemeyiz
    std::vector<cv::Point2d> in{cv::Point2d(u, v)}, out;
    cv::undistortPoints(in, out, K, D);   // normalize (X/Z, Y/Z)
    pub_bx_ = out[0].x; pub_by_ = out[0].y; pub_valid_ = true;  // display icin sakla
    geometry_msgs::msg::Vector3Stamped m;
    { std::lock_guard<std::mutex> lk(stamp_mtx_); m.header.stamp = last_stamp_; }
    m.header.frame_id = "camera";
    m.vector.x = out[0].x;
    m.vector.y = out[0].y;
    m.vector.z = (double)track_id;   // >=0 gecerli
    pub_->publish(m);
  }

  void publishInvalid() {
    pub_valid_ = false;
    geometry_msgs::msg::Vector3Stamped m;
    { std::lock_guard<std::mutex> lk(stamp_mtx_); m.header.stamp = last_stamp_; }
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
  std::string image_topic_, cinfo_topic_, nvinfer_cfg_, tracker_cfg_, tracker_cfg_file_;
  int W_, H_, lost_max_, pgie_interval_, reacq_delay_;
  double conf_th_, track_gate_frac_, reacq_gate_frac_, vel_ema_;
  double track_gate_px_, reacq_gate_px_;
  bool display_ = false;
  bool record_video_ = true, osd_active_ = false;
  std::string record_dir_, record_path_;
  double record_fps_ = 15.0;
  double record_scale_ = 1.0;
  std::set<int> vehicle_cls_;

  // ---- dayaniklilik ----
  int    copy_hw_ = 0, max_rss_mb_ = 1500;
  double stall_timeout_s_ = 3.0, test_fail_after_s_ = 0.0;
  bool   test_fail_silent_ = false;
  std::atomic<bool> test_fail_posted_{false};
  std::atomic<bool> fatal_{false};
  std::mutex  fatal_mtx_;
  std::string fatal_reason_;
  std::atomic<int64_t> last_in_ns_{0}, last_out_ns_{0};   // steady ns; out 0 = henuz cikti yok
  std::chrono::steady_clock::time_point t_start_;
  rclcpp::TimerBase::SharedPtr guard_timer_;

  // ---- OpenCV recorder (IR-tarzi: VideoWriter MJPG .avi, kare kare) ----
  std::mutex   rec_ov_mtx_;   // probe yazar, onImage okur
  RecOverlay   rec_ov_;
  cv::VideoWriter writer_;
  bool         writer_open_ = false;

  // OSD/kayit overlay'i icin son kilitli hedef (yalniz probe thread'inden erisilir)
  double pub_bx_ = 0, pub_by_ = 0;   // bearing X/Z, Y/Z
  double pub_u_ = 0, pub_v_ = 0;     // olculen piksel merkezi
  double pub_conf_ = 0;              // kilitli aracin confidence'i
  bool pub_valid_ = false;

  // ---- intrinsics ----
  std::mutex k_mtx_;
  cv::Mat K_, D_;
  bool have_K_ = false;

  // ---- kilit durumu (yalniz probe thread'inden erisilir) ----
  long locked_id_ = -1;
  double last_u_ = 0, last_v_ = 0;
  double vel_u_ = 0, vel_v_ = 0;   // piksel/kare hiz tahmini (pan takibi)
  bool have_last_ = false;
  int lost_ = 0;

  // ---- stamp ----
  std::mutex stamp_mtx_;
  builtin_interfaces::msg::Time last_stamp_;

  // ---- gstreamer ----
  GstElement *pipeline_ = nullptr, *appsrc_ = nullptr, *mux_ = nullptr,
             *pgie_ = nullptr, *tracker_ = nullptr;
  GMainLoop *loop_ = nullptr;
  std::thread gthread_;

  // ---- ROS ----
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_ci_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_alt_;

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
