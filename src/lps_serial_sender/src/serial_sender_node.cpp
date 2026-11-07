/**
 * lps_serial_sender — serial_sender_node
 *
 * Girisler:
 *   - AprilTag detections (Isaac ROS): ID 45 → avgPos,
 *     funnel tag'leri {0,20,24,36,45} → offset-based landing tahmini (debug)
 *   - IR kamera goruntusu → VPI OFA dense optical flow → global dx/dy
 *
 * Cikis: /dev/ttyTHS1 @ 50 Hz sabit frekans, 163B paket (packet_defs.hpp)
 *
 * Threading modeli:
 *   - rclcpp::spin TEK thread: tagCb + imgCb sirayla calisir.
 *   - vpi_thread_: frame kuyrugundan okur, opticflow_ yazar.
 *   - tx_thread_: sabit periyotta (sleep_until, mutlak deadline) paket basar.
 *     ROS timer KULLANILMAZ — timer executor thread'ini bekledigi icin
 *     tagCb/imgCb yogunken paket zamanlamasi titriyordu (scope'ta gorulen
 *     sirt sirta paketler). Gec kalinirsa takvim kaydirilir, burst atilmaz.
 *   - Paylasilan veri: frame kuyrugu (frame_mutex_), sensor paketleri
 *     (data_mutex_). Yeni paylasim eklerken mutex disiplinine dikkat.
 *
 * IR projektor detection su an DEVRE DISI — ir_projector_detector.hpp'de
 * hazir duruyor, asagida "IR-DISABLED" etiketli yorum bloklarindan acilir.
 */

#include <rclcpp/rclcpp.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>   // car_detector bearing (TEST)
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <lps_serial_sender/packet_defs.hpp>
#include <lps_serial_sender/serial_port.hpp>
#include <lps_serial_sender/optical_flow_vpi.hpp>
#include <lps_serial_sender/debug_recorder.hpp>
#include <lps_serial_sender/ir_projector_detector.hpp>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <pthread.h>
#include <sched.h>

/* ------------------------------------------------------------------ */
/* Tag metadata — offset-based landing point estimation icin           */
/* offset        : tag merkezi → landing point (camera frame, XY)      */
/* physical_size : gercek tag boyutu (PnP size scaling icin)           */
/* ------------------------------------------------------------------ */
// === Inis modu secimi (DERLEME ZAMANI) ===
//   0 = HUNI modu  : avgPos = 4 huni tag'inin outlier'siz ORTALAMASI,
//                    tag1..tag4 = her tag TEK TEK  (uçuş modu — VARSAYILAN)
//   1 = 45/24 modu : iki-tag test inişi (avgPos = 45 offset'li / 24 olcekli)
// Modu degistirmek icin SADECE bu satirdaki 0/1'i degistir + colcon build.
// (Not: 'true/false' degil integer — preprocessor'da net olsun diye.)
#define TEST_APRITLAG 0

struct TagMeta {
    cv::Vec3f offset;
    float     physical_size_m;
};

/* Quaternion → Roll/Pitch/Yaw (ZYX, RADYAN doner) */
static void quatToRPY(double qx, double qy, double qz, double qw,
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

/* Quaternion → 3x3 rotasyon matrisi (R_(kamera←tag)).
   Bu R, tag-frame'deki bir vektoru kamera frame'ine tasir:
   offset_kamera = R * offset_tagframe. Icinde montaj egimi + drone
   attitude birlikte gelir (canli olculur). */
static cv::Matx33f quatToMatrix(double qx, double qy, double qz, double qw)
{
    // Normalize (guvenlik icin)
    double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n < 1e-9) return cv::Matx33f::eye();
    qx/=n; qy/=n; qz/=n; qw/=n;
    return cv::Matx33f(
        (float)(1-2*(qy*qy+qz*qz)), (float)(2*(qx*qy-qz*qw)),   (float)(2*(qx*qz+qy*qw)),
        (float)(2*(qx*qy+qz*qw)),   (float)(1-2*(qx*qx+qz*qz)), (float)(2*(qy*qz-qx*qw)),
        (float)(2*(qx*qz-qy*qw)),   (float)(2*(qy*qz+qx*qw)),   (float)(1-2*(qx*qx+qy*qy)));
}

/* ------------------------------------------------------------------ */
class SerialSenderNode : public rclcpp::Node
{
public:
    SerialSenderNode() : Node("serial_sender")
    {
        start_time_ = std::chrono::steady_clock::now();
        // TUM timestamp'lerin ortak sifir noktasi (ROS clock, ns).
        // hrvs + apriltag + opticflow timestamp'leri = bu andan itibaren ms.
        // Boylece hepsi 0 civarindan baslar, ayni tabanda karsilastirilabilir
        // (orn. sensor gecikmesi = hrvs.timeStamp - apriltag.timeStamp).
        t0_ns_ = (int64_t)this->now().nanoseconds();

        serial_port_  = declare_parameter<std::string>("serial_port",  "/dev/ttyTHS1");
        baud_rate_    = declare_parameter<int>   ("baud_rate",    115200);
        send_rate_hz_ = declare_parameter<double>("send_rate_hz", 50.0);
        tag_topic_    = declare_parameter<std::string>("tag_topic",   "/tag_detections");
        image_topic_  = declare_parameter<std::string>("image_topic", "/camera/camera/infra1/image_rect_raw");
        int   grid_size  = declare_parameter<int>("grid_size",  4);
        int   num_levels = declare_parameter<int>("num_levels", 3);
        float ema_alpha  = (float)declare_parameter<double>("ema_alpha", 0.25);
        // false = periyodik log YOK (sadece acilis + hata loglari kalir)
        debug_        = declare_parameter<bool>("debug", false);

        // AprilTag node'unda set'lenmis size parametresi — per-ID size scaling icin referans.
        // FUNNEL (tag25h9) modu: node size=0.25 (duvar buyuk tag siyah kare=250mm).
        // Kucuk tag'ler (0.077) tag_meta'da per-ID scale ile duzeltilir.
        apriltag_node_size_ = (float)declare_parameter<double>("apriltag_node_size", 0.25);

        // Tag verisi bu kadar sure yenilenmezse valid=false gonderilir
        // (pipeline stall'inda bayat pozisyonu 50Hz'de tekrarlamamak icin)
        tag_timeout_ms_ = declare_parameter<int>("tag_timeout_ms", 500);

        // TEST: car_detector bearing'ini avgPos'a enjekte et (default KAPALI).
        car_avg_inject_ = declare_parameter<bool>("car_avg_inject", false);

        // TX thread CPU sabitleme (-1 = kapali). Jetson'da tum IRQ'lar
        // CPU0'da toplaniyor; TX thread USB/GPU interrupt firtinasinin
        // oldugu core'a denk gelirse RT oncelige ragmen 50-70ms gec
        // uyanabiliyor (olculdu: yuk altinda ~2 sn'de bir). Sakin bir
        // core'a sabitlemek bu kuyrugu keser.
        tx_cpu_ = (int)declare_parameter<int>("tx_cpu", 5);

        // Doku kapisi TABANI. Efektif esik sahneye uyarlanir:
        //   esik = max(taban, 1.5 x grad_p50)   (p50 = anlik gurultu tabani)
        // debug=true iken [OPTFLOW] logundaki thr= ile izlenir.
        // Taban 8 -> 4 (26_07_24): on-suzme sonrasi gradyanlar dustu, taban
        // baglayici hale gelip kapsamayi ve Q'yu eziyordu (beton ucusunda
        // g90=6 < taban 8). Dokulu sahnede etkisiz, gurultulu sahnede
        // uyarlanir kural koruyor. Ayrinti: optical_flow_vpi.hpp
        float flow_tex = (float)declare_parameter<double>("flow_texture_thresh", 4.0);

        // Akis girisine uygulanan gurultu on-suzmesi (0 = kapali).
        // OFA korelasyon tabanli; piksel gurultusu dogrudan eslesmeyi bozuyordu.
        // Olculen kazanc (bkz. optical_flow_vpi.hpp): calisma noktasinda
        // ortalama hata 0.109 -> 0.033 px, Q 44-55 -> 84-93.
        // Sahada suphelenirsen 0.0 vererek tek parametreyle geri al.
        float flow_pre = (float)declare_parameter<double>("flow_prefilter_sigma", 1.5);

        flow_ = std::make_unique<lps::OpticalFlowVPI>(grid_size, num_levels,
                                                      ema_alpha, flow_tex,
                                                      flow_pre);

        // Ucus kaydi: overlay'li video (tag koseler + akis HUD'u), ayri
        // dusuk oncelikli thread'de — ucus sonrasi "tespit nerede/neden
        // zorlandi" analizi icin. TX/VPI zamanlamasina dokunmaz.
        bool        rec_on    = declare_parameter<bool>("record_video", false);
        std::string rec_dir   = declare_parameter<std::string>(
                                    "record_dir", "/home/lapis/lps_records");
        double      rec_fps   = declare_parameter<double>("record_fps", 15.0);
        double      rec_scale = declare_parameter<double>("record_scale", 0.5);
        if (rec_on) {
            recorder_ = std::make_unique<lps::DebugRecorder>(
                            rec_dir, rec_fps, rec_scale);
            RCLCPP_INFO(get_logger(), "Ucus video kaydi: %s (%.0f fps, x%.2f)",
                        recorder_->path().c_str(), rec_fps, rec_scale);
        }

        // Ham optik akis CSV logu (analiz icin; EMA ONCESI kare-basi olcum).
        // Dosya video kaydiyla ayni dizine, ayni zaman damgasiyla duser.
        if (declare_parameter<bool>("flow_csv", false)) {
            char stamp[32];
            const std::time_t now = std::time(nullptr);
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S",
                          std::localtime(&now));
            const std::string path = rec_dir + "/flow_" + stamp + ".csv";
            flow_csv_.open(path);
            if (flow_csv_) {
                flow_csv_ << "t_ms,raw_dx,raw_dy,raw_q,ema_dx,ema_dy,ema_q,"
                             "consist,tex,g50,g90\n";
                flow_csv_.flush();   // ani kesintide baslik kaybolmasin
                RCLCPP_INFO(get_logger(), "Ham akis CSV: %s", path.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Akis CSV acilamadi: %s", path.c_str());
            }
        }

        // Huni kalibrasyon logu: her tespit edilen tag icin HAM poz
        // (Tpos + quaternion) + turevleri. Bu logdan tag-frame 'o' offset'i
        // kalibre edilir ve R*offset dogrulanir. Analiz icin; opsiyonel.
        if (declare_parameter<bool>("funnel_csv", false)) {
            char stamp[32];
            const std::time_t now = std::time(nullptr);
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S",
                          std::localtime(&now));
            const std::string path = rec_dir + "/funnel_" + stamp + ".csv";
            funnel_csv_.open(path);
            if (funnel_csv_) {
                funnel_csv_ << "t_ms,tag_id,tx,ty,tz,qx,qy,qz,qw,"
                               "roll_deg,pitch_deg,yaw_deg,"
                               "ox_cam,oy_cam,oz_cam,est_x,est_y,est_z\n";
                funnel_csv_.flush();
                RCLCPP_INFO(get_logger(), "Huni kalibrasyon CSV: %s", path.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Huni CSV acilamadi: %s", path.c_str());
            }
        }

        serial_fd_ = lps::openSerial(serial_port_, baud_rate_);

        tag_sub_ = create_subscription<
            isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            tag_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::tagCb, this, std::placeholders::_1));

        // TEST: car_detector bearing'ini apriltag avgPos'a enjekte et.
        // Default KAPALI -> funnel'a sifir etki. Acikken txThread avgPos'u
        // car bearing'iyle EZER (funnel yazimi degismez, sadece gonderim aninda).
        if (car_avg_inject_) {
            car_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
                "/car_detection/bearing", rclcpp::SensorDataQoS(),
                std::bind(&SerialSenderNode::carCb, this, std::placeholders::_1));
            RCLCPP_WARN(get_logger(),
                "[CAR-TEST] car_avg_inject ACIK -> avgPos = car bearing (funnel EZILIR)");
        }

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::imgCb, this, std::placeholders::_1));

        // IR-DISABLED: IR debug goruntu publisher'i
        // ir_debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
        //     "~/ir_debug_image", rclcpp::SensorDataQoS());
        // ir_detector_.setIntrinsics(647.50, 647.50, 646.49, 354.54);  // TODO: camera_info'dan al

        vpi_thread_ = std::thread(&SerialSenderNode::vpiThread, this);
        tx_thread_  = std::thread(&SerialSenderNode::txThread, this);

        // Debug loglama AYRI timer'da (executor thread) — TX dongusunde
        // konsol I/O'su bloklarsa paket zamanlamasi bozuluyordu
        if (debug_) {
            debug_timer_ = create_wall_timer(std::chrono::seconds(1),
                std::bind(&SerialSenderNode::debugLog, this));
        }

        RCLCPP_INFO(get_logger(),
            "SerialSender: tag='%s' img='%s' → %s @%d baud %.1f Hz | pkt=%zuB | debug=%s",
            tag_topic_.c_str(), image_topic_.c_str(),
            serial_port_.c_str(), baud_rate_, send_rate_hz_,
            sizeof(jetsonOtopilotPack_t), debug_ ? "ON" : "OFF");
    }

    ~SerialSenderNode()
    {
        tx_running_ = false;
        if (tx_thread_.joinable()) tx_thread_.join();

        vpi_running_ = false;
        frame_cv_.notify_all();
        if (vpi_thread_.joinable()) vpi_thread_.join();
        flow_.reset();   // VPI objeleri thread durduktan sonra yok edilir

        recorder_.reset();   // son kareyi yazar, dosyayi kapatir

        // CSV: VPI thread durduktan SONRA kapat (yazan o thread'di)
        if (flow_csv_.is_open()) flow_csv_.close();
        if (funnel_csv_.is_open()) funnel_csv_.close();

        if (serial_fd_ >= 0) ::close(serial_fd_);
    }

private:
    /* -------------------------------------------------------------- */
    /* AprilTag callback                                              */
    /*   1) ID 45 → avgPos → serial                                   */
    /*   2) Funnel tag'leri → offset-based landing tahmini (debug)    */
    /* -------------------------------------------------------------- */
    void tagCb(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        apriltagPack_t ap{};
        // Image capture timestamp — sensor exposure ani (callback zamani DEGIL).
        // Isaac ROS AprilTag input image header'ini propagate ediyor.
        // Drone delta-t hesabini gercek zaman farki uzerinden yapmak icin sart.
        ap.timeStamp = msSinceStart(
            (uint64_t)rclcpp::Time(msg->header.stamp).nanoseconds());

#if TEST_APRITLAG   // ===== 45/24 IKI-TAG TEST INISI =====
        // avgPos = 45 (buyuk, offset'li) ya da 24 (kucuk, olcekli).
        // tag1..tag4 KULLANILMAZ (bu modda huni yok). Huni modu icin #else'e bak.
        for (const auto &det : msg->detections) {
            // Sadece hedef tag'ler: 45 = buyuk (245mm), 24 = kucuk (95mm).
            // Hedef DEGILSE atla — yoksa alakasiz bir tag valid=true + (0,0,0)
            // gonderip break edip gercek tag'e hic ulasamiyordu.
            if (det.id != 45 && det.id != 24) continue;

            const auto &p = det.pose.pose.pose;
            ap.avgPos.valid = true;
            if(det.id == 45){
                // Buyuk tag: pozisyonu kucuk tag merkezine kaydir. Offset TAG
                // FRAME'inde (0,-0.215,0 = kucuk tag 21.5cm) ve R ile kamera
                // frame'ine cevrilir -> HER drone acisinda dogru (funnel mantigi;
                // eski sabit kamera-Y offset sadece dik bakista dogruydu).
                // NOT: offset tag'in dik/hizali yapistirildigini varsayar; egik
                // yapistirilmissa yon degisir (bir 45/24 logu ile dogrulanabilir).
                const auto &q = p.orientation;
                cv::Matx33f R = quatToMatrix(q.x, q.y, q.z, q.w);
                cv::Vec3f off = R * cv::Vec3f(0.0f, -0.215f, 0.0f);
                ap.avgPos.pos.x = (float)p.position.x + off[0];
                ap.avgPos.pos.y = (float)p.position.y + off[1];
                ap.avgPos.pos.z = (float)p.position.z + off[2];
            }else{ // det.id == 24
                // Kucuk tag: AprilTag node size=0.245'e gore PnP olceklendi,
                // gercek boyut 0.095 -> pozisyon vektorunu duzelt.
                float scale = 0.095f / apriltag_node_size_;
                ap.avgPos.pos.x = (float)p.position.x * scale;
                ap.avgPos.pos.y = (float)p.position.y * scale;
                ap.avgPos.pos.z = (float)p.position.z * scale;
            }

            float roll, pitch, yaw;
            quatToRPY(p.orientation.x, p.orientation.y,
                      p.orientation.z, p.orientation.w,
                      roll, pitch, yaw);
            // Firmware'e DERECE gidiyor
            constexpr float R2D = (float)(180.0 / M_PI);
            ap.avgPos.angle.roll  = roll  * R2D;
            ap.avgPos.angle.pitch = pitch * R2D;
            ap.avgPos.angle.yaw   = yaw   * R2D;
            break;
        }
#else               // ===== HUNI MODU (4 tag: ortalama + tek tek) =====

        // === Offset-based funnel landing estimation → PAKETE gider ===
        std::vector<cv::Vec3f> estimates;
        std::vector<cv::Vec3f> raws;
        std::vector<int>       used_ids;

        for (const auto &det : msg->detections) {
            auto it = tag_meta_.find(det.id);
            if (it == tag_meta_.end()) continue;

            const auto &meta = it->second;

            // Size scaling: AprilTag node parametre size'a gore PnP olceklendi,
            // gercek size farkliysa pozisyon vektorunu duzeltiyoruz.
            float scale = meta.physical_size_m / apriltag_node_size_;

            const auto &p = det.pose.pose.pose.position;
            const auto &q = det.pose.pose.pose.orientation;
            cv::Vec3f tag_pos(
                (float)p.x * scale,
                (float)p.y * scale,
                (float)p.z * scale);

            // Offset'i tag'in OLCULEN oryantasyonuyla dondur:
            //   offset_kamera = R * offset_tagframe
            // R = quat→matris (montaj egimi + drone attitude birlikte, canli).
            // NOT: meta.offset su an eski (duz-zemin) degerleriyle — PLACEHOLDER.
            // Gercek tag-frame 'o' vektoru bu ucusun logundan kalibre edilecek.
            cv::Matx33f R = quatToMatrix(q.x, q.y, q.z, q.w);
            cv::Vec3f   offset_cam = R * meta.offset;
            cv::Vec3f   est = tag_pos + offset_cam;

            raws.push_back(tag_pos);
            estimates.push_back(est);
            used_ids.push_back(det.id);

            // === Kalibrasyon logu: HAM poz (Tpos + quaternion) + turevleri ===
            if (funnel_csv_.is_open()) {
                float roll, pitch, yaw;
                quatToRPY(q.x, q.y, q.z, q.w, roll, pitch, yaw);
                const float R2D = 180.0f / (float)M_PI;
                funnel_csv_
                    << ap.timeStamp << ',' << det.id << ','
                    << tag_pos[0] << ',' << tag_pos[1] << ',' << tag_pos[2] << ','
                    << q.x << ',' << q.y << ',' << q.z << ',' << q.w << ','
                    << roll*R2D << ',' << pitch*R2D << ',' << yaw*R2D << ','
                    << offset_cam[0] << ',' << offset_cam[1] << ',' << offset_cam[2] << ','
                    << est[0] << ',' << est[1] << ',' << est[2] << '\n';
                if (++funnel_csv_lines_ % 30 == 0) funnel_csv_.flush();
            }
        }

        // Median tabanli outlier rejection + ortalama
        cv::Vec3f funnel(0, 0, 0);
        int n_inliers = 0;
        std::vector<PerTagLanding> per_tag;
        per_tag.reserve(estimates.size());

        if (!estimates.empty()) {
            std::vector<float> xs, ys, zs;
            xs.reserve(estimates.size());
            ys.reserve(estimates.size());
            zs.reserve(estimates.size());
            for (const auto &e : estimates) {
                xs.push_back(e[0]); ys.push_back(e[1]); zs.push_back(e[2]);
            }
            auto med = [](std::vector<float> &v) {
                std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                return v[v.size() / 2];
            };
            cv::Vec3f median(med(xs), med(ys), med(zs));

            constexpr float OUTLIER_M = 0.4f;
            cv::Vec3f sum(0, 0, 0);
            for (size_t i = 0; i < estimates.size(); ++i) {
                cv::Vec3f d = estimates[i] - median;
                bool used = (cv::norm(d) <= OUTLIER_M);
                per_tag.push_back({used_ids[i], raws[i], estimates[i], used});
                if (used) { sum += estimates[i]; n_inliers++; }
            }
            if (n_inliers > 0) funnel = sum * (1.0f / n_inliers);
        }

        // === Funnel sonuclarini PAKETE yaz (otopilota gider) ===
        // avgPos = outlier'siz ortalama (huni merkezi fuzyonu). POZISYON sadece
        // avgPos'ta gider; tek tek tag pozlari artik GONDERILMIYOR — her tag'in
        // yalnizca VALIDITY bit'i basiliyor (validity bitfield'ina).
        ap.avgPos.valid = (n_inliers > 0);
        ap.avgPos.pos.x = funnel[0];
        ap.avgPos.pos.y = funnel[1];
        ap.avgPos.pos.z = funnel[2];
        // angle: huni ortalamasinin tek dogal yonu yok — 0 birakiliyor.
        // ESKI PAKET: per-tag validity biti YOK; sadece avgPos.valid + pos gider.
        // (tag1-4 alanlari pakette var ama hep 0 — firmware ile uyumlu.)
#endif

        
        std::vector<lps::DebugRecorder::TagOverlay> overlays;
        if (recorder_) {
            overlays.reserve(msg->detections.size());
            for (const auto &det : msg->detections) {
                lps::DebugRecorder::TagOverlay t;
                t.id = det.id;
                if (tag_meta_.find(det.id) == tag_meta_.end()) continue;  // huni tag'leri (duvar+kucuk)
                for (int i = 0; i < 4; ++i)
                    t.corners[i] = cv::Point2f((float)det.corners[i].x,
                                               (float)det.corners[i].y);
                overlays.push_back(t);
            }
        }

        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            apriltag_               = ap;
            last_tag_msg_tp_        = std::chrono::steady_clock::now();
#if !TEST_APRITLAG   // huni debug degiskenleri SADECE huni modunda uretiliyor
            funnel_landing_         = funnel;
            funnel_landing_n_seen_  = (uint8_t)estimates.size();
            funnel_landing_n_used_  = (uint8_t)n_inliers;
            funnel_per_tag_         = std::move(per_tag);
#endif
            tag_overlays_           = std::move(overlays);
        }
    }

    /* -------------------------------------------------------------- */
    /* TEST: car_detector bearing -> car_avg state (txThread avgPos'u  */
    /* bununla ezer, SADECE car_avg_inject acikken).                   */
    void carCb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        car_valid_ = (msg->vector.z >= 0.0);    // z<0 => kilit yok
        if (car_valid_) {
            car_pos_.x = (float)msg->vector.x;   // bearing X/Z
            car_pos_.y = (float)msg->vector.y;   // bearing Y/Z
            car_pos_.z = 0.0f;
            car_tp_    = std::chrono::steady_clock::now();
        }
    }

    /* -------------------------------------------------------------- */
    /* Image callback — frame'i kuyruğa koyar, anında döner            */
    /* Kopya YOK: cv_bridge view + msg shared_ptr canli tutulur        */
    /* -------------------------------------------------------------- */
    void imgCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "cv_bridge: %s", e.what());
            return;
        }
        if (!cv_ptr || cv_ptr->image.empty()) return;

        // IR-DISABLED: IR projektor detection
        // cv::Mat ir_vis;
        // bool want_vis = ir_debug_pub_ &&
        //                 ir_debug_pub_->get_subscription_count() > 0;
        // auto ir_res = ir_detector_.detect(cv_ptr->image,
        //                                   want_vis ? &ir_vis : nullptr);
        // if (want_vis) { /* cv_bridge ile ir_vis publish */ }

        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            latest_frame_ptr_ = cv_ptr;   // msg'yi canli tutar, clone yok
            latest_frame_ns_  = rclcpp::Time(msg->header.stamp).nanoseconds();
            new_frame_        = true;
        }
        frame_cv_.notify_one();

        // Ucus kaydi: durum snapshot'i + frame pointer'i recorder'a ver
        // (kopya yok; cv_ptr keepalive olarak gecer, yazim ayri thread'de)
        if (recorder_) {
            lps::DebugRecorder::State st;
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                st.tags      = tag_overlays_;
                st.tag_valid = apriltag_.avgPos.valid;
                st.tag_stale = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_tag_msg_tp_).count()
                    > tag_timeout_ms_;
                st.px    = apriltag_.avgPos.pos.x;
                st.py    = apriltag_.avgPos.pos.y;
                st.pz    = apriltag_.avgPos.pos.z;
                st.roll  = apriltag_.avgPos.angle.roll;
                st.pitch = apriltag_.avgPos.angle.pitch;
                st.yaw   = apriltag_.avgPos.angle.yaw;
                st.dx      = opticflow_.motionPos.x;
                st.dy      = opticflow_.motionPos.y;
                st.quality = opticflow_.quality;
            }
            st.consist  = flow_consist_dbg_.load();
            st.tex      = flow_tex_dbg_.load();
            st.grad_p50 = flow_p50_dbg_.load();
            st.grad_p90 = flow_p90_dbg_.load();
            st.t_ms = msSinceStart(
                (uint64_t)rclcpp::Time(msg->header.stamp).nanoseconds());
            recorder_->push(cv_ptr->image, cv_ptr, st);
        }
    }

    /* -------------------------------------------------------------- */
    /* VPI thread — frame kuyrugundan al, optical flow hesapla         */
    /*                                                                 */
    /* -------------------------------------------------------------- */
    void vpiThread()
    {
        while (vpi_running_.load()) {
            cv_bridge::CvImageConstPtr frame_ptr;
            uint64_t frame_ns = 0;
            {
                std::unique_lock<std::mutex> lk(frame_mutex_);
                frame_cv_.wait(lk, [this]{
                    return new_frame_ || !vpi_running_.load();
                });
                if (!vpi_running_.load()) break;
                frame_ptr  = std::move(latest_frame_ptr_);
                frame_ns   = latest_frame_ns_;
                new_frame_ = false;
            }
            if (!frame_ptr) continue;
            const cv::Mat &frame = frame_ptr->image;

            try {
                if (!flow_->initialized()) {
                    flow_->init(frame);
                    RCLCPP_INFO(get_logger(), "VPI OFA init: %dx%d → padded %dx%d",
                                frame.cols, frame.rows,
                                flow_->paddedWidth(), flow_->paddedHeight());
                    continue;
                }

                lps::OpticalFlowVPI::Result r;
                flow_->process(frame, r);

                opticFlowPack_t of{};
                // Image capture timestamp (VPI processing time DEGIL)
                of.timestamp    = msSinceStart(frame_ns);
                of.motionPos.x  = r.dx;
                of.motionPos.y  = r.dy;
                of.quality      = r.quality;

                // Ham akis CSV'si — EMA ONCESI kare-basi olcum.
                // Bu thread'de yazmak guvenli: ofstream tamponlu, satir ~60 bayt,
                // maliyet mikrosaniye. Yarim saniyede bir flush: ani kesinti
                // (guc gitmesi / kill) halinde en fazla ~15 satir kaybedilir.
                if (flow_csv_.is_open()) {
                    flow_csv_ << of.timestamp << ','
                              << r.raw_dx << ',' << r.raw_dy << ','
                              << (unsigned)r.raw_quality << ','
                              << r.dx << ',' << r.dy << ','
                              << (unsigned)r.quality << ','
                              << r.consist << ',' << r.tex_ratio << ','
                              << (unsigned)r.tex_p50 << ',' << (unsigned)r.tex_p90
                              << '\n';
                    if (++csv_lines_ % 15 == 0) flow_csv_.flush();
                }

                { std::lock_guard<std::mutex> lk(data_mutex_); opticflow_ = of; }
                flow_consist_dbg_ = r.consist;      // sadece debug logu icin
                flow_tex_dbg_     = r.tex_ratio;
                flow_p50_dbg_     = r.tex_p50;
                flow_p90_dbg_     = r.tex_p90;
                flow_thr_dbg_     = r.tex_thresh;
            } catch (const std::exception &e) {
                if (!flow_->initialized()) {
                    // Init basarisiz — donanim/format sorunu, tekrar denemek anlamsiz
                    RCLCPP_FATAL(get_logger(),
                        "VPI init basarisiz, optical flow DEVRE DISI: %s", e.what());
                    break;
                }
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "VPI hatasi (frame atlandi): %s", e.what());
            }
        }
    }

    /* TX thread — sabit frekansta seri porta paket gonderir           */

    void txThread()
    {

        {
            sched_param sp{};
            sp.sched_priority = 50;
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
                RCLCPP_INFO(get_logger(),
                    "TX thread RT oncelik alinamadi (izin yok) — normal scheduler");
            }
        }

        
        if (tx_cpu_ >= 0) {
            cpu_set_t cpus;
            CPU_ZERO(&cpus);
            CPU_SET(tx_cpu_, &cpus);
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus) != 0) {
                RCLCPP_INFO(get_logger(),
                    "TX thread CPU%d'e sabitlenemedi — serbest calisacak", tx_cpu_);
            }
        }

        const auto period = std::chrono::nanoseconds(
            (int64_t)(1e9 / send_rate_hz_));
        auto next = std::chrono::steady_clock::now() + period;

        while (tx_running_.load()) {
            std::this_thread::sleep_until(next);
            if (!tx_running_.load()) break;

            sendPacket();

            next += period;
            auto now = std::chrono::steady_clock::now();
            if (next < now) {
                next = now;
            }
        }
    }

    void sendPacket()
    {
        frame_number_++;   // monoton, HIC sifirlanmaz — firmware paket kaybini bundan anlar

        apriltagPack_t  at_copy;
        opticFlowPack_t of_copy;
        std::chrono::steady_clock::time_point last_tag_tp;
        bool        car_valid_copy = false;
        float_3D_t  car_pos_copy{};
        std::chrono::steady_clock::time_point car_tp_copy;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            at_copy     = apriltag_;
            of_copy     = opticflow_;
            last_tag_tp = last_tag_msg_tp_;
            car_valid_copy = car_valid_;
            car_pos_copy   = car_pos_;
            car_tp_copy    = car_tp_;
        }

        // Staleness: tag pipeline'i durduysa bayat "valid" pozu gondermeye devam etme
        auto tag_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_tag_tp).count();
        if (tag_age_ms > tag_timeout_ms_) {
            at_copy.avgPos.valid = false;
        }

        // TEST: car bearing enjeksiyonu — avgPos'u car ile EZ (funnel override).
        // SADECE car_avg_inject acikken. Kendi staleness'i var (tag_timeout_ms).
        if (car_avg_inject_) {
            auto car_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - car_tp_copy).count();
            if (car_valid_copy && car_age_ms <= tag_timeout_ms_) {
                at_copy.avgPos.pos = car_pos_copy;   // bearing X/Z, Y/Z, 0
                at_copy.avgPos.valid = true;
            } else {
                at_copy.avgPos.valid = false;        // car kayip -> gecersiz
            }
        }

        // DEBUG: gonderilen avgPos'u 1 sn'de bir bas (debug=true iken).
        if (debug_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[TX] avgPos valid=%d pos=(%.3f, %.3f, %.3f)%s",
                (int)at_copy.avgPos.valid,
                at_copy.avgPos.pos.x, at_copy.avgPos.pos.y, at_copy.avgPos.pos.z,
                car_avg_inject_ ? "  [CAR-INJECT]" : "");
        }

        jetsonOtopilotPack_ut pkt;
        std::memset(pkt.data, 0, sizeof(jetsonOtopilotPack_t));

        pkt.pack.header.sync_byte   = SYNC_BYTE;
        pkt.pack.header.msg_id      = MSG_ID;
        pkt.pack.header.payloadSize = static_cast<uint8_t>(sizeof(jetsonVariables_t));

        uint32_t hsum = 0;
        for (int i = 0; i < 3; ++i) hsum += pkt.data[i];
        pkt.pack.header.headerchk = static_cast<uint8_t>(~(hsum % 256) + 1);

        // HRVS — paket gonderim ani + monoton frame sayaci.
        // steady_clock: sistem saati step'lerinden etkilenmez (26_07_19
        // ucusundaki NTP sicramasi ROS clock'u da sicratmisti). Ayni
        // sifir noktasi (node acilisi), kamera-domain guard'ina girmez.
        pkt.pack.variables.hrvs.timeStamp  = (uint32_t)
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time_).count();
        pkt.pack.variables.hrvs.frameNumer = frame_number_;

        pkt.pack.variables.aprtiltag = at_copy;
        pkt.pack.variables.opticflow = of_copy;

        uint32_t csum = 0;
        const int payStart = (int)sizeof(header_t);
        const int payEnd   = payStart + (int)sizeof(jetsonVariables_t);
        for (int i = payStart; i < payEnd; ++i) csum += pkt.data[i];
        pkt.pack.chksum = static_cast<uint8_t>(~(csum % 256) + 1);

        switch (lps::writePacket(serial_fd_, pkt.data, sizeof(jetsonOtopilotPack_t))) {
            case lps::WriteResult::OK:
                break;
            case lps::WriteResult::PARTIAL:
                // Paket yarim gitti — firmware sync_byte+chksum ile resync yapar,
                // bu paket kayip. Sik goruluyorsa baud/send_rate orani yanlis demektir.
                partial_write_count_++;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Serial PARTIAL write (toplam %u) — paket kayip, firmware resync eder",
                    partial_write_count_);
                break;
            case lps::WriteResult::FAILED:
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Serial write hatasi: %s", strerror(errno));
                break;
        }
    }

    /* -------------------------------------------------------------- */
    /* Debug log — executor thread'inde 1 Hz timer. TX thread'inden    */
    /* AYRI: konsol I/O'su bloklarsa paket zamanlamasi etkilenmesin.   */
    /* -------------------------------------------------------------- */
    void debugLog()
    {
        apriltagPack_t  at_copy;
        opticFlowPack_t of_copy;
        cv::Vec3f  funnel_copy;
        uint8_t    funnel_seen, funnel_used;
        std::vector<PerTagLanding> per_tag_copy;
        std::chrono::steady_clock::time_point last_tag_tp;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            at_copy      = apriltag_;
            of_copy      = opticflow_;
            funnel_copy  = funnel_landing_;
            funnel_seen  = funnel_landing_n_seen_;
            funnel_used  = funnel_landing_n_used_;
            per_tag_copy = funnel_per_tag_;
            last_tag_tp  = last_tag_msg_tp_;
        }
        auto tag_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_tag_tp).count();
        bool stale = tag_age_ms > tag_timeout_ms_;

        // RCLCPP_INFO(get_logger(),
        //     "[APRILTAG] valid=%s%s pos=(%.3f, %.3f, %.3f) rpy_deg=(%.1f, %.1f, %.1f)",
        //     (at_copy.avgPos.validity.valid && !stale) ? "Y" : "N",
        //     stale ? " (STALE)" : "",
        //     at_copy.avgPos.pos.x, at_copy.avgPos.pos.y, at_copy.avgPos.pos.z,
        //     at_copy.avgPos.angle.roll,
        //     at_copy.avgPos.angle.pitch,
        //     at_copy.avgPos.angle.yaw);

        // RCLCPP_INFO(get_logger(),
        //     "[FUNNEL]   seen=%u used=%u landing(cam)=(%.3f, %.3f, %.3f)",
        //     funnel_seen, funnel_used,
        //     funnel_copy[0], funnel_copy[1], funnel_copy[2]);

        // for (const auto &pt : per_tag_copy) {
        //     RCLCPP_INFO(get_logger(),
        //         "  ID %2d %s raw=(%.3f, %.3f, %.3f) landing=(%.3f, %.3f, %.3f)",
        //         pt.id, pt.used ? "OK " : "OUT",
        //         pt.raw[0], pt.raw[1], pt.raw[2],
        //         pt.landing[0], pt.landing[1], pt.landing[2]);
        // }

        RCLCPP_INFO(get_logger(),
            "[OPTFLOW]  dx=%.3f dy=%.3f quality=%u consist=%.2f tex=%.2f "
            "grad_p50=%u grad_p90=%u thr=%.0f",
            of_copy.motionPos.x, of_copy.motionPos.y, of_copy.quality,
            flow_consist_dbg_.load(), flow_tex_dbg_.load(),
            (unsigned)flow_p50_dbg_.load(), (unsigned)flow_p90_dbg_.load(),
            flow_thr_dbg_.load());
    }

    /** Kamera-domain timestamp'ini (ns) node-baslangic tabanli ms'ye cevirir.
     *  Tum paket timestamp'leri bu tabani kullanir — 0'dan baslar,
     *  ayni tabanda oldugu icin alanlar arasi fark alinabilir.
     *
     *  SICRAMA KORKULUGU (26_07_19 ucusu: sistem saati ucus ortasinda
     *  +2g 3.5s step atti, HUD t 18s→185403s; NTP bayat-saat duzeltmesi):
     *  ardisik damgalar arasi fark ±2 s'yi asarsa taban kaydirilir, eksen
     *  nominal 33 ms ilerleyerek SUREKLI kalir. Tag damgalari goruntuden
     *  ~100 ms geride gelir; kucuk negatif farklar normaldir, esige girmez.
     *  Gecis aninda kuyruktaki eski-domain mesajlari 1-2 rebase daha
     *  tetikleyebilir; her rebase en fazla 33 ms hata enjekte eder. */
    uint32_t msSinceStart(uint64_t stamp_ns)
    {
        static constexpr int64_t kStampJumpNs = 2000000000LL;  // 2 s
        static constexpr int64_t kNominalDtNs =   33000000LL;  // ~30fps
        std::lock_guard<std::mutex> lk(ts_guard_mutex_);
        const int64_t s = (int64_t)stamp_ns;
        if (last_stamp_ns_ != 0) {
            const int64_t d = s - last_stamp_ns_;
            if (d > kStampJumpNs || d < -kStampJumpNs) {
                t0_ns_ += d - kNominalDtNs;
                RCLCPP_WARN(get_logger(),
                    "[TS-GUARD] timestamp sicramasi yutuldu: %+.2f s "
                    "(saat step'i / surucu domain degisimi). Eksen surekli.",
                    (double)d / 1e9);
            }
        }
        last_stamp_ns_ = s;
        const int64_t rel = s - t0_ns_;
        if (rel <= 0) return 0;
        return static_cast<uint32_t>(rel / 1000000LL);
    }

    /* params */
    std::string serial_port_, tag_topic_, image_topic_;
    int    baud_rate_ = 115200;
    double send_rate_hz_ = 50.0;
    bool   debug_ = true;
    float  apriltag_node_size_ = 0.25f;   // tag25h9 huni duvar buyuk tag = 250mm
    int    tag_timeout_ms_ = 500;
    int    tx_cpu_ = 5;   // TX thread'in sabitlendigi core (-1 = kapali)

    /* Tag offset-based landing estimation (debug only, serial'a gitmiyor)
       Offset'ler camera frame'inde, R = 0.30m varsayimi (mekanik olcumle revize edilecek) */
    // Offset'ler TAG-FRAME'de (o vektoru): huni merkezi = tag_pos + R*offset,
    // R = tag'in olculen oryantasyonu (montaj egimi + drone attitude birlikte).
    // 26_08_11 KALIBRASYONU — 3 huni log'u birlikte cozuldu (co-visibility
    // kisiti: tum tag'ler ayni merkezi gostersin). Sonuc: 4 tag merkezi
    // 3-5 mm icinde ayni noktada bulusuyor (per-tag consensus sapmasi).
    // Eski duz-zemin (+-0.30, Z=0) degerleri 40-51° montaj egimini yok
    // sayiyordu; buradaki Z bileseni (~-0.06..-0.18) o egimi tasiyor.
    // Tag 24 (E) en zayif belirlenen (~1-2 cm); ucusta ortalama+outlier yumusatir.
    // ID'ler tag36h11 (dik yapistirma): 0(N)/20(E)/24(S)/36(W). W tekrar 36
    // (36h11'de var). Offset'ler yeni yapistirma sonrasi funnel log'undan
    // centroid-anchor ile kalibre edilecek.
    const std::map<int, TagMeta>  tag_meta_ = {
        // ===== tag25h9 HUNI — KALIBRE (26_08_20, funnel_20260820_163329) =====
        // Hedef = DIP (kucuk tag centroid = asil inis noktasi). Her tag:
        //   o_i = mean_kare R_i^-1(kucuk_centroid - T_i). 8 tag da AYNI dip noktayi
        //   hedefler -> avgPos her yerde tutarli. Dogrulama: uyum medyan 3.6mm,
        //   duvar-kucuk merkez 3B farki 3.7mm (Z farki 172mm->3.7mm), k-fold std
        //   0.1-0.4mm. Gauge-free (dip centroid anchor). Duvar size 0.25, kucuk 0.077.
        { 0, {cv::Vec3f(+0.0023f, +0.2998f, -0.0712f), 0.25f}},  // N
        {20, {cv::Vec3f(-0.0019f, +0.2986f, -0.0789f), 0.25f}},  // E
        {24, {cv::Vec3f(+0.0027f, +0.3008f, -0.0735f), 0.25f}},  // S
        {34, {cv::Vec3f(+0.0014f, +0.2993f, -0.0757f), 0.25f}},  // W
        // Alt-orta KUCUK tag'ler (tag25h9, 77mm) ID 10/12/14/16 (2x2: 10=sol-ust
        //   12=sag-ust 14=sol-alt 16=sag-alt). avgPos'a katilir (paket slotu YOK).
        {10, {cv::Vec3f(+0.0429f, +0.0430f, -0.0010f), 0.077f}},  // sol-ust
        {12, {cv::Vec3f(-0.0435f, +0.0432f, +0.0045f), 0.077f}},  // sag-ust
        {14, {cv::Vec3f(+0.0432f, -0.0427f, -0.0095f), 0.077f}},  // sol-alt
        {16, {cv::Vec3f(-0.0434f, -0.0428f, +0.0010f), 0.077f}},  // sag-alt
    };

    struct PerTagLanding {
        int       id;
        cv::Vec3f raw;       // tag merkez pozisyonu camera frame'de (scaled, offset YOK)
        cv::Vec3f landing;   // raw + offset (landing tahmini)
        bool      used;      // false ise outlier reject'lendi
    };

    /* IR-DISABLED: IR projektor detector (ir_projector_detector.hpp)
    lps::IRProjectorDetector ir_detector_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_debug_pub_;
    */

    /* serial */
    int      serial_fd_ = -1;
    uint32_t frame_number_ = 0;          // monoton paket sayaci
    uint32_t partial_write_count_ = 0;

    /* subscriptions & debug timer */
    rclcpp::Subscription<
        isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr car_sub_;  // TEST
    rclcpp::TimerBase::SharedPtr debug_timer_;

    /* TX thread — sabit periyotlu paket gonderimi */
    std::thread       tx_thread_;
    std::atomic<bool> tx_running_{true};

    /* paylasilan sensor verisi — spin thread ↔ vpi thread */
    std::mutex      data_mutex_;
    apriltagPack_t  apriltag_{};
    opticFlowPack_t opticflow_{};
    std::chrono::steady_clock::time_point last_tag_msg_tp_{};   // epoch = "hic gelmedi"

    /* TEST: car_detector bearing enjeksiyonu (car_avg_inject) */
    bool        car_avg_inject_ = false;
    bool        car_valid_ = false;
    float_3D_t  car_pos_{};
    std::chrono::steady_clock::time_point car_tp_{};
    cv::Vec3f funnel_landing_{0, 0, 0};
    uint8_t   funnel_landing_n_seen_ = 0;
    uint8_t   funnel_landing_n_used_ = 0;
    std::vector<PerTagLanding> funnel_per_tag_;

    /* frame kuyrugu — spin thread → vpi thread (tek slot, eskiyi ezer) */
    std::mutex                 frame_mutex_;
    std::condition_variable    frame_cv_;
    cv_bridge::CvImageConstPtr latest_frame_ptr_;   // ROS msg'yi canli tutar
    uint64_t                   latest_frame_ns_ = 0;
    bool                       new_frame_ = false;
    std::atomic<bool>          vpi_running_{true};
    // Quality bilesenleri — pakete girmez, 1 Hz debug logu icindir
    std::atomic<float>         flow_consist_dbg_{0.0f};
    std::atomic<float>         flow_tex_dbg_{0.0f};
    std::atomic<uint8_t>       flow_p50_dbg_{0};
    std::atomic<uint8_t>       flow_p90_dbg_{0};
    std::atomic<float>         flow_thr_dbg_{0.0f};
    std::thread                vpi_thread_;

    std::unique_ptr<lps::OpticalFlowVPI> flow_;

    /* ucus video kaydi (opsiyonel, record_video param) */
    std::unique_ptr<lps::DebugRecorder> recorder_;
    std::ofstream flow_csv_;    // ham akis logu (VPI thread'inden yazilir)
    uint64_t      csv_lines_ = 0;
    std::ofstream funnel_csv_;  // huni kalibrasyon logu (tagCb'den yazilir)
    uint64_t      funnel_csv_lines_ = 0;
    std::vector<lps::DebugRecorder::TagOverlay> tag_overlays_;  // data_mutex_

    std::chrono::steady_clock::time_point start_time_;
    int64_t t0_ns_ = 0;         // ortak sifir noktasi (ns); TS-GUARD kaydirabilir
    int64_t last_stamp_ns_ = 0; // TS-GUARD: son gorulen kamera-domain damgasi
    std::mutex ts_guard_mutex_; // msSinceStart cok thread'den cagriliyor
};

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Tek thread'li executor: tag + img callback'leri sirayla calisir.
    // VPI ve serial TX kendi thread'lerinde (node icinde).
    rclcpp::spin(std::make_shared<SerialSenderNode>());
    rclcpp::shutdown();
    return 0;
}
