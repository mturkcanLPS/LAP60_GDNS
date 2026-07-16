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
        t0_ns_ = (uint64_t)this->now().nanoseconds();

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

        // AprilTag node'unda set'lenmis size parametresi — per-ID size scaling icin referans
        apriltag_node_size_ = (float)declare_parameter<double>("apriltag_node_size", 0.984);

        // Tag verisi bu kadar sure yenilenmezse valid=false gonderilir
        // (pipeline stall'inda bayat pozisyonu 50Hz'de tekrarlamamak icin)
        tag_timeout_ms_ = declare_parameter<int>("tag_timeout_ms", 500);

        // TX thread CPU sabitleme (-1 = kapali). Jetson'da tum IRQ'lar
        // CPU0'da toplaniyor; TX thread USB/GPU interrupt firtinasinin
        // oldugu core'a denk gelirse RT oncelige ragmen 50-70ms gec
        // uyanabiliyor (olculdu: yuk altinda ~2 sn'de bir). Sakin bir
        // core'a sabitlemek bu kuyrugu keser.
        tx_cpu_ = (int)declare_parameter<int>("tx_cpu", 5);

        // Doku kapisi TABANI. Efektif esik sahneye uyarlanir:
        //   esik = max(taban, 1.5 x grad_p50)   (p50 = anlik gurultu tabani)
        // debug=true iken [OPTFLOW] logundaki thr= ile izlenir.
        float flow_tex = (float)declare_parameter<double>("flow_texture_thresh", 8.0);

        flow_ = std::make_unique<lps::OpticalFlowVPI>(grid_size, num_levels,
                                                      ema_alpha, flow_tex);

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

        serial_fd_ = lps::openSerial(serial_port_, baud_rate_);

        tag_sub_ = create_subscription<
            isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            tag_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SerialSenderNode::tagCb, this, std::placeholders::_1));

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

        for (const auto &det : msg->detections) {
            if (det.id != 45) continue;

            const auto &p = det.pose.pose.pose;
            ap.avgPos.valid = true;
            ap.avgPos.pos.x = (float)p.position.x;
            ap.avgPos.pos.y = (float)p.position.y;
            ap.avgPos.pos.z = (float)p.position.z;
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

        // === Offset-based funnel landing estimation (debug only) ===
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
            cv::Vec3f tag_pos(
                (float)p.x * scale,
                (float)p.y * scale,
                (float)p.z * scale);

            // Offset direkt camera frame'inde — yaw rotasyonu yok
            // (drone level + heading sabit varsayimi; kamera egikse tahmini
            //  otopilot IMU/ivme ile duzeltir)
            raws.push_back(tag_pos);
            estimates.push_back(tag_pos + meta.offset);
            used_ids.push_back(det.id);
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

        // Video overlay icin TUM tespitlerin koseleri (recorder aciksa)
        std::vector<lps::DebugRecorder::TagOverlay> overlays;
        if (recorder_) {
            overlays.reserve(msg->detections.size());
            for (const auto &det : msg->detections) {
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
            apriltag_               = ap;
            last_tag_msg_tp_        = std::chrono::steady_clock::now();
            funnel_landing_         = funnel;
            funnel_landing_n_seen_  = (uint8_t)estimates.size();
            funnel_landing_n_used_  = (uint8_t)n_inliers;
            funnel_per_tag_         = std::move(per_tag);
            tag_overlays_           = std::move(overlays);
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
    /* Tum govde try/catch icinde: VPI hatasi node'u COKERTMEZ         */
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

    /* -------------------------------------------------------------- */
    /* TX thread — sabit frekansta seri porta paket gonderir           */
    /* Mutlak deadline pacing: executor'daki callback yogunlugundan    */
    /* etkilenmez. Gec kalinirsa telafi paketi hemen gider, hicbir     */
    /* slot atlanmaz (paket kaybi yerine bir kerelik sik aralik).      */
    /* -------------------------------------------------------------- */
    void txThread()
    {
        // RT oncelik dene — wakeup latency'yi dusurur. Izin yoksa (root /
        // CAP_SYS_NICE gerekir) sessizce normal scheduler'da devam eder.
        {
            sched_param sp{};
            sp.sched_priority = 50;
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
                RCLCPP_INFO(get_logger(),
                    "TX thread RT oncelik alinamadi (izin yok) — normal scheduler");
            }
        }

        // IRQ firtinasindan uzak sakin bir core'a sabitle (tx_cpu param)
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
                // Gec kalindi — slotu ATLAMA (eski ROS timer semantigi):
                // telafi paketi hemen gitsin, takvim ondan sonra otursun.
                // Boylece stall paket KAYBINA degil, bir kerelik sik
                // aralikli iki pakete donusur.
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
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            at_copy     = apriltag_;
            of_copy     = opticflow_;
            last_tag_tp = last_tag_msg_tp_;
        }

        // Staleness: tag pipeline'i durduysa bayat "valid" pozu gondermeye devam etme
        auto tag_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_tag_tp).count();
        if (tag_age_ms > tag_timeout_ms_) {
            at_copy.avgPos.valid = false;
        }

        jetsonOtopilotPack_ut pkt;
        std::memset(pkt.data, 0, sizeof(jetsonOtopilotPack_t));

        pkt.pack.header.sync_byte   = SYNC_BYTE;
        pkt.pack.header.msg_id      = MSG_ID;
        pkt.pack.header.payloadSize = static_cast<uint8_t>(sizeof(jetsonVariables_t));

        uint32_t hsum = 0;
        for (int i = 0; i < 3; ++i) hsum += pkt.data[i];
        pkt.pack.header.headerchk = static_cast<uint8_t>(~(hsum % 256) + 1);

        // HRVS — paket gonderim ani (uptime ms) + monoton frame sayaci
        pkt.pack.variables.hrvs.timeStamp  =
            msSinceStart((uint64_t)this->now().nanoseconds());
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
        //     (at_copy.avgPos.valid && !stale) ? "Y" : "N",
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

    /** ROS clock timestamp'ini (ns) node-baslangic tabanli ms'ye cevirir.
     *  Tum paket timestamp'leri bu tabani kullanir — 0'dan baslar,
     *  ayni tabanda oldugu icin alanlar arasi fark alinabilir. */
    uint32_t msSinceStart(uint64_t stamp_ns) const
    {
        if (stamp_ns <= t0_ns_) return 0;
        return static_cast<uint32_t>((stamp_ns - t0_ns_) / 1000000ULL);
    }

    /* params */
    std::string serial_port_, tag_topic_, image_topic_;
    int    baud_rate_ = 921600;
    double send_rate_hz_ = 20.0;
    bool   debug_ = true;
    float  apriltag_node_size_ = 0.984f;
    int    tag_timeout_ms_ = 500;
    int    tx_cpu_ = 5;   // TX thread'in sabitlendigi core (-1 = kapali)

    /* Tag offset-based landing estimation (debug only, serial'a gitmiyor)
       Offset'ler camera frame'inde, R = 0.30m varsayimi (mekanik olcumle revize edilecek) */
    const std::map<int, TagMeta> tag_meta_ = {
        // N tag — kamera frame'de -Y'de gorunur, offset +Y ile ortalar
        { 0, {cv::Vec3f( 0.000f, +0.300f, 0.0f), 0.280f}},
        // S tag — kamera frame'de +Y'de gorunur, offset -Y
        {20, {cv::Vec3f( 0.000f, -0.300f, 0.0f), 0.280f}},
        // E tag — kamera frame'de +X'te gorunur, offset -X
        {24, {cv::Vec3f(-0.300f,  0.000f, 0.0f), 0.280f}},
        // W tag — kamera frame'de -X'te gorunur, offset +X
        {36, {cv::Vec3f(+0.300f,  0.000f, 0.0f), 0.280f}},
        // Buyuk debug/uzak tag — huni merkezinde varsayilan (offset=0)
        {45, {cv::Vec3f( 0.000f,  0.000f, 0.0f), 0.984f}},
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
    rclcpp::TimerBase::SharedPtr debug_timer_;

    /* TX thread — sabit periyotlu paket gonderimi */
    std::thread       tx_thread_;
    std::atomic<bool> tx_running_{true};

    /* paylasilan sensor verisi — spin thread ↔ vpi thread */
    std::mutex      data_mutex_;
    apriltagPack_t  apriltag_{};
    opticFlowPack_t opticflow_{};
    std::chrono::steady_clock::time_point last_tag_msg_tp_{};   // epoch = "hic gelmedi"
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
    std::vector<lps::DebugRecorder::TagOverlay> tag_overlays_;  // data_mutex_

    std::chrono::steady_clock::time_point start_time_;
    uint64_t t0_ns_ = 0;   // tum timestamp'lerin ortak sifir noktasi (ROS ns)
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
