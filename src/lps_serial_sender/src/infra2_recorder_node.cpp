/**
 * infra2_recorder_node — IKINCI IR kamera (infra2): optik akis + video kaydi
 *
 * infra2 goruntusu uzerinde VPI optik akis hesaplar ve sonucu yayinlar
 * (flow_topic, geometry_msgs/Vector3Stamped: x=dx, y=dy, z=quality 0-100;
 * header.stamp = akisin hesaplandigi karenin kamera zamani). serial_sender bu
 * topic'i ve /tag_detections_infra2'yi dinleyip paketin SONUNDAKI
 * opticflow2 / aprtiltag2 alanlarina koyar. Bu node seri portu kendisi
 * YAZMAZ (seri kod icermiyor); paketin tek yazari serial_sender.
 *
 * Ayrica AprilTag fuzyonu + akisi infra1 videosuyla AYNI overlay'le
 * (debug_recorder.hpp) lps_infra2_*.avi olarak kaydeder. infra1 verisiyle
 * BIRLESTIRILMEZ: infra2 alanlari pakette ayri, kendi alanlarinda gider.
 *
 * Kiyas icin hesap infra1 ile BIREBIR ayni:
 *   - tag -> avgPos: tag_fusion.hpp (serial_sender ile ortak; mod TEST_APRITLAG)
 *   - optik akis: optical_flow_vpi.hpp (ayni sinif, launch'ta ayni parametreler)
 * Beklenen tek sistematik fark: stereo baseline. infra2, infra1'in 50.06 mm
 * saginda (D435i kalibrasyonu, rs-enumerate-devices -c) -> ayni sahnede
 * infra2'nin pos.x'i ~0.050 m KUCUK cikar. Bu hata degil, kamera konumu.
 *
 * Ucus yolunu korumak icin launch'ta 'nice -n 10' ile dusuk oncelikte kosar.
 */
#include <rclcpp/rclcpp.hpp>
#include "lps_serial_sender/stamp_seconds.hpp"     // damga -> saniye, istisnasiz (20_10)
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "lps_serial_sender/optical_flow_vpi.hpp"
#include "lps_serial_sender/debug_recorder.hpp"
#include "lps_serial_sender/tag_fusion.hpp"
#include "lps_serial_sender/frame_gap_stats.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class Infra2RecorderNode : public rclcpp::Node
{
public:
    Infra2RecorderNode() : Node("infra2_recorder")
    {
        start_time_ = std::chrono::steady_clock::now();

        image_topic_ = declare_parameter<std::string>("image_topic",
                           "/camera/camera/infra2/image_rect_raw");
        tag_topic_   = declare_parameter<std::string>("tag_topic", "/tag_detections_infra2");
        // serial_sender ile AYNI deger verilmeli (launch'ta ortak degiskenden gelir)
        node_size_   = (float)declare_parameter<double>("apriltag_node_size", 0.25);

        // Optik akis — serial_sender ile AYNI parametreler (launch'ta ortak sozluk)
        const int   grid   = declare_parameter<int>("grid_size", 4);
        const int   levels = declare_parameter<int>("num_levels", 3);
        const float ema    = (float)declare_parameter<double>("ema_alpha", 0.25);
        const float tex    = (float)declare_parameter<double>("flow_texture_thresh", 4.0);
        const float pre    = (float)declare_parameter<double>("flow_prefilter_sigma", 1.5);
        // OFA donanimi infra1 akisiyla paylasiliyor: infra1 '[OPTICFLOW] Hz'
        // duserse bu akis kapatilip (flow_enabled:=false) sadece tag kaydi alinabilir.
        flow_enabled_ = declare_parameter<bool>("flow_enabled", true);
        // Akis sonucu buradan yayinlanir -> serial_sender opticflow2'ye koyar.
        flow_topic_ = declare_parameter<std::string>("flow_topic", "/infra2/optical_flow");
        // infra1 ile ayni: akis kare-araligi duzenliligi (duzenli/atlanan %)
        gap_stats_ = std::make_unique<lps::FrameGapStats>(
            declare_parameter<double>("camera_fps", 30.0));

        const std::string dir = declare_parameter<std::string>("record_dir", "/home/lapis/lps_records");
        const double fps      = declare_parameter<double>("record_fps", 15.0);
        const double scale    = declare_parameter<double>("record_scale", 0.5);
        const std::string pfx = declare_parameter<std::string>("record_prefix", "lps_infra2");
        label_ = declare_parameter<std::string>("record_label",
                     "INFRA2 (seriale giden: aprtiltag2/opticflow2)");

        const int session     = declare_parameter<int>("record_session", 0);   // launch verir
        recorder_ = std::make_unique<lps::DebugRecorder>(dir, fps, scale, pfx, session);

        if (flow_enabled_) {
            flow_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
                flow_topic_, rclcpp::SensorDataQoS());
            flow_ = std::make_unique<lps::OpticalFlowVPI>(grid, levels, ema, tex, pre);
            vpi_thread_ = std::thread(&Infra2RecorderNode::vpiThread, this);
        }

        tag_sub_ = create_subscription<
            isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            tag_topic_, rclcpp::SensorDataQoS(),
            std::bind(&Infra2RecorderNode::tagCb, this, std::placeholders::_1));
        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&Infra2RecorderNode::imgCb, this, std::placeholders::_1));

        status_timer_ = create_wall_timer(std::chrono::seconds(5),
            std::bind(&Infra2RecorderNode::statusCb, this));

        RCLCPP_INFO(get_logger(),
            "INFRA2: img='%s' tag='%s' akis=%s (-> '%s', serial opticflow2) kayit -> %s",
            image_topic_.c_str(), tag_topic_.c_str(), flow_enabled_ ? "ACIK" : "KAPALI",
            flow_topic_.c_str(), recorder_->path().c_str());
    }

    ~Infra2RecorderNode()
    {
        vpi_running_ = false;
        frame_cv_.notify_all();
        if (vpi_thread_.joinable()) vpi_thread_.join();
        flow_.reset();       // VPI objeleri thread durduktan SONRA
        recorder_.reset();   // son kareyi yazar, dosyayi kapatir
    }

private:
    /* AprilTag (infra2) -> infra1 ile AYNI fuzyon (tag_fusion.hpp) */
    void tagCb(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        tag_cb_count_++;
        const lps::TagFusion f = lps::fuseTags(lps::toTagDets(*msg), node_size_);
        if (f.valid) tag_valid_count_++;

        std::vector<lps::DebugRecorder::TagOverlay> overlays;
        overlays.reserve(msg->detections.size());
        for (const auto &det : msg->detections) {
            lps::DebugRecorder::TagOverlay t;
            t.id = det.id;
            for (int i = 0; i < 4; ++i)
                t.corners[i] = cv::Point2f((float)det.corners[i].x, (float)det.corners[i].y);
            overlays.push_back(t);
        }
        std::lock_guard<std::mutex> lk(data_mutex_);
        fusion_       = f;
        tag_overlays_ = std::move(overlays);
    }

    /* Goruntu: akis thread'ine kopya + kayda state (kopyasiz, keepalive) */
    void imgCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        img_cb_count_++;
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
            return;
        }
        if (!cv_ptr || cv_ptr->image.empty()) return;

        // Pozlama sagligi (15 karede bir, 1/8 kucultulmus karede — ucuz).
        // 25_09 11:10 kaydinda infra2 8.5 dk boyunca %100 doygun (bembeyaz)
        // kaydetti ve kimse fark etmedi; durum satiri bunu artik bagiriyor.
        if (img_cb_count_ % 15 == 1) {
            cv::Mat s;
            cv::resize(cv_ptr->image, s, cv::Size(), 0.125, 0.125, cv::INTER_AREA);
            bright_x10_ = (int)(cv::mean(s)[0] * 10.0);
            sat_pct_    = (int)(100.0 * cv::countNonZero(s >= 250) / (double)s.total());
        }

        if (flow_enabled_) {
            {
                std::lock_guard<std::mutex> lk(frame_mutex_);
                latest_frame_ = cv_ptr->image.clone();   // VPI thread icin (serial_sender gibi)
                latest_stamp_ = msg->header.stamp;       // yayinlanan akisin zamani
                new_frame_    = true;
            }
            frame_cv_.notify_one();
        }

        lps::DebugRecorder::State st;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            st.tags      = tag_overlays_;
            st.tag_valid = fusion_.valid;
            st.px = fusion_.pos[0]; st.py = fusion_.pos[1]; st.pz = fusion_.pos[2];
            st.roll = fusion_.roll; st.pitch = fusion_.pitch; st.yaw = fusion_.yaw;
            st.dx      = flow_res_.dx;
            st.dy      = flow_res_.dy;
            st.quality = flow_res_.quality;
            st.consist = flow_res_.consist;     // infra1 HUD'unda 0; burada dolu
            st.tex     = flow_res_.tex_ratio;
            st.grad_p50 = flow_res_.tex_p50;
            st.grad_p90 = flow_res_.tex_p90;
        }
        st.ir_enabled = false;   // IR ucgeni sadece infra1'de (uçus yolu)
        st.t_ms    = msSinceStart();
        st.stamp_s = lps::stampSeconds(msg->header.stamp);
        st.label   = label_;
        recorder_->push(cv_ptr->image, cv_ptr, st);
    }

    /* VPI akisi — serial_sender::vpiThread ile ayni akis (seriale yazmadan) */
    void vpiThread()
    {
        while (vpi_running_.load()) {
            cv::Mat frame;
            builtin_interfaces::msg::Time stamp;
            {
                std::unique_lock<std::mutex> lk(frame_mutex_);
                frame_cv_.wait(lk, [this]{ return new_frame_ || !vpi_running_.load(); });
                if (!vpi_running_.load()) break;
                frame      = std::move(latest_frame_);
                stamp      = latest_stamp_;
                new_frame_ = false;
            }
            if (!flow_->initialized()) {
                try {
                    flow_->init(frame);
                    RCLCPP_INFO(get_logger(), "INFRA2 VPI OFA init: %dx%d -> padded %dx%d",
                                frame.cols, frame.rows,
                                flow_->paddedWidth(), flow_->paddedHeight());
                } catch (const std::exception &e) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "INFRA2 VPI init hatasi: %s", e.what());
                }
                continue;
            }
            lps::OpticalFlowVPI::Result r;
            try {
                flow_->process(frame, r);
            } catch (const std::exception &e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "INFRA2 VPI akis hatasi: %s", e.what());
                continue;
            }
            flow_count_++;
            gap_stats_->add(lps::stampSeconds(stamp));
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                flow_res_ = r;
            }
            // serial_sender -> opticflow2. infra1'in pakete koydugu alanlarin
            // AYNISI: EMA'li dx/dy (px/kare) + EMA'li kapsama-cezali quality.
            geometry_msgs::msg::Vector3Stamped m;
            m.header.stamp    = stamp;
            m.header.frame_id = "camera_infra2_optical_frame";
            m.vector.x = r.dx;
            m.vector.y = r.dy;
            m.vector.z = r.quality;
            flow_pub_->publish(m);
        }
    }

    /* 5 sn'de bir durum — kaydin canli oldugu sahada gorulsun */
    void statusCb()
    {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_status_tp_).count();
        last_status_tp_ = now;
        const uint32_t img = img_cb_count_.exchange(0), tag = tag_cb_count_.exchange(0);
        const uint32_t tv = tag_valid_count_.exchange(0), fl = flow_count_.exchange(0);
        lps::OpticalFlowVPI::Result r;
        { std::lock_guard<std::mutex> lk(data_mutex_); r = flow_res_; }
        int reg = -1, skip = -1;
        gap_stats_->take(reg, skip);
        RCLCPP_INFO(get_logger(),
            "[INFRA2] img %.1f Hz | tag %.1f Hz (gecerli %.0f%%) | akis %.1f Hz "
            "dx=%+.2f dy=%+.2f Q=%u | aralik duzenli %%%d atlanan %%%d | parlaklik %.0f doygun %%%d%s",
            img / dt, tag / dt, tag ? 100.0 * tv / tag : 0.0, fl / dt,
            r.dx, r.dy, (unsigned)r.quality, reg, skip,
            bright_x10_.load() / 10.0, sat_pct_.load(),
            recorder_->failed() ? " | KAYIT HATASI" : "");
        if (img == 0)
            RCLCPP_WARN(get_logger(), "[INFRA2] goruntu gelmiyor — enable_infra2 acik mi? (%s)",
                        image_topic_.c_str());
        else if (sat_pct_.load() >= 50)
            RCLCPP_WARN(get_logger(),
                "[INFRA2] goruntunun %%%d'i DOYGUN (beyaz) — kayit kullanilamaz. "
                "Sag lens (infra2) kapali/ortulu mu, onunde parlak bir sey var mi?",
                sat_pct_.load());
    }

    uint32_t msSinceStart() const
    {
        return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();
    }

    std::string image_topic_, tag_topic_, label_, flow_topic_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr flow_pub_;
    float node_size_ = 0.25f;
    bool  flow_enabled_ = true;

    rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::unique_ptr<lps::DebugRecorder>   recorder_;
    std::unique_ptr<lps::OpticalFlowVPI>  flow_;
    std::unique_ptr<lps::FrameGapStats>   gap_stats_;   // akis kare-araligi duzenliligi

    // tagCb/vpiThread yazar, imgCb/statusCb okur
    std::mutex data_mutex_;
    lps::TagFusion fusion_;
    std::vector<lps::DebugRecorder::TagOverlay> tag_overlays_;
    lps::OpticalFlowVPI::Result flow_res_;

    // VPI thread iletisimi
    std::thread             vpi_thread_;
    std::mutex              frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat                 latest_frame_;
    builtin_interfaces::msg::Time latest_stamp_;
    bool                    new_frame_ = false;
    std::atomic<bool>       vpi_running_{true};

    std::atomic<uint32_t> img_cb_count_{0}, tag_cb_count_{0}, tag_valid_count_{0}, flow_count_{0};
    std::atomic<int>      bright_x10_{0}, sat_pct_{0};   // pozlama sagligi (imgCb yazar)
    std::chrono::steady_clock::time_point start_time_, last_status_tp_{std::chrono::steady_clock::now()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Infra2RecorderNode>());
    rclcpp::shutdown();
    return 0;
}
