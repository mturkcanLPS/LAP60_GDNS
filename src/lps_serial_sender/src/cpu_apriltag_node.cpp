/**
 * cpu_apriltag_node — CPU AprilTag (AprilRobotics libapriltag) tespiti
 *
 * Isaac ROS (GPU/cuAprilTags) ile A/B icin: AYNI mesaj tipini
 * (isaac_ros_apriltag_interfaces/AprilTagDetectionArray) yayinlar, boylece
 * serial_sender degismeden calisir. Isaac'in vermedigi knob'lar burada acik:
 *   quad_decimate, quad_sigma, decode_sharpening, refine_edges, nthreads
 * ve tag_family (36h11 / 25h9 / 16h5 ...) parametre.
 *
 * Kullanim: launch'ta Isaac container yerine (ya da yaninda ayri topic'e)
 * bu node'u calistir. Ayni anda ikisini farkli output_topic ile kosturup
 * serial_sender'i birine baglayarak canli kiyas yapilabilir.
 *
 * Poz: libapriltag'in estimate_tag_pose'u camera_info intrinsics + tag_size
 * ile R,t verir; position=t, orientation=R->quaternion olarak doldurulur
 * (serial_sender'in bekledigi det.pose.pose.pose ile ayni).
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>

#include <atomic>
#include <cmath>
#include <cstring>
#include <string>

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/common/matd.h"   // MATD_EL makrosu (matd_get export edilmemis)
#include "apriltag/tag36h11.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
}

class CpuAprilTagNode : public rclcpp::Node
{
public:
    CpuAprilTagNode() : Node("cpu_apriltag")
    {
        image_topic_  = declare_parameter<std::string>("image_topic",
                            "/camera/camera/infra1/image_rect_raw");
        cinfo_topic_  = declare_parameter<std::string>("camera_info_topic",
                            "/camera/camera/infra1/camera_info");
        output_topic_ = declare_parameter<std::string>("output_topic", "/tag_detections");
        family_name_  = declare_parameter<std::string>("tag_family", "tag36h11");
        tag_size_     = declare_parameter<double>("size", 0.25);  // tag25h9 huni duvar buyuk tag=250mm

        // --- AprilTag knob'lari (Isaac'in vermedigi) ---
        //   quad_decimate: tespit downsample'i. 1.0=tam cozunurluk (kucuk/uzak
        //     tag'ler), 2.0=hizli ama kucuk tag kacar. Bizim derdimize 1.0 iyi.
        //   quad_sigma: segmentasyon oncesi blur (gurultulu goruntude faydali)
        //   decode_sharpening: bit cozerken keskinlestirme (bulanik/dusuk
        //     kontrast/egik tag'ler icin 0.25-1.0)
        //   refine_edges: koseleri incelt (poz dogrulugu)
        double quad_decimate    = declare_parameter<double>("quad_decimate", 1.0);
        double quad_sigma       = declare_parameter<double>("quad_sigma", 0.0);
        double decode_sharpen   = declare_parameter<double>("decode_sharpening", 0.25);
        bool   refine_edges     = declare_parameter<bool>("refine_edges", true);
        int    nthreads         = declare_parameter<int>("nthreads", 2);

        td_ = apriltag_detector_create();
        tf_ = makeFamily(family_name_);
        if (!tf_) {
            RCLCPP_FATAL(get_logger(), "Bilinmeyen tag_family: %s", family_name_.c_str());
            throw std::runtime_error("bad tag_family");
        }
        // add_family = add_family_bits(...,2): 2 bit duzeltmeye izin. tag36h11
        // (hamming 11) ile false positive zaten yok -> circle icin eklenen
        // max_hamming/decision_margin knob'lari kaldirildi.
        apriltag_detector_add_family(td_, tf_);
        td_->quad_decimate    = (float)quad_decimate;
        td_->quad_sigma       = (float)quad_sigma;
        td_->refine_edges     = refine_edges;
        td_->decode_sharpening = decode_sharpen;
        td_->nthreads         = nthreads;

        pub_ = create_publisher<
            isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
            output_topic_, rclcpp::SensorDataQoS());

        cinfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            cinfo_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CpuAprilTagNode::cinfoCb, this, std::placeholders::_1));

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CpuAprilTagNode::imgCb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "CPU AprilTag: family=%s size=%.3f decimate=%.1f sharpen=%.2f "
            "sigma=%.1f | %s -> %s",
            family_name_.c_str(), tag_size_, quad_decimate, decode_sharpen,
            quad_sigma, image_topic_.c_str(), output_topic_.c_str());
    }

    ~CpuAprilTagNode()
    {
        if (td_) apriltag_detector_destroy(td_);
        if (tf_) destroyFamily(family_name_, tf_);
    }

private:
    apriltag_family_t *makeFamily(const std::string &n)
    {
        if (n == "tag36h11")        return tag36h11_create();
        if (n == "tag36h10")        return tag36h10_create();
        if (n == "tag25h9")         return tag25h9_create();
        if (n == "tag16h5")         return tag16h5_create();
        if (n == "tagCircle21h7")   return tagCircle21h7_create();
        if (n == "tagCircle49h12")  return tagCircle49h12_create();
        if (n == "tagCustom48h12")  return tagCustom48h12_create();
        if (n == "tagStandard41h12")return tagStandard41h12_create();
        if (n == "tagStandard52h13")return tagStandard52h13_create();
        return nullptr;
    }
    void destroyFamily(const std::string &n, apriltag_family_t *tf)
    {
        if (n == "tag36h11")        tag36h11_destroy(tf);
        else if (n == "tag36h10")   tag36h10_destroy(tf);
        else if (n == "tag25h9")    tag25h9_destroy(tf);
        else if (n == "tag16h5")    tag16h5_destroy(tf);
        else if (n == "tagCircle21h7")   tagCircle21h7_destroy(tf);
        else if (n == "tagCircle49h12")  tagCircle49h12_destroy(tf);
        else if (n == "tagCustom48h12")  tagCustom48h12_destroy(tf);
        else if (n == "tagStandard41h12") tagStandard41h12_destroy(tf);
        else if (n == "tagStandard52h13") tagStandard52h13_destroy(tf);
    }

    void cinfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // K = [fx 0 cx; 0 fy cy; 0 0 1]
        fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
        have_cinfo_ = (fx_ > 0.0 && fy_ > 0.0);
    }

    // 3x3 rotasyon matrisi -> quaternion (x,y,z,w). matd_get export edilmemis,
    // MATD_EL makrosuyla dogrudan data[] erisimi (satir-major, ncols=3).
    static void rotToQuat(const matd_t *R, double &qx, double &qy, double &qz, double &qw)
    {
        double m00=MATD_EL(R,0,0), m01=MATD_EL(R,0,1), m02=MATD_EL(R,0,2);
        double m10=MATD_EL(R,1,0), m11=MATD_EL(R,1,1), m12=MATD_EL(R,1,2);
        double m20=MATD_EL(R,2,0), m21=MATD_EL(R,2,1), m22=MATD_EL(R,2,2);
        double tr = m00 + m11 + m22;
        if (tr > 0) {
            double s = std::sqrt(tr + 1.0) * 2;
            qw = 0.25 * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            double s = std::sqrt(1.0 + m00 - m11 - m22) * 2;
            qw = (m21 - m12) / s; qx = 0.25 * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
        } else if (m11 > m22) {
            double s = std::sqrt(1.0 + m11 - m00 - m22) * 2;
            qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25 * s; qz = (m12 + m21) / s;
        } else {
            double s = std::sqrt(1.0 + m22 - m00 - m11) * 2;
            qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25 * s;
        }
    }

    void imgCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!have_cinfo_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "camera_info bekleniyor (poz icin intrinsics lazim)");
            return;
        }
        cv::Mat gray;
        try {
            gray = cv_bridge::toCvShare(msg, "mono8")->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
            return;
        }
        if (gray.empty()) return;

        image_u8_t im{ gray.cols, gray.rows, (int)gray.step, gray.data };
        zarray_t *dets = apriltag_detector_detect(td_, &im);

        isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray out;
        out.header = msg->header;

        const int n = zarray_size(dets);
        for (int i = 0; i < n; ++i) {
            apriltag_detection_t *det;
            zarray_get(dets, i, &det);

            apriltag_detection_info_t info;
            info.det = det; info.tagsize = tag_size_;
            info.fx = fx_; info.fy = fy_; info.cx = cx_; info.cy = cy_;
            apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);   // R (3x3), t (3x1) camera frame

            isaac_ros_apriltag_interfaces::msg::AprilTagDetection d;
            d.family = family_name_;
            d.id = det->id;
            d.center.x = det->c[0];
            d.center.y = det->c[1];
            for (int k = 0; k < 4; ++k) {
                d.corners[k].x = det->p[k][0];
                d.corners[k].y = det->p[k][1];
            }
            d.pose.header = msg->header;
            d.pose.pose.pose.position.x = MATD_EL(pose.t, 0, 0);
            d.pose.pose.pose.position.y = MATD_EL(pose.t, 1, 0);
            d.pose.pose.pose.position.z = MATD_EL(pose.t, 2, 0);
            double qx, qy, qz, qw;
            rotToQuat(pose.R, qx, qy, qz, qw);
            d.pose.pose.pose.orientation.x = qx;
            d.pose.pose.pose.orientation.y = qy;
            d.pose.pose.pose.orientation.z = qz;
            d.pose.pose.pose.orientation.w = qw;
            out.detections.push_back(d);

            // matd_destroy export edilmemis; matd tek calloc blogu -> free() dogru
            free(pose.R);
            free(pose.t);
        }
        apriltag_detections_destroy(dets);
        pub_->publish(out);
    }

    // params
    std::string image_topic_, cinfo_topic_, output_topic_, family_name_;
    double tag_size_ = 0.25;

    apriltag_detector_t *td_ = nullptr;
    apriltag_family_t   *tf_ = nullptr;

    double fx_=0, fy_=0, cx_=0, cy_=0;
    std::atomic<bool> have_cinfo_{false};

    rclcpp::Publisher<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CpuAprilTagNode>());
    rclcpp::shutdown();
    return 0;
}
