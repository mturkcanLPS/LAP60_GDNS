#include <rclcpp/rclcpp.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <sstream>
#include <string>

#define SYNC_BYTE 0x33
#define MSG_ID 0x11

class UdpSender {
public:
  UdpSender(const std::string& host, int port)
  {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      throw std::runtime_error("UDP socket() failed");
    }

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port));

    if (::inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) != 1) {
      ::close(sock_);
      throw std::runtime_error("inet_pton failed for host=" + host);
    }
  }

  ~UdpSender() {
    if (sock_ >= 0) ::close(sock_);
  }

  void sendPacket(const char* msg, size_t size) {
    if (sock_ < 0) return;
    ::sendto(sock_, msg, size, 0,
             reinterpret_cast<const sockaddr*>(&addr_), sizeof(addr_));
  }

private:
  int sock_{-1};
  sockaddr_in addr_{};
};
#pragma pack(1)

typedef struct  {
  uint8_t sync_byte;
  uint8_t msgID;
  uint8_t payloadSize;
  uint8_t hchk;
}Header_t;
typedef struct {
  uint32_t timestamp;
  uint16_t tag_id;
  uint8_t validity;
  uint64_t sec;
  uint64_t nsec;
  int16_t pos_x;
  int16_t pos_y;
  int16_t pos_z;
  int16_t orientation_x;
  int16_t orientation_y;
  int16_t orientation_z;
  int16_t orientation_w;
}Tag_packet_t;

typedef struct {
  Header_t header;
  Tag_packet_t tag;
  uint8_t pchk;
}ApRos_packet_t;

typedef union {
    
    ApRos_packet_t  pack;
    uint8_t data[sizeof(ApRos_packet_t)];

}jetsonApRosPack_ut;

#pragma pack()

class AprilTagUdpNode : public rclcpp::Node
{
public:
  AprilTagUdpNode() : Node("apriltag_udp_sender")
  {
    start_time_ = std::chrono::steady_clock::now();
    topic_ = declare_parameter<std::string>("topic", "/tag_detections");
    host_  = declare_parameter<std::string>("host", "127.0.0.1");
    port_  = declare_parameter<int>("port", 5545);
    target_id_ = declare_parameter<int>("target_id", -1); // -1 = hepsi

    sender_ = std::make_unique<UdpSender>(host_, port_);

    sub_ = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      topic_, rclcpp::SensorDataQoS(),
      std::bind(&AprilTagUdpNode::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening %s -> UDP %s:%d (target_id=%d)",
                topic_.c_str(), host_.c_str(), port_, target_id_);
  }

private:
  void cb(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {

    jetsonApRosPack_ut packet;
    memset(packet.data, 0x00, sizeof(ApRos_packet_t));
    packet.pack.header.sync_byte = SYNC_BYTE;
    packet.pack.header.msgID = MSG_ID;
    packet.pack.header.payloadSize = sizeof(Tag_packet_t);
    uint32_t tempChk = 0;
    for (int i = 0; i < 3; i++)
    {
        tempChk += packet.data[i];
    }
    packet.pack.header.hchk = tempChk % 256;
    packet.pack.header.hchk = ~(packet.pack.header.hchk) + 1;

    packet.pack.tag.sec = msg->header.stamp.sec;
    packet.pack.tag.nsec = msg->header.stamp.nanosec;

     auto now = std::chrono::steady_clock::now();

        packet.pack.tag.timestamp = 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time_
        ).count();
  
         
    msg->detections.size() > 0 ? packet.pack.tag.validity = true : packet.pack.tag.validity = false;
   
    for (const auto& det : msg->detections) 
    {
      if (target_id_ >= 0 && static_cast<int>(det.id) != target_id_) continue;
      

      packet.pack.tag.tag_id = det.id;

      packet.pack.tag.pos_x = static_cast<int16_t>(det.pose.pose.pose.position.x * 1000);
      packet.pack.tag.pos_y = static_cast<int16_t>(det.pose.pose.pose.position.y * 1000);
      packet.pack.tag.pos_z = static_cast<int16_t>(det.pose.pose.pose.position.z * 1000);
      packet.pack.tag.orientation_x = static_cast<int16_t>(det.pose.pose.pose.orientation.x * 1000);
      packet.pack.tag.orientation_y = static_cast<int16_t>(det.pose.pose.pose.orientation.y * 1000);
      packet.pack.tag.orientation_z = static_cast<int16_t>(det.pose.pose.pose.orientation.z * 1000);
      packet.pack.tag.orientation_w = static_cast<int16_t>(det.pose.pose.pose.orientation.w * 1000);

    

//      sender_->sendPacket((char*)&packet.data, sizeof(ApRos_packet_t));
      
    }
    uint32_t ptempChk = 0;
    for (int i = 4; i < packet.pack.header.payloadSize + 4; i++)
    {
          ptempChk += packet.data[i];
    }
    packet.pack.pchk = ptempChk % 256;
    packet.pack.pchk = ~(packet.pack.pchk) + 1;
    sender_->sendPacket((char*)&packet.data, sizeof(ApRos_packet_t));
  }
  std::chrono::steady_clock::time_point start_time_;
  std::string topic_, host_;
  int port_{5005};
  int target_id_{-1};

  std::unique_ptr<UdpSender> sender_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagUdpNode>());
  rclcpp::shutdown();
  return 0;
}
