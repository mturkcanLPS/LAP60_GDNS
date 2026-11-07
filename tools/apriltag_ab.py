#!/usr/bin/env python3
"""AprilTag A/B: Isaac (GPU) vs CPU (libapriltag) — ayni goruntude kiyas.

Kullanim:
  1) Kamerayi + iki dedektoru calistir:
       ros2 launch lps_bringup bringup.launch.py cpu_apriltag:=true
     (Isaac -> /tag_detections, CPU -> /tag_detections_cpu)
  2) Huniyi/tag'leri kameraya goster, sonra:
       python3 tools/apriltag_ab.py
  3) Birkaç saniye topla, Ctrl+C ile ozeti gor.

Her tag ID icin: iki dedektorde kac karede goruldu + toplam tespit.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from collections import defaultdict


class AB(Node):
    def __init__(self):
        super().__init__('apriltag_ab')
        self.cnt = {'GPU': defaultdict(int), 'CPU': defaultdict(int)}
        self.frames = {'GPU': 0, 'CPU': 0}
        self.frames_with = {'GPU': 0, 'CPU': 0}
        self.create_subscription(AprilTagDetectionArray, '/tag_detections',
                                 lambda m: self.cb('GPU', m), qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, '/tag_detections_cpu',
                                 lambda m: self.cb('CPU', m), qos_profile_sensor_data)
        self.get_logger().info('dinliyor: /tag_detections (GPU) + /tag_detections_cpu (CPU). Ctrl+C = ozet')

    def cb(self, src, msg):
        self.frames[src] += 1
        if msg.detections:
            self.frames_with[src] += 1
        for d in msg.detections:
            self.cnt[src][d.id] += 1

    def summary(self):
        print('\n==== AprilTag A/B ozeti ====')
        for src in ('GPU', 'CPU'):
            fr = self.frames[src]
            print(f'{src}: {fr} mesaj, {self.frames_with[src]} tanesinde >=1 tag '
                  f'(%{100*self.frames_with[src]/max(1,fr):.0f})')
        ids = sorted(set(self.cnt['GPU']) | set(self.cnt['CPU']))
        print(f"{'tag_id':>8}{'GPU':>10}{'CPU':>10}   (kare basi tespit sayisi)")
        for i in ids:
            print(f"{i:>8}{self.cnt['GPU'][i]:>10}{self.cnt['CPU'][i]:>10}")
        tot_g = sum(self.cnt['GPU'].values()); tot_c = sum(self.cnt['CPU'].values())
        print(f"{'TOPLAM':>8}{tot_g:>10}{tot_c:>10}   CPU/GPU = x{tot_c/max(1,tot_g):.2f}")


def main():
    rclpy.init()
    n = AB()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.summary()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
