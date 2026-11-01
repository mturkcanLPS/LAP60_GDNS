#!/usr/bin/env python3
"""car_detector icin sentetik renkli kamera: bir hava fotografini 15 Hz'de yayinlar.

Masada arac yokken tespit + kilit + bearing zincirini sinamak icin. IZOLE bir
ROS_DOMAIN_ID'de calistir (gercek kamera topic'lerine karismasin).

  ROS_DOMAIN_ID=77 fake_color_pub.py --image foo.jpg [--fx 640 --fy 640]

camera_info K: fx/fy parametre, cx/cy goruntu merkezi, distorsiyon SIFIR ->
beklenen bearing = ((u - cx)/fx, (v - cy)/fy) (car_detector K'yi mux boyutuna
olcekler, oran ayni kalir).
"""
import argparse
import array
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


class FakeColor(Node):
    def __init__(self, a):
        super().__init__('fake_color')
        bgr = cv2.imread(a.image)
        if bgr is None:
            raise SystemExit(f'goruntu okunamadi: {a.image}')
        self.rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.h, self.w = self.rgb.shape[:2]
        self.fx, self.fy = a.fx, a.fy
        self.img_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', qos_profile_sensor_data)
        self.ci_pub = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', qos_profile_sensor_data)
        # Mesajlar BIR KEZ kurulur, her karede sadece stamp degisir. Eskiden her karede
        # m.data = bytes -> rclpy 2.7 MB'i eleman eleman dogruluyordu: %100 CPU, ~2 fps.
        # array.array hizli yoldan (memcpy) gecer.
        self.m = Image()
        self.m.header.frame_id = 'camera_color_optical_frame'
        self.m.height, self.m.width, self.m.encoding, self.m.step = self.h, self.w, 'rgb8', self.w * 3
        self.m.data = array.array('B', self.rgb.tobytes())
        self.ci = CameraInfo()
        self.ci.width, self.ci.height = self.w, self.h
        self.ci.distortion_model = 'plumb_bob'
        self.ci.d = [0.0] * 5
        self.ci.k = [self.fx, 0.0, self.w / 2, 0.0, self.fy, self.h / 2, 0.0, 0.0, 1.0]
        self.create_timer(1.0 / a.fps, self.tick)
        self.get_logger().info(f'{a.image} {self.w}x{self.h} fx={self.fx} fy={self.fy} '
                               f'cx={self.w/2} cy={self.h/2} @{a.fps} Hz')

    def tick(self):
        st = self.get_clock().now().to_msg()
        self.m.header.stamp = st
        self.img_pub.publish(self.m)
        self.ci.header = self.m.header
        self.ci_pub.publish(self.ci)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image', required=True)
    ap.add_argument('--fx', type=float, default=900.0)
    ap.add_argument('--fy', type=float, default=900.0)
    ap.add_argument('--fps', type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    rclpy.spin(FakeColor(a))


if __name__ == '__main__':
    main()
