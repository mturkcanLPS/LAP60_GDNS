import rclpy, sys, numpy as np, cv2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node('kare_al'); got = {}
def cb(m): got['m'] = m
n.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', cb, qos_profile_sensor_data)
while 'm' not in got: rclpy.spin_once(n, timeout_sec=0.5)
m = got['m']; img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
cv2.imwrite(sys.argv[1], img); print(m.width, m.height, m.encoding, 'max', img.max(), 'n>=250', int((img >= 250).sum()))
