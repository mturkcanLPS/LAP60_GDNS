import rclpy, time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
rclpy.init(); n = rclpy.create_node('say'); c = {'imu': 0, 'img': 0}; last = {}
def ci(m): c['imu'] += 1; last['imu'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
def cg(m): c['img'] += 1; last['img'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
n.create_subscription(Imu, '/camera/camera/imu', ci, qos_profile_sensor_data)
n.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', cg, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
print(c, {k: round(v - time.time(), 2) for k, v in last.items()})
