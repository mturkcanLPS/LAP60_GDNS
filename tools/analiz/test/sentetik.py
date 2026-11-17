"""Sentetik test: dokulu goruntu her karede +2 px kayar (15 Hz). IMU (200 Hz) bunu tamamen aciklayan
donme verir: rot_dx = -f*dth_y = +2 px -> gy = -2*fps/f. Beklenen: derot_dx ~ 0, tex_dx ~ +2."""
import rclpy, time, numpy as np, cv2, sys
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, Imu
from builtin_interfaces.msg import Time
rclpy.init(); n = rclpy.create_node('sentetik')
pi = n.create_publisher(Image, '/camera/camera/infra1/image_rect_raw', qos_profile_sensor_data)
pc = n.create_publisher(CameraInfo, '/camera/camera/infra1/camera_info', qos_profile_sensor_data)
pm = n.create_publisher(Imu, '/camera/camera/imu', qos_profile_sensor_data)
rng = np.random.default_rng(1); big = cv2.GaussianBlur((rng.random((720, 2600)) * 255).astype(np.uint8), (0, 0), 2.0)
crops = [big[:, x:x + 1280].tobytes() for x in range(0, 1201, 2)]
f, fps, shift = 647.5, 10.0, 2.0
gy = -shift * fps / f
def st(t):
    m = Time(); m.sec = int(t); m.nanosec = int((t - int(t)) * 1e9); return m
ci = CameraInfo(); ci.height, ci.width = 720, 1280
ci.k = [f, 0.0, 640.0, 0.0, f, 360.0, 0.0, 0.0, 1.0]; ci.d = [0.0] * 5; ci.distortion_model = 'plumb_bob'
ci.p = [f, 0.0, 640.0, 0.0, 0.0, f, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 12
t0 = time.time(); k = 0; ti = t0
while time.time() - t0 < dur:
    now = time.time()
    while ti <= now:
        m = Imu(); m.header.stamp = st(ti); m.angular_velocity.y = gy; m.linear_acceleration.z = -9.8; pm.publish(m); ti += 0.005
    if now - t0 >= k / fps:
        x0 = 1200 - int((time.time() - t0) * shift * fps) % 1200   # zamana bagli: +shift*fps px/s saga
        tk = time.time()
        msg = Image(); msg.header.stamp = st(tk); msg.header.frame_id = 'cam'; msg.height, msg.width = 720, 1280
        msg.encoding = 'mono8'; msg.step = 1280; msg.data = big[:, x0:x0 + 1280].tobytes(); pi.publish(msg)
        ci.header = msg.header; pc.publish(ci); k += 1
    time.sleep(0.001)
print('kare', k, 'fps', round(k / dur, 1))
