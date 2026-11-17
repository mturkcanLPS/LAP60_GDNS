"""Canli izleme: her 15 s'de bir satir — kamera basina duvar tag sayilari, mesafe, CPU sicakligi."""
import rclpy, time, collections, numpy as np
from rclpy.qos import qos_profile_sensor_data
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
def temp():
    try: return max(int(open(f'/sys/class/thermal/thermal_zone{i}/temp').read()) for i in (0, 1)) / 1000
    except Exception: return float('nan')
rclpy.init(); n = rclpy.create_node('kalib_izle'); C = collections.defaultdict(list)
for t, k in (('/tag_detections', 'i1'), ('/tag_detections_infra2', 'i2')):
    n.create_subscription(AprilTagDetectionArray, t,
        lambda m, k=k: C[k].append([(d.id, float(np.linalg.norm([d.pose.pose.pose.position.x, d.pose.pose.pose.position.y, d.pose.pose.pose.position.z]))) for d in m.detections if d.id in (0, 20, 24, 34, 10, 12, 14, 16)]),
        qos_profile_sensor_data)
t0 = time.time(); last = t0
while True:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - last >= 15:
        last = time.time(); parts = []
        for k in ('i1', 'i2'):
            L = C[k]; d = [x for f in L for _, x in f]
            sm = [f for f in L if any(i in (10, 12, 14, 16) for i, _ in f)]
            both = sum(1 for f in sm if any(i in (0, 20, 24, 34) for i, _ in f))
            parts.append(f"{k}: tagli {sum(1 for f in L if f)}/{len(L)}, >=2tag {sum(1 for f in L if len(f) >= 2)}, kucuklu {len(sm)} (duvarla birlikte {both}), mesafe " +
                         (f"{np.min(d):.2f}-{np.max(d):.2f}" if d else '-'))
        print(f"[{int(last - t0)} s] CPU {temp():.0f}C | " + ' | '.join(parts), flush=True); C.clear()
