"""Yeni cpu_apriltag_node'u (filtreli) bag'deki ham kareler ile besle; gercek/sahte say."""
import os, sys, glob, sqlite3, time, signal, subprocess, threading, collections
import numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
os.environ['ROS_DOMAIN_ID'] = '77'; S = '/home/lapis/huni_kalib'; N = int(sys.argv[1])
db = sqlite3.connect(glob.glob(S + '/kalib_145232/*.db3')[0])
T = {r[1]: r[0] for r in db.execute('select id, name from topics')}
ci = deserialize_message(db.execute('select data from messages where topic_id=? limit 1', (T['/camera/camera/infra1/camera_info'],)).fetchone()[0], CameraInfo)
node = subprocess.Popen(['/home/lapis/ros2_ws/install/lps_serial_sender/lib/lps_serial_sender/cpu_apriltag_node', '--ros-args',
    '-p', 'image_topic:=/ft/img', '-p', 'camera_info_topic:=/ft/ci', '-p', 'output_topic:=/ft/tags', '-p', 'tag_family:=tag25h9',
    '-p', 'size:=0.26', '-p', 'decode_sharpening:=1.0', '-p', 'nthreads:=4', '-p', 'max_hamming:=1', '-p', 'min_decision_margin:=30.0'],
    stdout=open(S + '/ft_node.log', 'w'), stderr=subprocess.STDOUT)
rclpy.init(); n = rclpy.create_node('ft'); pi = n.create_publisher(Image, '/ft/img', 10); pc = n.create_publisher(CameraInfo, '/ft/ci', 10)
C = collections.Counter(); got = [0]
def cb(m):
    got[0] += 1
    for d in m.detections:
        p = d.pose.pose.pose.position; dist = (p.x**2 + p.y**2 + p.z**2) ** 0.5
        C['gercek' if d.id in (0, 20, 24, 34) and dist < 2.5 else f'SAHTE id{d.id} {dist:.1f}m'] += 1
n.create_subscription(AprilTagDetectionArray, '/ft/tags', cb, qos_profile_sensor_data)
threading.Thread(target=lambda: rclpy.spin(n), daemon=True).start(); time.sleep(3)
for _ in range(5): pc.publish(ci); time.sleep(0.05)
sent = 0
for i, (data,) in enumerate(db.execute('select data from messages where topic_id=? order by timestamp', (T['/camera/camera/infra1/image_rect_raw'],))):
    if i % N: continue
    m = deserialize_message(data, Image); m.header.stamp.sec = i; pc.publish(ci); pi.publish(m); sent += 1; time.sleep(0.045)
time.sleep(2); node.send_signal(signal.SIGINT); node.wait(timeout=10); rclpy.shutdown()
print('gonderilen', sent, 'cevap', got[0], dict(C))
