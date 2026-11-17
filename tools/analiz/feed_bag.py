# bag'deki ham IR goruntulerini (her N. kare) [kare_no][1280x720] olarak stdout'a yaz. argv: bag cam N
import sys, glob, sqlite3, struct
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
db = sqlite3.connect(glob.glob(sys.argv[1] + '/*.db3')[0]); N = int(sys.argv[3])
tid = db.execute('select id from topics where name=?', (f'/camera/camera/{sys.argv[2]}/image_rect_raw',)).fetchone()[0]
out = sys.stdout.buffer
for i, (data,) in enumerate(db.execute('select data from messages where topic_id=? order by timestamp', (tid,))):
    if i % N: continue
    m = deserialize_message(data, Image); out.write(struct.pack('<I', i)); out.write(bytes(m.data))
