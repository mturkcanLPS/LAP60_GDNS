# AVI (yarim cozunurluk) her N. kareyi 1280x720'ye buyutup [kare_no][goruntu] olarak yaz. argv: avi N
import sys, os, struct, cv2, numpy as np
p, N = sys.argv[1], int(sys.argv[2]); sz = os.path.getsize(p); f = open(p, 'rb'); pos = f.read(1 << 20).find(b'movi') + 4; o = []
while True:
    f.seek(pos); h = f.read(8)
    if len(h) < 8: break
    cc, ln = h[:4], struct.unpack('<I', h[4:])[0]
    if cc in (b'LIST', b'RIFF'): pos += 12; continue
    if cc == b'00dc':
        if pos + 8 + ln > sz: break
        o.append(pos)
    pos += 8 + ln + (ln & 1)
out = sys.stdout.buffer
for i in range(0, len(o), N):
    f.seek(o[i]); ln = struct.unpack('<I', f.read(8)[4:])[0]
    g = cv2.imdecode(np.frombuffer(f.read(ln), np.uint8), cv2.IMREAD_GRAYSCALE)
    if g is None: continue
    out.write(struct.pack('<I', i)); out.write(cv2.resize(g, (1280, 720), interpolation=cv2.INTER_CUBIC).tobytes())
