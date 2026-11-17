import os, pty, select, sys, time
link, sure = sys.argv[1], float(sys.argv[2])
m, s = pty.openpty()
if os.path.lexists(link): os.remove(link)
os.symlink(os.ttyname(s), link)
t0 = time.time()
while time.time() - t0 < sure:
    r, _, _ = select.select([m], [], [], 0.2)
    if r:
        try: os.read(m, 65536)
        except OSError: pass
os.remove(link)
