#!/usr/bin/env python3
"""LPS yigini bellek izleme — sizinti / bellek baskisi testi (root gerekmez).

Her --interval saniyede: yigin sureclerinin RSS'i (+swap), sistem MemAvailable,
swap kullanimi ve (jtop varsa) GPU/NVMM paylasimli bellek. Sonda surec basina
baslangic/bitis/tepe ve DOGRUSAL EGILIM (MB/dk) yazar: surekli buyume = sizinti.
    mem_watch.py --duration 300 --interval 10
"""
import argparse, os, time

NODES = {
    'realsense':       ['realsense2_camera_node'],
    'apriltag infra1': ['cpu_apriltag_node', '__node:=cpu_apriltag '],
    'apriltag infra2': ['cpu_apriltag_node', '__node:=cpu_apriltag_infra2'],
    'serial_sender':   ['serial_sender_node'],
    'infra2_recorder': ['infra2_recorder_node'],
    'car_detector':    ['car_detector_node'],
}


def procs():
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode() + ' '
        except OSError:
            continue
        for k, keys in NODES.items():
            if all(s in cmd for s in keys) and 'mem_watch' not in cmd:
                out[k] = int(pid)
    return out


def rss_mb(pid):
    v = {}
    for l in open(f'/proc/{pid}/status'):
        if l.startswith(('VmRSS:', 'VmSwap:', 'Threads:')):
            k, n = l.split()[:2]; v[k[:-1]] = int(n)
    return v.get('VmRSS', 0) / 1024.0, v.get('VmSwap', 0) / 1024.0, v.get('Threads', 0)


def meminfo():
    m = {}
    for l in open('/proc/meminfo'):
        k, n = l.split()[:2]; m[k[:-1]] = int(n) / 1024.0
    return m['MemAvailable'], m['SwapTotal'] - m['SwapFree']


def gpu_shared_mb(j):
    try:
        return j.memory['RAM']['shared'] / 1024.0
    except Exception:
        return float('nan')


def slope_per_min(ts, ys):
    n = len(ts)
    if n < 3: return float('nan')
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    return 60.0 * sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=300)
    ap.add_argument('--interval', type=float, default=10)
    a = ap.parse_args()
    try:
        from jtop import jtop
        j = jtop(); j.start()
    except Exception:
        j = None
    ps = procs()
    rows, t0 = [], time.time()
    while time.time() - t0 < a.duration:
        t = time.time() - t0
        avail, swap = meminfo()
        per = {}
        for k, p in ps.items():
            try: per[k] = rss_mb(p)
            except OSError: per[k] = None          # surec kapandi/coktu
        rows.append((t, avail, swap, gpu_shared_mb(j) if j else float('nan'), per))
        time.sleep(a.interval)
    if j: j.close()

    ts = [r[0] for r in rows]
    print(f'\n===== bellek izleme ({ts[-1]:.0f} s, {len(rows)} ornek) =====')
    for name, idx in (('sistem MemAvailable', 1), ('swap kullanimi', 2), ('GPU/NVMM paylasimli', 3)):
        ys = [r[idx] for r in rows]
        print(f'  {name:22s}: bas {ys[0]:7.0f} MB  son {ys[-1]:7.0f} MB  min {min(ys):7.0f}  '
              f'egilim {slope_per_min(ts, ys):+6.1f} MB/dk')
    print(f'  {"surec":16s} {"bas RSS":>8s} {"son RSS":>8s} {"tepe":>7s} {"egilim":>11s} {"1.yari":>9s} {"2.yari":>9s} {"swap":>6s} {"thread":>6s}')
    for k in ps:
        pts = [(r[0], r[4][k]) for r in rows if r[4].get(k)]
        if len(pts) < len(rows):
            print(f'  {k:16s} !!! surec olcum sirasinda KAPANDI ({len(pts)}/{len(rows)} ornek)')
        if not pts: continue
        rs = [p[1][0] for p in pts]
        # Isinma mi sizinti mi: isinmada 2. yarinin egilimi ~0'a iner, sizintida surer.
        h = len(pts) // 2
        s1 = slope_per_min([p[0] for p in pts[:h]], rs[:h])
        s2 = slope_per_min([p[0] for p in pts[h:]], rs[h:])
        print(f'  {k:16s} {rs[0]:7.0f}M {rs[-1]:7.0f}M {max(rs):6.0f}M '
              f'{slope_per_min([p[0] for p in pts], rs):+8.2f}M/dk {s1:+7.2f}M {s2:+7.2f}M '
              f'{pts[-1][1][1]:5.0f}M {pts[-1][1][2]:6d}')


if __name__ == '__main__':
    main()
