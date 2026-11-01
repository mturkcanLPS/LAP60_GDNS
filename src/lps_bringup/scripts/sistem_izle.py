#!/usr/bin/env python3
"""Genel sistem testi: tegrastats + node bazinda CPU + fan + donanim saatleri +
asiri akim (OC) / termal olaylar. Root gerekmez. Sonda ozet basar.

    sistem_izle.py BITIS_HH:MM:SS CIKTI_ONEKI
    -> CIKTI_ONEKI_tegrastats.log (ham, 5 s) + stdout'a ozet

2026-10-20: 19_10 ucusu sonrasi masa testleri (servis_izle.sh ile paralel).
"""
import datetime as dt
import glob
import os
import re
import statistics as st
import subprocess
import sys
import time

end = dt.datetime.combine(dt.date.today(), dt.time.fromisoformat(sys.argv[1])).timestamp()
pref = sys.argv[2]
HZ = os.sysconf('SC_CLK_TCK')
DEVF = {'GPU': '17000000.gpu', 'VIC': '15340000.vic', 'OFA': '15a50000.ofa'}
NODES = ('realsense', 'apriltag_infra1', 'apriltag_infra2', 'serial_sender',
         'infra2_recorder', 'car_detector')


def rd(p, d=None):
    try:
        return open(p).read().strip()
    except OSError:
        return d


def ocs():
    return {f'oc{i}': int(rd(f'/sys/class/hwmon/hwmon3/oc{i}_event_cnt', '0')) for i in (1, 2, 3)}


def node_of(pid):
    try:
        comm = rd(f'/proc/{pid}/comm')
        cmd = open(f'/proc/{pid}/cmdline', 'rb').read()
    except OSError:
        return None
    if comm == 'cpu_apriltag_no':
        return 'apriltag_infra2' if b'cpu_apriltag_infra2' in cmd else 'apriltag_infra1'
    return {'realsense2_came': 'realsense', 'serial_sender_n': 'serial_sender',
            'infra2_recorder': 'infra2_recorder', 'car_detector_no': 'car_detector'}.get(comm)


def ticks():
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        n = node_of(pid)
        if not n:
            continue
        try:
            f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
            out[n] = out.get(n, 0) + int(f[11]) + int(f[12])
        except (OSError, IndexError):
            pass
    return out


def total_cpu():
    v = list(map(int, open('/proc/stat').readline().split()[1:]))
    return sum(v), v[3] + v[4]


def s(v, u='', f='.0f'):
    if not v:
        return '-'
    v2 = sorted(v)
    p95 = v2[int(0.95 * (len(v2) - 1))]
    return f'ort {st.mean(v):{f}}{u}  p95 {p95:{f}}{u}  max {max(v):{f}}{u}'


t0 = time.time()
oc0 = ocs()
kl0 = sum(1 for _ in open('/var/log/kern.log', errors='ignore'))
tg = subprocess.Popen(['tegrastats', '--interval', '5000'], stdout=subprocess.PIPE, text=True)
os.set_blocking(tg.stdout.fileno(), False)
rows, nodes, fan, freqs = [], {}, [], {k: [] for k in DEVF}
pt, pw = ticks(), time.time()
tot0 = total_cpu()
logf = open(pref + '_tegrastats.log', 'w')
while time.time() < end:
    time.sleep(5)
    for line in iter(tg.stdout.readline, ''):
        if not line:
            break
        logf.write(line)
        rows.append(line)
    logf.flush()
    now = time.time()
    ct = ticks()
    dtt = now - pw
    for n, v in ct.items():
        if n in pt:
            nodes.setdefault(n, []).append(100.0 * (v - pt[n]) / HZ / dtt)
    pt, pw = ct, now
    fan.append((int(rd('/sys/class/hwmon/hwmon2/rpm', '0')), int(rd('/sys/class/hwmon/hwmon0/pwm1', '0'))))
    for k, d in DEVF.items():
        f = rd(f'/sys/class/devfreq/{d}/cur_freq')
        if f:
            freqs[k].append(int(f) / 1e6)
tg.terminate()
logf.close()
oc1 = ocs()
tot1 = total_cpu()


def col(pat, conv=float):
    out = []
    for r in rows:
        m = re.search(pat, r)
        if m:
            out.append(conv(m.group(1)))
    return out


cores = []
for r in rows:
    m = re.search(r'CPU \[([^\]]+)\]', r)
    if m:
        cores.append([int(x) for x in re.findall(r'(\d+)%@\d+', m.group(1))])
cpu_avg = [st.mean(c) for c in cores if c]
cfreq = col(r'%@(\d+)', int)
gr3d = col(r'GR3D_FREQ (\d+)%', int)
ramv = col(r'RAM (\d+)/', int)
swv = col(r'SWAP (\d+)/', int)
idle = 100 * (1 - (tot1[1] - tot0[1]) / max(1, tot1[0] - tot0[0]))

print(f'===== GENEL SISTEM TESTI  {dt.datetime.fromtimestamp(t0):%H:%M:%S} - {dt.datetime.now():%H:%M:%S}'
      f'  ({(time.time() - t0) / 60:.1f} dk, {len(rows)} tegrastats ornegi) =====')
print(f'CPU (6 cekirdek ort) : {s(cpu_avg, "%")}   | /proc/stat toplam: %{idle:.0f}')
for i in range(6):
    print(f'   cekirdek {i}       : {s([c[i] for c in cores if len(c) > i], "%")}')
print(f'CPU frekans          : {s(cfreq, " MHz")}')
print(f'GPU yuku (GR3D)      : {s(gr3d, "%")}')
for k, v in freqs.items():
    print(f'{k:4s} saat (devfreq)  : min {min(v):.0f}  max {max(v):.0f} MHz' if v else f'{k}: -')
print('Sicakliklar:')
for sn in ('cpu', 'gpu', 'soc0', 'soc1', 'soc2', 'tj'):
    tv = col(sn + r'@([\d.]+)C')
    print(f'   {sn:5s}             : {s(tv, " C", ".1f")}')
print('Guc:')
for pn in ('VDD_IN', 'VDD_CPU_GPU_CV', 'VDD_SOC'):
    pv = [x / 1000 for x in col(pn + r' (\d+)mW', int)]
    print(f'   {pn:15s}   : {s(pv, " W", ".2f")}')
print(f'RAM kullanimi        : {s(ramv, " MB")}')
print(f'Swap kullanimi       : {s(swv, " MB")}')
print(f'Fan                  : {s([f[0] for f in fan], " rpm")}  | pwm {s([f[1] for f in fan], "/255")}')
print('Node bazinda CPU (% = tek cekirdek, 600 = hepsi):')
for n in NODES:
    print(f'   {n:16s}  : {s(nodes.get(n, []), "%")}')
tot_nodes = [sum(x) for x in zip(*[nodes[n] for n in nodes])] if nodes else []
print(f'   {"LPS toplam":16s}  : {s(tot_nodes, "%")}')
print('Asiri akim (OC) olay : ' + '  '.join(f'{k}: +{oc1[k] - oc0[k]} (toplam {oc1[k]})' for k in oc1))
kl = [line for i, line in enumerate(open('/var/log/kern.log', errors='ignore')) if i >= kl0]
for k in ('cpu-throttle-alert', 'gpu-throttle-alert', 'hot-surface-alert', 'soc-throttle'):
    n = sum(1 for line in kl if k in line and '0 -> 1' in line)
    print(f'Termal olay {k:20s}: {n}')
print('Termal esikler (trip noktalari):')
for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
    trips = []
    for tp in sorted(glob.glob(z + '/trip_point_*_temp')):
        typ = rd(tp.replace('_temp', '_type'), '?')
        t = rd(tp)
        if t and t.lstrip('-').isdigit() and int(t) > 0:
            trips.append(f'{typ}={int(t) / 1000:.0f}')
    print(f'   {rd(z + "/type")}: {" ".join(trips)}')
