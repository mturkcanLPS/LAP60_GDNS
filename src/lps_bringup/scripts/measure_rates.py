#!/usr/bin/env python3
"""Calisan LPS yiginin CPU + kamera hizlarini olcer (iki kamera karsilastirmasi).

Kaynak: node'larin saniyelik loglari (--log DOSYA ya da --journal) + /proc CPU.
Root gerekmez (sadece okur). Ornek:
    measure_rates.py --journal --duration 60 --label baseline
    measure_rates.py --log /tmp/launch.log --duration 60 --label "30fps esit"
"""
import argparse, os, re, subprocess, time

NODES = {   # etiket -> /proc cmdline'da aranacak (hepsi eslesmeli)
    'realsense':         ['realsense2_camera_node'],
    'apriltag infra1':   ['cpu_apriltag_node', '__node:=cpu_apriltag '],
    'apriltag infra2':   ['cpu_apriltag_node', '__node:=cpu_apriltag_infra2'],
    'serial_sender':     ['serial_sender_node'],
    'infra2_recorder':   ['infra2_recorder_node'],
    'car_detector':      ['car_detector_node'],
}
TICK = os.sysconf('SC_CLK_TCK')


def procs():
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode() + ' '
        except OSError:
            continue
        for k, keys in NODES.items():
            if all(s in cmd for s in keys) and 'measure_rates' not in cmd:
                out[k] = int(pid)
    return out


def cpu_ticks(pid):
    f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
    return int(f[11]) + int(f[12])            # utime + stime


def udp_rcvbuf_errors():
    """Sistem geneli UDP RcvbufErrors (tampon tasmasindan dusen datagram)."""
    lines = [l.split() for l in open('/proc/net/snmp') if l.startswith('Udp:')]
    k, v = lines[0], lines[1]
    return int(v[k.index('RcvbufErrors')]), int(v[k.index('InDatagrams')])


def sys_busy():
    v = list(map(int, open('/proc/stat').readline().split()[1:]))
    idle = v[3] + v[4]
    return sum(v) - idle, sum(v)


PAT = {   # metrik -> (regex, grup)
    'infra1 tag Hz':   r'\[APRILTAG\]\s+([\d.]+) Hz',
    'infra1 akis Hz':  r'\[OPTICFLOW\]\s+([\d.]+) Hz',
    'infra1 duzenli%': r'\[OPTICFLOW\].*duzenli %(-?\d+)',
    'infra1 atlanan%': r'\[OPTICFLOW\].*atlanan %(-?\d+)',
    'infra2 goruntu Hz': r'\[INFRA2(?: TEST)?\] img ([\d.]+) Hz',
    'infra2 tag Hz':   r'\[INFRA2(?: TEST)?\] img [\d.]+ Hz \| tag ([\d.]+) Hz',
    'infra2 akis Hz':  r'\[INFRA2(?: TEST)?\].*akis ([\d.]+) Hz',
    'infra2 duzenli%': r'\[INFRA2(?: TEST)?\].*duzenli %(-?\d+)',
    'infra2 atlanan%': r'\[INFRA2(?: TEST)?\].*atlanan %(-?\d+)',
    'pakette tag2 Hz':  r'\[APRILTAG2\]\s+([\d.]+) Hz',
    'pakette akis2 Hz': r'\[OPTICFLOW2\]\s+([\d.]+) Hz',
    'arac bearing Hz':  r'\[CAR\]\s+([\d.]+) Hz',
}


def parse(text):
    vals = {k: [] for k in PAT}
    for line in text.splitlines():
        for k, rx in PAT.items():
            m = re.search(rx, line)
            if m and float(m.group(1)) >= 0:
                vals[k].append(float(m.group(1)))
    return vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60)
    ap.add_argument('--label', default='olcum')
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--log', help='launch cikti dosyasi')
    g.add_argument('--journal', action='store_true', help='apriltag_bringup servis logu')
    a = ap.parse_args()

    ps = procs()
    t0c = {k: cpu_ticks(p) for k, p in ps.items()}
    b0, a0 = sys_busy()
    e0, d0 = udp_rcvbuf_errors()
    log_off = os.path.getsize(a.log) if a.log else 0
    since = time.strftime('%Y-%m-%d %H:%M:%S')
    t0 = time.time()
    time.sleep(a.duration)
    dt = time.time() - t0
    b1, a1 = sys_busy()
    e1, d1 = udp_rcvbuf_errors()

    if a.log:
        with open(a.log, errors='replace') as f:
            f.seek(log_off); text = f.read()
    else:
        text = subprocess.run(['journalctl', '-u', 'apriltag_bringup', '--since', since,
                               '--no-pager'], capture_output=True, text=True).stdout
    vals = parse(text)

    print(f'\n===== {a.label} ({dt:.0f} s) =====')
    print(f'  toplam CPU: %{100.0 * (b1 - b0) / max(1, a1 - a0):.0f} (6 cekirdegin)')
    de, dd = e1 - e0, d1 - d0
    print(f'  UDP tampon tasmasi: {de} datagram dustu / {dd + de} (%{100.0 * de / max(1, dd + de):.1f})')
    for k, p in ps.items():
        try:
            c = 100.0 * (cpu_ticks(p) - t0c[k]) / TICK / dt
            print(f'  CPU {k:18s}: %{c:5.0f} (1 cekirdek = %100)')
        except OSError:
            print(f'  CPU {k:18s}: surec olcum sirasinda kapandi')
    for k, v in vals.items():
        if not v:
            continue
        s = sorted(v)
        p10 = s[max(0, int(0.1 * len(s)) - 1)]
        print(f'  {k:18s}: ort {sum(v)/len(v):5.1f}   en kotu %10: {p10:5.1f}   (n={len(v)})')


if __name__ == '__main__':
    main()
