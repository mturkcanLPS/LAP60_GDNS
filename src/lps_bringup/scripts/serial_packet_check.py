#!/usr/bin/env python3
"""Seri paket cozucu — otopilotun gorecegi baytlari firmware gibi cozer.

Iki kullanim:
  1) Sahte port (masa testi): pty acar, slave yolunu --ptyfile'a yazar;
     serial_sender'a serial_port:=<o yol> verilir. Otopilota hicbir sey gitmez.
       serial_packet_check.py --pty --ptyfile /tmp/pty --duration 60
  2) Gercek port dinleme YOK (tek yazar serial_sender; ayni porta ikinci okuyucu
     baglamak veriyi boler) — bu arac sadece pty icindir.

Iki mesaj tipi ayni akista (ayni header + checksum kurali):
  0x42 ana LPS paketi   payload 128, toplam 133 B, 50 Hz
  0x48 sistem sagligi   payload  26, toplam  31 B, ~1 Hz
Kontrol: sync/id, header checksum, payloadSize, payload checksum; sonra alanlar
acilir (ofsetler serial_sender_node.cpp jetsonVariables_t / healthPack_t ile birebir).
"""
import argparse, os, struct, sys, time, tty

SYNC, MSG_ID, MSG_ID_HEALTH = 0xBB, 0x42, 0x48
PAYLOAD = {MSG_ID: 128, MSG_ID_HEALTH: 26}          # msg_id -> payloadSize
OFF = dict(hrvs=(0, 16), aprtiltag=(16, 30), opticflow=(46, 13), ir=(59, 13),
           aprtiltag2=(72, 30), opticflow2=(102, 13), car=(115, 13))
# healthPack_t: timeStamp, valid, uptime, cpuLoad, cpuCoreMax, gpuLoad, cpuTemp, gpuTemp,
# tjTemp, ramUsed, swapUsed, diskUsed, fanRpm, inVoltage, inPower, ocEvents
HEALTH_FMT = '<I?IBBBbbbBBBHHHH'
assert struct.calcsize(HEALTH_FMT) == 26


def apriltag(b):
    ts, val, x, y, z, r, p, yw = struct.unpack('<IH6f', b)
    return dict(ts=ts, valid=val & 1, bits=(val >> 1) & 0xFF, pos=(x, y, z), rpy=(r, p, yw))


def flow(b):
    ts, q, dx, dy = struct.unpack('<IBff', b)
    return dict(ts=ts, q=q, dx=dx, dy=dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pty', action='store_true', required=True)
    ap.add_argument('--ptyfile', required=True)
    ap.add_argument('--duration', type=float, default=60)
    a = ap.parse_args()

    master, slave = os.openpty()
    tty.setraw(master)
    open(a.ptyfile, 'w').write(os.ttyname(slave))
    os.set_blocking(master, False)

    buf, pkts, hpkts, bad_h, bad_c = b'', [], [], 0, 0
    t0 = time.time()
    while time.time() - t0 < a.duration:
        try:
            buf += os.read(master, 4096)
        except BlockingIOError:
            time.sleep(0.005); continue
        while len(buf) >= 4:
            i = buf.find(bytes([SYNC]))
            if i < 0: buf = b''; break
            buf = buf[i:]
            if len(buf) < 4: break
            h = buf[:4]
            if h[1] not in PAYLOAD or (sum(h[:3]) + h[3]) & 0xFF != 0 or h[2] != PAYLOAD[h[1]]:
                bad_h += 1; buf = buf[1:]; continue
            n = PAYLOAD[h[1]]
            if len(buf) < 4 + n + 1: break
            pay, ck = buf[4:4 + n], buf[4 + n]
            if (sum(pay) + ck) & 0xFF != 0:
                bad_c += 1; buf = buf[1:]; continue
            (pkts if h[1] == MSG_ID else hpkts).append((time.time(), pay)); buf = buf[4 + n + 1:]

    span = (pkts[-1][0] - pkts[0][0]) if len(pkts) > 1 else 1
    hspan = (hpkts[-1][0] - hpkts[0][0]) if len(hpkts) > 1 else 1
    print(f"gecerli paket: 0x42 {len(pkts)} ({len(pkts)/span:.1f} Hz) | 0x48 saglik {len(hpkts)} "
          f"({(len(hpkts) - 1)/hspan if len(hpkts) > 1 else 0:.2f} Hz) | bozuk header: {bad_h} | bozuk checksum: {bad_c}")
    if not pkts:
        return 1
    seg = lambda pay, k: pay[OFF[k][0]:OFF[k][0] + OFF[k][1]]
    last = pkts[-1][1]
    for k in ('aprtiltag', 'aprtiltag2'):
        t = apriltag(seg(last, k))
        print(f"  {k:11s}: valid={t['valid']} pos=({t['pos'][0]:+.4f},{t['pos'][1]:+.4f},{t['pos'][2]:+.4f}) ts={t['ts']}")
    for k in ('opticflow', 'opticflow2'):
        f = flow(seg(last, k))
        print(f"  {k:11s}: dx={f['dx']:+.3f} dy={f['dy']:+.3f} q={f['q']} ts={f['ts']}")
    ts, val, bx, by = struct.unpack('<I?ff', seg(last, 'car'))
    print(f"  {'car':11s}: valid={int(val)} bearing=({bx:+.4f},{by:+.4f}) ts={ts}")
    # car alani paketler boyunca: kac pakette gecerli, gecerliyken bearing ortalamasi
    cars = [struct.unpack('<I?ff', seg(p, 'car')) for _, p in pkts]
    v = [c for c in cars if c[1]]
    if v:
        print(f"  car gecerli: {len(v)}/{len(cars)} paket | gecerliyken bearing ort "
              f"({sum(c[2] for c in v)/len(v):+.4f},{sum(c[3] for c in v)/len(v):+.4f})")
    else:
        print(f"  car gecerli: 0/{len(cars)} paket")
    if hpkts:
        h = struct.unpack(HEALTH_FMT, hpkts[-1][1])
        print(f"  {'saglik 0x48':11s}: valid={int(h[1])} up={h[2]}s cpu {h[3]}% (max cekirdek {h[4]}%) gpu {h[5]}% | "
              f"cpu {h[6]} gpu {h[7]} tj {h[8]} C | ram {h[9]}% swap {h[10]}% disk {h[11]}% | "
              f"fan {h[12]} rpm | giris {h[13]/1000:.2f} V {h[14]/1000:.2f} W | oc {h[15]} | ts={h[0]}")
        hts = [struct.unpack(HEALTH_FMT, p)[0] for _, p in hpkts]
        print(f"  saglik ornek timeStamp'leri tekil mi: {len(set(hts)) == len(hts)} ({len(hts)} paket)")
    else:
        print("  saglik 0x48: HIC gelmedi")
    return 0


if __name__ == '__main__':
    sys.exit(main())
