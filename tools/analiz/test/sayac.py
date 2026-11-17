"""FTDI portunun cekirdek hata sayaclari (TIOCGICOUNT). Port baskasinda kilitliyse acilamaz."""
import os, fcntl, struct, sys, json
fd = os.open(sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyFTDI', os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
v = struct.unpack('20i', fcntl.ioctl(fd, 0x545D, bytes(80))); os.close(fd)
print(json.dumps(dict(zip(['cts', 'dsr', 'rng', 'dcd', 'rx', 'tx', 'frame', 'overrun', 'parity', 'brk', 'buf_overrun'], v[:11]))))
