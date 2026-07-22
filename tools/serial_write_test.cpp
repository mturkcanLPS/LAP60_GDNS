/**
 * serial_write_test.cpp — UART yazma stratejisi A/B testi (loopback)
 *
 * Amac: serial_sender_node'un telde paket bozmasinin sebebi yazma stratejisi
 * mi, onu kodu bozmadan kanitlamak. Node ile BIREBIR ayni kosullar:
 *   163 bayt paket, 115200 baud, 40 Hz, ayni termios, ayni O_NONBLOCK acilis,
 *   ayni "gec kalinca telafi" zamanlamasi, ayni CPU pinning + SCHED_FIFO.
 * Tek degisken: paketin nasil yazildigi.
 *
 *   --mode single : node'un SUANKI davranisi — tek write(), yarim kalirsa birak
 *                   (beklenti: telde kesik paketler = osiloskoptaki bozulma)
 *   --mode loop   : pyserial'in davranisi — paket bitene kadar yaz, gerekirse
 *                   portun bosalmasini bekle (beklenti: tertemiz)
 *
 * DONANIM: TX (pin 8) — RX (pin 10) jumper'i takili olmali (loopback).
 *          serial_sender portu tutmamali (test modunda openSerial kapali).
 *
 * Derle:
 *   g++ -O2 tools/serial_write_test.cpp -o /tmp/serial_write_test
 *
 * Kullan (once mevcut davranis, sonra onerilen — ayni yuk altinda):
 *   /tmp/serial_write_test --mode single --seconds 20
 *   /tmp/serial_write_test --mode loop   --seconds 20
 */
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <chrono>
#include <thread>
#include <string>

using clk = std::chrono::steady_clock;

// Gercek paketle ayni boyut (sizeof(jetsonOtopilotPack_t) = 163)
static constexpr size_t PKT_LEN = 163;

struct Stats {
    uint64_t sent = 0;          // gonderilmeye calisilan paket
    uint64_t write_ok = 0;      // tamami yazilan
    uint64_t write_partial = 0; // YARIM yazilan (telde kesik paket!)
    uint64_t write_eagain = 0;  // hic yazilamayan (EAGAIN — paket komple dustu)
    uint64_t resumes = 0;       // loop modunda "kalani yaz" tekrari
    uint64_t waits = 0;         // loop modunda port bosalmasini bekleme
    uint64_t wait_timeout = 0;  // beklemeye ragmen bosalmadi (gercek stall)
    uint64_t rx_ok = 0;         // geri okunan ve BIREBIR dogru paket
    uint64_t rx_short = 0;      // eksik geri geldi (truncation kaniti)
    uint64_t rx_zero = 0;       // icinde 0x00 var (DMA sifirlama kaniti)
    uint64_t rx_mismatch = 0;   // uzunluk dogru ama icerik bozuk
    uint64_t rx_none = 0;       // hic geri gelmedi
    long     max_write_us = 0;  // en uzun write suresi (blokla(n)ma maliyeti)
    long     max_late_us = 0;   // tick'e en fazla ne kadar gec kalindi
};

static speed_t baudConst(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: fprintf(stderr, "Desteklenmeyen baud: %d\n", baud); exit(1);
    }
}

/* Node'un openSerial'i ile AYNI: O_NONBLOCK + raw + flow control kapali */
static int openPort(const std::string &port, int baud)
{
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "HATA: %s acilamadi: %s\n", port.c_str(), strerror(errno));
        fprintf(stderr, "Port baskasinda mi? serial_sender kapali olmali.\n");
        exit(1);
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr, "HATA: %s baska bir process tarafindan kullaniliyor.\n",
                port.c_str());
        exit(1);
    }
    ioctl(fd, TIOCEXCL);

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); exit(1); }
    speed_t spd = baudConst(baud);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);
    cfmakeraw(&tty);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |=  CLOCAL;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); exit(1); }
    return fd;
}

/* MOD 1 — node'un su anki davranisi: tek write, yarim kalirsa BIRAK.
   Yarim kalinca o baytlar ZATEN telde; "drop" etmek mumkun degil. */
static void writeSingle(int fd, const uint8_t *data, size_t len, Stats &st)
{
    ssize_t n = ::write(fd, data, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) st.write_eagain++;
        return;
    }
    if ((size_t)n != len) st.write_partial++;
    else                  st.write_ok++;
}

/* MOD 2 — pyserial'in davranisi: paket tamamen cikana kadar yaz.
   Yer yoksa portun bosalmasini bekle (poll POLLOUT), kalani yaz. */
static void writeLoop(int fd, const uint8_t *data, size_t len, Stats &st,
                      int budget_ms)
{
    size_t off = 0;
    const auto t_start = clk::now();
    while (off < len) {
        ssize_t n = ::write(fd, data + off, len - off);
        if (n > 0) {
            off += (size_t)n;
            if (off < len) st.resumes++;   // yarim kaldi ama BIRAKMIYORUZ
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd p{fd, POLLOUT, 0};
            int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                              clk::now() - t_start).count();
            int remain = budget_ms - elapsed;
            if (remain <= 0) { st.wait_timeout++; return; }
            st.waits++;
            if (poll(&p, 1, remain) <= 0) { st.wait_timeout++; return; }
            continue;
        }
        return;  // gercek hata
    }
    st.write_ok++;
}

/* Loopback'ten geri okuma — istenen bayt gelene ya da sure dolana kadar */
static size_t readBack(int fd, uint8_t *buf, size_t want, int timeout_ms)
{
    size_t got = 0;
    const auto t0 = clk::now();
    while (got < want) {
        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                          clk::now() - t0).count();
        if (elapsed >= timeout_ms) break;
        struct pollfd p{fd, POLLIN, 0};
        if (poll(&p, 1, timeout_ms - elapsed) <= 0) break;
        ssize_t n = ::read(fd, buf + got, want - got);
        if (n > 0)      got += (size_t)n;
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) break;
    }
    return got;
}

int main(int argc, char **argv)
{
    std::string port = "/dev/ttyTHS1", mode = "loop";
    int baud = 115200, seconds = 20, cpu = 5;
    double rate = 40.0;
    bool rt = true;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto val = [&](const char *what) -> const char * {
            if (i + 1 >= argc) {
                fprintf(stderr, "HATA: %s bir deger bekliyor\n", what); exit(1);
            }
            return argv[++i];
        };
        if      (a == "--port")    port    = val("--port");
        else if (a == "--mode")    mode    = val("--mode");
        else if (a == "--baud")    baud    = atoi(val("--baud"));
        else if (a == "--rate")    rate    = atof(val("--rate"));
        else if (a == "--seconds") seconds = atoi(val("--seconds"));
        else if (a == "--cpu")     cpu     = atoi(val("--cpu"));
        else if (a == "--no-rt")   rt = false;
        else { fprintf(stderr, "HATA: bilinmeyen arguman: %s\n", a.c_str()); exit(1); }
    }
    if (mode != "single" && mode != "loop") {
        fprintf(stderr, "--mode single|loop olmali\n"); return 1;
    }

    printf("========================================================\n");
    printf(" UART yazma stratejisi testi — MOD: %s\n", mode.c_str());
    printf("========================================================\n");
    printf(" %s @ %d baud | %zu bayt/paket | %.1f Hz | %d sn\n",
           port.c_str(), baud, PKT_LEN, rate, seconds);
    printf(" Hat kullanimi: %%%.0f  (jumper TX pin8 — RX pin10 takili olmali)\n",
           100.0 * (PKT_LEN * 10.0 * rate) / baud);
    printf("========================================================\n\n");

    // Payload: hic SIFIR bayt yok — telde 0x00 gorursek DMA sifirlamasidir
    uint8_t pkt[PKT_LEN], rx[PKT_LEN];
    for (size_t i = 0; i < PKT_LEN; ++i) pkt[i] = (uint8_t)((i % 255) + 1);

    int fd = openPort(port, baud);

    // Node'un TX thread'i ile ayni kosullar: RT oncelik + CPU pinning
    if (rt) {
        sched_param sp{};
        sp.sched_priority = 50;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
            printf("NOT: RT oncelik alinamadi (izin yok) — normal scheduler\n");
    }
    if (cpu >= 0) {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(cpu, &cpus);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus) != 0)
            printf("NOT: CPU%d'e sabitlenemedi — serbest calisacak\n", cpu);
    }

    Stats st;
    const auto period = std::chrono::nanoseconds((int64_t)(1e9 / rate));
    const auto t_end  = clk::now() + std::chrono::seconds(seconds);
    auto next = clk::now() + period;

    while (clk::now() < t_end) {
        std::this_thread::sleep_until(next);

        // Her paketi tek basina dogrulayabilmek icin girdiyi temizle
        tcflush(fd, TCIFLUSH);

        const auto tw = clk::now();
        st.sent++;
        if (mode == "single") writeSingle(fd, pkt, PKT_LEN, st);
        else                  writeLoop  (fd, pkt, PKT_LEN, st, 100);
        long wus = (long)std::chrono::duration_cast<std::chrono::microseconds>(
                       clk::now() - tw).count();
        if (wus > st.max_write_us) st.max_write_us = wus;

        // Loopback'ten geri oku — 163 bayt @115200 ≈ 14 ms
        size_t got = readBack(fd, rx, PKT_LEN, 25);
        if      (got == 0)        st.rx_none++;
        else if (got < PKT_LEN)   st.rx_short++;      // KESIK paket kaniti
        else {
            bool zero = false;
            for (size_t i = 0; i < PKT_LEN; ++i) if (rx[i] == 0) { zero = true; break; }
            if (zero)                              st.rx_zero++;
            else if (memcmp(rx, pkt, PKT_LEN) != 0) st.rx_mismatch++;
            else                                    st.rx_ok++;
        }

        next += period;
        const auto now = clk::now();
        if (next < now) {   // node ile ayni telafi mantigi
            long late = (long)std::chrono::duration_cast<std::chrono::microseconds>(
                            now - next).count();
            if (late > st.max_late_us) st.max_late_us = late;
            next = now;
        }
    }
    ::close(fd);

    const double pct = st.sent ? 100.0 / (double)st.sent : 0.0;
    printf("\n===================== SONUC (%s) =====================\n", mode.c_str());
    printf("Gonderilen paket        : %lu\n", (unsigned long)st.sent);
    printf("--- YAZMA tarafi ---\n");
    printf("  tamami yazildi        : %lu (%%%.1f)\n", (unsigned long)st.write_ok,  st.write_ok * pct);
    printf("  YARIM yazildi         : %lu (%%%.1f)  <-- telde kesik paket\n",
           (unsigned long)st.write_partial, st.write_partial * pct);
    printf("  EAGAIN (hic yazilmadi): %lu (%%%.1f)\n", (unsigned long)st.write_eagain, st.write_eagain * pct);
    if (mode == "loop") {
        printf("  'kalani yaz' tekrari  : %lu   <-- single modda bunlar KESIK PAKET olurdu\n",
               (unsigned long)st.resumes);
        printf("  port bosalmasi bekleme: %lu (timeout: %lu)\n",
               (unsigned long)st.waits, (unsigned long)st.wait_timeout);
    }
    printf("  en uzun write suresi  : %.2f ms  (periyot %.1f ms)\n",
           st.max_write_us / 1000.0, 1000.0 / rate);
    printf("--- TELDEN GERI OKUMA (loopback dogrulamasi) ---\n");
    printf("  BIREBIR DOGRU         : %lu (%%%.1f)\n", (unsigned long)st.rx_ok, st.rx_ok * pct);
    printf("  eksik/kesik geldi     : %lu (%%%.1f)\n", (unsigned long)st.rx_short, st.rx_short * pct);
    printf("  icinde 0x00 vardi     : %lu (%%%.1f)  <-- DMA sifirlama bug'i\n",
           (unsigned long)st.rx_zero, st.rx_zero * pct);
    printf("  icerik uyusmadi       : %lu (%%%.1f)\n", (unsigned long)st.rx_mismatch, st.rx_mismatch * pct);
    printf("  hic gelmedi           : %lu (%%%.1f)  (jumper takili mi?)\n",
           (unsigned long)st.rx_none, st.rx_none * pct);
    printf("  tick gecikmesi (max)  : %.2f ms\n", st.max_late_us / 1000.0);
    printf("======================================================\n");

    // Geri okuma hic olmadiysa TELDEN dogrulama yapilamamis demektir —
    // yazma tarafi temiz gorunse bile "TEMIZ" deyip yanlis guven vermeyelim.
    if (st.rx_ok == 0 && st.rx_none > st.sent / 2) {
        printf("SONUC: DOGRULAMA YAPILAMADI — telden veri geri gelmedi.\n");
        printf("       Jumper TX(pin8)-RX(pin10) takili mi? Yazma sayilari\n");
        printf("       yine de gecerli (YARIM/EAGAIN satirlarina bak).\n");
        return 3;
    }
    const bool clean = (st.write_partial == 0 && st.rx_short == 0 &&
                        st.rx_zero == 0 && st.rx_mismatch == 0 && st.rx_none == 0);
    printf("%s\n", clean ? "SONUC: TEMIZ — hic bozuk paket yok."
                         : "SONUC: BOZULMA VAR — yukaridaki sayilara bak.");
    return clean ? 0 : 2;
}
