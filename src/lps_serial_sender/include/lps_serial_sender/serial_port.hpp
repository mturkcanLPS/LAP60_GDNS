/**
 * serial_port.hpp — seri port acma/yazma yardimcilari
 */
#pragma once

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace lps {

inline speed_t toBaudConstant(int baud)
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
        default:
            // Sessizce 115200'e dusmek YOK — typo'lu baud (orn. 560800)
            // fark edilmeden yanlis hizda calismaya yol aciyordu.
            throw std::runtime_error(
                "Desteklenmeyen baud: " + std::to_string(baud) +
                " (gecerli: 9600/19200/38400/57600/115200/230400/460800/921600)");
    }
}

inline int openSerial(const std::string &port, int baud)
{
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) throw std::runtime_error("Serial acilamadi: " + port +
                                         " (" + strerror(errno) + ")");

    // TEK YAZICI GARANTISI — ayni porta ikinci bir process yazarsa
    // byte'lar telde birbirine karisir (checksum tutmaz, "arada bozulma").
    //  - flock : ikinci node instance'i acilista gurultuyle olur
    //  - TIOCEXCL: root olmayan diger process'lerin open()'i EBUSY alir
    //    (ModemManager gibi probe'culara karsi ilk savunma)
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        ::close(fd);
        throw std::runtime_error(
            "Port baska bir process tarafindan kullaniliyor: " + port +
            " (eski node instance'i mi kaldi?)");
    }
    ioctl(fd, TIOCEXCL);   // best-effort, basarisizligi kritik degil

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        throw std::runtime_error("tcgetattr hatasi");
    }
    speed_t spd = toBaudConstant(baud);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);
    cfmakeraw(&tty);
    tty.c_cflag &= ~CRTSCTS;      // hardware flow control kapalı
    tty.c_cflag |=  CLOCAL;       // modem sinyali bekleme
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        throw std::runtime_error("tcsetattr hatasi");
    }

    // tcsetattr KISMI basarida da 0 doner — baud gercekten uygulandi mi
    // geri okuyup dogrula, yoksa sessizce yanlis hizda calisilir.
    struct termios chk;
    if (tcgetattr(fd, &chk) != 0 ||
        cfgetospeed(&chk) != spd || cfgetispeed(&chk) != spd) {
        ::close(fd);
        throw std::runtime_error("Baud " + std::to_string(baud) +
                                 " donanima uygulanamadi: " + port);
    }
    return fd;
}

/**
 * Paketi yazar — TAMAMI cikana kadar israr eder. Sonuc:
 *   WriteResult::OK      — tamami yazildi
 *   WriteResult::PARTIAL — butce dolmasina ragmen bitirilemedi (telde kesik
 *                          paket kaldi); firmware sync_byte+chksum ile resync
 *                          yapar. ARTIK NADIR olmali — sik goruluyorsa hat
 *                          gercekten doymus demektir (baud/send_rate orani).
 *   WriteResult::FAILED  — tek bayt bile yazilamadi (errno set)
 *
 * NEDEN DONGU (26_07_22 teshisi):
 * Eskiden tek ::write() cagirilip yarim kalirsa paket "drop" ediliyordu.
 * Ama O_NONBLOCK write() ya hep ya hic DEGIL: tamponda 100 bayt yer varsa
 * 163 baytin 100'unu YAZAR, gerisini reddeder — yani "drop" karari
 * verildiginde o 100 bayt COKTAN telde. Yarim paketi geri almak mumkun
 * olmadigi icin strateji basindan hataliydi. Loglar bunu dogruladi:
 * bir oturumda 9 saniyede 350 PARTIAL (~saniyede 40 = pratikte HER paket
 * kesik) + toplam 878 EAGAIN. Osiloskoptaki bozulmanin kaynagi buydu.
 *
 * Cozum pyserial'in (Python testinin temiz cikmasinin sebebi) stratejisi:
 * yaz, yer kalmadiysa portun bosalmasini bekle (poll POLLOUT), kalani yaz.
 * Bekleme butce ile sinirli — TX thread'i sonsuza kadar tikamaz.
 * 163 bayt @115200 ≈ 14 ms'de bosalir; 20 ms butce bir paketlik tam drenaji
 * kapsar ve 40 Hz'in 25 ms'lik periyodunun altinda kalir.
 */
enum class WriteResult { OK, PARTIAL, FAILED };

inline WriteResult writePacket(int fd, const uint8_t *data, size_t len,
                               int budget_ms = 20)
{
    size_t off = 0;
    const auto t0 = std::chrono::steady_clock::now();

    while (off < len) {
        const ssize_t n = ::write(fd, data + off, len - off);
        if (n > 0) { off += (size_t)n; continue; }

        if (n < 0 && errno == EINTR) continue;   // sinyal kesti — devam

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            const int elapsed = (int)std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count();
            const int remain = budget_ms - elapsed;
            if (remain <= 0) break;              // butce doldu
            struct pollfd p{fd, POLLOUT, 0};
            if (::poll(&p, 1, remain) <= 0) break;
            continue;                            // port bosaldi — kalani yaz
        }
        break;                                   // gercek hata
    }

    if (off == len) return WriteResult::OK;
    return off > 0 ? WriteResult::PARTIAL : WriteResult::FAILED;
}

} // namespace lps
