/**
 * serial_port.hpp — seri port acma/yazma yardimcilari
 */
#pragma once

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <cerrno>
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
 * Paketi yazar. Sonuc:
 *   WriteResult::OK      — tamami yazildi
 *   WriteResult::PARTIAL — kismi yazildi (kernel TX buffer doldu);
 *                          paket bozuk gitti, firmware sync_byte+chksum ile resync yapar
 *   WriteResult::FAILED  — hic yazilamadi (errno set)
 *
 * O_NONBLOCK modda bekleme yapilmaz; sabit frekansli telemetride
 * kismi/başarisiz paketi tamamlamaya calismak yerine drop etmek dogru
 * (bir sonraki tick zaten taze veri gonderir).
 */
enum class WriteResult { OK, PARTIAL, FAILED };

inline WriteResult writePacket(int fd, const uint8_t *data, size_t len)
{
    ssize_t n = ::write(fd, data, len);
    if (n < 0)                 return WriteResult::FAILED;
    if ((size_t)n != len)      return WriteResult::PARTIAL;
    return WriteResult::OK;
}

} // namespace lps
