#include "pal/uart_linux.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

UARTLinux::UARTLinux() : handle{-1} {}

int UARTLinux::initUART(const char *comPort)
{
    handle = open(comPort, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);

    if (handle == -1)
    {
        return ReturnCode::OPENING_ERROR;
    }

    return ReturnCode::OK;
}

int UARTLinux::configureUART(int baudrate, Parity parity, StopBits stopBits, int dataBits)
{
    struct termios serial;

    serial.c_iflag = 0;
    serial.c_oflag = 0;
    serial.c_lflag = 0;
    serial.c_cflag = CREAD;

    serial.c_cc[VMIN] = 0;
    serial.c_cc[VTIME] = 0;

    switch (baudrate)
    {
    case 50:
        serial.c_cflag |= B50;
        break;
    case 75:
        serial.c_cflag |= B75;
        break;
    case 110:
        serial.c_cflag |= B110;
        break;
    case 134:
        serial.c_cflag |= B134;
        break;
    case 150:
        serial.c_cflag |= B150;
        break;
    case 200:
        serial.c_cflag |= B200;
        break;
    case 300:
        serial.c_cflag |= B300;
        break;
    case 600:
        serial.c_cflag |= B600;
        break;
    case 1200:
        serial.c_cflag |= B1200;
        break;
    case 1800:
        serial.c_cflag |= B1800;
        break;
    case 2400:
        serial.c_cflag |= B2400;
        break;
    case 4800:
        serial.c_cflag |= B4800;
        break;
    case 9600:
        serial.c_cflag |= B9600;
        break;
    case 19200:
        serial.c_cflag |= B19200;
        break;
    case 38400:
        serial.c_cflag |= B38400;
        break;
    case 57600:
        serial.c_cflag |= B57600;
        break;
    case 115200:
        serial.c_cflag |= B115200;
        break;
    case 230400:
        serial.c_cflag |= B230400;
        break;
    case 460800:
        serial.c_cflag |= B460800;
        break;
    case 500000:
        serial.c_cflag |= B500000;
        break;
    case 576000:
        serial.c_cflag |= B576000;
        break;
    case 921600:
        serial.c_cflag |= B921600;
        break;
    case 1000000:
        serial.c_cflag |= B1000000;
        break;
    case 1152000:
        serial.c_cflag |= B1152000;
        break;
    case 1500000:
        serial.c_cflag |= B1500000;
        break;
    case 2000000:
        serial.c_cflag |= B2000000;
        break;
    case 2500000:
        serial.c_cflag |= B2500000;
        break;
    case 3000000:
        serial.c_cflag |= B3000000;
        break;
    case 3500000:
        serial.c_cflag |= B3500000;
        break;
    case 4000000:
        serial.c_cflag |= B4000000;
        break;
    default:
        return ReturnCode::INVALID_BAUDRATE;
    }

    switch (dataBits)
    {
    case 5:
        serial.c_cflag |= CS5;
        break;
    case 6:
        serial.c_cflag |= CS6;
        break;
    case 7:
        serial.c_cflag |= CS7;
        break;
    case 8:
        serial.c_cflag |= CS8;
        break;
    default:
        return ReturnCode::INVALID_BYTESIZE;
    }

    switch (stopBits)
    {
    case StopBits::ONE:
        break;
    case StopBits::TWO:
        serial.c_cflag |= CSTOPB;
        break;
    default:
        return ReturnCode::INVALID_STOPBITS;
    }

    switch (parity)
    {
    case Parity::NO:
        break;
    case Parity::EVEN:
        serial.c_cflag |= PARODD;
        break;
    case Parity::ODD:
        serial.c_cflag |= PARENB | PARODD;
        break;
    default:
        return ReturnCode::INVALID_PARITY;
    }

    if (tcsetattr(handle, TCSANOW, &serial) < 0)
    {
        return ReturnCode::CONFIGURATION_ERROR;
    }

    if (tcflush(handle, TCIFLUSH) < 0)
    {
        return ReturnCode::CONFIGURATION_ERROR;
    }

    return ReturnCode::OK;
}

bool UARTLinux::readUART(uint8_t *buffer, int bytesToRead, int *bytesRead) const
{
    *bytesRead = read(handle, buffer, bytesToRead);

    if (*bytesRead >= 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool UARTLinux::writeUART(uint8_t *buffer, int bytesToWrite, int *bytesWritten) const
{
    *bytesWritten = write(handle, buffer, bytesToWrite);

    if (*bytesWritten == bytesToWrite)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool UARTLinux::closeUART()
{
    return (close(handle) == 0);
}