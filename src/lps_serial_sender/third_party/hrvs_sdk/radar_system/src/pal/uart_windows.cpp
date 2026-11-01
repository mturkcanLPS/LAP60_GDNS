#include <string>
#include "pal/uart_windows.h"

int UARTWindows::initUART(const char *comPort)
{
    std::string comPortPath = "\\\\.\\";
    comPortPath += comPort;

    this->handle = CreateFile(comPortPath.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

    if (this->handle == INVALID_HANDLE_VALUE)
    {
        return ReturnCode::INIT_ERR;
    }

    return ReturnCode::OK;
}

int UARTWindows::configureUART(int baudrate, Parity parity, StopBits stopBits, int dataBits)
{
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(this->handle, &dcbSerialParams))
    {
        return ReturnCode::GET_COMM_STATE_ERR;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = dataBits;

    switch (parity)
    {
    case IUART::Parity::NO:
        dcbSerialParams.Parity = NOPARITY;
        break;
    case IUART::Parity::ODD:
        dcbSerialParams.Parity = ODDPARITY;
        break;
    case IUART::Parity::EVEN:
        dcbSerialParams.Parity = EVENPARITY;
        break;
    default:
        return ReturnCode::SET_COMM_STATE_ERR;
    }

    switch (stopBits)
    {
    case IUART::StopBits::ONE:
        dcbSerialParams.StopBits = ONESTOPBIT;
        break;
    case IUART::StopBits::TWO:
        dcbSerialParams.StopBits = TWOSTOPBITS;
        break;
    default:
        return ReturnCode::SET_COMM_STATE_ERR;
    }

    if (!SetCommState(this->handle, &dcbSerialParams))
    {
        return ReturnCode::SET_COMM_STATE_ERR;
    }

    // Set Serial Timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 10;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 10;

    if (!SetCommTimeouts(this->handle, &timeouts))
    {
        return ReturnCode::SET_COMM_TIMEOUTS_ERR;
    }

    return ReturnCode::OK;
}

bool UARTWindows::readUART(uint8_t *buffer, int bytesToRead, int *bytesRead) const
{
    int status = 0;
    DWORD bytesReadDword = 0;
    status = ReadFile(this->handle, buffer, bytesToRead, &bytesReadDword, NULL);
    *bytesRead = bytesReadDword;
    // status with value 0 means fail
    return status;
}

bool UARTWindows::writeUART(uint8_t *buffer, int bytesToWrite, int *bytesWritten) const
{
    int status = 0;
    DWORD bytesSentDword = 0;
    status = WriteFile(this->handle, buffer, bytesToWrite, &bytesSentDword, NULL);
    // status with value 0 means fail
    *bytesWritten = bytesSentDword;
    return status;
}

bool UARTWindows::closeUART()
{
    return CloseHandle(this->handle);
}

