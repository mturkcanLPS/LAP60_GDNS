#include "pal/gpio_pin_arduino.h"
#include <stdio.h>
#include <time.h>
#include <dos.h>
#include <windows.h>

GPIOPinArduino::GPIOPinArduino(hrvs_sdk::IUART* uart)
{
    pUart = uart;
}

GPIOPinArduino::~GPIOPinArduino()
{
    pUart->closeUART();
}

bool GPIOPinArduino::writePinState(State state) const
{
    // variables for sending msg
    uint8_t arduinoMsg[] = {ARDUINO_SYNC_CHAR, state};
    int writtenBytes;
    
    // variables for reading response
    uint8_t arduinoRespMsg[ARDUINO_RESP_SIZE];

    // sending msg to arduino
    if( pUart->writeUART(arduinoMsg, sizeof(arduinoMsg), &writtenBytes) && 
        writtenBytes == static_cast<int>(sizeof(arduinoMsg)))
    {
        // read arduino response
        if(!readResponseFromUart(arduinoRespMsg, ARDUINO_RESP_SIZE))
        {
            return false;
        }

        // verify GPIO state
        if(arduinoRespMsg[1] == static_cast<uint8_t>(state))
        {
            return true;
        }
    }

    return false;
}

bool GPIOPinArduino::readResponseFromUart(uint8_t *response, int responseLength) const
{
    uint8_t inputBuffer[INPUT_BUFFER_LENGTH];
    int bytesReceived = 0;
    int totalBytesReceived = 0;
    unsigned readCtr = 0;
    unsigned syncIdx = 0;
    bool syncReceived = false;

    do
    {
        if(readCtr != 0)
        {
            Sleep(UART_WAIT);
        }
        
        if(!pUart->readUART(&inputBuffer[totalBytesReceived], responseLength - totalBytesReceived, &bytesReceived))
        {
            return false;
        }

        // skipping bytes until SYNC character is identified
        if(!syncReceived)
        {
            for(unsigned i = 0; i < bytesReceived; i++)
            {
                if(inputBuffer[i] == ARDUINO_SYNC_CHAR)
                {
                    syncReceived = true;
                    break;
                }

                syncIdx++;
            }

            bytesReceived -= syncIdx;
            syncIdx = 0;
        }

        totalBytesReceived += bytesReceived;
        readCtr++;
    }
    while(totalBytesReceived != responseLength && readCtr < UART_MAX_TRIALS);

    if(totalBytesReceived == responseLength)
    {
        for(unsigned i = 0; i < responseLength; i++)
        {
            response[i] = inputBuffer[i];
        }

        return true;
    }

    return false;
}

bool GPIOPinArduino::closePin()
{
    return 0;
}