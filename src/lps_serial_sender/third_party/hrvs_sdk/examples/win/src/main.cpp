/*
Description: This file contains example of how you can: use API to retrieve measured data from radar system.
*/

#include "radar_system.h"
#include "pal/uart_windows.h"
#include "pal/gpio_pin_arduino.h"
#include "pal/time_stdlib.h"
#include "pal/thread_stdlib.h"
#include <iostream>
#include <string.h>
#include <unistd.h>

using namespace hrvs_sdk;

unsigned RADARS_NUMBER = 1;

// EAP UART config parameters:
constexpr char EAP_UART_PORT[] = "COM22";
constexpr int EAP_UART_BAUDRATE = 921600;
constexpr IUART::Parity EAP_UART_PARITY = IUART::Parity::NO;
constexpr IUART::StopBits EAP_UART_STOP_BITS = IUART::StopBits::ONE;
constexpr int EAP_UART_DATABITS = 8;

// Arduino UART (used for triggering measurements by creating GPIO pulses) config parameters:
constexpr char ARDUINO_UART_PORT[] = "COM21";
constexpr int ARDUINO_UART_BAUDRATE = 115200;
constexpr IUART::Parity ARDUINO_UART_PARITY = IUART::Parity::NO;
constexpr IUART::StopBits ARDUINO_UART_STOP_BITS = IUART::StopBits::ONE;
constexpr int ARDUINO_UART_DATABITS = 8;

void configureEapUart(UARTWindows *eapUartPtr)
{
    UARTWindows::ReturnCode uartReturnCode;

    uartReturnCode = static_cast<UARTWindows::ReturnCode>(eapUartPtr->initUART(EAP_UART_PORT));
    if(uartReturnCode == UARTWindows::ReturnCode::OK)
    {
        std::cout << "hrvs_example_win: HRVS EAP UART initialization - passed" << std::endl;
    }
    else
    {
        std::cerr << "hrvs_example_win: HRVS EAP UART initialization  - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }

    uartReturnCode = static_cast<UARTWindows::ReturnCode>(eapUartPtr->configureUART(EAP_UART_BAUDRATE, EAP_UART_PARITY, EAP_UART_STOP_BITS, EAP_UART_DATABITS));

    if (uartReturnCode == UARTWindows::ReturnCode::OK)
    {
        std::cout << "hrvs_example_win: HRVS EAP UART configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "hrvs_example_win: HRVS EAP UART configuration - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }
}

void configureArduinoUart(UARTWindows *arduinoUartPtr)
{
    UARTWindows::ReturnCode uartReturnCode;

    uartReturnCode = static_cast<UARTWindows::ReturnCode>(arduinoUartPtr->initUART(ARDUINO_UART_PORT));
    if(uartReturnCode == UARTWindows::ReturnCode::OK)
    {
        std::cout << "hrvs_example_win: Arduino UART initialization - passed" << std::endl;
    }
    else
    {
        std::cerr << "hrvs_example_win: Arduino UART initialization  - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }

    uartReturnCode = static_cast<UARTWindows::ReturnCode>(arduinoUartPtr->configureUART(ARDUINO_UART_BAUDRATE, ARDUINO_UART_PARITY, ARDUINO_UART_STOP_BITS, ARDUINO_UART_DATABITS));

    if (uartReturnCode == UARTWindows::ReturnCode::OK)
    {
        std::cout << "hrvs_example_win: Arduino UART configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "hrvs_example_win: Arduino UART configuration - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }
}

void parseStatusVariable(const RadarSystem& radarSystem, const ProcessingConfig & processingConfig)
{
    unsigned systemFrameNumber;
    RadarSystem::SystemState systemState;
    unsigned long long status; 
    unsigned resetCounter = 0;
    radarSystem.getRadarSystemState(systemFrameNumber, systemState, status, resetCounter);

    // example of how you can get more information about occurred error by applying bit masks on HRVS SDK status variable
    if(GET_STATUS_ITEM2(status) == RadarSystem::ReturnCode::ERR_RADAR_MSG)
    {
        switch(static_cast<RadarSystem::RadarMessageError>(GET_STATUS_ITEM3(status)))
        {
            case RadarSystem::RadarMessageError::MSG_TIMEOUT:
                std::cerr << "hrvs_example_win: parseStatusVariable() - timeout when receiving '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)]  << "' from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl;
                break;
            case RadarSystem::RadarMessageError::MSG_CORRUPTED:
                std::cerr << "hrvs_example_win: parseStatusVariable() - data of received '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)]  << "' from radar " << unsigned(GET_STATUS_ITEM8(status))  << '/' << processingConfig.numberOfRadars << " are corrupted" << std::endl;
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_MSG_TYPE:
                std::cerr << "hrvs_example_win: parseStatusVariable() - system received radar message '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)] << "' from radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " when expected message was '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)] << "' from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl; 
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_RADAR:
                std::cerr << "hrvs_example_win: parseStatusVariable() - system received radar message '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)] << "' from radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " when it should be received from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl;
                break;
            case RadarSystem::RadarMessageError::RADAR_ERR:
                std::cerr << "hrvs_example_win: parseStatusVariable() - radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " failure" << std::endl;
                break;
            default:
                break;
        }
    }
    // method can be extended to handle any other error (not just RadarSystem::ReturnCode::ERR_RADAR_MSG) 
    // for more details see composition of status variable inside doxygen documentation for getRadarSystemState()
}

int main(int argc, char *argv[])
{
    /************   PARSE CMD ARGUMENTS   ************/

    // default settings:
    int measurementsCount = 5;

    // processing command line arguments:
    int c;
    while ((c = getopt(argc, argv, "m:")) != -1)
    {
        switch (c)
        {
        case 'm':
            measurementsCount = atoi(optarg);
            break;
        default:
            std::cout << "Usage: ./hrvs_example_win.out [-m MEASUREMENTS_COUNT]" << std::endl;
            return EXIT_FAILURE;
        }
    }

    /************   1.CONFIGURE INTERFACES   ************/
    
    // OPTION A: create RadarSystem object in normal mode 
    RadarSystem radarSystem;
    
    // OPTION B: create RadarSystem object in debug mode (use for debugging purposes only, may slow down the system)
    // char debugFolderPath[] = "C:\\HRVS_SDK\\Debug";   // folder has to exist already!
    // RadarSystem radarSystem{true, debugFolderPath};
    
    RadarSystem::ReturnCode rsReturnCode;

    UARTWindows eapUart;
    UARTWindows arduinoUart;
    configureEapUart(&eapUart);
    configureArduinoUart(&arduinoUart);
    GPIOPinArduino arduinoGpio(&arduinoUart);
    TimeStdlib timeSource;
    ThreadStdlib currentThread;
    
    rsReturnCode = radarSystem.configureInterfaces(&eapUart, &arduinoGpio, &timeSource, &currentThread);
    
    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        std::cout << "hrvs_example_win: PAL interfaces configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "hrvs_example_win: PAL interfaces configuration - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        return EXIT_FAILURE;
    }

    /************   2.CONFIGURE RADAR SYSTEM   ************/

    // Configuration settings for radar system:  
    ProcessingConfig processingConfig;
    processingConfig.numberOfRadars = RADARS_NUMBER;
    RadarConfig radarConfig;
    bool autoResetMode = true;
    bool persistMode = true;

    radarSystem.setConfigurations(processingConfig, radarConfig, autoResetMode, persistMode);

    std::cout << "hrvs_example_win: radar system configuration - passed" << std::endl;

    /************   3. HRVS CONFIGURATION   ************/

    rsReturnCode = radarSystem.hrvsConfiguration();

    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        std::cout << "hrvs_example_win: HRVS configuration - passed" << std::endl;
    }
    else
    {
        parseStatusVariable(radarSystem, processingConfig);
        std::cerr << "hrvs_example_win:  HRVS configuration - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        return EXIT_FAILURE;
    }

    /************   4. TRIGGER MEASUREMENT AND GET RADAR DATA   ************/

    int performedMeasurements = 0;
    // declaration of variables for data retrieved by HRVS SDK
    unsigned systemFrameNumber;
    float bodyVelocity[3];
    float avgRange;
    float radarMinRange;
    float radarVelocity;
    float radarSnr;

    while (performedMeasurements < measurementsCount)
    {
        std::cout << "#######################################################################" << std::endl;

        // trigger one measurement and load declared variables with latest postprocessed data from radars
        // OPTION A: trigger measurement without injecting radar velocity and range data
        rsReturnCode = radarSystem.runMeasurement();

        // OPTION B: inject radar velocity and range data every other measurement and trigger measurement
        // (velocity and range returned by radar system will be replaced by injected data)
        // static bool injectData = true;
        // if (injectData)
        // {
        //     radarSystem.injectVelocity(0.0, RadarSystem::RadarId::RADAR1);
        //     radarSystem.injectVelocity(0.0, RadarSystem::RadarId::RADAR2);
        //     radarSystem.injectVelocity(0.0, RadarSystem::RadarId::RADAR3);
        //     radarSystem.injectVelocity(0.0, RadarSystem::RadarId::RADAR4);
        //     radarSystem.injectRange(22.0, RadarSystem::RadarId::RADAR1);
        //     radarSystem.injectRange(22.0, RadarSystem::RadarId::RADAR2);
        //     radarSystem.injectRange(22.0, RadarSystem::RadarId::RADAR3);
        //     radarSystem.injectRange(22.0, RadarSystem::RadarId::RADAR4);
        //     injectData = false;
        // }
        // else
        // {
        //     radarSystem.cancelInjectVelocity(RadarSystem::RadarId::RADAR1);
        //     radarSystem.cancelInjectVelocity(RadarSystem::RadarId::RADAR2);
        //     radarSystem.cancelInjectVelocity(RadarSystem::RadarId::RADAR3);
        //     radarSystem.cancelInjectVelocity(RadarSystem::RadarId::RADAR4);
        //     radarSystem.cancelInjectRange(RadarSystem::RadarId::RADAR1);
        //     radarSystem.cancelInjectRange(RadarSystem::RadarId::RADAR2);
        //     radarSystem.cancelInjectRange(RadarSystem::RadarId::RADAR3);
        //     radarSystem.cancelInjectRange(RadarSystem::RadarId::RADAR4);
        //     injectData = true;
        // }
        // rsReturnCode = radarSystem.runMeasurement();

        if(rsReturnCode == RadarSystem::ReturnCode::OK)
        {
            std::cout << "hrvs_example_win: performed measurement n." << performedMeasurements + 1 << " - passed" << std::endl;

            // retrieving data from radar system by public getters
            radarSystem.getBodyVelocity(systemFrameNumber, bodyVelocity);
            std::cout << "Body velocity for frame n." << systemFrameNumber << " - x: " << bodyVelocity[0] << " m/s, y: " << bodyVelocity[1] << " m/s, z: " << bodyVelocity[2] << " m/s" << std::endl;

            radarSystem.getAvgRange(systemFrameNumber, avgRange);
            std::cout << "Average range for frame n." << systemFrameNumber << " : " << avgRange << " m" << std::endl;

            // radars are indexed from 1
            for(uint8_t i = 1; i <= processingConfig.numberOfRadars; i++)
            {
                std::cout << "### RADAR " << unsigned(i) << " ###" << std::endl;
                
                radarSystem.getRadarMinrange(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarMinRange);
                std::cout << "Min range for frame n." << systemFrameNumber << " : " << radarMinRange << " m" << std::endl;

                radarSystem.getRadarVelocity(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarVelocity);
                std::cout << "Velocity for frame n." << systemFrameNumber << " : " << radarVelocity << " m/s" << std::endl;

                radarSystem.getRadarSnr(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarSnr);
                std::cout << "SNR for frame n." << systemFrameNumber << " : " << radarSnr << " dB" << std::endl;
            }
        }
        else
        {   
            parseStatusVariable(radarSystem, processingConfig);
            std::cerr << "hrvs_example_win: performed measurement n." << performedMeasurements + 1 << " - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        }

        performedMeasurements++;
    }

    std::cout << "#######################################################################" << std::endl;
    
    /************   5. CLOSE RADAR SYSTEM   ************/

    // release resources of radar system:
    radarSystem.closeRadarSystem();

    return EXIT_SUCCESS;
}