/*
Description: This file contains example of how you can: use API to retrieve measured data from radar system.
*/

#include "radar_system.h"
#include "pal/gpio_pin_linux.h"
#include "pal/uart_linux.h"
#include "pal/time_stdlib.h"
#include "pal/thread_stdlib.h"
#include <stdlib.h>
#include <iostream>
#include <getopt.h>

using namespace hrvs_sdk;

unsigned RADARS_NUMBER = 1;

// UART config parameters:
constexpr char UART_PORT[] = "/dev/ttyTHS1";
constexpr int UART_BAUDRATE = 921600;
constexpr IUART::Parity UART_PARITY = IUART::Parity::NO;
constexpr IUART::StopBits UART_STOP_BITS = IUART::StopBits::ONE;
constexpr int UART_DATABITS = 8;

// GPIO config parameters:
constexpr unsigned PIN_ID = 79;
char PIO_ID[] = "gpio79";


void configureUart(UARTLinux *uartPtr)
{
    UARTLinux::ReturnCode uartReturnCode;

    uartReturnCode = static_cast<UARTLinux::ReturnCode>(uartPtr->initUART(UART_PORT));
    if(uartReturnCode == UARTLinux::ReturnCode::OK)
    {
        std::cout << "example_config: HRVS EAP UART initialization - passed" << std::endl;
    }
    else
    {
        std::cerr << "example_config: HRVS EAP UART initialization  - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }

    uartReturnCode = static_cast<UARTLinux::ReturnCode>(uartPtr->configureUART(UART_BAUDRATE, UART_PARITY, UART_STOP_BITS, UART_DATABITS));

    if (uartReturnCode == UARTLinux::ReturnCode::OK)
    {
        std::cout << "example_config: HRVS EAP UART configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "example_config: HRVS EAP UART configuration - failed (return code " << unsigned(uartReturnCode) << ')' << std::endl;
        exit(EXIT_FAILURE);
    }
}

void configureGpio(GPIOPinLinux *gpioPinPtr)
{
    if (gpioPinPtr->configure(PIN_ID, PIO_ID))
    {
        std::cout << "example_config: GPIO SYNC_IN configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "example_config: GPIO SYNC_IN configuration - failed" << std::endl;
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
                std::cerr << "example_config: parseStatusVariable() - timeout when receiving '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)]  << "' from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl;
                break;
            case RadarSystem::RadarMessageError::MSG_CORRUPTED:
                std::cerr << "example_config: parseStatusVariable() - data of received '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)]  << "' from radar " << unsigned(GET_STATUS_ITEM8(status))  << '/' << processingConfig.numberOfRadars << " are corrupted" << std::endl;
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_MSG_TYPE:
                std::cerr << "example_config: parseStatusVariable() - system received radar message '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)] << "' from radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " when expected message was '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)] << "' from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl; 
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_RADAR:
                std::cerr << "example_config: parseStatusVariable() - system received radar message '" << IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)] << "' from radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " when it should be received from radar " << unsigned(GET_STATUS_ITEM7(status)) << '/' << processingConfig.numberOfRadars << std::endl;
                break;
            case RadarSystem::RadarMessageError::RADAR_ERR:
                std::cerr << "example_config: parseStatusVariable() - radar " << unsigned(GET_STATUS_ITEM8(status)) << '/' << processingConfig.numberOfRadars << " failure" << std::endl;
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
            std::cout << "Usage: ./example_config.out [-m MEASUREMENTS_COUNT]" << std::endl;
            return EXIT_FAILURE;
        }
    }

    /************   1.CONFIGURE INTERFACES   ************/
    
    RadarSystem radarSystem;
    RadarSystem::ReturnCode rsReturnCode;

    UARTLinux uart;
    GPIOPinLinux gpioPin;
    TimeStdlib timeSource;
    ThreadStdlib currentThread;
    
    configureUart(&uart);
    configureGpio(&gpioPin);

    rsReturnCode = radarSystem.configureInterfaces(&uart, &gpioPin, &timeSource, &currentThread);
    
    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        std::cout << "example_config: PAL interfaces configuration - passed" << std::endl;
    }
    else
    {
        std::cerr << "example_config: PAL interfaces configuration - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        return EXIT_FAILURE;
    }

    /************   2.CONFIGURE RADAR SYSTEM   ************/

    // Configuration settings for radar system:  
    ProcessingConfig processingConfig;
    processingConfig.numberOfRadars = RADARS_NUMBER;
    
    // Range filter configuration:
    RadarConfig radarConfig;
    radarConfig.radarType = RadarType::RADAR_60GHZ_PROTO_1_1_CONFIG; // Range filter is available only for RADAR_60GHZ_PROTO_1_1_CONFIG radar type!!!
    radarConfig.rangeFilterEnable = true;                            // Enable range filter
    radarConfig.rangeFilterMin = 0.5f;                               // Min value of range filter (in meters)
    radarConfig.rangeFilterMax = 1.0f;                               // Max value of range filter (in meters) 

    bool autoResetMode = true;
    bool persistMode = true;

    radarSystem.setConfigurations(processingConfig, radarConfig, autoResetMode, persistMode);

    std::cout << "example_config: radar system configuration - passed" << std::endl;

    /************   3. HRVS CONFIGURATION   ************/

    rsReturnCode = radarSystem.hrvsConfiguration();

    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        std::cout << "example_config: HRVS configuration - passed" << std::endl;
    }
    else
    {
        parseStatusVariable(radarSystem, processingConfig);
        std::cerr << "example_config:  HRVS configuration - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
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
        rsReturnCode = radarSystem.runMeasurement();
        if(rsReturnCode == RadarSystem::ReturnCode::OK)
        {
            std::cout << "example_config: performed measurement n." << performedMeasurements + 1 << " - passed" << std::endl;

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
            std::cerr << "example_config: performed measurement n." << performedMeasurements + 1 << " - failed (return code " << unsigned(rsReturnCode) << ')' << std::endl;
        }

        performedMeasurements++;
    }

    std::cout << "#######################################################################" << std::endl;
    
    /************   5. CLOSE RADAR SYSTEM   ************/

    // release resources of radar system:
    radarSystem.closeRadarSystem();

    return EXIT_SUCCESS;
}