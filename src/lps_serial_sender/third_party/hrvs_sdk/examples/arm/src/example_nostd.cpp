/*
Description: This file contains example of how you can: use API to retrieve measured data from radar system without usage any C++ standard library.
*/

#include "radar_system.h"
#include "pal/gpio_pin_linux.h"
#include "pal/uart_linux.h"
#include "pal/time_linux.h"
#include "pal/thread_linux.h"
#include <stdlib.h>
#include <stdio.h>
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
        printf("example_nostd: HRVS EAP UART initialization - passed\n");
    }
    else
    {
        fprintf(stderr, "example_nostd: HRVS EAP UART initialization  - failed (return code %u)\n", uartReturnCode);
        exit(EXIT_FAILURE);
    }

    uartReturnCode = static_cast<UARTLinux::ReturnCode>(uartPtr->configureUART(UART_BAUDRATE, UART_PARITY, UART_STOP_BITS, UART_DATABITS));

    if (uartReturnCode == UARTLinux::ReturnCode::OK)
    {
        printf("example_nostd: HRVS EAP UART configuration - passed\n");
    }
    else
    {
        fprintf(stderr, "example_nostd: HRVS EAP UART configuration - failed (return code %u)\n", uartReturnCode);
        exit(EXIT_FAILURE);
    }
}

void configureGpio(GPIOPinLinux *gpioPinPtr)
{
    if (gpioPinPtr->configure(PIN_ID, PIO_ID))
    {
        printf("example_nostd: GPIO SYNC_IN configuration - passed\n");
    }
    else
    {
        fprintf(stderr, "example_nostd: GPIO SYNC_IN configuration - failed\n");
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
                fprintf(stderr, "example_nostd: parseStatusVariable() - timeout when receiving '%s' from radar %u/%u\n", IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)], GET_STATUS_ITEM7(status), processingConfig.numberOfRadars);
                break;
            case RadarSystem::RadarMessageError::MSG_CORRUPTED:
                fprintf(stderr, "example_nostd: parseStatusVariable() - data of received '%s' from radar %u/%u are corrupted\n", IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)], GET_STATUS_ITEM8(status), processingConfig.numberOfRadars);
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_MSG_TYPE:
                fprintf(stderr, "example_nostd: parseStatusVariable() - system received radar message '%s' from radar %u/%u when expected message was '%s' from radar %u/%u\n", IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)], GET_STATUS_ITEM8(status), processingConfig.numberOfRadars, IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM5(status)], GET_STATUS_ITEM7(status), processingConfig.numberOfRadars);
                break;
            case RadarSystem::RadarMessageError::UNEXPECTED_RADAR:
                fprintf(stderr, "example_nostd: parseStatusVariable() - system received radar message '%s' from radar %u/%u when it should be received from radar %u/%u\n", IRadarMsg::MESSAGE_NAME[GET_STATUS_ITEM6(status)], GET_STATUS_ITEM8(status), processingConfig.numberOfRadars, GET_STATUS_ITEM7(status), processingConfig.numberOfRadars);
                break;
            case RadarSystem::RadarMessageError::RADAR_ERR:
                fprintf(stderr, "example_nostd: parseStatusVariable() - radar %u/%u failure\n", GET_STATUS_ITEM8(status), processingConfig.numberOfRadars);
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
            printf("Usage: ./example_nostd.out [-m MEASUREMENTS_COUNT]\n");
            return EXIT_FAILURE;
        }
    }

    /************   1.CONFIGURE INTERFACES   ************/
    
    RadarSystem radarSystem;
    RadarSystem::ReturnCode rsReturnCode;

    UARTLinux uart;
    GPIOPinLinux gpioPin;
    TimeLib timeSource;
    ThreadLib currentThread;
    
    configureUart(&uart);
    configureGpio(&gpioPin);

    rsReturnCode = radarSystem.configureInterfaces(&uart, &gpioPin, &timeSource, &currentThread);
    
    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        printf("example_nostd: PAL interfaces configuration - passed\n");
    }
    else
    {
        fprintf(stderr, "example_nostd: PAL interfaces configuration - failed (return code %u)\n", rsReturnCode);
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

    printf("example_nostd: radar system configuration - passed\n");

    /************   3. HRVS CONFIGURATION   ************/

    rsReturnCode = radarSystem.hrvsConfiguration();

    if (rsReturnCode == RadarSystem::ReturnCode::OK)
    {
        printf("example_nostd: HRVS configuration - passed\n");
    }
    else
    {
        parseStatusVariable(radarSystem, processingConfig);
        fprintf(stderr, "example_nostd: HRVS configuration - failed (return code %u)\n",  rsReturnCode);
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
        printf("#######################################################################\n");
        
        // trigger one measurement and load declared variables with latest postprocessed data from radars
        rsReturnCode = radarSystem.runMeasurement();
        if(rsReturnCode == RadarSystem::ReturnCode::OK)
        {
            printf("example_nostd: performed measurement n.%u - passed\n", performedMeasurements + 1);

            // retrieving data from radar system by public getters
            radarSystem.getBodyVelocity(systemFrameNumber, bodyVelocity);
            printf("Body velocity for frame n.%u - x: %f m/s, y: %f m/s, z: %f m/s\n", systemFrameNumber, bodyVelocity[0], bodyVelocity[1], bodyVelocity[2]);

            radarSystem.getAvgRange(systemFrameNumber, avgRange);
            printf("Average range for frame n.%u : %f m\n", systemFrameNumber, avgRange);

            // radars are indexed from 1
            for(uint8_t i = 1; i <= processingConfig.numberOfRadars; i++)
            {
                printf("### RADAR %u ###\n", i);

                radarSystem.getRadarMinrange(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarMinRange);
                printf("Min range for frame n.%u : %f m\n", systemFrameNumber, radarMinRange);

                radarSystem.getRadarVelocity(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarVelocity);
                printf("Velocity for frame n.%u : %f m/s\n", systemFrameNumber, radarMinRange);

                radarSystem.getRadarSnr(static_cast<RadarSystem::RadarId>(i), systemFrameNumber, radarSnr);
                printf("SNR for frame n.%u : %f dB\n", systemFrameNumber, radarMinRange);
            }
        }
        else
        {   
            parseStatusVariable(radarSystem, processingConfig); 
            fprintf(stderr, "example_nostd: performed measurement n.%u - failed (return code %u)\n", performedMeasurements + 1, rsReturnCode);
        }

        performedMeasurements++;
    }

    printf("#######################################################################\n");
    
    /************   5. CLOSE RADAR SYSTEM   ************/

    // release resources of radar system:
    radarSystem.closeRadarSystem();

    return EXIT_SUCCESS;
}