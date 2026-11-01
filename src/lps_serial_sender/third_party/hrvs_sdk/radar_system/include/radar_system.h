/*
 * SOFTWARE RIGHTS NOTICE:
 *
 * COPYRIGHT 2023 HONEYWELL INTERNATIONAL INC.  ALL RIGHTS RESERVED.
 * THE INFORMATION CONTAINED HEREIN MY NOT BE REPRODUCED, USED, DISTRIBUTED
 * OR DISCLOSED TO OTHERS WITHOUT THE WRITTEN CONSENT OF HONEYWELL.
 * NOTHING CONTAINED HEREIN SHALL BE CONSTRUED AS CONFERRING BY IMPLICATION,
 * ESTOPPEL, OR OTHERWISE ANY LICENSE TO ANY PATENT, TRADEMARK, COPYRIGHT
 * OR OTHER INTELLECTUAL PROPERTY RIGHT OF HONEYWELL OR ANY THIRD PARTY.
 *
 */

#ifndef RADAR_SYSTEM_H
#define RADAR_SYSTEM_H

#include <string.h>

#include "messages/message.h"
#include "pal_interfaces/i_uart.h"
#include "pal_interfaces/i_gpio_pin.h"
#include "pal_interfaces/i_thread.h"
#include "pal_interfaces/i_time.h"
#include "configurations/userConfigs.h"

#ifndef NO_INFO
#include "utils/async_log_writer.h"
#endif // #ifndef NO_INFO


#ifdef MSG_LOGGING
#include "configurations/loggingConfig.h"
#include "utils/async_file_writer.h"
#endif // #ifdef MSG_LOGGING

/******************************   MACROS   ******************************/

#ifndef SDK_VER_MAJ
#define SDK_VER_MAJ 0
#endif // #ifndef SDK_VER_MAJ

#ifndef SDK_VER_MIN
#define SDK_VER_MIN 0
#endif // #ifndef SDK_VER_MIN

#ifndef SDK_VER_BUILD
#define SDK_VER_BUILD 0
#endif // #ifndef SDK_VER_BUILD

// Check if SDK_LIB_NAME is defined
#ifndef SDK_LIB_NAME
#define SDK_LIB_NAME "default_lib.lib"
#endif

// status variable constants
# define STATUS_DEFAULT_VALUE                         0ULL
# define STATUS_NULLPTR_EAP_UART_POS                  16
# define STATUS_NULLPTR_GPIO_POS                      17
# define STATUS_NULLPTR_THREAD_POS                    18
# define STATUS_NULLPTR_TIME_POS                      19
# define STATUS_CLOSE_EAP_UART_POS                    8
# define STATUS_CLOSE_GPIO_POS                        9
# define STATUS_EAP_MASK_IN_POS                       16
# define STATUS_EAP_MASK_OUT_POS                      17

// macros for setting values to status variable (more info at documentation of getRadarSystemState()):
# define SET_STATUS_BIT(status, pos, value)           ((status & ~(1ULL << pos))     | (unsigned long long) value << pos)   
# define SET_STATUS_ITEM1(status, value)              ((status & 0xFFFFFFFFFFFFFF00) | (unsigned long long) value)
# define SET_STATUS_ITEM2(status, value)              ((status & 0xFFFFFFFFFFFF00FF) | (unsigned long long) value << 8)
# define SET_STATUS_ITEM3(status, value)              ((status & 0xFFFFFFFFFF00FFFF) | (unsigned long long) value << 16)
# define SET_STATUS_ITEM4(status, value)              ((status & 0xFFFFFFFF00FFFFFF) | (unsigned long long) value << 24)
# define SET_STATUS_ITEM5(status, value)              ((status & 0xFFFFFF00FFFFFFFF) | (unsigned long long) value << 32)
# define SET_STATUS_ITEM6(status, value)              ((status & 0xFFFF00FFFFFFFFFF) | (unsigned long long) value << 40)
# define SET_STATUS_ITEM7(status, value)              ((status & 0xFF00FFFFFFFFFFFF) | (unsigned long long) value << 48)
# define SET_STATUS_ITEM8(status, value)              ((status & 0x00FFFFFFFFFFFFFF) | (unsigned long long) value << 56)

// macros for parsing values from status variable (more info at documentation of getRadarSystemState()):
# define GET_STATUS_BIT(status, pos)                  ((bool)    (status  & (1ULL << pos)))
# define GET_STATUS_ITEM1(status)                     ((uint8_t) (status  & 0x00000000000000FF))
# define GET_STATUS_ITEM2(status)                     ((uint8_t) ((status & 0x000000000000FF00) >> 8))
# define GET_STATUS_ITEM3(status)                     ((uint8_t) ((status & 0x0000000000FF0000) >> 16))
# define GET_STATUS_ITEM4(status)                     ((uint8_t) ((status & 0x00000000FF000000) >> 24))
# define GET_STATUS_ITEM5(status)                     ((uint8_t) ((status & 0x000000FF00000000) >> 32))
# define GET_STATUS_ITEM6(status)                     ((uint8_t) ((status & 0x0000FF0000000000) >> 40))
# define GET_STATUS_ITEM7(status)                     ((uint8_t) ((status & 0x00FF000000000000) >> 48))
# define GET_STATUS_ITEM8(status)                     ((uint8_t) ((status & 0xFF00000000000000) >> 56))

/*****************************   HRVS SDK   *****************************/

namespace hrvs_sdk
{       
        /********************   FORWARD CLASS DECLARATIONS   ********************/

        class MinRangeMsg;
        class ObjectInfoMsg;
        class FilteredPointsMsg;
        class ProcessedData;

        /***************************   RADAR SYSTEM  ****************************/

        /**
        * @brief This class implements the interface to Honeywell Velocity Radar System (HRVS). \n
                 The interface allows to configure, execute the measurement and process the measurement data.
        * @note Used terms: \n
                        - Radar System - HRVS SDK API. \n
                        - Radar System Host - A platform the Radar System is running on. \n
                        - HRVS - Honeywell Velocity Radar System. \n
                        - Radar - Instance of radar the HRVS consists from. \n
                        - UART - Universal Asynchronous Receiver Transmitter. \n
                        - GPIO - General Purpose Input Output.
        */
        class RadarSystem
        {
           
        /******************************   ENUMS   *******************************/
        
        public:
                /**
                 * @brief States of radar system machine. Possible transitions can be seen on image included in documentation.
                 */
                enum SystemState : uint8_t
                {
                        INITIAL_STATE,         /**< Initial state. */
                        INTERFACES_CONFIGURED, /**< PAL interfaces have been assigned to radar system. */
                        CONFIGURATIONS_SET,    /**< Radar system and HRVS configurations have been entered by user. */
                        RADARS_CONFIGURED,     /**< HRVS is configured. */
                        MEASUREMENT_OK,        /**< HRVS measurement succeeded (all data received, valid,  successfully processed, can be retrieved by getter methods). */ 
                        MEASUREMENT_FAIL,      /**< HRVS measurement failed (something failed, radars data can not be retrieved by getter methods). */        
                        CLOSED
                };

                /**
                 * @brief Methods avaiable by API.
                 */
                enum Methods : uint8_t
                {
                        CONFIGURE_INTERFACES     // configureInterfaces()
                        , SET_CONFIGURATIONS     // setConfigurations()
                        , HRVS_CONFIGURATION     // hrvsConfiguration()
                        , RUN_MEASUREMENT        // runMeasurement()
                        , GET_BODY_VELOCITY      // getBodyVelocity()
                        , GET_AVG_RANGE          // getAvgRange()
                        , GET_RADAR_MINRANGE     // getRadarMinrange()
                        , GET_RADAR_VELOCITY     // getRadarVelocity()
                        , GET_RADAR_SNR          // getRadarSnr()
                        , GET_RADAR_SYSTEM_STATE // getRadarSystemState()
                        , GET_SDK_VERSION        // getSDKVersion()
                        , GET_HRVS_VERSION       // getHRVSVersion()
                        , RESET                  // reset()
                        , CLOSE_RADAR_SYSTEM     // closeRadarSystem()                      
                };

                /**
                 * @brief All return codes of API non-void methods.
                 */
                enum ReturnCode : uint8_t
                {
                        OK,                             /**< Method succesfully completed. */                             
                        METHOD_NOT_ALLOWED,             /**< System is in state in which method can not be called. Possible transitions can be seen on image included in documentation. */
                        NULLPTR_ASSIGNMENT,             /**< Nullpointer assignment. */
                        ERR_UART,                       /**< Error caused by user provided UART interface instance (some interface method has thrown false value). */
                        ERR_GPIO,                       /**< Error caused by user provided GPIO interface instance (some interface method has thrown false value). */
                        ERR_RADAR_MSG,                  /**< Error arised during radar message reception from HRVS or radar message verification. */
                        INDEX_OUT_OF_RANGE,             /**< Entered index is out of range. */
                        HRVS_SDK_ERROR,                 /**< Error caused by SDK implementation (e.g. insufficient buffer lengths). */   
                        EAP_MODEL_ERROR                 /**< Error signaled by EAP model (maskIn / maskOut are not zero). */          
                };

                /**
                 * @brief Enumeration for individual HRVS radars.
                 */
                enum RadarId : uint8_t
                {
                        UNSET_RADAR_ID,
                        RADAR1,
                        RADAR2,
                        RADAR3,
                        RADAR4
                };

                /************   ENUMS USED ONLY IN STATUS VARIABLE   ************/
                
                /**
                 * @brief Intance of UART interface which caused ReturnCode::ERR_UART | ReturnCode::ERR_RADAR_MSG error.
                 */
                enum UartInstance : uint8_t
                {       
                        NO_UART_INSTANCE,       /**< No instance set. */
                        EAP_UART                /**< UART communicating through EAP port. Note: HRVS SDK provided for users contains only this instance. */   
                };

                /**
                 * @brief Enum used to give more info about ReturnCode::ERR_UART error.
                 */
                enum UartError : uint8_t
                {
                        NO_UART_ERR,            /**< No error. */
                        WRITING_UART_ERR,       /**< UART interface method writeUART() has thrown false. */
                        READING_UART_ERR        /**< UART interface method readUART() has thrown false. */
                };


                /**
                 * @brief Enum used to give more info about ReturnCode::ERR_GPIO error.
                 */
                enum GpioError : uint8_t
                {
                        NO_GPIO_ERR,                    /**< No error set. */
                        WRITING_GPIO_ERR                /**< GPIO interface method writePinState() has thrown false. */
                };

                /**
                 * @brief Enum used to give more info about ReturnCode::ERR_RADAR_MSG error. 
                 */
                enum RadarMessageError : uint8_t
                {
                        NO_MSG_ERR,             /**< No error set. */
                        MSG_TIMEOUT,            /**< Timeout while receiving message via UART from HRVS. */
                        MSG_CORRUPTED,          /**< Received message from HRVS is corrupted (computed and received checksum does not match). */
                        UNEXPECTED_MSG_TYPE,    /**< Received message with unexpected type (expected message type ID and received message type ID does not match). */
                        UNEXPECTED_RADAR,       /**< Received message from unexpected radar (expected radar ID and received message radar ID does not match). */           
                        RADAR_ERR               /**< Radar error identified by special radarId (0xFF) in minrange message. */
                };                

                /**
                 * @brief Enum used to give more info about ReturnCode::HRVS_SDK_ERROR error.
                 */
                enum HrvsSdkErrorType : uint8_t
                {
                        NO_SDK_ERR,                     /**< No error set. */
                        SMALL_IN_BUFFER_LENGTH,         /**< Insufficient length for buffer storing data received from EAP UART. */
                        SMALL_OUT_BUFFER_LENGTH         /**< Insufficient length for buffer storing data to be send via EAP UART. */
                };

        /****************************   CONSTANTS   *****************************/
        public:
                static constexpr unsigned MAX_NUMBER_OF_RADARS = 4;                       /**< Max number of HRVS radars.*/
                static constexpr unsigned MAX_DEBUG_FOLDER_PATH_LEN = 256;                /**< Max length of path to folder where folder with HRVS SDK logs, folder with name "hrvsSdkLogs_N", is created when debug mode is enabled. */
        private:
                static constexpr unsigned IN_BUFFER_SIZE = 65536;
                static constexpr unsigned OUT_BUFFER_SIZE = 64;

                static constexpr unsigned LOADING_TIME_MS = 2000;                       /**< Waiting period to boot up. */
                static constexpr unsigned ACK_WAITING_TIME_MS = 500;                   /**< Waiting period to get acknowledgment message. */
                static constexpr unsigned RECEIVE_DATA_TIMEOUT_MS = 1000;               /**< Max waiting period for getting data. */
                static constexpr unsigned TRIGGER_PULSE_DURATION_MS = 1;               /**< Duration of pulse for triggering measurement on radar. */
                static constexpr unsigned RESET_ICB_DELAY = 500;                       /**< Period for holding reset GPIO in state 1.  */
                static constexpr unsigned UART_SLEEP_TIME = 100;                        /**< If UART reading does not return any data, sleep for this time and try again until timeout. */
                static constexpr unsigned ERR_MEASUREMENT_COOLDOWN = 300;               /**< After error during reception of measurement data in previous measurement run, 
                                                                                             wait at least this time before flushing data on EAP UART and triggering next measurement. */

                static constexpr unsigned NUM_OF_POINTS_IN_FRAME = 10;
                static constexpr unsigned NOISE_GAIN_THRESHOLD = 200;
                static constexpr float NOISE_GAIN_STEP = 0.01;
                static constexpr float NOISE_GAIN_LIMIT = 1.04f;                        /**< Noise Gain Limit in dB. */

                static constexpr unsigned SDK_VERSION_LEN = 3;
                static constexpr unsigned SDK_VERSION_MAJ = SDK_VER_MAJ;
                static constexpr unsigned SDK_VERSION_MIN = SDK_VER_MIN;
                static constexpr unsigned SDK_VERSION_BUILD = SDK_VER_BUILD;
                static constexpr unsigned HRVS_VERSION_LEN = 4;

                static constexpr char* SYSTEM_STATE_NAMES[] =   {
                                                                (char*)"INITIAL_STATE",
                                                                (char*)"INTERFACES_CONFIGURED",
                                                                (char*)"CONFIGURATIONS_SET",
                                                                (char*)"RADARS_CONFIGURED",
                                                                (char*)"MEASUREMENT_OK",
                                                                (char*)"MEASUREMENT_FAIL",
                                                                (char*)"CLOSED"
                                                                };

                static constexpr char EAP_COMMIT[] = "0xffba6cc";                                      /**< Commit of EAP model. */
                static constexpr float DEFAULT_CLOCK = 1.0f;                                           /**< Value of clock EAP model parameter. */
                static constexpr unsigned EAP_TEST_MODE = 0;                                           /**< Value of testMode EAP model parameter. */
                static constexpr uint8_t DEFAULT_GROUND_AIR_MODE = 1;                                  /**< Value of groundAirMode EAP model parameter. */
                static constexpr unsigned char DFS_LIMIT_REACHED[MAX_NUMBER_OF_RADARS] = {0, 0, 0, 0}; /**< Value of dfsLimitReached EAP model parameter. */
                static constexpr float CLOSE_RANGE60 = 22.0f;                                          /**< Value of closeRange60 EAP model parameter. */
                static constexpr float CLOSE_RANGE80 = 18.0f;                                          /**< Value of closeRange80 EAP model parameter. */
                static constexpr unsigned char ENABLE_OVERFLOW = 0;                                    /**< Value of enableOverflow EAP model parameter. */
                
                static constexpr uint16_t INVALID_FRAME_NUMBER = -1;                    /**< Value indicating invalid frame number. 
                                                                                             Verification of expected frame numbers is skipped with this value. */

                
                static constexpr unsigned RESET_MAX_ERR_TRIALS = 3;                     /**< When autoResetMode mode is enabled (for more info about autoResetMode see method setConfigurations()), 
                                                                                             HRVS is resetted only when method hrvsConfiguration() or runMeasurement() returns error any return code
                                                                                             at least number of times, specified by this constant, in a row. */
                static constexpr unsigned RESET_MAX_RANGE_FRAMES = 5;                   /**< How many frames after reset is max range computed by EAP model ignored (system meanwhile uses max range computed before reset). */
                
                static constexpr unsigned RADAR_CONTROL_MSG_CHSUM_ERR = 0xBBBB;         /**> Value in minrange message frameNumber field, which indicates that radar received control message with checksum error. */

                static constexpr char DEFAULT_DEBUG_FOLDER_PATH[] = "./";               /**< Default path of folder where folder with HRVS SDK logs, folder with name "hrvsSdkLogs_N", is created when debug mode is enabled. */

#ifndef NO_INFO
                static constexpr char HRVS_SDK_LOGS_FILE[] = "hrvsSdkLogs.txt";          /**< Name of file where are stored debug HRVS SDK logs of easyLogging framework. */
#endif // #ifdef NO_INFO

#if defined(MSG_LOGGING) || !defined(NO_INFO)
                static constexpr char LOG_FOLDER_NAME_PREFIX[] = "hrvsSdkLogs_";                                /**< Prefix of folder name where are stored debug log files. */      
                static constexpr unsigned MAX_LOG_FOLDER_NAME_LEN = sizeof(LOG_FOLDER_NAME_PREFIX) + 3;         /**< Max length of folder name where are stored debug log files. */        
#endif // #if defined(MSG_LOGGING) || !defined(NO_INFO)

        /********************************* API ********************************/

        public:
                /**
                 * @brief Constructor of RadarSystem.
                 *
                 * @param[in] debugModeEnable Debug mode enables injection of velocities and ranges to radars (feature available in all SDK libraries).
                 *                            Enables logging all messages transmitted between Radar System Host and HRVS to csv files inside "hrvsSdkLogs_N" folders (feature available only in SDK libraries compiled with C++ standard library). 
                 *                            Enables extended TRACE logging of HRVS SDK operations  (feature available only in SDK libraries compiled with C++ standard library).
                 *                            Use this option only for testing purposes, as it may slow down the system.
                 * @param[in] debugFolderPath Path to folder where folder with HRVS SDK logs, folder with name "hrvsSdkLogs_N", is created when debug mode is enabled.
                 *                            Default path "./" is used when debug mode is enabled and this parameter is nullptr.
                 */
                RadarSystem(bool debugModeEnable = false, const char *debugFolderPath = nullptr);

                /**
                 * @brief Releases resources of RadarSystem, if they were not already released by method closeRadarSystem().
                 */
                ~RadarSystem();

                /**
                 * @brief Configures communication interfaces between Radar System Host and HRVS.
                 * 
                 * @param[in] eapUart Instance of class implementing IUART interface. The interface must initialized and properly configured.
                 * @param[in] gpioPin Instance of class implementing IGPIOPin interface. The interface must initialized and properly configured.
                 * @param[in] timeSource Instance of class implementing ITime interface.
                 * @param[in] thread Instance of class implementing IThread interface.
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::NULLPTR_ASSIGNMENT.
                 * 
                 * @note HRVS should be powered on and physically interconnected with Radar System Host at the time of calling this method.
                 */
                ReturnCode configureInterfaces(IUART *eapUart, IGPIOPin *gpioPin, ITime *timeSource, IThread *thread);

                /**
                 * @brief Sets HRVS configuration parameters.
                 *
                 * @param[in] processingConfig Settings required for computation of 3D body velocity.
                 * @param[in] radarConfig Radar configuration parameters.
                 * @param[in] autoResetMode In case method hrvsConfiguration() or runMeasurement() ended with error RESET_MAX_ERR_TRIALS of times in a row, 
                 * their next call will automatically call reset() method before next execution of hrvsConfiguration() or runMeasurement().
                 * @param[in] persistMode Methods hrvsConfiguration() and runMeasurement() are indefenitely repeated until method ends succesfully.
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED.
                 */
                ReturnCode setConfigurations(
                const ProcessingConfig &processingConfig
                , const RadarConfig &radarConfig
                , bool autoResetMode
                , bool persistMode
                );

                /**
                 * @brief Sends configuration message to HRVS and waits for reception of configuration acknowledgment message from all connected radars.
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::ERR_UART | ReturnCode::ERR_GPIO | ReturnCode::ERR_RADAR_MSG | ReturnCode::HRVS_SDK_ERROR.
                 */
                ReturnCode hrvsConfiguration();

                /**
                 * @brief Can replace measured velocity by injected velocity for each radar for debug purposes.
                 *
                 * @param[in] injectedVelocity Injected velocity [m/s].
                 * @param[in] radarId Id of radar to inject velocity.
                 *
                 * @warning Injected velocity is used only when debug mode is enabled.
                 * @warning To Disable injected velocity, call method cancelInjectVelocity().
                 */
                void injectVelocity(float injectedVelocity, RadarId radarId);

                /**
                 * @brief Cancels injected velocity for specified radar.
                 *
                 * @param[in] radarId Id of radar to cancel injected velocity.
                 * @return true Injected velocity was successfully canceled.
                 * @return false Velocity was not injected for specified radar.
                 */
                bool cancelInjectVelocity(RadarId radarId);

                /**
                 * @brief Injects range for specified radar for debug purposes.
                 *
                 * @param[in] injectedRange Injected range [m].
                 * @param[in] radarId Id of radar to inject range.
                 *
                 * @warning Injected range is used only when debug mode is enabled.
                 * @warning To Disable injected range, call method cancelInjectRange().
                 */
                void injectRange(float injectedRange, RadarId radarId);

                /**
                 * @brief Cancels injected range for specified radar.
                 *
                 * @param[in] radarId Id of radar to cancel injected range.
                 * @return true Injected range was successfully canceled.
                 * @return false Range was not injected for specified radar.
                 */
                bool cancelInjectRange(RadarId radarId);

                /**
                 * @brief Triggers one measurement and processes measurement data (unpacking, storing, logging).
                 * @param[in] inVelBodyValid (optional) Set this flag to true if you want to pass body velocity (parameters inVelBodyX, inVelBodyY, inVelBodyZ) to EAP model.
                 * @param[in] inVelBodyX (optional) Input body velocity in x-axis [m/s] passed to EAP model which improve its performance. Data are used only when inVelBodyValid is set to true.
                 * @param[in] inVelBodyY (optional) Input body velocity in y-axis [m/s] passed to EAP model which improve its performance. Data are used only when inVelBodyValid is set to true.
                 * @param[in] inVelBodyZ (optional) Input body velocity in z-axis [m/s] passed to EAP model which improve its performance. Data are used only when inVelBodyValid is set to true.
                 * @note Use this method, if you want further process measured data from radars by own algorithm.
                 * After calling this method, you can extract measured data by getter methods.
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::ERR_UART | ReturnCode::ERR_GPIO | ReturnCode::ERR_RADAR_MSG | ReturnCode::HRVS_SDK_ERROR | ReturnCode::EAP_MODEL_ERROR.
                 */
                ReturnCode runMeasurement(bool inVelBodyValid = false,
                                          float inVelBodyX = 0.0f,
                                          float inVelBodyY = 0.0f,
                                          float inVelBodyZ = 0.0f);

                /**
                 * @brief Get computed 3D body velocity [m/s] by EAP model in current measurement cycle.
                 *
                 * @param[out] radarSystemFrameNumber Reference where is copied current radar system frame number.
                 * @param[out] array Array with 3 dimensions, where are copied x,y,z body velocity [m/s].
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED.
                 * 
                 * @note When ReturnCode::METHOD_NOT_ALLOWED is returned, output reference parameters stay unchanged.
                 */
                ReturnCode getBodyVelocity(unsigned &radarSystemFrameNumber, float (&array)[3]);

                /**
                 * @brief Get average range [m] computed by averaging distances from largest objects which have been detected by radars.
                 *
                 * @param[out] radarSystemFrameNumber Reference where is copied current radar system frame number.
                 * @param[out] avgRange Reference where is copied average range [m].
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED.
                 * 
                 * @note When ReturnCode::METHOD_NOT_ALLOWED is returned, output reference parameters stay unchanged.
                 */
                ReturnCode getAvgRange(unsigned &radarSystemFrameNumber, float &avgRange);

                /**
                 * @brief Get found minimum range [m] received from radar specified by its ID in current measurement cycle.
                 *
                 * @param[in] radarId Id of radar from which you want data. \n
                 * Possible values are: <RadarId::RADAR1, number_of_used_radars>. \n
                 * If value is not in expected range, method returns return code INDEX_OUT_OF_RANGE.
                 * @param[out] radarSystemFrameNumber Reference where is copied current radar system frame number.
                 * @param[out] minRangeValue Reference where is copied radar minimum range [m].
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::INDEX_OUT_OF_RANGE.
                 * 
                 * @note When ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::INDEX_OUT_OF_RANGE is returned, output reference parameters stay unchanged.
                 */
                ReturnCode getRadarMinrange(RadarId radarId, unsigned &radarSystemFrameNumber, float &minRangeValue);

                /**
                 * @brief Get velocity [m/s] received from radar specified by its ID in current measurement cycle.
                 *
                 * @param[in] radarId Id of radar from which you want data. \n
                 * Possible values are: <RadarId::RADAR1, number_of_used_radars>. \n
                 * If value is not in expected range, method returns code INDEX_OUT_OF_RANGE.
                 * @param[out] radarSystemFrameNumber Reference where is copied current radar system frame number.
                 * @param[out] radarVelocity Reference where is copied radar velocity [m/s].
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::INDEX_OUT_OF_RANGE.
                 * 
                 * @note When ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::INDEX_OUT_OF_RANGE is returned, output reference parameters stay unchanged.
                 */
                ReturnCode getRadarVelocity(RadarId radarId, unsigned &radarSystemFrameNumber, float &radarVelocity);

                /**
                 * @brief Get signal to noise ratio (SNR) [dB] received from radar specified by its ID in current measurement cycle.
                 *
                 * @param[in] radarId Id of radar from which you want data. \n
                 * Possible values are: <RadarId::RADAR1, number_of_used_radars>. \n
                 * If value is not in expected range, method returns code INDEX_OUT_OF_RANGE.
                 * @param[out] radarSystemFrameNumber Reference where is copied current radar system frame number.
                 * @param[out] radarSnr Reference where is copied radar SNR [dB].
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::INDEX_OUT_OF_RANGE.
                 * 
                 * @note When ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::INDEX_OUT_OF_RANGE is returned, output reference parameters stay unchanged.
                 */
                ReturnCode getRadarSnr(RadarId radarId, unsigned &radarSystemFrameNumber, float &radarSnr);
                    
                /**
                 * @brief Get radar system state.
                 * 
                 * @param[out] systemFrameNumber Reference where is copied current radar system frame number.
                 * @param[out] systemState Reference where is copied current radar system state.
                 * @param[out] status Reference where is copied status of lastly perfomed radar system method. \n
                 * \n
                 * ********************************************************************\n
                 * Status variable description: \n
                 * 
                 * Return code of HRVS SDK API methods provide just limited information about arrised error. \n
                 * More detailed information about arrised error can be extracted from variable with status of \n
                 * lastly performed HRVS SDK API method (further simply called status variable). Status variable \n
                 * is accessible by one of output parameters of method: getSystemStatus(). Content of status \n
                 * variable depends on arrised error type, i.e. depends on return code of HRVS SDK API method: \n
                 * \n
                 * ******************************* VOID ******************************* \n
                 * \n
                 *  All void methods, except closeRadarSystem(), fill status variable followingly: \n
                 *  \n
                 * MSB                                                   54b                                                                 8b      LSB \n
                 *  |----------------------------------------------------------------------------------------------------------------|----------------| \n
                 *  |                                                   UNUSED                                                       |     METHOD     | \n
                 *  |----------------------------------------------------------------------------------------------------------------|----------------| \n
                 *  \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *  \n
                 * ******************************************************************** \n
                 * \n
                 * Method closeRadarSystem() fills status variable followingly: \n
                 * \n
                 * MSB                                                54b                                                       1b 1b        8b      LSB \n
                 *  |----------------------------------------------------------------------------------------------------------|--|--|----------------| \n
                 *  |                                                UNUSED                                                    |C1|C0|     METHOD     | \n
                 *  |----------------------------------------------------------------------------------------------------------|--|--|----------------| \n
                 * \n
                 *  Items: \n
                 *      METHOD - Methods::CLOSE_RADAR_SYSTEM \n
                 *      C0 - EAP UART close status (0 - success / 1 - fail) \n 
                 *      C1 - GPIO close status (0 - success / 1 - fail) \n
                 * \n      
                 * ************************** ReturnCode::OK ************************** \n
                 * \n
                 * MSB                                               48b                                                      8b               8b     LSB \n
                 *  |------------------------------------------------------------------------------------------------|----------------|----------------| \n
                 *  |                                              UNUSED                                            |   RETURN CODE  |     METHOD     | \n
                 *  |------------------------------------------------------------------------------------------------|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::OK \n
                 * \n
                 * ****************** ReturnCode::METHOD_NOT_ALLOWED ****************** \n
                 * \n
                 * MSB                                               48b                                                      8b               8b     LSB \n
                 *  |------------------------------------------------------------------------------------------------|----------------|----------------| \n
                 *  |                                              UNUSED                                            |   RETURN CODE  |     METHOD     | \n
                 *  |------------------------------------------------------------------------------------------------|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::METHOD_NOT_ALLOWED \n
                 * \n
                 * ****************** ReturnCode::NULLPTR_ASSIGNMENT ****************** \n
                 * \n
                 * MSB                                        44 bit                                         1b 1b 1b 1b      8b                  8b     LSB \n
                 *  |---------------------------------------------------------------------------------------|--|--|--|--|----------------|----------------| \n
                 *  |                                         UNUSED                                        |N4|N3|N2|N1|   RETURN CODE  |     METHOD     | \n
                 *  |---------------------------------------------------------------------------------------|--|--|--|--|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::NULLPTR_ASSIGNMENT \n
                 *      N1 - EAP UART interface assignment (0 - OK / 1 - nullptr) \n
                 *      N2 - GPIO interface assignment (0 - OK / 1 - nullptr) \n
                 *      N3 - thread interface assignment (0 - OK / 1 - nullptr) \n
                 *      N4 - time interface assignment (0 - OK / 1 - nullptr) \n    
                 * \n
                 * *********************** ReturnCode::ERR_UART *********************** \n
                 * \n
                 * MSB                                      40b                              8b               8b                8b               8b     LSB \n
                 *  |----------------------------------------------------------------|----------------|----------------|----------------|----------------| \n
                 *  |                                      UNUSED                    |   UART ERROR   |  UART INSTANCE |   RETURN CODE  |     METHOD     | \n
                 *  |----------------------------------------------------------------|----------------|----------------|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::ERR_UART \n
                 *      UART INSTANCE - instance of UART interface causing error (see enum UartInstance) \n
                 *      UART ERROR - type of error (see enum UartError) \n
                 * \n
                 * *********************** ReturnCode::ERR_GPIO *********************** \n
                 * \n
                 * MSB                                     40b                                                8b               8b              8b      LSB \n
                 *  |--------------------------------------------------------------------------------|----------------|----------------|----------------| \n
                 *  |                                     UNUSED                                     |   GPIO ERROR   |   RETURN CODE  |     METHOD     | \n
                 *  |--------------------------------------------------------------------------------|----------------|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::ERR_GPIO \n
                 *      GPIO ERROR - type of error (see enum GpioError) \n
                 * \n
                 * ********************* ReturnCode::ERR_RADAR_MSG ******************** \n
                 * \n
                 * MSB      8b                8b               8b               8b              8b                8b               8b              8b      LSB \n    
                 *  |----------------|----------------|----------------|----------------|----------------|----------------|----------------|----------------| \n
                 *  | REC. RADAR ID  | EXP. RADAR ID  | REC. MSG TYPE  |  EXP. MSG TYP  |    MSG ERROR   |  UART INSTANCE |   RETURN CODE  |     METHOD     | \n
                 *  |----------------|----------------|----------------|----------------|----------------|----------------|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::ERR_RADAR_MSG \n
                 *      UART INSTANCE - instance of UART interface causing error (see enum UartInstance) \n
                 *      MSG ERROR  - type of error (see enum RadarMessageError) \n
                 *      EXP. MSG TYPE - expected message type ID in received message (see enum IRadarMsg::MessageType) \n
                 *      REC. MSG TYPE - parsed message type ID from received message (see enum IRadarMsg::MessageType) \n
                 *      EXP. RADAR ID - expected radar ID in received message (see enum RadarId) \n
                 *      REC. RADAR ID - parsed radar ID from received message (see enum RadarId) \n
                 * \n
                 * ****************** ReturnCode::INDEX_OUT_OF_RANGE ****************** \n
                 * \n
                 * MSB                                             48b                                                       8b                8b     LSB \n    
                 *  |------------------------------------------------------------------------------------------------|----------------|----------------| \n
                 *  |                                              UNUSED                                            |   RETURN CODE  |     METHOD     | \n
                 *  |------------------------------------------------------------------------------------------------|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::INDEX_OUT_OF_RANGE \n
                 * \n
                 * ******************* ReturnCode::HRVS_SDK_ERROR ********************* \n
                 * \n  
                 * MSB                                     40b                                                8b              8b              8b       LSB \n
                 *  |--------------------------------------------------------------------------------|----------------|----------------|----------------| \n
                 *  |                                    UNUSED                                      |    SDK ERROR   |   RETURN CODE  |     METHOD     | \n
                 *  |--------------------------------------------------------------------------------|----------------|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - ID of lastly performed method (see enum Methods) \n
                 *      RETURN CODE - ReturnCode::HRVS_SDK_ERROR \n
                 *      SDK ERROR - type of error (see enum HrvsSdkErrorType) \n
                 * \n
                 * ******************** ReturnCode::EAP_MODEL_ERROR ******************* \n
                 * \n
                 * MSB                                         46 bit                                              1b 1b      8b                  8b     LSB \n
                 *  |---------------------------------------------------------------------------------------------|--|--|----------------|----------------| \n
                 *  |                                         UNUSED                                              |N2|N1|   RETURN CODE  |     METHOD     | \n
                 *  |---------------------------------------------------------------------------------------------|--|--|----------------|----------------| \n
                 * \n
                 * Items: \n
                 *      METHOD - Methods::RUN_MEASUREMENT \n
                 *      RETURN CODE - ReturnCode::EAP_MODEL_ERROR \n
                 *      N1 - 1 if maskIn is not zero, 0 otherwise \n
                 *      N2 - 1 if maskOut is not zero, 0 otherwise \n
                 * \n
                 * ******************************************************************** \n
                 * Note: Unused bits are always 0 for all types of status variable. \n
                 * 
                 * @param[out] resetErrorCounter Reference where is copied count of errors caused by method hrvsConfiguration() or runMeasurement() in row (see variable resetErrorCounter).
                 */
                void getRadarSystemState(unsigned &systemFrameNumber, SystemState &systemState, unsigned long long &status, unsigned &resetErrorCounter) const;
                
                /**
                 * @brief Get version of HRVS SDK.
                 * 
                 * @param[out] version Reference where is copied SDK version. \n
                 *                     Version format: version[0] - major, version[1] - minor, version[2] - build
                 */
                void getSDKVersion(unsigned (&version)[SDK_VERSION_LEN]);

                /**
                 * @brief Get SW version of HRVS radars.
                 * 
                 * @param[out] version Reference where is copied HRVS SW version. \n
                 *                     Version format: version[0][0] - radar1 major, version[0][1] - radar1 minor, version[0][2] - radar1 build, version[0][3] - radar1 type \n
                 *                                                                                             ... \n
                 *                                     version[3][0] - radar4 major, version[3][1] - radar4 minor, version[3][2] - radar4 build, version[3][3] - radar4 type \n                    
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED.
                 * 
                 * @note When ReturnCode::METHOD_NOT_ALLOWED is returned, output reference parameters stay unchanged.
                 */
                ReturnCode getHRVSVersion(unsigned (&version)[MAX_NUMBER_OF_RADARS][HRVS_VERSION_LEN]);

                /**
                 * @brief Powers off and powers on HRVS and reinitialises radar system state variables.
                 * 
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::METHOD_NOT_ALLOWED | ReturnCode::ERR_GPIO.
                 */
                ReturnCode reset();
                
                /**
                 * @brief As logs printed on console are printed in asynchronous way, this method can be used to flush logs and wait until all logs are printed.
                 * @note This method also flushes logs with messages transmitted between Radar System Host and HRVS to csv files (feature available only if debug mode is enabled).
                 */
                void flushLogs();

                /**
                 * @brief Calls flushLogs() method and releases resources of RadarSystem object.
                 *  
                 * @note After method is successfully performed, resource of RadarSystem object are released and object can not be used again.
                 */
                void closeRadarSystem();

        /****************************   METHODS   *****************************/

        private:
                /**
                 * @brief Changes state of radar system.
                 * 
                 * @param[in] newState New state of radar system. 
                 */
                void changeSystemState(SystemState newState);

                /**
                 * @brief Resets all radar system state variables and external aiding model.
                 */
                void resetRadarSystem();

                /**
                 * @brief Sends message to HRVS.
                 * 
                 * @param[in] msg Message to be send to HRVS.
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::ERR_UART.
                 */
                ReturnCode sendMsgToHRVS(IRadarInput &msg);

                
                /**
                 * @brief Moves unhandled bytes to the beginning of the input buffer.
                 *
                 * @param[in] idx Index of input buffer, where starts unhandled bytes.
                 */
                void shiftInputBuffer(unsigned idx);

                /**
                 * @brief Finds synchronization word and type of message in inbut buffer.
                 * Automatically shifts processed bytes in input buffer.
                 * 
                 * @return MessageType Detected message type.
                 */
                IRadarMsg::MessageType detectMessageType();

                /**
                 * @brief Receives and verifies message from EAP UART.
                 * 
                 * @param[out] msgToBeReceived Message to be received from EAP UART.
                 * @param[in] expectedRadarId ID of radar from which is expected message.
                 * @param[in] expectedRadarFrameNumber Expected radar frame number of received message. 
                 * When expected and received frame number do not match, only warning log will be printed (no error is returned). 
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::ERR_UART | ReturnCode::ERR_RADAR_MSG | ReturnCode::HRVS_SDK_ERROR. 
                 */
                ReturnCode receiveEapUartMsg(IRadarOutput &msgToBeReceived, RadarId expectedRadarId, uint16_t expectedRadarFrameNumber);
                
                /**
                 * @brief Triggers measurement by sending GPIO pin signal followed with control message.
                 *
                 * @return Possible return codes: ReturnCode::OK | ReturnCode::ERR_UART | ReturnCode::ERR_GPIO | ReturnCode::HRVS_SDK_ERROR.
                 */
                ReturnCode triggerMeasurement();
                
                /**
                 * @brief Process data from received min range message.
                 * 
                 * @param[in] minRangeMsg Contant reference to received min range message.
                 * @param[in] radarId ID of radar from which is expected message.
                 */
                void handleMinRangeMsg(const MinRangeMsg &minRangeMsg, RadarId radarId);
                
                /**
                 * @brief Process data from received object info message.
                 * 
                 * @param[in] objectInfoMsg Contant reference to received object info message.
                 * @param[in] radarId ID of radar from which is expected message. 
                 * @param[out] processedData Reference to object containing processed data of object info message.
                 */
                void handleObjectInfoMsg(const ObjectInfoMsg &objectInfoMsg, RadarId radarId, ProcessedData &processedData);
                
                /**
                 * @brief Adjusts noise gain by data received from filter point messages.
                 */
                void adjustNoiseGain();    

                /**
                 * @brief Process data from filtered point message.
                 * 
                 * @param[in] filteredPointMsg Contant reference to received filtered point message.
                 * @param[in] radarId ID of radar from which is expected message. 
                 */
                void handleFilteredPointMsg(const FilteredPointsMsg &filteredPointMsg, RadarId radarId);

                /**
                 * @brief Computes 3D body velocity. Computation is done after receiving all messages from HRVS radars.
                 *
                 * @param[in] processedData Reference to object containing processed data of object info message.
                 * @param[in] velBodyValid Flag indicating whether input body velocity is valid.
                 * @param[in] velBody Array with 3 dimensions, where are copied x,y,z input body velocity [m/s].
                 * @return Possible return codes: ReturnCode::OK.
                 */
                ReturnCode computeBodyVelocity(ProcessedData &processedData, bool velBodyValid, const float (&velBody)[3]);

                /**
                 * @brief Method flushes: any available data from EAP UART interface and already received data in input buffer.
                 */
                void flushReadEapUart(); 
#ifdef MSG_LOGGING
                /**
                 * @brief Opens logging files.
                 * @warning Existing files are rewritten! Make new directories, if you want to log multiple runs (log files of run will be stored in separate directories and files won't be rewritten).
                 */
                void openLog();

                /**
                 * @brief Prints corresponding headers to opened csv files and STDOUT.
                 */
                void printHeaders();

                /**
                 * @brief Logs radar message sent/received through EAP UART (only if logging of message type is enabled by logging configuration).
                 * @param[in] radarMsg Config message to log.
                 */
                void logRadarMsg(IRadarMsg &radarMsg);


                /**
                 * @brief Logs inputs and outputs of EAP model.
                 */
                void logEapData(const ProcessedData &processedData,
                                bool velBodyValid,
                                const float (&velBody)[3],
                                const float validRatio,
                                const unsigned pointsOut[MAX_NUMBER_OF_RADARS],
                                const float refBeamOut[MAX_NUMBER_OF_RADARS],
                                const unsigned char zeroTht[MAX_NUMBER_OF_RADARS],
                                double maskIn,
                                unsigned short maskOut);


                /**
                 * @brief Closes logging files.
                 */
                void closeLog();
#endif // #ifdef MSG_LOGGING
        
#if defined(MSG_LOGGING) || !defined(NO_INFO)
        /**
         * @brief Method checks if directory, which path is passed as argument, exists.
         * 
         * @param[in] dirName Path to directory.
         * @return true Directory exists.
         * @return false Directory does not exist.
         */
        bool checkIfDirectoryExists(const char *dirName);
        
        
        /**
         * @brief Method finds out name of new debug directory inside debug directory path.
         * 
         * @param[in] debugDirPath Path to debug directory.
         * @param[out] debugDirName Reference where is copied name of new debug directory.
         */
        void getNewDebugDirectoryName(const char *debugDirPath, char (&debugDirName)[MAX_LOG_FOLDER_NAME_LEN]);

        /**
         * @brief Method creates directory with given name.
         *
         * @param[in] dirname Name of directory to be created.
         * @return True if directory was created successfully, false otherwise.
         */
        bool createDirectory(const char *dirname);
#endif // #if defined(MSG_LOGGING) || !defined(NO_INFO)

        /****************************** VARIABLES *******************************/
        private:
                // debug mode:
                bool debugMode;                                 /**< Debug mode enables injection of velocities and ranges to radars and extended logging features of transmitted messages and HRVS SDK operations. */
#ifndef NO_INFO
                AsyncLogWriter asyncLogWriter;                  /**< Object for logging messages with easylogging library in another thread. */
#endif // #ifndef NO_INFO
#if defined(MSG_LOGGING) || !defined(NO_INFO)
                char debufFolderFullPath[MAX_DEBUG_FOLDER_PATH_LEN + 1 + MAX_LOG_FOLDER_NAME_LEN];       /**< Full path (even with found out "hrvsSdkLogs_N" suffix) of folder with HRVS SDK logs when debug mode is enabled. */
                bool debugFolderCreated;                                                                 /**< Whether debug folder was successfully created. */
#endif // #if defined(MSG_LOGGING) || !defined(NO_INFO)

                // instances implementing interfaces:
                IUART *eapUart;                        /**< UART communicating through EAP port.*/
                IGPIOPin *measurementTriggerPin;
                ITime *systemTimeSource;
                IThread *currentThread;

                // state:
                SystemState systemState;                /**< Current state of radar system. */
                unsigned long long statusVariable;      /**< Variable with encoded status of lastly performed API method (more info at documentation of getRadarSystemState()) */
                unsigned resetErrorCounter;             /**< Number of times method hrvsConfiguration() or runMeasurement() ended with error in row (used for deciding when to reset HRVS). */
                unsigned resetMaxRangeFramesCtr;        /**< Number of next frames in which calculated value of max range by EAP model will be ignored.  */
                unsigned syncWordIdx;                   /**< How many bytes of SYNC WORD of currently receiving message has been processed. */        


                // time info:
                unsigned systemFrameNumber;             /**< System frame number (is not resetted with reset() method). */
                uint16_t lastRadarFrameNumber;          /**< Radar frame number (is resetted with reset() method). */
                long long radarBootTime;                /**> Time of bringing power to radars (returned by systemTimeSource). */
                long long triggerMeasurementTime;       /**< Time of triggered last measurement (returned by systemTimeSource). */

                // UART:
                uint8_t inBuffer[IN_BUFFER_SIZE];
                uint8_t outBuffer[OUT_BUFFER_SIZE];
                unsigned bytesInInBuffer;

                // configuration entered by user:
                ProcessingConfig processingConfig;
                RadarConfig radarConfig;
                bool autoResetMode;
                bool persistMode;
                
                // configuration taken from HRVS config message (values are reused in radar system methods):
                float cfgStartFrequency;
                float cfgMaxVelocity;

                // SW loaded on HRVS (taken from config ack):
                unsigned hrvsSwVersion[MAX_NUMBER_OF_RADARS][HRVS_VERSION_LEN];
                
                // raw data from radar messages:
                float radarSnr[MAX_NUMBER_OF_RADARS];

                // output EAP model data:
                float maxRange;
                int radarBodyVelocitySizeOut[2];
                float radarBodyVelocity[3];
                float velOut[MAX_NUMBER_OF_RADARS];
                float ranOut[MAX_NUMBER_OF_RADARS];
                
                // variables for adjustation noise gain:
                unsigned averageNumPoints[NUM_OF_POINTS_IN_FRAME * 4];
                unsigned averageNumPointsIdx;
                unsigned averageNumPointsNumSamples;
                float noiseGain;                                        /**< Noise Gain in dB*/

                // injected data:
                bool injectedVelocityEnabled[MAX_NUMBER_OF_RADARS];
                float injectedVelocity[MAX_NUMBER_OF_RADARS];
                bool injectedRangeEnabled[MAX_NUMBER_OF_RADARS];
                float injectedRange[MAX_NUMBER_OF_RADARS];
                
#ifdef MSG_LOGGING
                LoggingConfig loggingConfig;

                // objects for logging data to files in another thread:
                AsyncFileWriter configMsgAw;
                AsyncFileWriter configAckAw;
                AsyncFileWriter controlMsgAw;
                AsyncFileWriter minRangeAw;
                AsyncFileWriter objectInfoAw;
                AsyncFileWriter filteredPointAw;
                AsyncFileWriter eapAw;

#endif // #ifdef MSG_LOGGING 
        };
}

#endif // #ifndef RADAR_SYSTEM_H
