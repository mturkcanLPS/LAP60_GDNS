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

#ifndef LOGGING_CONFIG_H
#define LOGGING_CONFIG_H

#ifdef MSG_LOGGING
namespace hrvs_sdk
{
    /**
     * @brief This class implements the setting of logging.
     * It dictates what data should be logged and the output of the logged data either .csv file or STDOUT.
     * @warning When specifying output directory, use forward slashes instead of backward slashes.
     */
    class LoggingConfig
    {
        /******************************   ENUMS   *******************************/
        public:
            /**
             * @brief Enumeration defining the possible output types of HRVS.
             */
            enum RadarOutputType
            {
                NONE_OUTPUT,
                CONFIGMSG_OUTPUT,
                CONFIGACK_OUTPUT,
                CONTROLMSG_OUTPUT,
                MINRANGE_OUTPUT,
                OBJECTINFO_OUTPUT,
                FILTEREDPOINTS_OUTPUT,
                EAP_OUTPUT,
            };

        /****************************   CONSTANTS   *****************************/
       
        public:
            // logging file names:
            static constexpr char CONFIG_MSG_FILENAME[] = "config_msg.csv";
            static constexpr char CONFIG_ACK_FILENAME[] = "config_ack.csv";
            static constexpr char CONTROL_MSG_FILENAME[] = "control_msg.csv";
            static constexpr char MINRANGE_FILENAME[] = "min_range.csv";
            static constexpr char OBJECT_INFO_FILENAME[] = "object_info.csv";
            static constexpr char FILTERED_POINTS_FILENAME[] = "filtered_points.csv";
            static constexpr char EAP_FILENAME[] = "eap.csv";

            // logging file csv headers:
            static constexpr char CONFIG_MSG_PROTO_1_0_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarID,rxChannelEn,txChannelEn,startFrequency,idleTime,rampEndTime,freqSlopeConst,txPowerOut,digOutSampleRate,framePeriodicity,frameTriggerDelay,Sat_primarySliceDuration,Sat_numSlices";
            static constexpr char CONFIG_MSG_PROTO_1_1_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarID,rxChannelEn,txChannelEn,startFrequency,idleTime,rampEndTime,freqSlopeConst,maxRange,maxVelocity,txPowerOut,digOutSampleRate,framePeriodicity,frameTriggerDelay,Sat_primarySliceDuration,Sat_numSlices,rangeFilterEnable,rangeFilterMin,rangeFilterMax,minRangeSetting";
            static constexpr char CONFIG_ACK_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarID,uniqueID_lotNo,uniqueID_waferNo,uniqueID_devX,uniqueID_devY,swVersionDMSS_maj,swVersionDMSS_min,swVersionDMSS_bld,swVersionDMSS_typ,hwVersionFE,hwVersionFE_0,hwVersionFE_1,hwVersionFE_2,hwVersionFE_3";
            static constexpr char CONTROL_MSG_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarID,radarframeNumber,ITOW,messageType,noiseGain,maxRange";
            static constexpr char MINRANGE_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarID,radarFrameNumber,ITOW,rangeRes,velRes,minRange,minRangeBigObj,VARrange,VARvelocity,snr,temp_RX0,temp_RX1,temp_RX2,temp_TX0,temp_TX1,temp_TX2,temp_PM,temp_DIG0,temp_DIG1";
            static constexpr char OBJECT_INFO_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarID,radarframeNumber,ITOW,pointsPerObj,peakPerObj,minRangePerObj,velPerObj,ranPerObj,minVelPerObj,maxVelPerObj";
            static constexpr char FILTERED_POINTS_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarID,radarframeNumber,ITOW,noiseThreshold,range,velocity,peak";
            static constexpr char EAP_CSV_HEADER[] = "systemFrameNumber,lastMeasTriggerTime,radarframeNumber,"
                                                     "numOfRadars,roll,pitch,yaw,"
                                                     "re_lvel_1,re_lvel_2,re_lvel_3,re_lvel_4,"
                                                     "re_rt_tht_1,re_rt_tht_2,re_rt_tht_3,re_rt_tht_4,"
                                                     "re_rt_psi_1,re_rt_psi_2,re_rt_psi_3,re_rt_psi_4,"
                                                     "re_lrange_1,re_lrange_2,re_lrange_3,re_lrange_4,"
                                                     "re_lpoint_1,re_lpoint_2,re_lpoint_3,re_lpoint_4,"
                                                     "re_pvel_1,re_pvel_2,re_pvel_3,re_pvel_4,"
                                                     "re_prange_1,re_prange_2,re_prange_3,re_prange_4,"
                                                     "re_ppoint_1,re_ppoint_2,re_ppoint_3,re_ppoint_4,"
                                                     "re_vel_body_1,re_vel_body_2,re_vel_body_3,re_vel_body_valid,"
                                                     "re_body_vel_x,re_body_vel_y,re_body_vel_z,"
                                                     "re_body_vel_status_1,re_body_vel_status_2,"
                                                     "re_max_range,re_valid_ratio,"
                                                     "re_vel_out_1,re_vel_out_2,re_vel_out_3,re_vel_out_4,"
                                                     "re_ran_out_1,re_ran_out_2,re_ran_out_3,re_ran_out_4,"
                                                     "re_points_out_1,re_points_out_2,re_points_out_3,re_points_out_4,"
                                                     "re_beam_out_1,re_beam_out_2,re_beam_out_3,re_beam_out_4,re_zero_tht_1,re_zero_tht_2,re_zero_tht_3,re_zero_tht_4,re_mask_in,re_mask_out";

            static constexpr unsigned MAX_DIR_NAME_SIZE = 1024;

        /*************************   CONFIG VARIABLES   *************************/

        public:
            char outputDirectory[MAX_DIR_NAME_SIZE] = ".";  /**<  Directory, where to create log files (default: files are created or rewritten in the working directory).*/
            bool configMsg = false;                         /**<  Whether config messsage should be logged in file (default: false). */
            bool configAck = false;                         /**<  Whether config acknowledgement messages from all radars should be logged in file (default: false). */
            bool controlMsg = false;                        /**<  Whether control messages should be logged in file (default: false). */
            bool minRange = false;                          /**<  Whether min range messsages should be logged in file  (default: false). */
            bool objectInfo = false;                        /**<  Whether object info messsages should be logged in file  (default: false). */
            bool filteredPoints = false;                    /**<  Whether filtered points messsages should be logged in file  (default: false). */
            bool eap = false;                               /**<  Whether external aiding process input and output parameters should be logged in file  (default: false). */

        RadarOutputType stdOutputType = RadarOutputType::NONE_OUTPUT; /**<  Which output type should be print on STDOUT (default: none).*/

        /**
         * @brief Modifies instance variables (logging flags) to log all possible data.
         */
        void logAllData()
        {
            configMsg = true;
            configAck = true;
            controlMsg = true;
            minRange = true;
            objectInfo = true;
            filteredPoints = true;
            eap = true;
        }
    };
}
#endif // #ifdef MSG_LOGGING

#endif // #ifndef LOGGING_CONFIG_H
