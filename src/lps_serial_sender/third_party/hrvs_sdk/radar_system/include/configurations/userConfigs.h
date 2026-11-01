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

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#include <cstdint>

namespace hrvs_sdk
{
    /**
     * @brief All radar types supported by SDK.
     */
    enum RadarType
    {
        RADAR_60GHZ_PROTO_1_0_CONFIG, /**< Type of 60 GHz radar using protocol 1.0. */
        RADAR_60GHZ_PROTO_1_1_CONFIG, /**< Type of 60 GHz radar using protocol 1.1. */
        RADAR_80GHZ_PROTO_1_0_CONFIG  /**< Type of 80 GHz radar using protocol 1.0. */
    };

    /**
     * @name Default values for user configuration parameters.
     */
    //@{
    static constexpr unsigned DEFAULT_RADARS_NUMBER = 4;                                     /**< Default number of radars. */
    static constexpr float DEFAULT_ROLL_OFF = 0.0;                                           /**< Default roll off in degrees. */
    static constexpr float DEFAULT_PITCH_OFF = 0.0;                                          /**< Default pitch off in degrees. */
    static constexpr float DEFAULT_YAW_OFF = 0.0;                                            /**< Default yaw off in degrees. */
    static constexpr float DEFAULT_SLOPE_RADAR_1 = 15.0;                                     /**< Default slope of radar 1 in degrees. */
    static constexpr float DEFAULT_SLOPE_RADAR_2 = 15.0;                                     /**< Default slope of radar 2 in degrees. */
    static constexpr float DEFAULT_SLOPE_RADAR_3 = 15.0;                                     /**< Default slope of radar 3 in degrees. */
    static constexpr float DEFAULT_SLOPE_RADAR_4 = 15.0;                                     /**< Default slope of radar 4 in degrees. */
    static constexpr float DEFAULT_PSI_OFFSET_RADAR_1 = 0.0;                                 /**< Default psi offset of radar 1 in degrees. */
    static constexpr float DEFAULT_PSI_OFFSET_RADAR_2 = 0.0;                                 /**< Default psi offset of radar 2 in degrees. */
    static constexpr float DEFAULT_PSI_OFFSET_RADAR_3 = 0.0;                                 /**< Default psi offset of radar 3 in degrees. */
    static constexpr float DEFAULT_PSI_OFFSET_RADAR_4 = 0.0;                                 /**< Default psi offset of radar 4 in degrees. */
    static constexpr RadarType DEFAULT_RADAR_TYPE = RadarType::RADAR_60GHZ_PROTO_1_1_CONFIG; /**< Default radar type. */
    static constexpr unsigned DEFAULT_FRAME_TRIGGER_DELAY = 1;                               /**< Default frame trigger delay. */
    static constexpr bool DEFAULT_RANGE_FILTER_ENABLE = false;                               /**< Default range filter enable. */
    static constexpr float DEFAULT_RANGE_FILTER_MIN = 0.0;                                   /**< Default range filter minimum. */
    static constexpr float DEFAULT_RANGE_FILTER_MAX = 0.0;                                   /**< Default range filter maximum. */
    //@}

    /**
     * @brief This class implements the configuration parameters, 
     * that are applied in the processing of the measurement data and computation of 3D body velocity.
     */
    class ProcessingConfig
    {
    public:
        unsigned numberOfRadars = DEFAULT_RADARS_NUMBER;    /**< Number of Radars in HRVS */
        float rollOff = DEFAULT_ROLL_OFF;                   /**< HRVS Roll Off in degrees */
        float pitchOff = DEFAULT_PITCH_OFF;                 /**< HRVS Pitch Off in degrees */
        float yawOff = DEFAULT_YAW_OFF;                     /**< HRVS Yaw Off in degrees */
        float slopeRadar1 = DEFAULT_SLOPE_RADAR_1;          /**< Slope of radar 1 in degrees */
        float slopeRadar2 = DEFAULT_SLOPE_RADAR_2;          /**< Slope of radar 2 in degrees */
        float slopeRadar3 = DEFAULT_SLOPE_RADAR_3;          /**< Slope of radar 3 in degrees */
        float slopeRadar4 = DEFAULT_SLOPE_RADAR_4;          /**< Slope of radar 4 in degrees */
        float psiOffsetRadar1 = DEFAULT_PSI_OFFSET_RADAR_1; /**< Psi Offset of radar 1 in degrees */
        float psiOffsetRadar2 = DEFAULT_PSI_OFFSET_RADAR_2; /**< Psi Offset of radar 2 in degrees */
        float psiOffsetRadar3 = DEFAULT_PSI_OFFSET_RADAR_3; /**< Psi Offset of radar 3 in degrees */
        float psiOffsetRadar4 = DEFAULT_PSI_OFFSET_RADAR_4; /**< Psi Offset of radar 4 in degrees */
    };

    /**
     * @brief This class holds radar configuration parameters, which can be set by the user.
     */
    class RadarConfig
    {
    public:
        RadarType radarType = DEFAULT_RADAR_TYPE;                      /**< Type of radar (used to determine the protocol version). */
        uint16_t frameTriggerDelay = DEFAULT_FRAME_TRIGGER_DELAY;      /**< Frame Trigger Delay in microseconds.
                                                                            radar #1 is configured with: framTriggerDelay * 1,
                                                                            radar #2 is configured with: framTriggerDelay * 2,
                                                                            radar #3 is configured with: framTriggerDelay * 3,
                                                                            radar #4 is configured with: framTriggerDelay * 4 */
        bool rangeFilterEnable = DEFAULT_RANGE_FILTER_ENABLE;          /**< Enables filtering of measurements by measured range.
                                                                            Significantly improves performance of EAP model in ground applications.
                                                                            Value of parameter ignored in protocol 1.0. */
        float rangeFilterMin = DEFAULT_RANGE_FILTER_MIN;               /**< Minimal range of measurement in meters. Values below this range are filtered out.
                                                                            Parameter rangeFilterEnable must be true for this parameter to be used.
                                                                            Value of parameter ignored in protocol 1.0. */
        float rangeFilterMax = DEFAULT_RANGE_FILTER_MAX;               /**< Maximal range of measurement in meters. Values above this range are filtered out.
                                                                            Parameter rangeFilterEnable must be true for this parameter to be used.
                                                                            Value of parameter ignored in protocol 1.0. */
    };
}

#endif
