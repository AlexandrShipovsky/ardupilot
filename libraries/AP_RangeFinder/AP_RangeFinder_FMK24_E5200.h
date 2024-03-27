#pragma once

#include "AP_RangeFinder_config.h"

/*
 questions:
 - bug on page 22 on malfunction codes
 - fixed length seems strange at 28 bytes
 - definition of snr field is missing in documentation
 - roll/pitch limits are in conflict, 3.2 vs 
*/
#if AP_RANGEFINDER_FMK24_E5200_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"
#include <GCS_MAVLink/GCS.h>

class AP_RangeFinder_FMK24_E5200 : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RangeFinder FMK24");
        return new AP_RangeFinder_FMK24_E5200(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 57600;
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get a reading
    bool get_reading(float &reading_m) override;
    uint8_t payload[128];
    uint8_t payload_size = 0;
};
#endif