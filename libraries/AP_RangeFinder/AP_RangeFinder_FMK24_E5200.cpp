/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_FMK24_E5200_ENABLED

#include "AP_RangeFinder_FMK24_E5200.h"
#include <GCS_MAVLink/GCS.h>

// get_reading - read a value from the sensor
bool AP_RangeFinder_FMK24_E5200::get_reading(float &reading_m)
{
    if (uart == nullptr || uart->available() == 0)
    {
        return false;
    }

    bool has_data = false;

    uint32_t available = uart->available();

    while (available > 0)
    {
        payload[payload_size] = uart->read();
        if (payload[payload_size] == '\n')
        {
            // ---------------
            // Sync up with the header
            // We use data about the nearest obstacle
            // The FMK24-E5200 can transmit the distance to several targets at the same time, but we use the closest one
            for (uint8_t i = 0; i < payload_size - 2; i++)
            {
                if ((payload[i] == '1') && (payload[i + 1] == ':'))
                {
                    // target found
                    i += 2;
                    uint8_t buffer[payload_size - i + 1];
                    payload[payload_size] = '\0'; // for correct atoi
                    memcpy(buffer, &payload[i], ARRAY_SIZE(buffer));
                    reading_m = atoi((char *)buffer) * 0.01;

                    has_data = true;

                    if (reading_m > max_distance_cm() * 0.01)
                    {
                        reading_m = max_distance_cm() * 0.01;
                        state.status = RangeFinder::Status::OutOfRangeHigh;
                        has_data = false;
                    }
                    else
                    {
                        state.status = RangeFinder::Status::Good;
                    }

                    payload_size = 0;
                    return has_data;
                }
            }
        }
        available--;
        payload_size++;
        if (payload_size == ARRAY_SIZE(payload))
        {
            payload_size = 0;
        }
    }

    state.status = RangeFinder::Status::NoData;
    return has_data;
}

#endif // AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
