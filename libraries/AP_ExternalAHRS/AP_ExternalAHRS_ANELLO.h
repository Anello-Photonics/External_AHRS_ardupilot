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
/*
  ANELLO fused navigation over a dedicated MAVLink 1/2 serial stream.

  Setup (reboot after changing the device or serial protocol):
    AHRS_EKF_TYPE = 11
    EAHRS_TYPE = 12
    EAHRS_SENSORS = 0
    SERIALx_PROTOCOL = 36
    SERIALx_BAUD = the ANELLO output baud rate

  Configure ANELLO to stream ATTITUDE and/or ATTITUDE_QUATERNION at 50 Hz
  or faster, and GLOBAL_POSITION_INT at 5 Hz or faster. EAHRS_RATE does
  not configure the sender. Quaternions take priority while fresh (250 ms);
  Euler attitude is the fallback. Position expires after 1000 ms.

  Input axes must match the vehicle body frame (X forward, Y right,
  Z down), with attitude referenced to NED. No mounting rotation is applied.
  GLOBAL_POSITION_INT supplies MSL altitude, not relative_alt, and NED
  velocity (including positive-down vz). repr_offset_q is display-only.

  The port must carry only the ANELLO solution; messages are accepted from
  any system/component ID. This is a receive-only driver, with its own
  MAVLink parser and no GCS channel. Onboard sensors remain in use; these
  messages supply no acceleration, raw GPS fix, compass or pressure.
  Body rates are recorded in the external state but are not injected as
  raw IMU samples. Do not select ExternalAHRS as a GPS sensor type.

  Health indicates valid recent attitude and position messages only.
  The sender must suppress invalid/unconverged navigation solutions:
  these messages cannot communicate alignment, fix quality or variances.
 */
#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ANELLO_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_ANELLO : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_ANELLO(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state);

    int8_t get_port() const override
    {
        return uart == nullptr ? -1 : port_num;
    }
    const char *get_name() const override
    {
        return "ANELLO";
    }
    bool healthy() const override;
    bool initialised() const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void update() override;
    uint8_t num_gps_sensors() const override
    {
        return 0;
    }

private:
    friend class AP_ExternalAHRS_ANELLO_Test;

    static constexpr uint32_t ATTITUDE_TIMEOUT_MS = 250;
    static constexpr uint32_t POSITION_TIMEOUT_MS = 1000;

    void parse_byte(uint8_t byte);
    void handle_message(const mavlink_message_t &msg);
    void publish_attitude(const Quaternion &quat, const Vector3f &gyro, uint32_t now_ms);
    void check_freshness(uint32_t now_ms);

    AP_HAL::UARTDriver *uart;
    int8_t port_num = -1;
    // Private parser storage avoids consuming or interfering with a GCS channel.
    mavlink_message_t rx_message {};
    mavlink_status_t rx_status {};

    // Protected by state.sem, along with the frontend navigation state.
    bool have_attitude = false;
    bool have_position = false;
    bool have_quaternion_message = false;
    uint32_t last_attitude_ms = 0;
    uint32_t last_position_ms = 0;
    uint32_t last_quaternion_ms = 0;
};

#endif // AP_EXTERNAL_AHRS_ANELLO_ENABLED
