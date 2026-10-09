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
#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ANELLO_ENABLED

#include "AP_ExternalAHRS_ANELLO.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_ANELLO::AP_ExternalAHRS_ANELLO(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    // These messages contain fused estimates, not raw GPS/IMU/baro/mag data.
    set_default_sensors(0);
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ANELLO ExternalAHRS: no UART");
        return;
    }
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    uart->begin(sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0));
}

void AP_ExternalAHRS_ANELLO::update()
{
    if (uart != nullptr) {
        // Bound work even if a sender continuously fills the serial buffer.
        const uint32_t available = MIN(uart->available(), 4096U);
        for (uint32_t i = 0; i < available; i++) {
            uint8_t byte;
            if (!uart->read(byte)) {
                break;
            }
            parse_byte(byte);
        }
    }
    WITH_SEMAPHORE(state.sem);
    check_freshness(AP_HAL::millis());
}

void AP_ExternalAHRS_ANELLO::parse_byte(uint8_t byte)
{
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_frame_char_buffer(&rx_message, &rx_status, byte, &msg, &status) == MAVLINK_FRAMING_OK) {
        handle_message(msg);
    }
}

void AP_ExternalAHRS_ANELLO::handle_message(const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(state.sem);
    const uint32_t now_ms = AP_HAL::millis();

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(&msg, &packet);
        if (!isfinite(packet.roll) || !isfinite(packet.pitch) || !isfinite(packet.yaw)) {
            return;
        }
        // Prefer quaternion data while that stream is fresh.
        if (have_quaternion_message && now_ms - last_quaternion_ms <= ATTITUDE_TIMEOUT_MS) {
            return;
        }
        Quaternion quat;
        quat.from_euler(packet.roll, packet.pitch, packet.yaw);
        publish_attitude(quat, Vector3f(packet.rollspeed, packet.pitchspeed, packet.yawspeed), now_ms);
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
        mavlink_attitude_quaternion_t packet;
        mavlink_msg_attitude_quaternion_decode(&msg, &packet);
        Quaternion quat(packet.q1, packet.q2, packet.q3, packet.q4);
        const float norm = quat.length();
        // Allow rounding error, but reject zero, non-finite and malformed rotations.
        if (!isfinite(norm) || norm < 0.5f || norm > 1.5f ||
            !isfinite(packet.rollspeed) || !isfinite(packet.pitchspeed) || !isfinite(packet.yawspeed)) {
            return;
        }
        quat.normalize();
        publish_attitude(quat, Vector3f(packet.rollspeed, packet.pitchspeed, packet.yawspeed), now_ms);
        last_quaternion_ms = now_ms;
        have_quaternion_message = true;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        if (packet.lat < -900000000 || packet.lat > 900000000 ||
            packet.lon < -1800000000 || packet.lon > 1800000000) {
            return;
        }
        const Location location(packet.lat, packet.lon, packet.alt / 10, Location::AltFrame::ABSOLUTE);
        if (!location.initialised()) {
            return;
        }
        state.location = location;
        state.velocity = Vector3f(packet.vx, packet.vy, packet.vz) * 0.01f;
        state.have_location = true;
        state.have_velocity = true;
        state.last_location_update_us = AP_HAL::micros();
        last_position_ms = now_ms;
        have_position = true;
        break;
    }
    default:
        return;
    }

    if (!state.have_origin && state.have_quaternion && state.have_location) {
        state.origin = state.location;
        state.have_origin = true;
    }
}

void AP_ExternalAHRS_ANELLO::publish_attitude(const Quaternion &quat, const Vector3f &gyro, uint32_t now_ms)
{
    if (gyro.is_nan() || gyro.is_inf()) {
        return;
    }
    state.quat = quat;
    state.gyro = gyro;
    state.have_quaternion = true;
    have_attitude = true;
    last_attitude_ms = now_ms;
}

void AP_ExternalAHRS_ANELLO::check_freshness(uint32_t now_ms)
{
    state.have_quaternion = have_attitude && now_ms - last_attitude_ms <= ATTITUDE_TIMEOUT_MS;
    state.have_location = have_position && now_ms - last_position_ms <= POSITION_TIMEOUT_MS;
    state.have_velocity = state.have_location;
}

bool AP_ExternalAHRS_ANELLO::initialised() const
{
    WITH_SEMAPHORE(state.sem);
    return have_attitude && have_position;
}

bool AP_ExternalAHRS_ANELLO::healthy() const
{
    WITH_SEMAPHORE(state.sem);
    const uint32_t now_ms = AP_HAL::millis();
    return have_attitude && have_position &&
           now_ms - last_attitude_ms <= ATTITUDE_TIMEOUT_MS &&
           now_ms - last_position_ms <= POSITION_TIMEOUT_MS;
}

bool AP_ExternalAHRS_ANELLO::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    const char *reason = nullptr;
    if (uart == nullptr) {
        reason = "no UART";
    } else if (!initialised()) {
        reason = "waiting for attitude and position";
    } else if (!healthy()) {
        reason = "attitude or position is stale";
    } else if (AP::externalAHRS().has_sensor(AP_ExternalAHRS::AvailableSensor::IMU)) {
        reason = "set EAHRS_SENSORS=0 (no acceleration input)";
    }
    if (reason != nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "ANELLO ExternalAHRS: %s", reason);
        return false;
    }
    return true;
}

void AP_ExternalAHRS_ANELLO::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    status = {};
    status.flags.initalized = have_attitude && have_position;
    const uint32_t now_ms = AP_HAL::millis();
    status.flags.attitude = have_attitude && now_ms - last_attitude_ms <= ATTITUDE_TIMEOUT_MS;
    const bool position_valid = have_position && now_ms - last_position_ms <= POSITION_TIMEOUT_MS;
    status.flags.horiz_vel = position_valid;
    status.flags.vert_vel = position_valid;
    status.flags.horiz_pos_abs = position_valid;
    status.flags.vert_pos = position_valid;
    status.flags.pred_horiz_pos_abs = position_valid;
    status.flags.horiz_pos_rel = position_valid && state.have_origin;
    status.flags.pred_horiz_pos_rel = status.flags.horiz_pos_rel;
    // The selected messages do not report GNSS fix quality or filter variances.
}

#endif // AP_EXTERNAL_AHRS_ANELLO_ENABLED
