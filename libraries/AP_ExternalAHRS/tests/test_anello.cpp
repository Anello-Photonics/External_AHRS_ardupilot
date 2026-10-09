#include <AP_gtest.h>
#include <memory>
#include <AP_ExternalAHRS/AP_ExternalAHRS_ANELLO.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_EXTERNAL_AHRS_ANELLO_ENABLED

static AP_SerialManager serial_manager;
static GCS_Dummy dummy_gcs;

class AP_ExternalAHRS_ANELLO_Test : public ::testing::Test
{
protected:
    // ArduPilot allocation clears objects before their constructors run.
    std::unique_ptr<AP_ExternalAHRS> frontend_storage {new AP_ExternalAHRS()};
    AP_ExternalAHRS &frontend = *frontend_storage;
    AP_ExternalAHRS_ANELLO driver {&frontend, frontend.state};
    mavlink_status_t tx_status {};

    void feed(const mavlink_message_t &msg, bool corrupt = false)
    {
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
        const uint16_t length = mavlink_msg_to_send_buffer(bytes, &msg);
        if (corrupt) {
            bytes[length - 1] ^= 0x80;
        }
        for (uint16_t i = 0; i < length; i++) {
            driver.parse_byte(bytes[i]);
        }
    }

    void attitude(float roll = 0.2f, bool corrupt = false, float rollspeed = 0.01f)
    {
        mavlink_attitude_t packet {};
        packet.roll = roll;
        packet.pitch = -0.1f;
        packet.yaw = 0.4f;
        packet.rollspeed = rollspeed;
        mavlink_message_t msg;
        mavlink_msg_attitude_encode_status(42, 200, &tx_status, &msg, &packet);
        feed(msg, corrupt);
    }

    void quaternion(float w, float x, float y, float z)
    {
        mavlink_attitude_quaternion_t packet {};
        packet.q1 = w;
        packet.q2 = x;
        packet.q3 = y;
        packet.q4 = z;
        mavlink_message_t msg;
        mavlink_msg_attitude_quaternion_encode_status(42, 200, &tx_status, &msg, &packet);
        feed(msg);
    }

    void position(int32_t latitude = 374221234)
    {
        mavlink_global_position_int_t packet {};
        packet.lat = latitude;
        packet.lon = -1220845678;
        packet.alt = 123450;
        packet.relative_alt = 543210;
        packet.vx = 123;
        packet.vy = -456;
        packet.vz = 78;
        packet.hdg = 9000;
        mavlink_message_t msg;
        mavlink_msg_global_position_int_encode_status(42, 200, &tx_status, &msg, &packet);
        feed(msg);
    }

    void expire_attitude()
    {
        driver.last_attitude_ms = AP_HAL::millis() - driver.ATTITUDE_TIMEOUT_MS - 1;
        driver.last_quaternion_ms = driver.last_attitude_ms;
        driver.check_freshness(AP_HAL::millis());
    }

    void expire_position()
    {
        driver.last_position_ms = AP_HAL::millis() - driver.POSITION_TIMEOUT_MS - 1;
        driver.check_freshness(AP_HAL::millis());
    }
};

TEST_F(AP_ExternalAHRS_ANELLO_Test, Mavlink2ConversionsAndOrigin)
{
    EXPECT_FALSE(driver.initialised());
    EXPECT_FALSE(driver.healthy());
    position();
    EXPECT_FALSE(driver.initialised());
    EXPECT_FALSE(frontend.state.have_origin);
    attitude();
    EXPECT_TRUE(driver.initialised());
    EXPECT_TRUE(driver.healthy());
    EXPECT_EQ(frontend.state.location.lat, 374221234);
    EXPECT_EQ(frontend.state.location.lng, -1220845678);
    EXPECT_EQ(frontend.state.location.alt, 12345);
    EXPECT_EQ(frontend.state.location.get_alt_frame(), Location::AltFrame::ABSOLUTE);
    EXPECT_NEAR(frontend.state.velocity.x, 1.23f, 1e-5f);
    EXPECT_NEAR(frontend.state.velocity.y, -4.56f, 1e-5f);
    EXPECT_NEAR(frontend.state.velocity.z, 0.78f, 1e-5f);
    EXPECT_NEAR(frontend.state.gyro.x, 0.01f, 1e-5f);
    float roll, pitch, yaw;
    frontend.state.quat.to_euler(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.2f, 1e-5f);
    EXPECT_NEAR(pitch, -0.1f, 1e-5f);
    EXPECT_NEAR(yaw, 0.4f, 1e-5f);
    ASSERT_TRUE(frontend.state.have_origin);
    position(374222234);
    EXPECT_EQ(frontend.state.origin.lat, 374221234);
    EXPECT_EQ(driver.num_gps_sensors(), 0);
    EXPECT_FALSE(frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::IMU));
}

TEST_F(AP_ExternalAHRS_ANELLO_Test, Mavlink1)
{
    tx_status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    attitude();
    position();
    EXPECT_TRUE(driver.healthy());
}

TEST_F(AP_ExternalAHRS_ANELLO_Test, QuaternionPreferenceNormalizationAndFallback)
{
    quaternion(1.1f, 0, 0, 0);
    attitude();
    EXPECT_FLOAT_EQ(frontend.state.quat.q1, 1.0f);
    EXPECT_FLOAT_EQ(frontend.state.quat.q2, 0.0f);
    expire_attitude();
    EXPECT_FALSE(frontend.state.have_quaternion);
    attitude();
    EXPECT_TRUE(frontend.state.have_quaternion);
    EXPECT_GT(fabsf(frontend.state.quat.q2), 0.01f);
}

TEST_F(AP_ExternalAHRS_ANELLO_Test, QuaternionWxyzOrder)
{
    quaternion(cosf(0.4f), 0, 0, sinf(0.4f));
    float roll, pitch, yaw;
    frontend.state.quat.to_euler(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-5f);
    EXPECT_NEAR(pitch, 0.0f, 1e-5f);
    EXPECT_NEAR(yaw, 0.8f, 1e-5f);
}

TEST_F(AP_ExternalAHRS_ANELLO_Test, InvalidDataAndChecksumRecovery)
{
    quaternion(0, 0, 0, 0);
    quaternion(NAN, 0, 0, 0);
    quaternion(INFINITY, 0, 0, 0);
    quaternion(2, 0, 0, 0);
    attitude(NAN);
    attitude(0.2f, false, INFINITY);
    attitude(0.2f, false, NAN);
    attitude(0.2f, true);
    EXPECT_FALSE(frontend.state.have_quaternion);
    position(900000001);
    EXPECT_FALSE(frontend.state.have_location);
    attitude();
    position();
    EXPECT_TRUE(driver.healthy());
    expire_attitude();
    attitude(INFINITY);
    quaternion(0, 0, 0, 0);
    EXPECT_FALSE(driver.healthy());
}

TEST_F(AP_ExternalAHRS_ANELLO_Test, IndependentTimeoutsAndRecovery)
{
    attitude();
    position();
    nav_filter_status status;
    driver.get_filter_status(status);
    EXPECT_TRUE(status.flags.attitude);
    EXPECT_TRUE(status.flags.horiz_pos_abs);
    EXPECT_FALSE(status.flags.gps_quality_good);
    expire_attitude();
    EXPECT_FALSE(driver.healthy());
    EXPECT_TRUE(driver.initialised());
    EXPECT_FALSE(frontend.state.have_quaternion);
    EXPECT_TRUE(frontend.state.have_location);
    attitude();
    EXPECT_TRUE(driver.healthy());
    expire_position();
    EXPECT_FALSE(driver.healthy());
    EXPECT_TRUE(frontend.state.have_quaternion);
    EXPECT_FALSE(frontend.state.have_location);
    EXPECT_FALSE(frontend.state.have_velocity);
    driver.get_filter_status(status);
    EXPECT_TRUE(status.flags.attitude);
    EXPECT_FALSE(status.flags.horiz_pos_abs);
    EXPECT_FALSE(status.flags.horiz_vel);
    position();
    EXPECT_TRUE(driver.healthy());
}

#endif // AP_EXTERNAL_AHRS_ANELLO_ENABLED

AP_GTEST_MAIN()
