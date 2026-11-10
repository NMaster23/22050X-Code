#pragma once

#include "lemlib/api.hpp"
#include <cstdint>

namespace robot_config {

// ============================================================================
// 22050X HARDWARE PORTS & SUBSYSTEM SPECIFICATIONS
// Extracted from 22050X-Driver, 22050X-Autonomous, and PID-Tuner
// ============================================================================

// Drivetrain Motors (6:1 Blue Cartridge, 600 RPM, 4" Omniwheels)
constexpr int8_t MOTOR_LEFT_FRONT_PORT = 20;
constexpr int8_t MOTOR_LEFT_BACK_PORT = -10;
constexpr int8_t MOTOR_RIGHT_FRONT_PORT = -7;
constexpr int8_t MOTOR_RIGHT_BACK_PORT = 9;

// 2-Motor 6:1 Blue-Cartridge Cascade Lift
constexpr int8_t MOTOR_CASCADE_LEFT_PORT = 1;
constexpr int8_t MOTOR_CASCADE_RIGHT_PORT = -2;

// Intake & End-Effector / Claw Hardware Aligners
constexpr int8_t MOTOR_INTAKE_PORT = 19;
constexpr int8_t END_EFFECTOR_ROT_PORT = 18;
constexpr int8_t END_EFFECTOR_ROT_SENSE_PORT = 13;
constexpr int8_t END_EFFECTOR_PORT = 14;

// Odometry & Inertial Sensors
constexpr uint8_t IMU_PORT = 3;
constexpr uint8_t ODOM_X_PORT = 3;
constexpr uint8_t ODOM_Y_PORT = 0;

// Tracking Wheel Offsets & Sizes
constexpr float HORIZONTAL_TRACKING_WHEEL_DIAMETER = lemlib::Omniwheel::NEW_2;
constexpr float HORIZONTAL_TRACKING_OFFSET_IN = -5.75f;
constexpr float VERTICAL_TRACKING_WHEEL_DIAMETER = lemlib::Omniwheel::NEW_2;
constexpr float VERTICAL_TRACKING_OFFSET_IN = -2.50f;

// Drivetrain Geometry
constexpr float DRIVETRAIN_TRACK_WIDTH_IN = 10.0f;
constexpr float DRIVETRAIN_WHEEL_DIAMETER_IN = lemlib::Omniwheel::NEW_4;
constexpr float DRIVETRAIN_RPM = 600.0f;

// Cascade Lift Kinematics (6:1 Blue Cartridge, 32.14 deg motor rotation per inch of lift travel)
constexpr float CASCADE_DEG_PER_INCH = 32.142857f;
constexpr float CASCADE_MAX_HEIGHT_IN = 28.0f;

// Watchdog Safety Thresholds
constexpr float WATCHDOG_STALL_CURRENT_MA = 2350.0f;
constexpr float WATCHDOG_STALL_VEL_RPM = 5.0f;
constexpr uint32_t WATCHDOG_STALL_TIME_MS = 400;
constexpr uint32_t WATCHDOG_HEARTBEAT_TIMEOUT_MS = 150;
constexpr float PROS_THERMAL_LIMIT_C = 55.0f;

// ============================================================================
// [BEGIN_TUNED_GAINS]
// Automatically patched by vision_tuner (--mode full-suite)
// ============================================================================

// LemLib Drivetrain Horizontal Drift
constexpr float TUNED_HORIZONTAL_DRIFT = 2.0f;
constexpr float TUNED_BOOMERANG_LEAD = 0.6f;

// LemLib Lateral Controller Settings (moveToPoint / moveToPose lateral)
constexpr float LATERAL_KP = 10.0f;
constexpr float LATERAL_KI = 0.0f;
constexpr float LATERAL_KD = 30.0f;
constexpr float LATERAL_WINDUP_RANGE = 3.0f;
constexpr float LATERAL_SMALL_ERROR = 1.0f;
constexpr float LATERAL_SMALL_ERROR_TIMEOUT = 100.0f;
constexpr float LATERAL_LARGE_ERROR = 3.0f;
constexpr float LATERAL_LARGE_ERROR_TIMEOUT = 500.0f;
constexpr float LATERAL_SLEW = 20.0f;

// LemLib Angular Controller Settings (turnToHeading / turnToPoint)
constexpr float ANGULAR_KP = 2.0f;
constexpr float ANGULAR_KI = 0.0f;
constexpr float ANGULAR_KD = 10.0f;
constexpr float ANGULAR_WINDUP_RANGE = 3.0f;
constexpr float ANGULAR_SMALL_ERROR = 1.0f;
constexpr float ANGULAR_SMALL_ERROR_TIMEOUT = 100.0f;
constexpr float ANGULAR_LARGE_ERROR = 3.0f;
constexpr float ANGULAR_LARGE_ERROR_TIMEOUT = 500.0f;
constexpr float ANGULAR_SLEW = 0.0f;

// LemLib Swing Controller Settings (swingToHeading / swingToPoint)
constexpr float SWING_KP = 2.4f;
constexpr float SWING_KI = 0.0f;
constexpr float SWING_KD = 12.0f;
constexpr float SWING_WINDUP_RANGE = 3.0f;
constexpr float SWING_SMALL_ERROR = 1.0f;
constexpr float SWING_SMALL_ERROR_TIMEOUT = 100.0f;
constexpr float SWING_LARGE_ERROR = 3.0f;
constexpr float SWING_LARGE_ERROR_TIMEOUT = 500.0f;
constexpr float SWING_SLEW = 0.0f;

// Cascade Lift lemlib::PID Settings (2-Motor 6:1 Blue Cartridge)
constexpr float CASCADE_KP = 14.5f;
constexpr float CASCADE_KI = 0.08f;
constexpr float CASCADE_KD = 42.0f;
constexpr float CASCADE_WINDUP_RANGE = 2.5f;
constexpr bool CASCADE_SIGN_FLIP_RESET = true;
constexpr float CASCADE_GRAVITY_FF_MV = 1150.0f;

// ============================================================================
// [END_TUNED_GAINS]
// ============================================================================

inline lemlib::ControllerSettings makeLateralSettings() {
    return lemlib::ControllerSettings(
        LATERAL_KP,
        LATERAL_KI,
        LATERAL_KD,
        LATERAL_WINDUP_RANGE,
        LATERAL_SMALL_ERROR,
        LATERAL_SMALL_ERROR_TIMEOUT,
        LATERAL_LARGE_ERROR,
        LATERAL_LARGE_ERROR_TIMEOUT,
        LATERAL_SLEW
    );
}

inline lemlib::ControllerSettings makeAngularSettings() {
    return lemlib::ControllerSettings(
        ANGULAR_KP,
        ANGULAR_KI,
        ANGULAR_KD,
        ANGULAR_WINDUP_RANGE,
        ANGULAR_SMALL_ERROR,
        ANGULAR_SMALL_ERROR_TIMEOUT,
        ANGULAR_LARGE_ERROR,
        ANGULAR_LARGE_ERROR_TIMEOUT,
        ANGULAR_SLEW
    );
}

inline lemlib::ControllerSettings makeSwingSettings() {
    return lemlib::ControllerSettings(
        SWING_KP,
        SWING_KI,
        SWING_KD,
        SWING_WINDUP_RANGE,
        SWING_SMALL_ERROR,
        SWING_SMALL_ERROR_TIMEOUT,
        SWING_LARGE_ERROR,
        SWING_LARGE_ERROR_TIMEOUT,
        SWING_SLEW
    );
}

} // namespace robot_config
