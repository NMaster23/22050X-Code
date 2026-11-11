#pragma once

#include "main.h"
#include "pid.hpp"
#include <vector>

namespace tuner_config {

// ============================================================================
// 1. HARDWARE PORT CONFIGURATION (Imported directly from 22050X-Driver)
// ============================================================================

// Drivetrain Motor Ports (negative denotes reversed motor)
constexpr int8_t MOTOR_LEFT_FRONT_PORT = 20;
constexpr int8_t MOTOR_LEFT_BACK_PORT = -10;
constexpr int8_t MOTOR_RIGHT_FRONT_PORT = -7;
constexpr int8_t MOTOR_RIGHT_BACK_PORT = 9;

const std::vector<int8_t> LEFT_DRIVE_PORTS = {MOTOR_LEFT_FRONT_PORT, MOTOR_LEFT_BACK_PORT};     // {20, -10}
const std::vector<int8_t> RIGHT_DRIVE_PORTS = {MOTOR_RIGHT_FRONT_PORT, MOTOR_RIGHT_BACK_PORT};  // {-7, 9}

// Cascade Lift Motor Ports
constexpr int8_t MOTOR_CASCADE_LEFT_PORT = 1;
constexpr int8_t MOTOR_CASCADE_RIGHT_PORT = -2;
const std::vector<int8_t> CASCADE_PORTS = {MOTOR_CASCADE_LEFT_PORT, MOTOR_CASCADE_RIGHT_PORT};   // {1, -2}

// Intake & End-Effector Ports
constexpr int8_t MOTOR_INTAKE_PORT = 19;
constexpr int8_t END_EFFECTOR_ROT_PORT = 18;
constexpr int8_t END_EFFECTOR_ROT_SENSE_PORT = 13;
constexpr int8_t END_EFFECTOR_PORT = 14;

// Sensors
constexpr uint8_t IMU_PORT = 3;
constexpr uint8_t ODOM_X_PORT = 3;
constexpr uint8_t ODOM_Y_PORT = 0;

// Motor Cartridges
constexpr pros::v5::MotorGears DRIVETRAIN_GEARSET = pros::v5::MotorGears::blue; // 600 RPM
constexpr pros::v5::MotorGears CASCADE_GEARSET = pros::v5::MotorGears::blue;    // 600 RPM
constexpr pros::v5::MotorGears MECHANISM_GEARSET = pros::v5::MotorGears::blue;

// Default mechanism to tune (Cascade lift on port 1)
constexpr int8_t MECHANISM_PORT = MOTOR_CASCADE_LEFT_PORT;


// ============================================================================
// 2. DRIVETRAIN DIMENSIONS & KINEMATICS (From 22050X-Driver)
// ============================================================================

// Wheel diameter in inches (4.0" Omniwheels: lemlib::Omniwheel::NEW_4)
constexpr double WHEEL_DIAMETER_INCHES = 4.0;

// Gear ratio (motor teeth / wheel teeth; 1.0 for direct drive)
constexpr double GEAR_RATIO = 1.0;

// Track width in inches (center-to-center distance between wheel tracks)
constexpr double TRACK_WIDTH_INCHES = 10.0;

// Math constants
constexpr double PI = 3.14159265358979323846;

/**
 * @brief Converts average drivetrain motor degrees into linear travel inches
 */
inline double motor_deg_to_inches(double motor_deg) {
    double wheel_revs = (motor_deg / 360.0) * GEAR_RATIO;
    return wheel_revs * (PI * WHEEL_DIAMETER_INCHES);
}

/**
 * @brief Converts linear travel inches into required motor degrees
 */
inline double inches_to_motor_deg(double inches) {
    double wheel_revs = inches / (PI * WHEEL_DIAMETER_INCHES);
    return (wheel_revs / GEAR_RATIO) * 360.0;
}


// ============================================================================
// 3. DEFAULT PID GAINS & SETTLE SETTINGS
// (Adjustable live from controller, then persist here)
// ============================================================================

// --- Profile 1: Linear Drivetrain (Drive distance in inches) ---
inline PIDGains get_default_drive_linear_gains() {
    PIDGains g;
    g.kp = 8.5;             // Proportional gain (voltage output per inch of error)
    g.ki = 0.0;             // Integral gain
    g.kd = 25.0;            // Derivative gain (damping / overshoot reduction)
    g.windup_range = 3.0;   // Accumulate integral only within 3 inches
    g.sign_flip_reset = true;
    g.slew_rate = 15.0;     // Max slew rate per tick (limits wheel slip & jerk)
    g.min_output = -127.0;  // PROS motor voltage limit (-127 to +127)
    g.max_output = 127.0;
    return g;
}

inline SettleSettings get_default_drive_linear_settle() {
    SettleSettings s;
    s.small_error = 0.75;             // Settled if within 0.75 inches...
    s.small_error_timeout_ms = 150;    // ...for at least 150 ms
    s.large_error = 2.0;              // Settled if within 2.0 inches...
    s.large_error_timeout_ms = 400;   // ...for at least 400 ms
    s.max_time_ms = 3500;             // Hard timeout 3.5 seconds
    return s;
}

constexpr double DEFAULT_DRIVE_LINEAR_STEP = 24.0; // Default step distance in inches


// --- Profile 2: Angular Drivetrain (Turn heading in degrees) ---
inline PIDGains get_default_drive_angular_gains() {
    PIDGains g;
    g.kp = 1.8;             // Proportional gain (output per degree of heading error)
    g.ki = 0.0;
    g.kd = 12.0;            // Derivative gain (prevents oscillation/spinning out)
    g.windup_range = 5.0;   // Integral active within 5 degrees
    g.sign_flip_reset = true;
    g.slew_rate = 0.0;      // 0 = disabled for quick turning response
    g.min_output = -127.0;
    g.max_output = 127.0;
    return g;
}

inline SettleSettings get_default_drive_angular_settle() {
    SettleSettings s;
    s.small_error = 1.0;              // Settled if within 1.0 degree...
    s.small_error_timeout_ms = 150;    // ...for at least 150 ms
    s.large_error = 3.0;              // Settled if within 3.0 degrees...
    s.large_error_timeout_ms = 400;   // ...for at least 400 ms
    s.max_time_ms = 3000;             // Hard timeout 3.0 seconds
    return s;
}

constexpr double DEFAULT_DRIVE_ANGULAR_STEP = 90.0; // Default step angle in degrees


// --- Profile 3: Single Motor / Mechanism (e.g. Cascade Lift in motor degrees) ---
inline PIDGains get_default_mechanism_gains() {
    PIDGains g;
    g.kp = 0.45;
    g.ki = 0.005;
    g.kd = 2.0;
    g.windup_range = 25.0;  // Integral active within 25 degrees
    g.sign_flip_reset = true;
    g.slew_rate = 0.0;
    g.min_output = -127.0;
    g.max_output = 127.0;
    return g;
}

inline SettleSettings get_default_mechanism_settle() {
    SettleSettings s;
    s.small_error = 5.0;              // Settled if within 5 degrees
    s.small_error_timeout_ms = 150;
    s.large_error = 15.0;
    s.large_error_timeout_ms = 350;
    s.max_time_ms = 3000;
    return s;
}

constexpr double DEFAULT_MECHANISM_STEP = 360.0; // Default step in motor degrees

} // namespace tuner_config
