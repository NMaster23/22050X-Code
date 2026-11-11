#include "main.h"
#include <cmath>
#include <algorithm>
#include <cstdio>

// ============================================================================
// >>> EDIT YOUR PID VALUES AND TARGET DISTANCE HERE <<<
// ============================================================================

// PID GAINS
double kP = 8.5;               // Proportional: Push power. Increase if too slow, decrease if oscillating.
double kI = 0.0;               // Integral: Fixes small steady-state error (start at 0, usually 0.001 - 0.01).
double kD = 25.0;              // Derivative: Brake/damping. Increase to stop overshooting or bouncing.

// TARGET
double TARGET_INCHES = 24.0;   // How far forward to drive in inches (use negative to go backward)

// SETTLING / TIMEOUT
double ALLOWED_ERROR_INCHES = 0.5; // Stops when within this distance of target (in inches)
int TIMEOUT_MS = 3500;             // Maximum time in milliseconds before cutting power

// ============================================================================
// HARDWARE PORTS (from 22050X-Driver)
// ============================================================================
const int MOTOR_LEFT_FRONT = 20;
const int MOTOR_LEFT_BACK = -10;
const int MOTOR_RIGHT_FRONT = -7;
const int MOTOR_RIGHT_BACK = 9;
const int IMU_PORT = 3;

// Drivetrain geometry
const double WHEEL_DIAMETER = 4.0; // 4 inch omniwheels
const double PI = 3.14159265358979323846;
const double INCHES_PER_DEGREE = (PI * WHEEL_DIAMETER) / 360.0; // ~0.0349 inches per motor degree

// Hardware devices
pros::MotorGroup left_drive({MOTOR_LEFT_FRONT, MOTOR_LEFT_BACK}, pros::v5::MotorGears::blue);
pros::MotorGroup right_drive({MOTOR_RIGHT_FRONT, MOTOR_RIGHT_BACK}, pros::v5::MotorGears::blue);
pros::Imu imu(IMU_PORT);
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ============================================================================
// HELPER: Convert Motor Encoders to Linear Inches
// ============================================================================
double get_distance_inches() {
    double left_deg = left_drive.get_position();
    double right_deg = right_drive.get_position();
    double avg_deg = (left_deg + right_deg) * 0.5;
    return avg_deg * INCHES_PER_DEGREE;
}

// ============================================================================
// SIMPLE PID DRIVE FUNCTION
// ============================================================================
void drive_forward_pid(double target_dist) {
    printf("\n==================================================\n");
    printf(" DRIVING FORWARD PID: %.2f inches\n", target_dist);
    printf(" Gains: kP=%.3f, kI=%.3f, kD=%.3f\n", kP, kI, kD);
    printf("==================================================\n");

    // Reset encoders
    left_drive.tare_position();
    right_drive.tare_position();
    left_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    double start_heading = 0.0;
    bool has_imu = (imu.is_installed() && !imu.is_calibrating());
    if (has_imu) {
        start_heading = imu.get_rotation();
    }

    double error = target_dist;
    double prev_error = error;
    double integral = 0.0;

    uint32_t start_time = pros::millis();
    uint32_t prev_time = start_time;
    int settled_cycles = 0; // Number of consecutive cycles within allowed error

    while (true) {
        uint32_t current_time = pros::millis();
        double dt = (current_time - prev_time) / 1000.0;
        if (dt <= 0.0) dt = 0.01;
        prev_time = current_time;

        // Emergency abort if B button is pressed on controller
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            printf("Aborted by user!\n");
            break;
        }

        // Hard timeout
        if (current_time - start_time > static_cast<uint32_t>(TIMEOUT_MS)) {
            printf("Timed out after %d ms!\n", TIMEOUT_MS);
            break;
        }

        // Calculate current position & error
        double current_dist = get_distance_inches();
        error = target_dist - current_dist;

        // Proportional
        double p_out = kP * error;

        // Integral with anti-windup (only accumulate when close and not crossing zero)
        if (std::abs(error) < 3.0) {
            integral += error * dt;
        } else {
            integral = 0.0;
        }
        if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) {
            integral = 0.0; // Reset on zero crossing
        }
        double i_out = kI * integral;

        // Derivative
        double derivative = (error - prev_error) / dt;
        double d_out = kD * derivative;

        // Total forward voltage output (-127 to +127)
        double output = std::clamp(p_out + i_out + d_out, -127.0, 127.0);

        // IMU heading correction (keeps robot driving straight)
        double turn_correction = 0.0;
        if (has_imu) {
            double current_heading = imu.get_rotation();
            double heading_error = start_heading - current_heading;
            turn_correction = std::clamp(heading_error * 2.0, -30.0, 30.0);
        }

        // Send power to motors
        left_drive.move(output + turn_correction);
        right_drive.move(output - turn_correction);

        // Print live feedback to Brain LCD & Terminal
        uint32_t elapsed_ms = current_time - start_time;
        pros::lcd::print(0, "TARGET:  %.2f in", target_dist);
        pros::lcd::print(1, "CURRENT: %.2f in", current_dist);
        pros::lcd::print(2, "ERROR:   %.2f in", error);
        pros::lcd::print(3, "OUT:     %.1f | TIME: %lums", output, (unsigned long)elapsed_ms);
        pros::lcd::print(4, "GAINS: P=%.2f I=%.3f D=%.2f", kP, kI, kD);
        pros::lcd::print(5, "[B] = EMERGENCY STOP");

        printf("[PID] Time: %4lums | Target: %6.2f | Current: %6.2f | Error: %6.2f | Out: %6.1f\n",
               (unsigned long)elapsed_ms, target_dist, current_dist, error, output);

        // Check if settled within allowed error threshold
        if (std::abs(error) <= ALLOWED_ERROR_INCHES) {
            settled_cycles++;
            if (settled_cycles >= 10) { // Must stay inside tolerance for 100ms
                printf("Settled cleanly at %.2f in (Error: %.3f in)!\n", current_dist, error);
                break;
            }
        } else {
            settled_cycles = 0;
        }

        prev_error = error;
        pros::delay(10);
    }

    // Stop motors and hold position
    left_drive.brake();
    right_drive.brake();

    double final_dist = get_distance_inches();
    double final_err = target_dist - final_dist;
    pros::lcd::print(6, "DONE: Final Err = %.2f in", final_err);
    printf("==================================================\n");
    printf(" FINISHED: Final Dist: %.2f in | Final Err: %.2f in\n", final_dist, final_err);
    printf("==================================================\n\n");
    master.rumble(". - .");
}

// ============================================================================
// PROS LIFECYCLE
// ============================================================================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Initializing...");

    left_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    if (IMU_PORT > 0) {
        imu.reset();
        int attempts = 0;
        while (imu.is_calibrating() && attempts < 300) {
            pros::delay(10);
            attempts++;
        }
    }

    pros::lcd::set_text(0, "Ready! Starting PID move...");
    pros::delay(200);
}

void disabled() {}

void competition_initialize() {}

/**
 * Runs when started in autonomous mode
 */
void autonomous() {
    // Automatically drives forward using your PID values as soon as you run it!
    drive_forward_pid(TARGET_INCHES);
}

/**
 * Runs when started in driver / opcontrol mode
 */
void opcontrol() {
    // 1. Automatically drives forward using your PID values as soon as program starts!
    drive_forward_pid(TARGET_INCHES);

    // 2. Afterwards, you can press Button A to run the test again,
    //    or use joysticks to drive around!
    while (true) {
        // Press Button A on the controller to run the PID test again
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            drive_forward_pid(TARGET_INCHES);
        }

        // Manual arcade drive to reposition the robot between runs
        int left_y = master.get_analog(ANALOG_LEFT_Y);
        int right_x = master.get_analog(ANALOG_RIGHT_X);

        if (std::abs(left_y) > 5 || std::abs(right_x) > 5) {
            left_drive.move(left_y + right_x);
            right_drive.move(left_y - right_x);
        } else {
            left_drive.move(0);
            right_drive.move(0);
        }

        pros::delay(10);
    }
}
