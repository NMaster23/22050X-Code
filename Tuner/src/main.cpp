#include "main.h"
#include "pid_tuner.hpp"

// Global PID Tuner and Drive Controller Instance
PIDTuner tuner;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * All other competition modes are blocked by initialize.
 */
void initialize() {
    tuner.initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code.
 *
 * HOW TO DRIVE THE BOT WITH PID:
 * You can call tuner.drive_distance() and tuner.turn_to_angle() directly!
 * These functions use your tuned PID constants and the IMU heading lock.
 */
void autonomous() {
    // -------------------------------------------------------------
    // EXAMPLE AUTONOMOUS PID DRIVING ROUTINE:
    // -------------------------------------------------------------
    
    // 1. Drive forward 24.0 inches with PID heading lock
    tuner.drive_distance(24.0, 3500);
    pros::delay(250);

    // 2. Turn in place 90.0 degrees clockwise with Angular PID
    tuner.turn_to_angle(90.0, 3000);
    pros::delay(250);

    // 3. Drive forward another 18.0 inches
    tuner.drive_distance(18.0, 3000);
    pros::delay(250);

    // 4. Reverse 12.0 inches backwards
    tuner.drive_distance(-12.0, 2500);
    pros::delay(250);

    // 5. Turn back to 0.0 degrees (original field orientation)
    tuner.turn_to_angle(0.0, 3000);

    // Or run a 4-corner box test routine:
    // tuner.run_box_test(24.0);
}

/**
 * Runs the operator control code.
 *
 * In tuning & testing mode, opcontrol runs the interactive loop:
 * - Button A: Runs the active PID step test (drives the target distance or turns to target angle).
 * - Hold R1 + Button A: Runs a forward 24" -> reverse 24" repeatability test!
 * - Hold R1 + Button Y: Runs a 4-corner box test!
 * - Button B: Emergency stops any running motion immediately.
 * - Joysticks: Manual arcade drive to reposition the bot when idle.
 */
void opcontrol() {
    tuner.run_interactive_loop();
}
