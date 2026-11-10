#include "main.h"
#include "pid_tuner.hpp"

// Global PID Tuner Instance
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
 * the VEX Competition Switch, following either autonomous or opcontrol.
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
 * This example shows how to use your tuned PID controllers in autonomous:
 * you can execute your tuned step tests or call the PID loops directly.
 */
void autonomous() {
    // Example: Run a tuned step test in autonomous
    tuner.execute_step_test();
}

/**
 * Runs the operator control code.
 *
 * In tuning mode, opcontrol runs the interactive PID tuner loop.
 * You can adjust Kp, Ki, Kd and step targets in real time from the V5 Controller,
 * trigger step tests with Button A, view live feedback on the Brain & Controller,
 * and manually reposition the robot with the joysticks when idle!
 */
void opcontrol() {
    tuner.run_interactive_loop();
}
