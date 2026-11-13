#include "main.h"
#include "lemlib/api.hpp"
#include <thread>
#include <chrono>

const int MOTOR_LEFT_FRONT_PORT = 20;
const int MOTOR_LEFT_BACK_PORT = -10;
const int MOTOR_RIGHT_FRONT_PORT = -7;
const int MOTOR_RIGHT_BACK_PORT = 9;
const int MOTOR_CASCADE_LEFT_PORT = 1;
const int MOTOR_CASCADE_RIGHT_PORT = -2;
const double CASCADE_MAX_ROTATIONS = 10.0;
const double MIN_DRIVE_SCALE = 0.25;

// VEX Override Pin/Cup: 6.5 in height. 1.25x = 8.125 inches (~3.0 motor rotations)
const double OVERRIDE_PIN_CUP_HEIGHT_INCHES = 6.5;
const double CASCADE_INCHES_PER_ROTATION = 2.7;
const double CASCADE_UP_TARGET_ROTATIONS = (1.25 * OVERRIDE_PIN_CUP_HEIGHT_INCHES) / CASCADE_INCHES_PER_ROTATION; // ~3.0 rotations

const double END_EFFECTOR_GEAR_RATIO = 3.0;    // 3:1 ratio (sensor rotates 3x per 1x mechanism)
const double END_EFFECTOR_UP_TARGET = 75.0;     // Target raised position in degrees (adjust as desired: e.g. 70, 75, 90)
const double END_EFFECTOR_DOWN_TARGET = 0.0;   // Physical 0 deg dropped / rest position
const int MOTOR_INTAKE_PORT = 19;

const int END_EFFECTOR_ROT_PORT = 18;
const int END_EFFECTOR_ROT_SENSE_PORT = 13;
const int END_EFFECTOR_PORT = 14;

const int IMU_PORT = 12;

const int ODOM_X_PORT = 3;
const int ODOM_Y_PORT = 17;

bool rotating_macro = false;

pros::Motor left_front(MOTOR_LEFT_FRONT_PORT);
pros::Motor left_back(MOTOR_LEFT_BACK_PORT);
pros::Motor right_front(MOTOR_RIGHT_FRONT_PORT);
pros::Motor right_back(MOTOR_RIGHT_BACK_PORT);

pros::Motor cascade_left(MOTOR_CASCADE_LEFT_PORT);
pros::Motor cascade_right(MOTOR_CASCADE_RIGHT_PORT);

pros::Motor end_effector_rot(END_EFFECTOR_ROT_PORT);
pros::Rotation end_effector_rot_sense(END_EFFECTOR_ROT_SENSE_PORT);
pros::Motor intake(MOTOR_INTAKE_PORT);
pros::Motor end_effector(END_EFFECTOR_PORT);

pros::MotorGroup left_drive({MOTOR_LEFT_FRONT_PORT, MOTOR_LEFT_BACK_PORT});
pros::MotorGroup right_drive({MOTOR_RIGHT_FRONT_PORT, MOTOR_RIGHT_BACK_PORT});
pros::MotorGroup cascade({MOTOR_CASCADE_LEFT_PORT, MOTOR_CASCADE_RIGHT_PORT});
pros::Imu imu(IMU_PORT);

pros::Rotation odom_x(ODOM_X_PORT);
pros::Rotation odom_y(ODOM_Y_PORT);

lemlib::TrackingWheel horizontal_tracking(&odom_x, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel vertical_tracking(&odom_y, lemlib::Omniwheel::NEW_2, 0);

lemlib::OdomSensors sensors(
	&vertical_tracking,
	nullptr,
	&horizontal_tracking,
	nullptr,
	&imu
);

lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Drivetrain drivetrain(&left_drive,
							&right_drive,
							10,
							lemlib::Omniwheel::NEW_4,
							600,
							2
);

lemlib::Chassis chassis(drivetrain,
						lateral_controller,
						angular_controller,
						sensors
);


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	chassis.calibrate();
	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	chassis.setPose(0, 0, 0);
	chassis.moveToPose(0, -51, 0, 4000, {.forwards = false});
	chassis.waitUntilDone();
}
// x1: 
// y1: 
// r1: 