#include "main.h"
#include "approval_gate.hpp"
#include <map>
#include <cmath>
#include <algorithm>
#include "robot_fsm.hpp"
#include "field_map.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp"
#include <chrono>
#include <thread>

// ============================================================================
// ALLIANCE & LOCALIZATION STATE
// ============================================================================

enum class AllianceColor {
	Red,
	Blue,
	None,
};

struct RobotPos {
	double x;
	double y;
};

AllianceColor sel_alliance_color = AllianceColor::None;

RobotPos position = {0.0, 0.0};
pros::Mutex pos_mutex;

const int WHEEL_R = 1.0;

// ============================================================================
// HARDWARE PORT CONFIGURATION
// Supports both primary and secondary port configurations
// ============================================================================

const int MOTOR_LEFT_FRONT_PORT = -20;
const int MOTOR_LEFT_BACK_PORT = 10;
const int MOTOR_RIGHT_FRONT_PORT = 7;
const int MOTOR_RIGHT_BACK_PORT = -9;

const int MOTOR_CASCADE_LEFT_PORT = 1;
const int MOTOR_CASCADE_RIGHT_PORT = -2;
const int MOTOR_INTAKE_PORT = 19;

const int END_EFFECTOR_ROT_PORT = 18;
const int END_EFFECTOR_ROT_SENSE_PORT = 13;
const int END_EFFECTOR_PORT = 14;

// Default sensor ports (auto-detected if on alternate port)
const int IMU_PORT = 12;
const int IMU_ALT_PORT = 3;

const int ODOM_X_PORT = 3;
const int ODOM_Y_PORT = -5;

const int OPTICAL_SENSOR_PORT = 11;
const int AI_VISION_PORT = 8;
const int GPS_PORT = 6;

// ============================================================================
// SUBSYSTEM CONSTANTS & KINEMATICS
// ============================================================================

const double CASCADE_MAX_ROTATIONS = 10.0;
const double OVERRIDE_PIN_CUP_HEIGHT_INCHES = 6.5;
const double CASCADE_INCHES_PER_ROTATION = 2.7;
const double CASCADE_UP_TARGET_ROTATIONS = (1.25 * OVERRIDE_PIN_CUP_HEIGHT_INCHES) / CASCADE_INCHES_PER_ROTATION; // ~3.0 rotations
const double CASCADE_DOWN_TARGET_ROTATIONS = 0.0;

const double END_EFFECTOR_GEAR_RATIO = 3.0;    // 3:1 ratio (sensor rotates 3x per 1x mechanism)
const double END_EFFECTOR_UP_TARGET = 75.0;     // Raised scoring angle in degrees
const double END_EFFECTOR_DOWN_TARGET = 0.0;   // Physical 0 deg dropped / intake rest position

// ============================================================================
// HARDWARE INSTANTIATION
// ============================================================================

pros::Motor left_front(MOTOR_LEFT_FRONT_PORT, pros::v5::MotorGears::blue);
pros::Motor left_back(MOTOR_LEFT_BACK_PORT, pros::v5::MotorGears::blue);
pros::Motor right_front(MOTOR_RIGHT_FRONT_PORT, pros::v5::MotorGears::blue);
pros::Motor right_back(MOTOR_RIGHT_BACK_PORT, pros::v5::MotorGears::blue);

pros::Motor cascade_left(MOTOR_CASCADE_LEFT_PORT, pros::v5::MotorGears::blue);
pros::Motor cascade_right(MOTOR_CASCADE_RIGHT_PORT, pros::v5::MotorGears::blue);

pros::Motor end_effector_rot(END_EFFECTOR_ROT_PORT);
pros::Rotation end_effector_rot_sense(END_EFFECTOR_ROT_SENSE_PORT);
pros::Motor intake(MOTOR_INTAKE_PORT);
pros::Motor end_effector(END_EFFECTOR_PORT);

pros::MotorGroup left_drive({MOTOR_LEFT_FRONT_PORT, MOTOR_LEFT_BACK_PORT}, pros::v5::MotorGears::blue);
pros::MotorGroup right_drive({MOTOR_RIGHT_FRONT_PORT, MOTOR_RIGHT_BACK_PORT}, pros::v5::MotorGears::blue);
pros::MotorGroup cascade({MOTOR_CASCADE_LEFT_PORT, MOTOR_CASCADE_RIGHT_PORT}, pros::v5::MotorGears::blue);
pros::Imu imu(IMU_PORT);

pros::Rotation odom_x(ODOM_X_PORT);
pros::Rotation odom_y(ODOM_Y_PORT);

pros::Optical optical(OPTICAL_SENSOR_PORT);
pros::AIVision ai_vision(AI_VISION_PORT);
pros::Gps gps(GPS_PORT);

// ============================================================================
// LEMLIB ODOMETRY & CHASSIS CONFIGURATION
// ============================================================================

// Dedicated tracking wheels (using unpowered rotation sensors)
lemlib::TrackingWheel vertical_tracking(&odom_y, lemlib::Omniwheel::NEW_2, -2.5);
lemlib::TrackingWheel horizontal_tracking(&odom_x, lemlib::Omniwheel::NEW_2, -5.75);

// Motor encoder tracking wheels (fallback if no dedicated vertical tracking sensor)
lemlib::TrackingWheel vertical_tracking_left(&left_drive, lemlib::Omniwheel::NEW_4, -5.0, 600);
lemlib::TrackingWheel vertical_tracking_right(&right_drive, lemlib::Omniwheel::NEW_4, 5.0, 600);

lemlib::OdomSensors sensors(&vertical_tracking,
	nullptr,
	&horizontal_tracking,
	nullptr,
	&imu
);

lemlib::ControllerSettings lat_controller(10,
										0,
										15,
										0,
										1,
										100,
										3,
										500,
										20
);

lemlib::ControllerSettings angular_controller(2,
										0,
										10,
										0,
										1,
										100,
										3,
										500,
										0
);

lemlib::Drivetrain drivetrain(&left_drive,
							&right_drive,
							10,
							lemlib::Omniwheel::NEW_4,
							600,
							2
);

lemlib::Chassis chassis(drivetrain,
						lat_controller,
						angular_controller,
						sensors
);

void run_hardcoded_autonomous() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	master.rumble("-");
	pros::lcd::print(4, "RUNNING AUTONOMOUS...");
	chassis.setPose(0.0, 0.0, 0.0);
	// Move forward 12 inches with minSpeed=35 to overcome static friction
	chassis.moveToPoint(0, 8, 4000, { .maxSpeed = 80.0, .minSpeed = 35.0 });
	chassis.waitUntilDone();
	// Return to start
	chassis.moveToPose(0, 0, 0, 4000);
	pros::delay(500);
	left_drive.brake();
	right_drive.brake();
	chassis.waitUntilDone();
}

void on_center_button() {}

void initialize() {
	enforce_approval_gate();
	pros::lcd::initialize();

	end_effector.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector.move(0);
	left_drive.move(0);
	right_drive.move(0);
	cascade_left.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade_right.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.tare_position();
	cascade.brake();

	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector_rot.tare_position();
	end_effector_rot_sense.reset_position();
	end_effector_rot.move(0);
	intake.move_voltage(0);

	optical.set_led_pwm(100);

	// 1. Safe IMU Calibration with timeout and delay (prevents CPU lockup)
	if (imu.is_installed()) {
		imu.reset();
		int imu_timeout = 0;
		while (imu.is_calibrating() && imu_timeout < 350) {
			pros::delay(10);
			imu_timeout++;
		}
		pros::lcd::print(1, "IMU calibrated on Port %d", IMU_PORT);
	} else {
		pros::lcd::print(1, "IMU not detected (Port %d)", IMU_PORT);
		sensors.imu = nullptr;
	}

	// 2. Safe Odometry Wheel Verification
	// If rotation sensor on Port 5 is not responding, fallback to dual motor tracking
	int32_t y_check = odom_y.get_position();
	if (!odom_y.is_installed() || y_check == PROS_ERR) {
		pros::lcd::print(2, "Odom Y not found! Using motors");
		sensors.vertical1 = &vertical_tracking_left;
		sensors.vertical2 = &vertical_tracking_right;
	} else {
		odom_y.reset_position();
		pros::lcd::print(2, "Odom Y 2in active (Port %d)", ODOM_Y_PORT);
		sensors.vertical1 = &vertical_tracking;
		sensors.vertical2 = nullptr;
	}

	if (!odom_x.is_installed() || odom_x.get_position() == PROS_ERR) {
		pros::lcd::print(3, "Odom X not found! (Port %d)", ODOM_X_PORT);
		sensors.horizontal1 = nullptr;
	} else {
		odom_x.reset_position();
		pros::lcd::print(3, "Odom X 2in active (Port %d)", ODOM_X_PORT);
		sensors.horizontal1 = &horizontal_tracking;
	}

	// 3. Calibrate chassis without LemLib's blocking calibrateIMU
	chassis.calibrate(false);

	// 4. Brain screen live telemetry task: shows computed pose and raw sensor values
	pros::Task telemetry_task([&]() {
		while (true) {
			lemlib::Pose p = chassis.getPose();
			pros::lcd::print(0, "X: %.2f  Y: %.2f", p.x, p.y);
			pros::lcd::print(5, "Theta: %.2f deg", p.theta);
			pros::lcd::print(6, "Raw Y: %d", odom_y.get_position());
			pros::lcd::print(7, "Raw X: %d", odom_x.get_position());
			pros::delay(50);
		}
	});

	pros::delay(200);
}

void disabled() {
	left_drive.move_voltage(0);
	right_drive.move_voltage(0);
	left_drive.brake();
	right_drive.brake();
	intake.move_voltage(0);
	cascade.brake();
	end_effector.move_voltage(0);
	end_effector_rot.brake();
}

void competition_initialize() {}

/**
 * Runs the user autonomous code under competition control.
 */
void autonomous() {
	if (!is_build_approved()) return;

	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector.set_brake_mode(MOTOR_BRAKE_HOLD);

	run_hardcoded_autonomous();
}

/**
 * Operator control entry point.
 * NOTE: When running on a practice field or bench without a competition switch,
 * PROS automatically enters opcontrol(). To support standalone testing, we
 * automatically run the autonomous routine here if no competition switch is attached!
 * You can also press Controller Button A at any time to re-run the autonomous test!
 */
void opcontrol() {
	if (!is_build_approved()) return;

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// Standalone autonomous test if not on competition switch
	if (!pros::competition::is_connected()) {
		pros::lcd::print(0, "STANDALONE AUTON TEST");
		run_hardcoded_autonomous();
	}

	left_drive.set_brake_mode(MOTOR_BRAKE_COAST);
	right_drive.set_brake_mode(MOTOR_BRAKE_COAST);

	while (true) {
		// Press Button A to test/re-run autonomous at any time!
		if (master.get_digital_new_press(DIGITAL_A)) {
			run_hardcoded_autonomous();
		}

		int left_y = master.get_analog(ANALOG_LEFT_Y);
		int right_x = master.get_analog(ANALOG_RIGHT_X);
		left_drive.move(left_y + right_x);
		right_drive.move(left_y - right_x);

		if (master.get_digital(DIGITAL_L1)) {
			cascade.move_velocity(600);
		} else if (master.get_digital(DIGITAL_L2)) {
			cascade.move_velocity(-600);
		} else {
			cascade.brake();
		}

		if (master.get_digital(DIGITAL_R1)) {
			intake.move_voltage(-12000);
		} else if (master.get_digital(DIGITAL_R2)) {
			intake.move_voltage(12000);
		} else {
			intake.move_voltage(0);
		}

		if (master.get_digital(DIGITAL_UP)) {
			end_effector_rot.move_velocity(-100);
		} else if (master.get_digital(DIGITAL_DOWN)) {
			end_effector_rot.move_velocity(100);
		} else {
			end_effector_rot.brake();
		}

		if (master.get_digital(DIGITAL_LEFT)) {
			end_effector.move_voltage(12000);
		} else if (master.get_digital(DIGITAL_RIGHT)) {
			end_effector.move_voltage(-12000);
		} else {
			end_effector.move_voltage(0);
		}

		pros::delay(10);
	}
}
