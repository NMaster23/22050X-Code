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

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor left_front(MOTOR_LEFT_FRONT_PORT);
pros::Motor left_back(MOTOR_LEFT_BACK_PORT);
pros::Motor right_front(MOTOR_RIGHT_FRONT_PORT);
pros::Motor right_back(MOTOR_RIGHT_BACK_PORT);

pros::Motor cascade_left(MOTOR_CASCADE_LEFT_PORT);
pros::Motor cascade_right(MOTOR_CASCADE_RIGHT_PORT);

pros::Motor end_effector_rot(END_EFFECTOR_ROT_PORT);
pros::Rotation end_effector_rot_sense(END_EFFECTOR_ROT_SENSE_PORT);
pros::Motor intake(MOTOR_INTAKE_PORT);
pros::Motor end_effector(-END_EFFECTOR_PORT);

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

lemlib::ControllerSettings lateral_controller(14, // proportional gain (kP)
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
                                              15, // derivative gain (kD)
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

double get_end_effector_degrees() {
	int32_t pos = end_effector_rot_sense.get_position();
	if (pos == PROS_ERR) {
		return -end_effector_rot.get_position() / END_EFFECTOR_GEAR_RATIO;
	}
	// Sensor rotates negatively when arm moves up; invert so UP is positive degrees
	return (-pos / 100.0) / END_EFFECTOR_GEAR_RATIO;
}

void end_effector_control(float degrees) {
	bool finished = false;
	rotating_macro = true;
	uint32_t start_time = pros::millis();
	double prev_err = degrees - get_end_effector_degrees();
	while (!finished && (pros::millis() - start_time < 3000)) {
		double current = get_end_effector_degrees();
		double error = degrees - current;
		double derivative = error - prev_err;
		prev_err = error;

		if (std::abs(error) <= 1.0) {
			finished = true;
			end_effector_rot.brake();
		} else {
			double kP = 200.0;
			double kD = 50.0;
			// Negative voltage moves UP (-100 velocity), positive voltage moves DOWN (+100 velocity)
			double voltage = -(error * kP + derivative * kD);
			if (voltage > 9500.0) voltage = 9500.0;
			if (voltage < -9500.0) voltage = -9500.0;
			if (std::abs(voltage) < 2000.0 && std::abs(error) > 1.0) {
				voltage = (error > 0) ? -2000.0 : 2000.0;
			}
			end_effector_rot.move_voltage((int)voltage);
		}
		pros::delay(10);
	}
	end_effector_rot.brake();
	rotating_macro = false;
}


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
    odom_x.set_position(0);
    odom_y.set_position(0);
    end_effector_rot_sense.set_position(0);
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
    end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
    end_effector.move_velocity(600);
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, -12, 750, {.forwards = false, .maxSpeed = 105});
	chassis.moveToPoint(0, 2, 750, {.minSpeed = 127});
	chassis.moveToPoint(0, -12, 750, {.forwards = false, .maxSpeed = 105});
    pros::Task([]() {
        end_effector_control(50);
    });
    end_effector_rot.brake();
	chassis.moveToPoint(0, 2, 750, {.minSpeed = 127});
	chassis.turnToHeading(65, 300);
	chassis.moveToPoint(-31.6, -25, 900, {.forwards = false});
	chassis.waitUntilDone();
    end_effector.move_velocity(-600);
    pros::delay(500);
    pros::Task([]() {
        end_effector_control(90);
    });
    chassis.moveToPose(0, -15, 90, 1000);
}
// x1: 0.0
// y1: -15
// r1: 90