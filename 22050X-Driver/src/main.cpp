#include "main.h"
#include <cmath>
#include "lemlib/api.hpp"
#include <thread>
#include <chrono>

const int MOTOR_LEFT_FRONT_PORT = 20;
const int MOTOR_LEFT_BACK_PORT = -10;
const int MOTOR_RIGHT_FRONT_PORT = -7;
const int MOTOR_RIGHT_BACK_PORT = 9;
const int MOTOR_CASCADE_LEFT_PORT = 1;
const int MOTOR_CASCADE_RIGHT_PORT = -2;
const int MOTOR_INTAKE_PORT = 19;

const int END_EFFECTOR_ROT_PORT = 18;
const int END_EFFECTOR_ROT_SENSE_PORT = 13;
const int END_EFFECTOR_PORT = 14;

const int IMU_PORT = 3;

const int ODOM_X_PORT = 3;
const int ODOM_Y_PORT = 0;

bool rotating_macro = false;

pros::Motor left_front(MOTOR_LEFT_FRONT_PORT);
pros::Motor left_back(MOTOR_LEFT_BACK_PORT);
pros::Motor right_front(MOTOR_RIGHT_FRONT_PORT);
pros::Motor right_back(MOTOR_RIGHT_BACK_PORT);

pros::Motor cascade_left(MOTOR_CASCADE_LEFT_PORT);
pros::Motor cascade_right(MOTOR_CASCADE_RIGHT_PORT);

pros::Motor end_effector_rot(END_EFFECTOR_ROT_PORT);
pros::Motor end_effector_rot_sense(END_EFFECTOR_ROT_SENSE_PORT);
pros::Motor intake(MOTOR_INTAKE_PORT);
pros::Motor end_effector(END_EFFECTOR_PORT);

pros::MotorGroup left_drive({MOTOR_LEFT_FRONT_PORT, MOTOR_LEFT_BACK_PORT});
pros::MotorGroup right_drive({MOTOR_RIGHT_FRONT_PORT, MOTOR_RIGHT_BACK_PORT});
pros::MotorGroup cascade({MOTOR_CASCADE_LEFT_PORT, MOTOR_CASCADE_RIGHT_PORT});
pros::Imu imu(IMU_PORT);

pros::Rotation odom_x(ODOM_X_PORT);
pros::Rotation odom_y(ODOM_Y_PORT);

lemlib::TrackingWheel horizontal_tracking(&odom_x, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking(&odom_y, lemlib::Omniwheel::NEW_2, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking,
	nullptr,
	&horizontal_tracking,
	nullptr,
	&imu
);

lemlib::ControllerSettings lat_controller(10,
										0,
										3,
										3,
										1,
										100,
										3,
										500,
										20
);

lemlib::ControllerSettings angular_controller(2,
										0,
										10,
										3,
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


void end_effector_control(float degrees) {
	bool finished = false;
	rotating_macro = true;
	while (!finished) {
		float current = end_effector_rot_sense.get_position();
		if (std::abs(current - degrees) <= 5.0f) {
			finished = true;
			end_effector_rot.brake();
		} else if (current > degrees) {
			end_effector_rot.move_voltage(-12000);
		} else {
			end_effector_rot.move_voltage(12000);
		}
		
		if (std::abs(end_effector_rot_sense.get_position() - degrees) <= 5) {
			finished = true;
			end_effector_rot.brake();
		}
		pros::delay(10);
	}
	rotating_macro = false;
}

void cascade_control(float to_pos) {
	bool finished = false;
	while (!finished) {
		float current = cascade.get_position();
		if (std::abs(current - to_pos) <= 3.0f) {
			finished = true;
			cascade.brake();
		} else if (current < to_pos) {
			cascade.move_voltage(12000);
		} else {
			cascade.move_voltage(-12000);
		};
		pros::delay(10);
	}
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {
	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	end_effector.move(0);
	left_drive.move(0);
	right_drive.move(0);
	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.move(0);
	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector_rot.move(0);
	intake.move_voltage(0);
	pros::delay(200);
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
void competition_initialize() {

}

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);

	bool reversing_end_effector = false;
	uint32_t reverse_start_time = 0;

	while (true) {
		int left_y = master.get_analog(ANALOG_LEFT_Y);
		int right_x = master.get_analog(ANALOG_RIGHT_X);
		int button_l1 = master.get_digital(DIGITAL_L1);
		int button_l2 = master.get_digital(DIGITAL_L2);
		if (button_l1) {
			cascade.move_velocity(600);
		} else if (button_l2) {
			cascade.move_velocity(-600);
		} else {
			cascade.move_velocity(0);
		}
		int button_r1 = master.get_digital(DIGITAL_R1);
		int button_r2 = master.get_digital(DIGITAL_R2);
		if (button_r1) {
			intake.move_voltage(-12000);
		} else if (button_r2) {
			intake.move_voltage(12000);
		} else {
			intake.move_voltage(0);
		}
		int button_left = master.get_digital(DIGITAL_LEFT);
		int button_right = master.get_digital(DIGITAL_RIGHT);
		int button_up = master.get_digital(DIGITAL_UP);
		int button_down = master.get_digital_new_press(DIGITAL_DOWN);
		if (button_down) {
			pros::Task cascade_task([] { cascade_control(0); });
		} else if (!rotating_macro) {
			end_effector_rot.brake();
		}

		int button_b = master.get_digital_new_press(DIGITAL_B);
		if (button_b) {
			reversing_end_effector = true;
			reverse_start_time = pros::millis();
		}

		if (reversing_end_effector) {
			if (pros::millis() - reverse_start_time >= 3000) {
				reversing_end_effector = false;
			}
			end_effector.move_voltage(-12000);
		} else {
			end_effector.move_voltage(12000);
		}

		left_drive.move(left_y + right_x);
		right_drive.move(left_y - right_x);
		pros::delay(10); 
	}
}