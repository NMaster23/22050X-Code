#include "main.h"
#pragma once
#include <map>

const int MOTOR_LEFT_FRONT_PORT = 20;
const int MOTOR_LEFT_BACK_PORT = -8;
const int MOTOR_RIGHT_FRONT_PORT = -7;
const int MOTOR_RIGHT_BACK_PORT = 9;
const int MOTOR_CASCADE_LEFT_PORT = 1;
const int MOTOR_CASCADE_RIGHT_PORT = -2;
const int MOTOR_INTAKE_PORT = 19;

const int END_EFFECTOR_ROT_PORT = 4;
const int END_EFFECTOR_PORT = 5;
const int IMU_PORT = 3;

const int ODOM_X_PORT = 0;
const int ODOM_Y_PORT = 0;

pros::Motor left_front(MOTOR_LEFT_FRONT_PORT);
pros::Motor left_back(MOTOR_LEFT_BACK_PORT);
pros::Motor right_front(MOTOR_RIGHT_FRONT_PORT);
pros::Motor right_back(MOTOR_RIGHT_BACK_PORT);

pros::Motor cascade_left(MOTOR_CASCADE_LEFT_PORT);
pros::Motor cascade_right(MOTOR_CASCADE_RIGHT_PORT);

pros::Motor end_effector_rot(END_EFFECTOR_ROT_PORT);
pros::Motor intake(MOTOR_INTAKE_PORT);
pros::Motor end_effector(END_EFFECTOR_PORT);

pros::MotorGroup left_drive({MOTOR_LEFT_FRONT_PORT, MOTOR_LEFT_BACK_PORT});
pros::MotorGroup right_drive({MOTOR_RIGHT_FRONT_PORT, MOTOR_RIGHT_BACK_PORT});
pros::MotorGroup cascade({MOTOR_CASCADE_LEFT_PORT, MOTOR_CASCADE_RIGHT_PORT});
pros::Imu imu(IMU_PORT);

pros::Rotation odom_x(ODOM_X_PORT);
pros::Rotation odom_y(ODOM_Y_PORT);

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
	while (true) {
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
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
		if (button_left) {
			end_effector.move_voltage(12000);
		} else if (button_right) {
			end_effector.move_voltage(-12000);
		} else {
			end_effector.move_voltage(0);
		}
		int button_up = master.get_digital(DIGITAL_UP);
		int button_down = master.get_digital(DIGITAL_DOWN);
		if (button_up) {
			end_effector_rot.move_velocity(-100);
		} else if (button_down) {
			end_effector_rot.move_velocity(100);
		} else {
			end_effector_rot.brake();
		}
		left_drive.move(dir + turn);
		right_drive.move(dir - turn);
		pros::delay(10); 
	}
}

void init() {
	odom_x.reset_position();
	odom_y.reset_position();
}