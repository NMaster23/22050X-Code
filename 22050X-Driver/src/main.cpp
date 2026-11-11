#include "main.h"
#include "approval_gate.hpp"
#include <cmath>
#include <algorithm>
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
const int ODOM_Y_PORT = 0;

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
	enforce_approval_gate();
	pros::lcd::initialize();
	end_effector.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector.move(0);
	left_drive.move(0);
	right_drive.move(0);
	cascade_left.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade_right.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.set_encoder_units(pros::MotorUnits::rotations);
	cascade.tare_position();
	cascade.brake();
	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector_rot.tare_position();
	end_effector_rot_sense.reset_position();
	end_effector_rot.move(0);
	intake.move_voltage(0);

	pros::Task telemetry_task([]() {
		while (true) {
			double arm_angle = get_end_effector_degrees();
			double sensor_pos = -(end_effector_rot_sense.get_position() / 100.0);
			pros::lcd::print(3, "Arm:  %5.1f deg", arm_angle);
			pros::lcd::print(4, "Sens: %5.1f deg (3:1)", sensor_pos);
			pros::lcd::print(5, "Casc: %5.2f rot", cascade.get_position());
			pros::delay(50);
		}
	});

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
void autonomous() {
	if (!is_build_approved()) return;
}

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
	if (!is_build_approved()) return;
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	cascade_left.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade_right.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.set_encoder_units(pros::MotorUnits::rotations);
	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector.set_brake_mode(MOTOR_BRAKE_HOLD);

	bool reversing_end_effector = false;
	uint32_t reverse_start_time = 0;

	bool cascade_macro_active = false;
	double cascade_target_pos = 0.0;

	bool end_effector_toggle_up = false;
	bool end_effector_macro_active = false;
	double end_effector_target = END_EFFECTOR_DOWN_TARGET;
	double prev_arm_error = 0.0;

	while (true) {
		int left_y = master.get_analog(ANALOG_LEFT_Y);
		int right_x = master.get_analog(ANALOG_RIGHT_X);
		int button_l1 = master.get_digital(DIGITAL_L1);
		int button_l2 = master.get_digital(DIGITAL_L2);

		if (button_l1) {
			cascade_macro_active = false;
			cascade.move_velocity(600);
		} else if (button_l2) {
			cascade_macro_active = false;
			cascade.move_velocity(-600);
		} else if (cascade_macro_active) {
			double cascade_pos = cascade.get_position();
			double cascade_error = cascade_target_pos - cascade_pos;
			if (std::abs(cascade_error) <= 0.08) {
				cascade_macro_active = false;
				cascade.move_velocity(0);
				cascade.brake();
			} else if (cascade_error > 0) {
				if (cascade_error <= 0.5) {
					cascade.move_velocity(300);
				} else {
					cascade.move_velocity(600);
				}
			} else {
				if (cascade_pos <= 1.0) {
					cascade.move_velocity(-300);
				} else {
					cascade.move_velocity(-600);
				}
			}
		} else {
			cascade.brake();
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
		int button_right = master.get_digital_new_press(DIGITAL_RIGHT);
		int button_up = master.get_digital(DIGITAL_UP);
		int button_down = master.get_digital(DIGITAL_DOWN);

		if (button_right) {
			end_effector_toggle_up = !end_effector_toggle_up;
			end_effector_macro_active = true;
			end_effector_target = end_effector_toggle_up ? END_EFFECTOR_UP_TARGET : END_EFFECTOR_DOWN_TARGET;
			prev_arm_error = end_effector_target - get_end_effector_degrees();

			cascade_macro_active = true;
			cascade_target_pos = end_effector_toggle_up ? CASCADE_UP_TARGET_ROTATIONS : 0.0;
		}

		if (button_up) {
			end_effector_macro_active = false;
			end_effector_rot.move_velocity(-100);
		} else if (button_down) {
			end_effector_macro_active = false;
			end_effector_rot.move_velocity(100);
		} else if (end_effector_macro_active) {
			double current = get_end_effector_degrees();
			double error = end_effector_target - current;
			double derivative = error - prev_arm_error;
			prev_arm_error = error;

			if (std::abs(error) <= 1.0) {
				end_effector_rot.brake();
			} else {
				double kP = 200.0;
				double kD = 50.0;
				// Inverted: negative voltage moves UP (-100 velocity), positive voltage moves DOWN (+100 velocity)
				double voltage = -(error * kP + derivative * kD);
				if (voltage > 9500.0) voltage = 9500.0;
				if (voltage < -9500.0) voltage = -9500.0;
				if (std::abs(voltage) < 2000.0 && std::abs(error) > 1.0) {
					voltage = (error > 0) ? -2000.0 : 2000.0;
				}
				end_effector_rot.move_voltage((int)voltage);
			}
		} else {
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
			end_effector.move_voltage(12000);
		} else {
			end_effector.move_voltage(-12000);
		}

		double cascade_rotations = std::max(0.0, cascade.get_position());
		double drive_scale = 1.0;
		double half_rotations = CASCADE_MAX_ROTATIONS * 0.5;
		if (cascade_rotations > half_rotations) {
			double fraction_above_half = (cascade_rotations - half_rotations) / (CASCADE_MAX_ROTATIONS - half_rotations);
			if (fraction_above_half > 1.0) fraction_above_half = 1.0;
			drive_scale = 1.0 - fraction_above_half * (1.0 - MIN_DRIVE_SCALE);
		}

		left_drive.move(std::round((left_y + right_x) * drive_scale));
		right_drive.move(std::round((left_y - right_x) * drive_scale));
		pros::delay(10); 
	}
}