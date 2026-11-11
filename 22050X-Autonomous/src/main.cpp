#include "main.h"
#include "approval_gate.hpp"
#include <map>
#include <cmath>
#include "robot_fsm.hpp"
#include "field_map.hpp"
#include "lemlib/api.hpp"
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

const int BTN_BLUE_POS[4] = {260, 60, 440, 180};
const int BTN_RED_POS[4] = {40, 60, 220, 180};

// ============================================================================
// HARDWARE PORT CONFIGURATION
// Aligned with 22050X-Driver and Tuner ground truth
// ============================================================================

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

const int IMU_PORT = 12;

const int ODOM_X_PORT = 3;
const int ODOM_Y_PORT = 5;

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

pros::Optical optical(OPTICAL_SENSOR_PORT);
pros::AIVision ai_vision(AI_VISION_PORT);
pros::Gps gps(GPS_PORT);

// ============================================================================
// LEMLIB ODOMETRY & CHASSIS CONFIGURATION
// ============================================================================

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

bool rotating_macro = false;

// ============================================================================
// LOW-LEVEL SUBSYSTEM CONTROL FUNCTIONS
// ============================================================================

void drive_for(float dist, int timeout, lemlib::Pose current, float max_speed) {
    float theta_rad = current.theta * M_PI / 180.0;
    float target_x = current.x + (dist * std::sin(theta_rad));
    float target_y = current.y + (dist * std::cos(theta_rad));
    chassis.moveToPoint(target_x, target_y, timeout, { .maxSpeed = max_speed });
}

double get_end_effector_degrees() {
	int32_t pos = end_effector_rot_sense.get_position();
	if (pos == PROS_ERR) {
		return -end_effector_rot.get_position() / END_EFFECTOR_GEAR_RATIO;
	}
	// Sensor rotates negatively when arm moves up; invert so UP is positive degrees
	return (-pos / 100.0) / END_EFFECTOR_GEAR_RATIO;
}

void end_effector_control(float degrees, uint32_t timeout_ms = 2500) {
	bool finished = false;
	rotating_macro = true;
	uint32_t start_time = pros::millis();
	double prev_err = degrees - get_end_effector_degrees();
	while (!finished && (pros::millis() - start_time < timeout_ms)) {
		double current = get_end_effector_degrees();
		double error = degrees - current;
		double derivative = error - prev_err;
		prev_err = error;

		if (std::abs(error) <= 1.5) {
			finished = true;
			end_effector_rot.brake();
		} else {
			double kP = 200.0;
			double kD = 50.0;
			// Negative voltage moves UP (-100 velocity), positive voltage moves DOWN (+100 velocity)
			double voltage = -(error * kP + derivative * kD);
			if (voltage > 9500.0) voltage = 9500.0;
			if (voltage < -9500.0) voltage = -9500.0;
			if (std::abs(voltage) < 2000.0 && std::abs(error) > 1.5) {
				voltage = (error > 0) ? -2000.0 : 2000.0;
			}
			end_effector_rot.move_voltage((int)voltage);
		}
		pros::delay(10);
	}
	end_effector_rot.brake();
	rotating_macro = false;
}

void cascade_control(float to_pos, uint32_t timeout_ms = 3000) {
	uint32_t start = pros::millis();
	while (pros::millis() - start < timeout_ms) {
		float current = cascade.get_position();
		float error = to_pos - current;
		if (std::abs(error) <= 0.08f) {
			break;
		} else if (error > 0) {
			if (error <= 0.5f) {
				cascade.move_velocity(300);
			} else {
				cascade.move_velocity(600);
			}
		} else {
			if (current <= 1.0f) {
				cascade.move_velocity(-300);
			} else {
				cascade.move_velocity(-600);
			}
		}
		pros::delay(10);
	}
	cascade.brake();
}

// ============================================================================
// SCORING ROUTINE (CASCADE LIFT + BANSHEE CLAW)
// ============================================================================

void score_sequence() {
	// Raise cascade lift to scoring height
	cascade_control(CASCADE_UP_TARGET_ROTATIONS, 2200);
	// Pitch banshee claw wrist to scoring position
	end_effector_control(END_EFFECTOR_UP_TARGET, 1500);
	// Outtake game piece via end-effector rollers
	end_effector.move_voltage(12000);
	pros::delay(1200);
	end_effector.move_voltage(0);
	// Lower wrist and cascade back to travel position
	end_effector_control(END_EFFECTOR_DOWN_TARGET, 1500);
	cascade_control(CASCADE_DOWN_TARGET_ROTATIONS, 2200);
}

// ============================================================================
// TOGGLE MECHANISM WITH OPTICAL COLOR SENSING
// Goes straight backwards to flip toggle until correct alliance color is detected
// ============================================================================

bool flip_toggle_until_color(uint32_t timeout_ms = 4000) {
	optical.set_led_pwm(100);
	uint32_t start_time = pros::millis();
	bool color_matched = false;

	// Back up straight towards toggle
	left_drive.move_voltage(-6000);
	right_drive.move_voltage(-6000);

	while (pros::millis() - start_time < timeout_ms) {
		double hue = optical.get_hue();
		if (sel_alliance_color == AllianceColor::Red) {
			if (hue >= 340.0 || (hue <= 20.0 && hue > 0.0)) {
				color_matched = true;
				break;
			}
		} else if (sel_alliance_color == AllianceColor::Blue) {
			if (hue >= 195.0 && hue <= 245.0) {
				color_matched = true;
				break;
			}
		} else {
			// If unselected, accept either Red or Blue toggle state
			if ((hue >= 340.0 || (hue <= 20.0 && hue > 0.0)) || (hue >= 195.0 && hue <= 245.0)) {
				color_matched = true;
				break;
			}
		}
		pros::delay(15);
	}

	left_drive.brake();
	right_drive.brake();
	pros::delay(100);
	return color_matched;
}

// ============================================================================
// INTAKE SORTING & INTAKE DECIDE
// Mode 1: Eject opposing piece
// Mode 2: Keep / score alliance piece
// ============================================================================

void intake_handling(int mode) {
	if (mode == 1) {
		// Eject opposing pin
		end_effector_control(END_EFFECTOR_UP_TARGET, 800);
		intake.move_voltage(12000);
		end_effector.move_voltage(12000);
		pros::delay(350);
		end_effector_control(END_EFFECTOR_DOWN_TARGET, 800);
		intake.move_voltage(-12000);
		end_effector.move_voltage(-12000);
	} else if (mode == 2) {
		// Keep / score alliance pin
		end_effector_control(END_EFFECTOR_DOWN_TARGET, 500);
		intake.move_voltage(-12000);
		end_effector.move_voltage(-12000);
	}
}

/*
1: Yellow (35..65 deg hue)
2: Blue   (195..245 deg hue)
3: Red    (>=340 || <=20 deg hue)
*/
int intake_decide_sample() {
	float hue = optical.get_hue();
	int pin = 0;
	if (hue >= 35.0 && hue <= 65.0) {
		pin = 1; // Yellow pin -> keep / score
		intake_handling(2);
	} else if (hue >= 195.0 && hue <= 245.0) {
		pin = 2; // Blue pin
		if (sel_alliance_color == AllianceColor::Blue) {
			intake_handling(2);
		} else {
			intake_handling(1);
		}
	} else if (hue >= 340.0 || (hue <= 20.0 && hue > 0.0)) {
		pin = 3; // Red pin
		if (sel_alliance_color == AllianceColor::Red) {
			intake_handling(2);
		} else {
			intake_handling(1);
		}
	}
	return pin;
}

void intake_pins_with_decide(float dist_inches, uint32_t timeout_ms) {
	optical.set_led_pwm(100);
	intake.move_voltage(-12000);
	end_effector.move_voltage(-12000);

	lemlib::Pose cur = chassis.getPose();
	float theta_rad = cur.theta * M_PI / 180.0;
	float target_x = cur.x + (dist_inches * std::sin(theta_rad));
	float target_y = cur.y + (dist_inches * std::cos(theta_rad));

	chassis.moveToPoint(target_x, target_y, timeout_ms, { .maxSpeed = 55.0 });

	uint32_t start_time = pros::millis();
	while (chassis.isInMotion() && (pros::millis() - start_time < timeout_ms)) {
		intake_decide_sample();
		pros::delay(20);
	}
	chassis.waitUntilDone();
}

void intake_cup(float dist_inches, uint32_t timeout_ms) {
	intake.move_voltage(-12000);
	end_effector.move_voltage(-12000);
	drive_for(dist_inches, timeout_ms, chassis.getPose(), 60.0);
	chassis.waitUntilDone();
	pros::delay(300); // Allow cup to fully seat into claw / cascade
}

// ============================================================================
// AI VISION SENSOR INTEGRATION
// Retained active for vision detection pipeline & tracking targets
// ============================================================================

void update_ai_vision_tracking() {
	int32_t count = ai_vision.get_object_count();
	if (count > 0 && count != PROS_ERR) {
		auto objs = ai_vision.get_all_objects();
		objects.clear();
		for (const auto& obj : objs) {
			Target t;
			t.id = obj.id;
			if (pros::AIVision::is_type(obj, pros::AivisionDetectType::color)) {
				t.center_x = obj.object.color.xoffset + (obj.object.color.width / 2.0f);
				t.center_y = obj.object.color.yoffset + (obj.object.color.height / 2.0f);
				t.edge_x = obj.object.color.xoffset;
				t.edge_y = obj.object.color.yoffset;
				t.width = obj.object.color.width;
				t.height = obj.object.color.height;
				t.confidence = 100;
			} else if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object)) {
				t.center_x = obj.object.element.xoffset + (obj.object.element.width / 2.0f);
				t.center_y = obj.object.element.yoffset + (obj.object.element.height / 2.0f);
				t.edge_x = obj.object.element.xoffset;
				t.edge_y = obj.object.element.yoffset;
				t.width = obj.object.element.width;
				t.height = obj.object.element.height;
				t.confidence = obj.object.element.score;
			} else {
				t.center_x = 0;
				t.center_y = 0;
				t.edge_x = 0;
				t.edge_y = 0;
				t.width = 0;
				t.height = 0;
				t.confidence = 0;
			}
			addObj(t);
		}
	}
}

// ============================================================================
// FSM WRAPPER CALLBACKS
// ============================================================================

void init(RobotState& robot_state) {
	odom_x.reset_position();
	odom_y.reset_position();
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(10);
	}
	robot_state = RobotState::Navigating;
}

void loading(RobotState& robot_state) {}

void loading_stat(RobotState& robot_state) {
	intake.move_voltage(-12000);
	end_effector.move_voltage(-12000);
}

void picking(RobotState& robot_state) {}
void navigation(RobotState& robot_state) {}

void score(RobotState& robot_state) {
	score_sequence();
	robot_state = RobotState::Navigating;
}

void toggle(RobotState& robot_state) {
	flip_toggle_until_color(4000);
	robot_state = RobotState::Navigating;
}

void intake_decide(RobotState& robot_state) {
	intake_decide_sample();
}

// ============================================================================
// HARDCODED AUTONOMOUS ROUTINE
// ============================================================================

void run_hardcoded_autonomous() {
	// Zero odometry at starting line
	chassis.setPose(0.0, 0.0, 0.0);

	// 1. Turn 45 degrees to the right
	chassis.turnToHeading(45.0, 1500);
	chassis.waitUntilDone();

	// 2. Go forward and score preloaded game piece
	drive_for(18.0, 2000, chassis.getPose(), 80.0);
	chassis.waitUntilDone();
	score_sequence();

	// 3. Go straight backwards to flip the toggle until optical sensor detects correct color
	flip_toggle_until_color(4000);

	// Clear slightly forward from toggle before rotating
	drive_for(4.0, 1000, chassis.getPose(), 70.0);
	chassis.waitUntilDone();

	// 4. Turn -90 degrees (relative to current heading)
	float target_heading = chassis.getPose().theta - 90.0f;
	chassis.turnToHeading(target_heading, 1500);
	chassis.waitUntilDone();

	// 5. Intake a few pins and use intake decide to decide which pins to keep or eject
	intake_pins_with_decide(22.0, 3000);

	// 6. Intake a cup from there as well
	intake_cup(14.0, 2000);

	// 7. Score it!
	score_sequence();
}

// ============================================================================
// ALLIANCE SELECTION GUI
// ============================================================================

void alliance_color(pros::Controller master) {
	pros::screen::set_eraser(RGB2COLOR(0, 0, 0));
	pros::screen::erase();
	pros::screen::set_eraser(RGB2COLOR(255, 255, 255));
	pros::screen::set_pen(RGB2COLOR(255, 0, 0));
	pros::screen::fill_rect(BTN_RED_POS[0], BTN_RED_POS[1], BTN_RED_POS[2], BTN_RED_POS[3]);
	pros::screen::set_pen(RGB2COLOR(255, 255, 255));
	pros::screen::print(TEXT_MEDIUM, (BTN_RED_POS[0] + 30), (BTN_RED_POS[2] + 50), "Red Alliance");
	pros::screen::set_pen(RGB2COLOR(0, 0, 255));
	pros::screen::fill_rect(BTN_BLUE_POS[0], BTN_BLUE_POS[1], BTN_BLUE_POS[2], BTN_BLUE_POS[3]);
	pros::screen::set_pen(RGB2COLOR(255, 255, 255));
	pros::screen::print(TEXT_MEDIUM, (BTN_BLUE_POS[0] + 30), (BTN_BLUE_POS[2] + 50), "Blue Alliance");
	pros::screen::set_pen(RGB2COLOR(255, 255, 255));
	pros::screen::print(TEXT_MEDIUM, 10, 10, "Select Alliance Color");
	while (true) {
		pros::screen_touch_status_s_t status = pros::screen::touch_status();
		if (status.press_count > 0) {
			if (status.x <= BTN_RED_POS[2] && status.x >= BTN_RED_POS[0] && status.y >= BTN_RED_POS[1] && status.y <= BTN_RED_POS[3]) {
				sel_alliance_color = AllianceColor::Red;
				master.rumble(". - .");
				break;
			} else if (status.x <= BTN_BLUE_POS[2] && status.x >= BTN_BLUE_POS[0] && status.y >= BTN_BLUE_POS[1] && status.y <= BTN_BLUE_POS[3]) {
				sel_alliance_color = AllianceColor::Blue;
				master.rumble("- . . .");
				break;
			}
		}
		pros::delay(20);
	}
	pros::screen::set_eraser(RGB2COLOR(0, 0, 0));
	pros::screen::erase();
	if (sel_alliance_color == AllianceColor::Red) {
		pros::screen::set_pen(RGB2COLOR(255, 0, 0));
		pros::screen::print(TEXT_MEDIUM, 3, "Red Alliance Selected");
	} else if (sel_alliance_color == AllianceColor::Blue) {
		pros::screen::set_pen(RGB2COLOR(0, 0, 255));
		pros::screen::print(TEXT_MEDIUM, 3, "Blue Alliance Selected");
	}
}

void on_center_button() {}

// ============================================================================
// SYSTEM INITIALIZATION & COMPETITION HOOKS
// ============================================================================

void initialize() {
	enforce_approval_gate();
	pros::Controller master(pros::E_CONTROLLER_MASTER);
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

	optical.set_led_pwm(100);

	chassis.calibrate();

	pros::delay(200);
	alliance_color(master);
}

void disabled() {}

void competition_initialize() {}

/**
 * Runs the user autonomous code.
 * Executes the hardcoded route with optical sensing, intake sorting, and banshee claw manipulation.
 */
void autonomous() {
	if (!is_build_approved()) return;

	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);

	run_hardcoded_autonomous();
}

/**
 * Operator control code placeholder
 */
void opcontrol() {
	if (!is_build_approved()) return;
}
