#include "main.h"
#include <map>
#include "robot_fsm.hpp"
#include "field_map.hpp"
#include "lemlib/api.hpp"

enum class AllianceColor {
	Red,
	Blue,
	None,
};

enum class AutonMode {
	HardCoded,
	Experimental,
};

struct RobotPos {
	double x;
	double y;
};

AutonMode sel_auton_mode = AutonMode::HardCoded;
AllianceColor sel_alliance_color = AllianceColor::None;

RobotPos position = {0.0, 0.0};
pros::Mutex pos_mutex;

const int WHEEL_R = 1.0;

const int BTN_BLUE_POS[4] = {260, 60, 440, 180};
const int BTN_RED_POS[4] = {40, 60, 220, 180};

const int MOTOR_LEFT_FRONT_PORT = 20;
const int MOTOR_LEFT_BACK_PORT = -10;
const int MOTOR_RIGHT_FRONT_PORT = -7;
const int MOTOR_RIGHT_BACK_PORT = 9;
const int MOTOR_CASCADE_LEFT_PORT = 1;
const int MOTOR_CASCADE_RIGHT_PORT = -2;
const int MOTOR_INTAKE_PORT = 19;

const int END_EFFECTOR_ROT_SENSE_PORT = 0;
const int END_EFFECTOR_ROT_PORT = 4;
const int END_EFFECTOR_PORT = 5;
const int IMU_PORT = 3;

const int ODOM_X_PORT = 0;
const int ODOM_Y_PORT = 0;

const int GPS_PORT = 0;
const int AI_VISION_PORT = 0;
const int OPTICAL_SENSOR_PORT = 0;

pros::Motor left_front(MOTOR_LEFT_FRONT_PORT);
pros::Motor left_back(MOTOR_LEFT_BACK_PORT);
pros::Motor right_front(MOTOR_RIGHT_FRONT_PORT);
pros::Motor right_back(MOTOR_RIGHT_BACK_PORT);

pros::Motor cascade_left(MOTOR_CASCADE_LEFT_PORT);
pros::Motor cascade_right(MOTOR_CASCADE_RIGHT_PORT);

pros::Motor end_effector_rot(END_EFFECTOR_ROT_PORT);
pros::Motor intake(MOTOR_INTAKE_PORT);
pros::Motor end_effector(END_EFFECTOR_PORT);
pros::Motor end_effector_rot_sense(END_EFFECTOR_ROT_SENSE_PORT);

pros::MotorGroup left_drive({MOTOR_LEFT_FRONT_PORT, MOTOR_LEFT_BACK_PORT});
pros::MotorGroup right_drive({MOTOR_RIGHT_FRONT_PORT, MOTOR_RIGHT_BACK_PORT});
pros::MotorGroup cascade({MOTOR_CASCADE_LEFT_PORT, MOTOR_CASCADE_RIGHT_PORT});
pros::Imu imu(IMU_PORT);

pros::Rotation odom_x(ODOM_X_PORT);
pros::Rotation odom_y(ODOM_Y_PORT);

pros::Gps gps(GPS_PORT);
pros::AIVision ai_vision(AI_VISION_PORT);
pros::Optical optical(OPTICAL_SENSOR_PORT);

lemlib::TrackingWheel horizontal_tracking(&odom_x, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking(&odom_y, lemlib::Omniwheel::NEW_2, -2.5);

lemlib::Odomsensors = sensors(&vertical_tracking,
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
										20,
);

lemlib::ControllerSettings angular_controller(2,
										0,
										10,
										3,
										1,
										100,
										3,
										500,
										0,
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
						sensors,
);

bool rotating_macro = false;

void end_effector_control(float degrees) {
	bool finished = false;
	rotating_macro = true;
	while (!finished) {
		if (end_effector_rot_sense.get_position() > degrees) {
			end_effector_rot.move_voltage(-12000);
		} else if (end_effector_rot_sense.get_position() < degrees) {
			end_effector_rot.move_voltage(12000);
		} else {
			finished = true;
			end_effector_rot.brake();
		}
		
		if (std::abs(end_effector_rot_sense.get_position() - degrees) <= 5) {
			finished = true;
			end_effector_rot.brake();
		}
		pros::delay(10);
	}
	rotating_macro = false;
}

void ai_vision_sensor(void *param) {
	ai_vision.reset();
	std::vector<pros::v5::AIVision::Object> items = ai_vision.get_all_objects();
	while (true) {
		std::vector<pros::v5::AIVision::Object, std::allocator<pros::v5::AIVision::Object>> objects = ai_vision.get_all_objects();
		for (const auto& obj: objects) {
			if (pros::v5::AIVision::is_type(obj, pros::AivisionDetectType::object)) {
				Target detected_obj = Target {
					.id = obj.id,
					.center_x = obj.object.element.xoffset + (obj.object.element.width / 2.0f),
					.center_y = obj.object.element.yoffset + (obj.object.element.height / 2.0f),
					.edge_x = obj.object.element.xoffset,
					.edge_y = obj.object.element.yoffset,
					.width = obj.object.element.width,
					.height = obj.object.element.height,
					.confidence = obj.object.element.score,
				};
				addPoint(detected_obj.center_x, detected_obj.center_y, std::to_string(obj.type));
				addObj(detected_obj);
			} else if (pros::v5::AIVision::is_type(obj, pros::AivisionDetectType::tag)) {
				Target detected_obj = Target {
					.id = obj.id,
					.center_x = (obj.object.tag.x0 + obj.object.tag.x1 + obj.object.tag.x2 + obj.object.tag.x3) / 4.0f,
					.center_y = (obj.object.tag.y0 + obj.object.tag.y1 + obj.object.tag.y2 + obj.object.tag.y3) / 4.0f,
					.edge_x = std::min({obj.object.tag.x0, obj.object.tag.x1, obj.object.tag.x2, obj.object.tag.x3}),
					.edge_y = std::min({obj.object.tag.y0, obj.object.tag.y1, obj.object.tag.y2, obj.object.tag.y3}),
					.width = std::max({obj.object.tag.x0, obj.object.tag.x1, obj.object.tag.x2, obj.object.tag.x3}) - std::min({obj.object.tag.x0, obj.object.tag.x1, obj.object.tag.x2, obj.object.tag.x3}),
					.height = std::max({obj.object.tag.y0, obj.object.tag.y1, obj.object.tag.y2, obj.object.tag.y3}) - std::min({obj.object.tag.y0, obj.object.tag.y1, obj.object.tag.y2, obj.object.tag.y3}),
					.confidence = 1,
				};
				addPoint(detected_obj.center_x, detected_obj.center_y, std::to_string(obj.type));
				addObj(detected_obj);
			} else {
				continue;
			}
		}
		pros::delay(10);
	}
}

void field_tracking(void* param) {
	while (true) {
		float gps_x = gps.get_position_x();
		float gps_y = gps.get_position_y();
		float x_pos = odom_x.get_position();
		float y_pos = odom_y.get_position();
		float dist_x = 2 * std::numbers::pi * WHEEL_R * (x_pos / 36000);
		float dist_y = 2 * std::numbers::pi * WHEEL_R * (y_pos / 36000);
		pros::delay(20);
	}
}

void init(RobotState& robot_state) {
	odom_x.reset_position();
	odom_y.reset_position();
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(10);
	}
	robot_state == RobotState::Navigating;
}

void loading(RobotState& robot_state) {

}

void loading_stat(RobotState& robot_state) {
	intake.move_voltage(-12000);
	end_effector.move_voltage(-12000);
}

void picking(RobotState& robot_state) {

}

void navigation(RobotState& robot_state) {

}

void score(RobotState& robot_state) {

}

void toggle(RobotState& robot_state) {
	
}
/*
1: Yellow
2: Blue
3: Red
*/
void intake_decide(RobotState& robot_state) {
	bool looking_for = true;
	optical.set_led_pwm(100);
	int pin = 0;
	while (looking_for) {
		float hue = optical.get_hue();
		if (hue >= 35 && hue <= 65) {
			pin = 1;
			intake_handling(2);
		} else if (hue >= 195 && hue <= 245) {
			pin = 2;
			if (sel_alliance_color == AllianceColor::Blue) {
				intake_handling(2);
			} else {
				intake_handling(1);
			}
		} else if (hue >= 340 && hue <= 20) {
			pin = 3;
			if (sel_alliance_color == AllianceColor::Red) {
				intake_handling(2);
			} else {
				intake_handling(1);
			}
		} else {
			pin = 0;
		}
	}
}

void intake_handling(int case) {
	if (case == 1) {
		// eject
		end_effector_control(120);
		intake.move_voltage(12000);
	} else if (case == 2) {
		// score
		end_effector_control(0);
		intake.move_voltage(12000);
	}
}

void deciding_fsm() {
	
}

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
	uint32_t c_s_t = pros::millis();
	while (pros::millis() - c_s_t < 5000) {
		if (master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2) && master.get_digital(DIGITAL_R1) && master.get_digital(DIGITAL_R2)) {
			sel_auton_mode = AutonMode::Experimental;
			master.rumble("- -");
			break;
		}
		pros::delay(20);
	}
	if (sel_alliance_color == AllianceColor::Red) {
		pros::screen::set_pen(RGB2COLOR(255, 0, 0));
		pros::screen::print(TEXT_MEDIUM, 3, "Red Alliance Selected");
	} else if (sel_alliance_color == AllianceColor::Blue) {
		pros::screen::set_pen(RGB2COLOR(0, 0, 255));
		pros::screen::print(TEXT_MEDIUM, 3, "Blue Alliance Selected");
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	end_effector.move(0);
	left_drive.move(0);
	right_drive.move(0);
	cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
	cascade.move(0);
	end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
	end_effector_rot.move(0);
	intake.move_voltage(0);
	pros::delay(200);
	alliance_color(master);
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
	if (sel_auton_mode == AutonMode::Experimental) {
		ai_vision.enable_detection_types(pros::AivisionModeType::tags, pros::AivisionModeType::objects);
		cascade.set_brake_mode(MOTOR_BRAKE_HOLD);
		end_effector_rot.set_brake_mode(MOTOR_BRAKE_HOLD);
		RobotState current_state = RobotState::Init;
		pros::Task vision_task(ai_vision_sensor);
		while (true) {
			HandleStates(current_state, init, loading, loading_stat, picking, navigation, score, toggle, intake_decide);
			pros::delay(20);
		}
	} else if (sel_auton_mode == AutonMode::HardCoded) {

	} else {
		pros::screen::print(TEXT_LARGE, 3, "Error");
	}
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
	/*while (true) {
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
	}*/
}