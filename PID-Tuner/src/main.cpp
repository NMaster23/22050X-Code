#include "main.h"
#include "approval_gate.hpp"
#include "lemlib/api.hpp"
#include "robot_config.hpp"
#include "serial_bridge.hpp"
#include "hot_reload_chassis.hpp"
#include "pros/apix.h"

#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>

// ============================================================================
// 22050X PHYSICAL HARDWARE DEFINITIONS
// ============================================================================

const int MOTOR_LEFT_FRONT_PORT = robot_config::MOTOR_LEFT_FRONT_PORT;
const int MOTOR_LEFT_BACK_PORT = robot_config::MOTOR_LEFT_BACK_PORT;
const int MOTOR_RIGHT_FRONT_PORT = robot_config::MOTOR_RIGHT_FRONT_PORT;
const int MOTOR_RIGHT_BACK_PORT = robot_config::MOTOR_RIGHT_BACK_PORT;
const int MOTOR_CASCADE_LEFT_PORT = robot_config::MOTOR_CASCADE_LEFT_PORT;
const int MOTOR_CASCADE_RIGHT_PORT = robot_config::MOTOR_CASCADE_RIGHT_PORT;
const int MOTOR_INTAKE_PORT = robot_config::MOTOR_INTAKE_PORT;

const int END_EFFECTOR_ROT_PORT = robot_config::END_EFFECTOR_ROT_PORT;
const int END_EFFECTOR_ROT_SENSE_PORT = robot_config::END_EFFECTOR_ROT_SENSE_PORT;
const int END_EFFECTOR_PORT = robot_config::END_EFFECTOR_PORT;

const int IMU_PORT = robot_config::IMU_PORT;
const int ODOM_X_PORT = robot_config::ODOM_X_PORT;
const int ODOM_Y_PORT = robot_config::ODOM_Y_PORT;

pros::Motor left_front(MOTOR_LEFT_FRONT_PORT, pros::v5::MotorGears::blue);
pros::Motor left_back(MOTOR_LEFT_BACK_PORT, pros::v5::MotorGears::blue);
pros::Motor right_front(MOTOR_RIGHT_FRONT_PORT, pros::v5::MotorGears::blue);
pros::Motor right_back(MOTOR_RIGHT_BACK_PORT, pros::v5::MotorGears::blue);

pros::Motor cascade_left(MOTOR_CASCADE_LEFT_PORT, pros::v5::MotorGears::blue);
pros::Motor cascade_right(MOTOR_CASCADE_RIGHT_PORT, pros::v5::MotorGears::blue);

pros::Motor end_effector_rot(END_EFFECTOR_ROT_PORT);
pros::Motor end_effector_rot_sense(END_EFFECTOR_ROT_SENSE_PORT);
pros::Motor intake(MOTOR_INTAKE_PORT, pros::v5::MotorGears::blue);
pros::Motor end_effector(END_EFFECTOR_PORT);

pros::MotorGroup left_drive({MOTOR_LEFT_FRONT_PORT, MOTOR_LEFT_BACK_PORT}, pros::v5::MotorGears::blue);
pros::MotorGroup right_drive({MOTOR_RIGHT_FRONT_PORT, MOTOR_RIGHT_BACK_PORT}, pros::v5::MotorGears::blue);
pros::MotorGroup cascade({MOTOR_CASCADE_LEFT_PORT, MOTOR_CASCADE_RIGHT_PORT}, pros::v5::MotorGears::blue);

pros::Imu imu(IMU_PORT);
pros::Rotation odom_x(ODOM_X_PORT);
pros::Rotation odom_y(ODOM_Y_PORT);

pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::TrackingWheel horizontal_tracking(
    &odom_x,
    robot_config::HORIZONTAL_TRACKING_WHEEL_DIAMETER,
    robot_config::HORIZONTAL_TRACKING_OFFSET_IN
);
lemlib::TrackingWheel vertical_tracking(
    &odom_y,
    robot_config::VERTICAL_TRACKING_WHEEL_DIAMETER,
    robot_config::VERTICAL_TRACKING_OFFSET_IN
);

// [BEGIN_MAIN_CONTROLLER_SETTINGS]
lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);

lemlib::ControllerSettings lat_controller(
    14.3732f, // kP (Camera-in-the-Loop Tuned)
    0.0122f, // kI
    22.0000f, // kD
    1.8843f, // windupRange
    0.3500f, // smallError (in)
    75.0f, // smallErrorTimeout (ms)
    1.2500f, // largeError (in)
    260.0f, // largeErrorTimeout (ms)
    13.3750f  // slew
);

lemlib::ControllerSettings angular_controller(
    2.2242f, // kP (Camera-in-the-Loop Tuned)
    0.0150f, // kI
    23.4720f, // kD
    6.0000f, // windupRange
    0.6500f, // smallError (deg)
    70.0f, // smallErrorTimeout (ms)
    2.0000f, // largeError (deg)
    240.0f, // largeErrorTimeout (ms)
    0.0000f  // slew
);

lemlib::Drivetrain drivetrain(
    &left_drive,
    &right_drive,
    robot_config::DRIVETRAIN_TRACK_WIDTH_IN,
    robot_config::DRIVETRAIN_WHEEL_DIAMETER_IN,
    robot_config::DRIVETRAIN_RPM,
    4.2000f // horizontalDrift (Camera-in-the-Loop Tuned)
);
// [END_MAIN_CONTROLLER_SETTINGS]

tuning_harness::DynamicChassisWrapper chassis_wrapper(
    drivetrain,
    lat_controller,
    angular_controller,
    sensors
);

#define chassis (chassis_wrapper.get())

tuning_harness::CascadeLiftController cascade_controller(cascade);
tuning_harness::HardwareWatchdog hardware_watchdog;

// ============================================================================
// TELEMETRY & DISPATCHER STATE
// ============================================================================

namespace {

pros::Mutex cmd_mutex;
serial_bridge::HostCommandPacket latest_cmd{};
bool has_pending_cmd = false;
uint32_t last_valid_rx_ms = 0;
uint16_t tx_seq = 0;
bool tuning_session_active = false;
bool boundary_abort_latched = false;
uint8_t active_controller_id = 0;
serial_bridge::RobotMotionStatus current_motion_status = serial_bridge::RobotMotionStatus::IDLE;

// Hold-pose disturbance rejection state
bool hold_pose_active = false;
lemlib::Pose hold_target_pose(24.0f, 24.0f, 90.0f);
uint32_t hold_pose_end_ms = 0;

// Return-to-origin state
bool return_to_origin_pending = false;

float average_group_velocity_rpm(pros::Motor& m1, pros::Motor& m2) {
    return static_cast<float>((m1.get_actual_velocity() + m2.get_actual_velocity()) * 0.5);
}

float total_group_current_ma(pros::Motor& m1, pros::Motor& m2) {
    return static_cast<float>(m1.get_current_draw() + m2.get_current_draw());
}

float max_group_temp_c(pros::Motor& m1, pros::Motor& m2) {
    return static_cast<float>(std::max(m1.get_temperature(), m2.get_temperature()));
}

void coast_all_subsystems() {
    chassis.cancelAllMotions();
    left_drive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_drive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_drive.move(0);
    right_drive.move(0);
    cascade_controller.emergencyCoast();
}

void restore_brake_modes() {
    left_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    cascade_controller.clearCoast();
}

/**
 * @brief Non-blocking serial reader task for /dev/sin (stdin)
 * Decodes COBS-framed HostCommandPackets and validates CRC16.
 */
void serial_rx_task_fn(void*) {
    uint8_t frame_buf[256];
    uint8_t decoded_buf[256];
    size_t frame_idx = 0;

    int sin_fd = open("/dev/sin", O_RDONLY | O_NONBLOCK);
    if (sin_fd < 0) {
        sin_fd = STDIN_FILENO;
    }

    while (true) {
        uint8_t byte_in = 0;
        ssize_t n = read(sin_fd, &byte_in, 1);
        if (n > 0) {
            if (byte_in == 0x00) {
                if (frame_idx > 0) {
                    size_t dec_len = serial_bridge::cobs_decode(frame_buf, frame_idx, decoded_buf);
                    if (dec_len == sizeof(serial_bridge::HostCommandPacket)) {
                        serial_bridge::HostCommandPacket candidate{};
                        std::memcpy(&candidate, decoded_buf, sizeof(candidate));
                        if (serial_bridge::validate_command_packet(candidate)) {
                            std::lock_guard<pros::Mutex> lock(cmd_mutex);
                            latest_cmd = candidate;
                            has_pending_cmd = true;
                            last_valid_rx_ms = pros::millis();
                        }
                    }
                    frame_idx = 0;
                }
            } else {
                if (frame_idx < sizeof(frame_buf) - 1) {
                    frame_buf[frame_idx++] = byte_in;
                } else {
                    frame_idx = 0; // Overflow protection
                }
            }
        } else {
            pros::delay(2);
        }
    }
}

/**
 * @brief Execute LemLib Motion Command from HostCommandPacket
 */
void dispatch_motion_command(const serial_bridge::HostCommandPacket& pkt, uint32_t now_ms) {
    using serial_bridge::MotionCommandType;
    namespace TriggerFlags = serial_bridge::TriggerFlags;

    active_controller_id = pkt.controller_id;
    tuning_session_active = true;

    // 1. Check Emergency / Boundary Abort Flag
    if ((pkt.trigger_flags & TriggerFlags::BOUNDARY_ABORT) != 0 ||
        static_cast<MotionCommandType>(pkt.motion_type) == MotionCommandType::EMERGENCY_STOP) {
        boundary_abort_latched = true;
        hold_pose_active = false;
        return_to_origin_pending = false;
        coast_all_subsystems();
        current_motion_status = serial_bridge::RobotMotionStatus::WATCHDOG_TRIPPED;
        return;
    }

    boundary_abort_latched = false;
    hardware_watchdog.reset();
    restore_brake_modes();

    // 2. Sync Vision Ground-Truth Pose if requested
    if ((pkt.trigger_flags & TriggerFlags::SYNC_VISION_POSE) != 0) {
        chassis.setPose(pkt.vision_x, pkt.vision_y, pkt.vision_theta, false);
    }

    // 3. Reset Cascade Lift Origin if requested
    if ((pkt.trigger_flags & TriggerFlags::RESET_LIFT_ORIGIN) != 0) {
        cascade_controller.resetOrigin();
    }

    // 4. Hot-reload LemLib ControllerSettings or Cascade Lift PID gains if requested
    if ((pkt.trigger_flags & TriggerFlags::HOT_RELOAD_GAINS) != 0) {
        if (static_cast<serial_bridge::ControllerId>(pkt.controller_id) ==
            serial_bridge::ControllerId::CASCADE_LIFT) {
            bool sign_flip = (pkt.trigger_flags & TriggerFlags::SIGN_FLIP_RESET) != 0;
            cascade_controller.hotReload(
                pkt.kp,
                pkt.ki,
                pkt.kd,
                pkt.windup_range,
                sign_flip,
                pkt.small_error,
                pkt.small_error_timeout,
                pkt.large_error,
                pkt.large_error_timeout,
                pkt.gravity_ff_mv
            );
        } else {
            chassis_wrapper.applyCommandGains(pkt);
        }
    }

    // 5. Execute requested motion if EXECUTE_MOTION flag is set
    if ((pkt.trigger_flags & TriggerFlags::EXECUTE_MOTION) == 0) {
        return;
    }

    int timeout = (pkt.timeout_ms > 0) ? static_cast<int>(pkt.timeout_ms) : 3000;
    float max_spd = (pkt.max_speed > 1.0f) ? pkt.max_speed : 127.0f;
    float min_spd = (pkt.min_speed >= 0.0f) ? pkt.min_speed : 0.0f;
    float early_exit = (pkt.early_exit_range >= 0.0f) ? pkt.early_exit_range : 0.0f;

    hold_pose_active = false;
    chassis.cancelAllMotions();

    auto cmd_type = static_cast<MotionCommandType>(pkt.motion_type);
    switch (cmd_type) {
        case MotionCommandType::HEARTBEAT:
            break;

        case MotionCommandType::MOVE_TO_POINT_FWD:
            chassis_wrapper.restoreAngularGains();
            chassis.moveToPoint(
                pkt.target_x,
                pkt.target_y,
                timeout,
                {
                    .forwards = true,
                    .maxSpeed = max_spd,
                    .minSpeed = min_spd,
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::MOVE_TO_POINT_REV:
            chassis_wrapper.restoreAngularGains();
            chassis.moveToPoint(
                pkt.target_x,
                pkt.target_y,
                timeout,
                {
                    .forwards = false,
                    .maxSpeed = max_spd,
                    .minSpeed = min_spd,
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::TURN_TO_HEADING:
            chassis_wrapper.restoreAngularGains();
            chassis.turnToHeading(
                pkt.target_theta,
                timeout,
                {
                    .direction = AngularDirection::AUTO,
                    .maxSpeed = static_cast<int>(max_spd),
                    .minSpeed = static_cast<int>(min_spd),
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::TURN_TO_POINT:
            chassis_wrapper.restoreAngularGains();
            chassis.turnToPoint(
                pkt.target_x,
                pkt.target_y,
                timeout,
                {
                    .forwards = true,
                    .direction = AngularDirection::AUTO,
                    .maxSpeed = static_cast<int>(max_spd),
                    .minSpeed = static_cast<int>(min_spd),
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::MOVE_TO_POSE: {
            chassis_wrapper.restoreAngularGains();
            float lead = (pkt.lead > 0.0f && pkt.lead <= 1.0f)
                ? pkt.lead
                : chassis_wrapper.getBoomerangLead();
            float h_drift = (pkt.horizontal_drift > 0.0f)
                ? pkt.horizontal_drift
                : chassis_wrapper.getHorizontalDrift();
            chassis.moveToPose(
                pkt.target_x,
                pkt.target_y,
                pkt.target_theta,
                timeout,
                {
                    .forwards = true,
                    .horizontalDrift = h_drift,
                    .lead = lead,
                    .maxSpeed = max_spd,
                    .minSpeed = min_spd,
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;
        }

        case MotionCommandType::SWING_TO_HEADING_LEFT:
            chassis_wrapper.activateSwingGains();
            chassis.swingToHeading(
                pkt.target_theta,
                DriveSide::LEFT,
                timeout,
                {
                    .direction = AngularDirection::AUTO,
                    .maxSpeed = max_spd,
                    .minSpeed = min_spd,
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::SWING_TO_HEADING_RIGHT:
            chassis_wrapper.activateSwingGains();
            chassis.swingToHeading(
                pkt.target_theta,
                DriveSide::RIGHT,
                timeout,
                {
                    .direction = AngularDirection::AUTO,
                    .maxSpeed = max_spd,
                    .minSpeed = min_spd,
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::SWING_TO_POINT_LEFT:
            chassis_wrapper.activateSwingGains();
            chassis.swingToPoint(
                pkt.target_x,
                pkt.target_y,
                DriveSide::LEFT,
                timeout,
                {
                    .forwards = true,
                    .direction = AngularDirection::AUTO,
                    .maxSpeed = max_spd,
                    .minSpeed = min_spd,
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::SWING_TO_POINT_RIGHT:
            chassis_wrapper.activateSwingGains();
            chassis.swingToPoint(
                pkt.target_x,
                pkt.target_y,
                DriveSide::RIGHT,
                timeout,
                {
                    .forwards = true,
                    .direction = AngularDirection::AUTO,
                    .maxSpeed = max_spd,
                    .minSpeed = min_spd,
                    .earlyExitRange = early_exit
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::CASCADE_LIFT_STEP:
            cascade_controller.setTargetHeightInches(pkt.target_lift_height, timeout, now_ms);
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::RETURN_TO_ORIGIN:
            chassis_wrapper.restoreAngularGains();
            if ((pkt.trigger_flags & TriggerFlags::SYNC_VISION_POSE) != 0) {
                chassis.setPose(pkt.vision_x, pkt.vision_y, pkt.vision_theta, false);
            }
            chassis.moveToPose(
                0.0f,
                0.0f,
                0.0f,
                timeout,
                {
                    .forwards = false,
                    .horizontalDrift = chassis_wrapper.getHorizontalDrift(),
                    .lead = 0.45f,
                    .maxSpeed = 100.0f,
                    .minSpeed = 0.0f,
                    .earlyExitRange = 0.0f
                },
                true
            );
            return_to_origin_pending = true;
            current_motion_status = serial_bridge::RobotMotionStatus::RETURNING_TO_ORIGIN;
            break;

        case MotionCommandType::HOLD_POSE_DISTURBANCE:
            chassis_wrapper.restoreAngularGains();
            hold_target_pose = lemlib::Pose(pkt.target_x, pkt.target_y, pkt.target_theta);
            hold_pose_end_ms = now_ms + static_cast<uint32_t>(timeout);
            hold_pose_active = true;
            chassis.lateralPID.reset();
            chassis.angularPID.reset();
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::CHAINED_ENDURANCE_LAP:
            chassis_wrapper.restoreAngularGains();
            chassis.moveToPose(
                pkt.target_x,
                pkt.target_y,
                pkt.target_theta,
                timeout,
                {
                    .forwards = true,
                    .horizontalDrift = chassis_wrapper.getHorizontalDrift(),
                    .lead = chassis_wrapper.getBoomerangLead(),
                    .maxSpeed = max_spd,
                    .minSpeed = (min_spd > 0.0f) ? min_spd : 45.0f,
                    .earlyExitRange = (early_exit > 0.0f) ? early_exit : 2.5f
                },
                true
            );
            current_motion_status = serial_bridge::RobotMotionStatus::IN_MOTION;
            break;

        case MotionCommandType::EMERGENCY_STOP:
            coast_all_subsystems();
            current_motion_status = serial_bridge::RobotMotionStatus::WATCHDOG_TRIPPED;
            break;
    }
}

/**
 * @brief Deterministic 100 Hz (10 ms) Control, Watchdog, and Telemetry Transceiver Task
 */
void deterministic_100hz_bridge_task_fn(void*) {
    uint32_t wake_time = pros::millis();
    uint8_t encoded_tx[128];

    int sout_fd = open("/dev/sout", O_WRONLY | O_NONBLOCK);
    if (sout_fd < 0) {
        sout_fd = STDOUT_FILENO;
    }

    while (true) {
        uint32_t now_ms = pros::millis();

        // 1. Process incoming HostCommandPacket if available
        serial_bridge::HostCommandPacket cmd_copy{};
        bool got_cmd = false;
        uint32_t last_rx_snapshot = 0;
        {
            std::lock_guard<pros::Mutex> lock(cmd_mutex);
            last_rx_snapshot = last_valid_rx_ms;
            if (has_pending_cmd) {
                cmd_copy = latest_cmd;
                has_pending_cmd = false;
                got_cmd = true;
            }
        }

        if (got_cmd) {
            dispatch_motion_command(cmd_copy, now_ms);
        }

        // 2. Read Hardware Sensors
        lemlib::Pose pose = chassis.getPose();
        float raw_imu = static_cast<float>(imu.get_heading());
        if ( std::isnan(raw_imu) || std::isinf(raw_imu) ) {
            raw_imu = 0.0f;
        }

        float l_vel = average_group_velocity_rpm(left_front, left_back);
        float r_vel = average_group_velocity_rpm(right_front, right_back);
        float l_cur = total_group_current_ma(left_front, left_back);
        float r_cur = total_group_current_ma(right_front, right_back);
        float l_tmp = max_group_temp_c(left_front, left_back);
        float r_tmp = max_group_temp_c(right_front, right_back);

        float lift_pos_in = cascade_controller.getHeightInches();
        float lift_vel = average_group_velocity_rpm(cascade_left, cascade_right);
        float lift_cur = total_group_current_ma(cascade_left, cascade_right);
        float lift_tmp = max_group_temp_c(cascade_left, cascade_right);

        float max_temp = std::max({l_tmp, r_tmp, lift_tmp});

        // 3. Evaluate Hardware and Boundary Watchdog
        uint8_t wd_flags = hardware_watchdog.evaluate(
            now_ms,
            last_rx_snapshot,
            tuning_session_active && (chassis.isInMotion() || cascade_controller.isActive() || hold_pose_active),
            boundary_abort_latched,
            l_cur,
            r_cur,
            l_vel,
            r_vel,
            lift_cur,
            lift_vel,
            max_temp
        );

        if (hardware_watchdog.isTripped()) {
            hold_pose_active = false;
            return_to_origin_pending = false;
            coast_all_subsystems();
            if ((wd_flags & (serial_bridge::WatchdogFlags::DRIVETRAIN_STALL |
                             serial_bridge::WatchdogFlags::LIFT_STALL)) != 0) {
                current_motion_status = serial_bridge::RobotMotionStatus::STALL_PROTECT;
            } else {
                current_motion_status = serial_bridge::RobotMotionStatus::WATCHDOG_TRIPPED;
            }
        } else {
            // 4. Update Cascade Lift PID + Gravity Feedforward Loop
            cascade_controller.update(now_ms);

            // 5. Update Stationary Impulse Disturbance Rejection Hold if active
            if (hold_pose_active) {
                if (now_ms >= hold_pose_end_ms) {
                    hold_pose_active = false;
                    left_drive.move(0);
                    right_drive.move(0);
                    current_motion_status = serial_bridge::RobotMotionStatus::IDLE;
                } else {
                    float dx = hold_target_pose.x - pose.x;
                    float dy = hold_target_pose.y - pose.y;
                    float rad = pose.theta * static_cast<float>(M_PI) / 180.0f;
                    // Project error onto robot's forward axis
                    float lat_err = dx * std::sin(rad) + dy * std::cos(rad);
                    float ang_err = std::remainder(hold_target_pose.theta - pose.theta, 360.0f);

                    float lat_out = std::clamp(chassis.lateralPID.update(lat_err), -127.0f, 127.0f);
                    float ang_out = std::clamp(chassis.angularPID.update(ang_err), -127.0f, 127.0f);

                    left_drive.move(static_cast<int32_t>(std::clamp(lat_out + ang_out, -127.0f, 127.0f)));
                    right_drive.move(static_cast<int32_t>(std::clamp(lat_out - ang_out, -127.0f, 127.0f)));
                }
            } else if (!chassis.isInMotion() && !cascade_controller.isActive()) {
                if (return_to_origin_pending) {
                    return_to_origin_pending = false;
                }
                if (current_motion_status == serial_bridge::RobotMotionStatus::IN_MOTION ||
                    current_motion_status == serial_bridge::RobotMotionStatus::RETURNING_TO_ORIGIN) {
                    current_motion_status = serial_bridge::RobotMotionStatus::IDLE;
                }
            }
        }

        // 6. Populate and Transmit COBS-Framed RobotTelemetryPacket at 100 Hz
        serial_bridge::RobotTelemetryPacket telem{};
        telem.seq = tx_seq++;
        telem.timestamp_ms = now_ms;
        telem.pose_x = pose.x;
        telem.pose_y = pose.y;
        telem.pose_theta = pose.theta;
        telem.raw_imu_heading = raw_imu;
        telem.left_vel_rpm = l_vel;
        telem.right_vel_rpm = r_vel;
        telem.left_current_ma = l_cur;
        telem.right_current_ma = r_cur;
        telem.left_temp_c = l_tmp;
        telem.right_temp_c = r_tmp;
        telem.cascade_pos_in = lift_pos_in;
        telem.cascade_vel_rpm = lift_vel;
        telem.cascade_current_ma = lift_cur;
        telem.cascade_temp_c = lift_tmp;
        telem.battery_voltage_mv = static_cast<uint32_t>(pros::battery::get_voltage());
        telem.motion_status = static_cast<uint8_t>(current_motion_status);
        telem.active_controller_id = active_controller_id;
        telem.watchdog_flags = wd_flags;
        telem.reserved = 0;
        serial_bridge::finalize_telemetry_packet(telem);

        size_t enc_len = serial_bridge::cobs_encode(
            reinterpret_cast<const uint8_t*>(&telem),
            sizeof(telem),
            encoded_tx
        );
        encoded_tx[enc_len] = 0x00; // COBS frame delimiter
        write(sout_fd, encoded_tx, enc_len + 1);

        pros::Task::delay_until(&wake_time, 10); // Exact 100 Hz (10 ms) deterministic period
    }
}

} // namespace

// ============================================================================
// PROS LIFECYCLE CALLBACKS
// ============================================================================

void initialize() {
    enforce_approval_gate();

    std::setvbuf(stdin, nullptr, _IONBF, 0);
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    pros::lcd::initialize();

    left_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    end_effector_rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    left_drive.move(0);
    right_drive.move(0);
    cascade.move(0);
    intake.move_voltage(0);
    end_effector.move_voltage(0);

    chassis_wrapper.calibrateOnce();
    chassis.setPose(0.0f, 0.0f, 0.0f);

    static pros::Task rx_task(serial_rx_task_fn, nullptr, TASK_PRIORITY_DEFAULT + 1, TASK_STACK_DEPTH_DEFAULT, "SerialRX");
    static pros::Task tx_task(deterministic_100hz_bridge_task_fn, nullptr, TASK_PRIORITY_DEFAULT + 2, TASK_STACK_DEPTH_DEFAULT, "Bridge100Hz");

    static pros::Task lcd_task([] {
        while (true) {
            lemlib::Pose p = chassis.getPose();
            pros::lcd::print(0, "22050X Pose: X=%6.2f Y=%6.2f T=%6.2f", p.x, p.y, p.theta);
            pros::lcd::print(1, "Lift: %5.2f in | Batt: %d mV", cascade_controller.getHeightInches(), pros::battery::get_voltage());
            pros::lcd::print(2, "Status: %u | WD: 0x%02X", static_cast<unsigned>(current_motion_status), hardware_watchdog.getFlags());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    if (!is_build_approved()) return;
    // In autonomous mode, if no host tuning session is active, execute calibrated baseline verify move
    if (!tuning_session_active) {
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 24, 3000);
        chassis.waitUntilDone();
    }
}

void opcontrol() {
    if (!is_build_approved()) return;

    while (true) {
        // When a host serial tuning command is actively running, yield manual joystick control
        if (!tuning_session_active ||
            (current_motion_status == serial_bridge::RobotMotionStatus::IDLE &&
             (pros::millis() - last_valid_rx_ms > 500))) {

            int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

            if (std::abs(left_y) > 5 || std::abs(right_x) > 5) {
                tuning_session_active = false;
                left_drive.move(left_y + right_x);
                right_drive.move(left_y - right_x);
            } else if (!chassis.isInMotion() && !hold_pose_active) {
                left_drive.move(0);
                right_drive.move(0);
            }

            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                cascade_controller.setTargetHeightInches(
                    std::min(cascade_controller.getHeightInches() + 1.5f, robot_config::CASCADE_MAX_HEIGHT_IN),
                    500,
                    pros::millis()
                );
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                cascade_controller.setTargetHeightInches(
                    std::max(cascade_controller.getHeightInches() - 1.5f, 0.0f),
                    500,
                    pros::millis()
                );
            }

            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake.move_voltage(-12000);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake.move_voltage(12000);
            } else {
                intake.move_voltage(0);
            }

            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                chassis.setPose(0, 0, 0);
                master.rumble(".");
            }
        }

        pros::delay(20);
    }
}
