#pragma once

#include "main.h"
#include "pid.hpp"
#include "tuner_config.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

/**
 * @brief Selectable tuning subsystem modes
 */
enum class TuningMode {
    DRIVE_LINEAR = 0,   // Drivetrain distance (inches)
    DRIVE_ANGULAR = 1,  // Drivetrain heading turn (degrees)
    MECHANISM = 2,      // Single motor or mechanism position (degrees)
    MODE_COUNT = 3
};

/**
 * @brief Selectable parameter to adjust with D-pad
 */
enum class TuningParam {
    KP = 0,
    KI = 1,
    KD = 2,
    TARGET = 3,
    PARAM_COUNT = 4
};

/**
 * @brief Interactive PID Tuner and Closed-Loop Drivetrain Controller
 *
 * Provides live parameter adjustment from V5 Controller, real-time
 * telemetry logging over serial/terminal, LLEMU Brain screen diagnostics,
 * step response profiling, heading-corrected straight driving,
 * in-place turns, and multi-step autonomous test routines.
 */
class PIDTuner {
public:
    PIDTuner()
        : left_drive_(tuner_config::LEFT_DRIVE_PORTS, tuner_config::DRIVETRAIN_GEARSET),
          right_drive_(tuner_config::RIGHT_DRIVE_PORTS, tuner_config::DRIVETRAIN_GEARSET),
          cascade_(tuner_config::CASCADE_PORTS, tuner_config::CASCADE_GEARSET),
          mechanism_(tuner_config::MECHANISM_PORT, tuner_config::MECHANISM_GEARSET),
          imu_(tuner_config::IMU_PORT),
          master_(pros::E_CONTROLLER_MASTER) {
        
        // Load default profiles from tuner_config
        linear_pid_ = PID(tuner_config::get_default_drive_linear_gains(), tuner_config::get_default_drive_linear_settle());
        angular_pid_ = PID(tuner_config::get_default_drive_angular_gains(), tuner_config::get_default_drive_angular_settle());
        mech_pid_ = PID(tuner_config::get_default_mechanism_gains(), tuner_config::get_default_mechanism_settle());

        targets_[static_cast<size_t>(TuningMode::DRIVE_LINEAR)] = tuner_config::DEFAULT_DRIVE_LINEAR_STEP;
        targets_[static_cast<size_t>(TuningMode::DRIVE_ANGULAR)] = tuner_config::DEFAULT_DRIVE_ANGULAR_STEP;
        targets_[static_cast<size_t>(TuningMode::MECHANISM)] = tuner_config::DEFAULT_MECHANISM_STEP;
    }

    /**
     * @brief Initialize sensors and LCD display
     */
    void initialize() {
        pros::lcd::initialize();
        pros::lcd::set_text(0, "=== 22050X PID TUNER ===");
        pros::lcd::set_text(1, "Calibrating IMU & Motors...");

        left_drive_.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        right_drive_.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        cascade_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        mechanism_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        left_drive_.tare_position();
        right_drive_.tare_position();
        cascade_.tare_position();
        mechanism_.tare_position();

        if (tuner_config::IMU_PORT > 0) {
            imu_.reset();
            int attempts = 0;
            while (imu_.is_calibrating() && attempts < 350) {
                pros::delay(10);
                attempts++;
            }
        }

        update_brain_lcd();
        printf("\n======================================================\n");
        printf(" 22050X PID TUNER & DRIVE CONTROLLER INITIALIZED\n");
        printf(" Hardware Ports:\n");
        printf("   Left Drive:  [20, -10]\n");
        printf("   Right Drive: [-7, 9]\n");
        printf("   Cascade:     [1, -2]\n");
        printf("   IMU Smart:   Port 3\n");
        printf(" Format: [CSV] time_ms,target,measured,error,output\n");
        printf("======================================================\n\n");
    }

    // ========================================================================
    // HIGH-LEVEL PID DRIVE FUNCTIONS (Use these to test driving your robot!)
    // ========================================================================

    /**
     * @brief Drive the robot a specific linear distance in inches using the tuned Linear PID.
     * Incorporates automatic heading stabilization using the IMU to guarantee driving straight.
     *
     * @param target_inches Distance to drive (positive = forward, negative = reverse)
     * @param timeout_ms Maximum allowed time before aborting (default: 3500ms)
     * @param use_heading_lock Keep robot locked onto its initial heading while driving
     * @return true if settled cleanly within tolerance, false if timed out or aborted
     */
    bool drive_distance(double target_inches, int timeout_ms = 3500, bool use_heading_lock = true) {
        test_running_ = true;
        abort_requested_ = false;

        left_drive_.tare_position();
        right_drive_.tare_position();

        double start_heading = get_current_heading();
        uint32_t start_time = pros::millis();
        uint32_t prev_time = start_time;

        SettleSettings settle = linear_pid_.get_settle();
        settle.max_time_ms = timeout_ms;
        linear_pid_.set_settle(settle);
        linear_pid_.start_test(target_inches, 0.0, start_time);

        printf("\n[DRIVE_DISTANCE] Starting move: %.2f inches (Heading lock: %s, Start: %.2f deg)\n",
               target_inches, use_heading_lock ? "ON" : "OFF", start_heading);
        printf("[CSV] time_ms,target,measured,error,output\n");

        while (true) {
            uint32_t current_time = pros::millis();
            double dt = (current_time - prev_time) / 1000.0;
            if (dt <= 0.0) dt = 0.01;
            prev_time = current_time;

            // Check manual abort from controller
            if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                abort_requested_ = true;
                printf("[DRIVE_DISTANCE] Aborted by user.\n");
                break;
            }

            // Current linear distance traveled in inches
            double measured_in = get_linear_distance_inches();
            double linear_err = target_inches - measured_in;
            double linear_out = linear_pid_.compute(target_inches, measured_in, dt);

            // Heading correction PID to prevent drift
            double turn_out = 0.0;
            if (use_heading_lock) {
                double current_heading = get_current_heading();
                double heading_error = std::remainder(start_heading - current_heading, 360.0);
                turn_out = angular_pid_.get_gains().kp * heading_error;
                turn_out = std::clamp(turn_out, -40.0, 40.0); // Clamp heading correction
            }

            // Apply motor output
            left_drive_.move(linear_out + turn_out);
            right_drive_.move(linear_out - turn_out);

            // Log CSV
            uint32_t elapsed_ms = current_time - start_time;
            printf("[CSV] %lu,%.3f,%.3f,%.3f,%.2f\n",
                   (unsigned long)elapsed_ms, target_inches, measured_in, linear_err, linear_out);

            // Check settle condition
            bool done = linear_pid_.update_settle(linear_err, measured_in, current_time);
            if (done) {
                break;
            }

            pros::delay(10);
        }

        left_drive_.brake();
        right_drive_.brake();

        const StepMetrics& m = linear_pid_.get_metrics();
        printf("[DRIVE_DISTANCE] Done: %s | Settled: %s | Time: %lums | FinalErr: %.3f in\n\n",
               abort_requested_ ? "ABORTED" : "FINISHED",
               m.settled ? "YES" : "NO",
               (unsigned long)m.total_time_ms,
               m.steady_state_error);

        test_running_ = false;
        return m.settled && !abort_requested_;
    }

    /**
     * @brief Turn the robot in place to a specific target heading using the tuned Angular PID.
     *
     * @param target_degrees Target absolute heading in degrees (0 - 360 or relative)
     * @param timeout_ms Maximum allowed time before aborting (default: 3000ms)
     * @return true if settled cleanly, false if timed out or aborted
     */
    bool turn_to_angle(double target_degrees, int timeout_ms = 3000) {
        test_running_ = true;
        abort_requested_ = false;

        uint32_t start_time = pros::millis();
        uint32_t prev_time = start_time;
        double initial_heading = get_current_heading();

        SettleSettings settle = angular_pid_.get_settle();
        settle.max_time_ms = timeout_ms;
        angular_pid_.set_settle(settle);
        angular_pid_.start_test(target_degrees, initial_heading, start_time);

        printf("\n[TURN_TO_ANGLE] Starting turn: %.2f deg (Initial: %.2f deg)\n", target_degrees, initial_heading);
        printf("[CSV] time_ms,target,measured,error,output\n");

        while (true) {
            uint32_t current_time = pros::millis();
            double dt = (current_time - prev_time) / 1000.0;
            if (dt <= 0.0) dt = 0.01;
            prev_time = current_time;

            if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                abort_requested_ = true;
                printf("[TURN_TO_ANGLE] Aborted by user.\n");
                break;
            }

            double current_heading = get_current_heading();
            double error = std::remainder(target_degrees - current_heading, 360.0);
            double output = angular_pid_.compute(target_degrees, target_degrees - error, dt);

            // Clockwise turn: left forward, right backward
            left_drive_.move(output);
            right_drive_.move(-output);

            uint32_t elapsed_ms = current_time - start_time;
            printf("[CSV] %lu,%.3f,%.3f,%.3f,%.2f\n",
                   (unsigned long)elapsed_ms, target_degrees, current_heading, error, output);

            bool done = angular_pid_.update_settle(error, target_degrees - error, current_time);
            if (done) {
                break;
            }

            pros::delay(10);
        }

        left_drive_.brake();
        right_drive_.brake();

        const StepMetrics& m = angular_pid_.get_metrics();
        printf("[TURN_TO_ANGLE] Done: %s | Settled: %s | Time: %lums | FinalErr: %.3f deg\n\n",
               abort_requested_ ? "ABORTED" : "FINISHED",
               m.settled ? "YES" : "NO",
               (unsigned long)m.total_time_ms,
               m.steady_state_error);

        test_running_ = false;
        return m.settled && !abort_requested_;
    }

    /**
     * @brief Move cascade lift / mechanism to target position in motor degrees
     */
    bool move_mechanism(double target_degrees, int timeout_ms = 3000) {
        test_running_ = true;
        abort_requested_ = false;

        uint32_t start_time = pros::millis();
        uint32_t prev_time = start_time;
        double initial_pos = cascade_.get_position();

        SettleSettings settle = mech_pid_.get_settle();
        settle.max_time_ms = timeout_ms;
        mech_pid_.set_settle(settle);
        mech_pid_.start_test(target_degrees, initial_pos, start_time);

        printf("\n[MOVE_MECHANISM] Target: %.2f deg\n", target_degrees);

        while (true) {
            uint32_t current_time = pros::millis();
            double dt = (current_time - prev_time) / 1000.0;
            if (dt <= 0.0) dt = 0.01;
            prev_time = current_time;

            if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                abort_requested_ = true;
                break;
            }

            double current_pos = cascade_.get_position();
            double error = target_degrees - current_pos;
            double output = mech_pid_.compute(target_degrees, current_pos, dt);

            cascade_.move(output);

            bool done = mech_pid_.update_settle(error, current_pos, current_time);
            if (done) break;

            pros::delay(10);
        }

        cascade_.brake();
        test_running_ = false;
        return mech_pid_.get_metrics().settled && !abort_requested_;
    }

    /**
     * @brief Run a back-and-forth drive test (Forward X inches -> pause -> Reverse X inches)
     * Tests linear PID accuracy, settling, and return-to-origin precision.
     */
    void run_forward_backward_test(double distance_inches = 24.0) {
        printf("\n=========================================\n");
        printf(" STARTING FORWARD-BACKWARD PID TEST\n");
        printf("=========================================\n");
        master_.rumble("- .");

        // Forward leg
        printf("Leg 1: Driving Forward %.1f inches...\n", distance_inches);
        drive_distance(distance_inches, 3500, true);
        if (abort_requested_) return;
        pros::delay(500);

        // Reverse leg
        printf("Leg 2: Reversing %.1f inches back to origin...\n", distance_inches);
        drive_distance(-distance_inches, 3500, true);
        if (abort_requested_) return;

        printf("=========================================\n");
        printf(" FORWARD-BACKWARD TEST COMPLETE\n");
        printf("=========================================\n\n");
        master_.rumble(". - .");
    }

    /**
     * @brief Run a 4-corner box test routine (Drive 24" -> Turn 90° -> repeat 4x).
     * Tests combined Linear + Angular PID loops in a real autonomous routine.
     */
    void run_box_test(double side_inches = 24.0) {
        printf("\n=========================================\n");
        printf(" STARTING 4-CORNER BOX TEST (%.1f in)\n", side_inches);
        printf("=========================================\n");
        master_.rumble("- -");

        if (tuner_config::IMU_PORT > 0) {
            imu_.set_rotation(0.0);
        }

        double angles[4] = {90.0, 180.0, 270.0, 0.0};

        for (int i = 0; i < 4; i++) {
            printf("\nSide %d/4: Driving %.1f inches...\n", i + 1, side_inches);
            drive_distance(side_inches, 3500, true);
            if (abort_requested_) return;
            pros::delay(400);

            printf("Corner %d/4: Turning to %.1f degrees...\n", i + 1, angles[i]);
            turn_to_angle(angles[i], 3000);
            if (abort_requested_) return;
            pros::delay(400);
        }

        printf("=========================================\n");
        printf(" BOX TEST COMPLETE - RETURNED TO ORIGIN\n");
        printf("=========================================\n\n");
        master_.rumble(". - .");
    }

    /**
     * @brief Execute the single step test for the currently active tuning mode
     */
    void execute_step_test() {
        double target = get_active_target();
        switch (current_mode_) {
            case TuningMode::DRIVE_LINEAR:
                drive_distance(target, 3500, true);
                break;
            case TuningMode::DRIVE_ANGULAR:
                turn_to_angle(target, 3000);
                break;
            case TuningMode::MECHANISM:
                move_mechanism(target, 3000);
                break;
            default:
                break;
        }
    }

    /**
     * @brief Main interactive tuning loop. Call inside opcontrol().
     */
    void run_interactive_loop() {
        uint32_t last_controller_update = 0;
        uint32_t last_lcd_update = 0;

        while (true) {
            uint32_t now = pros::millis();

            // 1. Process controller button inputs
            handle_controller_input();

            // 2. If a test is not running, allow manual driving to reposition the robot
            if (!test_running_) {
                handle_manual_drive();
            }

            // 3. Update V5 Controller screen (throttled to 120ms to prevent packet drops)
            if (now - last_controller_update >= 120) {
                update_controller_screen();
                last_controller_update = now;
            }

            // 4. Update Brain LCD screen (every 100ms)
            if (now - last_lcd_update >= 100) {
                update_brain_lcd();
                last_lcd_update = now;
            }

            pros::delay(10);
        }
    }

    // Accessors for autonomous integration
    PID& get_linear_pid() { return linear_pid_; }
    PID& get_angular_pid() { return angular_pid_; }
    PID& get_mech_pid() { return mech_pid_; }
    pros::MotorGroup& left_drive() { return left_drive_; }
    pros::MotorGroup& right_drive() { return right_drive_; }
    pros::MotorGroup& cascade() { return cascade_; }
    pros::Imu& imu() { return imu_; }

private:
    pros::MotorGroup left_drive_;
    pros::MotorGroup right_drive_;
    pros::MotorGroup cascade_;
    pros::Motor mechanism_;
    pros::Imu imu_;
    pros::Controller master_;

    PID linear_pid_;
    PID angular_pid_;
    PID mech_pid_;

    double targets_[static_cast<size_t>(TuningMode::MODE_COUNT)] = {24.0, 90.0, 360.0};

    TuningMode current_mode_ = TuningMode::DRIVE_LINEAR;
    TuningParam current_param_ = TuningParam::KP;

    const std::vector<double> increments_ = {0.001, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0};
    size_t inc_idx_ = 3; // Default increment: 0.1

    bool test_running_ = false;
    bool abort_requested_ = false;

    PID& get_active_pid() {
        switch (current_mode_) {
            case TuningMode::DRIVE_LINEAR: return linear_pid_;
            case TuningMode::DRIVE_ANGULAR: return angular_pid_;
            case TuningMode::MECHANISM: return mech_pid_;
            default: return linear_pid_;
        }
    }

    double& get_active_target() {
        return targets_[static_cast<size_t>(current_mode_)];
    }

    double get_increment() const {
        return increments_[inc_idx_];
    }

    const char* get_mode_name(TuningMode mode) const {
        switch (mode) {
            case TuningMode::DRIVE_LINEAR: return "DRV_LIN";
            case TuningMode::DRIVE_ANGULAR: return "DRV_ANG";
            case TuningMode::MECHANISM: return "MECH";
            default: return "UNKNOWN";
        }
    }

    const char* get_param_name(TuningParam param) const {
        switch (param) {
            case TuningParam::KP: return "Kp";
            case TuningParam::KI: return "Ki";
            case TuningParam::KD: return "Kd";
            case TuningParam::TARGET: return "Tgt";
            default: return "?";
        }
    }

    double get_linear_distance_inches() {
        double l_deg = left_drive_.get_position();
        double r_deg = right_drive_.get_position();
        double avg_deg = (l_deg + r_deg) * 0.5;
        return tuner_config::motor_deg_to_inches(avg_deg);
    }

    double get_current_heading() {
        if (tuner_config::IMU_PORT > 0 && imu_.is_installed()) {
            return imu_.get_rotation(); // Continuous heading degrees
        } else {
            // Differential wheel heading fallback
            double l_in = tuner_config::motor_deg_to_inches(left_drive_.get_position());
            double r_in = tuner_config::motor_deg_to_inches(right_drive_.get_position());
            double diff_in = r_in - l_in;
            return (diff_in / tuner_config::TRACK_WIDTH_INCHES) * (180.0 / tuner_config::PI);
        }
    }

    void handle_controller_input() {
        // Button Y: Cycle Tuning Mode (Drive Linear -> Drive Angular -> Mechanism)
        if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            current_mode_ = static_cast<TuningMode>((static_cast<int>(current_mode_) + 1) % static_cast<int>(TuningMode::MODE_COUNT));
            master_.rumble(".");
        }

        // Button X: Cycle Tuning Parameter (Kp -> Ki -> Kd -> Target)
        if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            current_param_ = static_cast<TuningParam>((static_cast<int>(current_param_) + 1) % static_cast<int>(TuningParam::PARAM_COUNT));
        }

        // DPAD LEFT: Decrease increment step size
        if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            if (inc_idx_ > 0) inc_idx_--;
        }

        // DPAD RIGHT: Increase increment step size
        if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (inc_idx_ < increments_.size() - 1) inc_idx_++;
        }

        // DPAD UP / DOWN: Increase / decrease active parameter value
        double delta = get_increment();
        if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            adjust_param(delta);
        } else if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            adjust_param(-delta);
        }

        // Combined Test Triggers (Hold R1 while pressing buttons):
        if (master_.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                run_forward_backward_test(get_active_target());
                return;
            }
            if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                run_box_test(get_active_target());
                return;
            }
        }

        // Button A: Run standard Step Response Test for current profile
        if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            execute_step_test();
        }
    }

    void adjust_param(double delta) {
        PID& active_pid = get_active_pid();
        PIDGains& g = active_pid.gains();

        switch (current_param_) {
            case TuningParam::KP:
                g.kp = std::max(0.0, g.kp + delta);
                break;
            case TuningParam::KI:
                g.ki = std::max(0.0, g.ki + delta);
                break;
            case TuningParam::KD:
                g.kd = std::max(0.0, g.kd + delta);
                break;
            case TuningParam::TARGET:
                get_active_target() += delta;
                break;
            default:
                break;
        }
    }

    void handle_manual_drive() {
        int left_y = master_.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_x = master_.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (std::abs(left_y) < 5) left_y = 0;
        if (std::abs(right_x) < 5) right_x = 0;

        if (left_y != 0 || right_x != 0) {
            left_drive_.move(left_y + right_x);
            right_drive_.move(left_y - right_x);
        } else {
            left_drive_.move(0);
            right_drive_.move(0);
        }

        // Cascade lift manual jog on L1 / L2
        if (master_.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cascade_.move(127);
        } else if (master_.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            cascade_.move(-127);
        } else {
            cascade_.move(0);
        }
    }

    void update_controller_screen() {
        PID& active_pid = get_active_pid();
        const PIDGains& g = active_pid.get_gains();
        double tgt = get_active_target();
        double inc = get_increment();

        char line0[32];
        char line1[32];
        char line2[32];

        const char* sel = get_param_name(current_param_);
        snprintf(line0, sizeof(line0), "[%s] SEL:%s", get_mode_name(current_mode_), sel);
        snprintf(line1, sizeof(line1), "P:%.2f I:%.3f D:%.2f", g.kp, g.ki, g.kd);
        snprintf(line2, sizeof(line2), "Tgt:%.1f Inc:%.3f", tgt, inc);

        master_.print(0, 0, "%-20s", line0);
        pros::delay(30);
        master_.print(1, 0, "%-20s", line1);
        pros::delay(30);
        master_.print(2, 0, "%-20s", line2);
    }

    void update_brain_lcd() {
        PID& active_pid = get_active_pid();
        const PIDGains& g = active_pid.get_gains();
        const StepMetrics& m = active_pid.get_metrics();
        double current_val = (current_mode_ == TuningMode::DRIVE_LINEAR) ? get_linear_distance_inches() :
                             (current_mode_ == TuningMode::DRIVE_ANGULAR) ? get_current_heading() : cascade_.get_position();

        pros::lcd::print(0, "MODE: %s | SEL: %s (+/- %.3f)",
                         get_mode_name(current_mode_), get_param_name(current_param_), get_increment());
        pros::lcd::print(1, "GAINS: P=%.4f I=%.4f D=%.4f", g.kp, g.ki, g.kd);
        pros::lcd::print(2, "TARGET: %.2f | CURRENT: %.2f | ERR: %.2f",
                         get_active_target(), current_val, get_active_target() - current_val);
        pros::lcd::print(3, "LAST RUN: %s | Settled: %s (%lu ms)",
                         m.total_time_ms > 0 ? "DONE" : "IDLE",
                         m.settled ? "YES" : (m.timed_out ? "TIMEOUT" : "NO"),
                         (unsigned long)m.settling_time_ms);
        pros::lcd::print(4, "METRICS: Rise: %lums | Overshoot: %.2f (%.1f%%)",
                         (unsigned long)m.rise_time_ms, m.peak_overshoot, m.peak_overshoot_pct);
        pros::lcd::print(5, "[A]=TEST | [B]=ABORT | [X]=PARAM | [Y]=MODE");
        pros::lcd::print(6, "[R1+A]=FWD-BACK TEST | [R1+Y]=BOX 4-CORNER");
        pros::lcd::print(7, "Left:[20,-10] Right:[-7,9] IMU:3");
    }
};
