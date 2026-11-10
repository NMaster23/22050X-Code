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
 * @brief Interactive PID Tuner Engine
 *
 * Provides live parameter adjustment from V5 Controller, real-time
 * telemetry logging over serial/terminal, LLEMU Brain screen diagnostics,
 * step response profiling, and manual driving repositioning.
 */
class PIDTuner {
public:
    PIDTuner()
        : left_drive_(tuner_config::LEFT_DRIVE_PORTS, tuner_config::DRIVETRAIN_GEARSET),
          right_drive_(tuner_config::RIGHT_DRIVE_PORTS, tuner_config::DRIVETRAIN_GEARSET),
          mechanism_(tuner_config::MECHANISM_PORT, tuner_config::MECHANISM_GEARSET),
          imu_(tuner_config::IMU_PORT),
          master_(pros::E_CONTROLLER_MASTER) {
        
        // Load default profiles
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
        pros::lcd::set_text(0, "=== VEX V5 PID TUNER ===");
        pros::lcd::set_text(1, "Calibrating sensors...");

        left_drive_.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        right_drive_.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        mechanism_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        left_drive_.tare_position();
        right_drive_.tare_position();
        mechanism_.tare_position();

        if (tuner_config::IMU_PORT > 0) {
            imu_.reset();
            int attempts = 0;
            while (imu_.is_calibrating() && attempts < 300) {
                pros::delay(10);
                attempts++;
            }
        }

        update_brain_lcd();
        printf("\n======================================================\n");
        printf(" VEX V5 PID TUNER INITIALIZED\n");
        printf(" Use PROS Terminal to view real-time step response logs\n");
        printf(" Format: [CSV] time_ms,target,measured,error,output\n");
        printf("======================================================\n\n");
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

    /**
     * @brief Execute a step response test with the currently selected profile
     */
    void execute_step_test() {
        test_running_ = true;
        abort_requested_ = false;

        // Visual / haptic alert
        master_.rumble(".");
        pros::delay(50);

        PID& active_pid = get_active_pid();
        double target = get_active_target();

        // Tare feedback sensors before test
        left_drive_.tare_position();
        right_drive_.tare_position();
        mechanism_.tare_position();
        if (tuner_config::IMU_PORT > 0) {
            imu_.set_heading(0.0);
            imu_.set_rotation(0.0);
        }

        uint32_t start_time = pros::millis();
        double initial_value = get_sensor_feedback();
        active_pid.start_test(target, initial_value, start_time);

        printf("\n--- START STEP TEST [%s] ---\n", get_mode_name(current_mode_));
        printf("Gains: Kp=%.4f, Ki=%.4f, Kd=%.4f, Target=%.2f\n",
               active_pid.get_gains().kp, active_pid.get_gains().ki, active_pid.get_gains().kd, target);
        printf("[CSV] time_ms,target,measured,error,output\n");

        uint32_t prev_time = start_time;

        while (true) {
            uint32_t current_time = pros::millis();
            double dt = (current_time - prev_time) / 1000.0;
            if (dt <= 0.0) dt = 0.01;
            prev_time = current_time;

            // Check for manual abort from controller
            if (master_.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                abort_requested_ = true;
                printf("Test aborted by user.\n");
                break;
            }

            double measured = get_sensor_feedback();
            double error = target - measured;
            double output = active_pid.compute(target, measured, dt);

            // Apply motor output
            apply_motor_output(output);

            // Log CSV row
            uint32_t elapsed_ms = current_time - start_time;
            printf("[CSV] %lu,%.3f,%.3f,%.3f,%.2f\n",
                   (unsigned long)elapsed_ms, target, measured, error, output);

            // Check settle condition
            bool finished = active_pid.update_settle(error, measured, current_time);
            if (finished) {
                break;
            }

            pros::delay(10);
        }

        // Stop motors
        left_drive_.brake();
        right_drive_.brake();
        mechanism_.brake();

        // Summarize test results
        const StepMetrics& m = active_pid.get_metrics();
        printf("--- STEP TEST FINISHED [%s] ---\n", abort_requested_ ? "ABORTED" : (m.settled ? "SETTLED" : "TIMED OUT"));
        printf("Elapsed Time:   %lu ms\n", (unsigned long)m.total_time_ms);
        printf("Rise Time (90%%): %lu ms\n", (unsigned long)m.rise_time_ms);
        printf("Settling Time:  %lu ms\n", (unsigned long)m.settling_time_ms);
        printf("Peak Overshoot: %.2f (%.1f%%)\n", m.peak_overshoot, m.peak_overshoot_pct);
        printf("Final Error:    %.3f\n", m.steady_state_error);
        printf("---------------------------------------\n\n");

        if (abort_requested_) {
            master_.rumble("---");
        } else if (m.settled) {
            master_.rumble(". - .");
        } else {
            master_.rumble("- -");
        }

        test_running_ = false;
    }

    // Accessors for autonomous integration
    PID& get_linear_pid() { return linear_pid_; }
    PID& get_angular_pid() { return angular_pid_; }
    PID& get_mech_pid() { return mech_pid_; }
    pros::MotorGroup& left_drive() { return left_drive_; }
    pros::MotorGroup& right_drive() { return right_drive_; }
    pros::Motor& mechanism() { return mechanism_; }
    pros::Imu& imu() { return imu_; }

private:
    pros::MotorGroup left_drive_;
    pros::MotorGroup right_drive_;
    pros::Motor mechanism_;
    pros::Imu imu_;
    pros::Controller master_;

    PID linear_pid_;
    PID angular_pid_;
    PID mech_pid_;

    double targets_[static_cast<size_t>(TuningMode::MODE_COUNT)] = {24.0, 90.0, 360.0};

    TuningMode current_mode_ = TuningMode::DRIVE_LINEAR;
    TuningParam current_param_ = TuningParam::KP;

    // Available increment steps
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

    double get_sensor_feedback() {
        switch (current_mode_) {
            case TuningMode::DRIVE_LINEAR: {
                // Average drivetrain position converted to linear inches
                double l_deg = left_drive_.get_position();
                double r_deg = right_drive_.get_position();
                double avg_deg = (l_deg + r_deg) * 0.5;
                return tuner_config::motor_deg_to_inches(avg_deg);
            }
            case TuningMode::DRIVE_ANGULAR: {
                if (tuner_config::IMU_PORT > 0 && imu_.is_installed()) {
                    return imu_.get_rotation(); // Continuous heading in degrees
                } else {
                    // Fallback to wheel encoder differential heading
                    double l_in = tuner_config::motor_deg_to_inches(left_drive_.get_position());
                    double r_in = tuner_config::motor_deg_to_inches(right_drive_.get_position());
                    double diff_in = r_in - l_in;
                    return (diff_in / tuner_config::TRACK_WIDTH_INCHES) * (180.0 / tuner_config::PI);
                }
            }
            case TuningMode::MECHANISM: {
                return mechanism_.get_position(); // Degrees
            }
            default:
                return 0.0;
        }
    }

    void apply_motor_output(double output) {
        switch (current_mode_) {
            case TuningMode::DRIVE_LINEAR:
                left_drive_.move(output);
                right_drive_.move(output);
                break;
            case TuningMode::DRIVE_ANGULAR:
                // Clockwise turn: left forwards, right backwards
                left_drive_.move(output);
                right_drive_.move(-output);
                break;
            case TuningMode::MECHANISM:
                mechanism_.move(output);
                break;
            default:
                break;
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

        // Button A: Run Step Response Test
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

        // Deadband filter
        if (std::abs(left_y) < 5) left_y = 0;
        if (std::abs(right_x) < 5) right_x = 0;

        if (left_y != 0 || right_x != 0) {
            left_drive_.move(left_y + right_x);
            right_drive_.move(left_y - right_x);
        } else {
            left_drive_.move(0);
            right_drive_.move(0);
        }

        // Manual mechanism control on shoulder buttons
        if (master_.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            mechanism_.move(127);
        } else if (master_.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            mechanism_.move(-127);
        } else {
            mechanism_.move(0);
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

        // Format selected parameter highlight
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
        double current_val = get_sensor_feedback();

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
        pros::lcd::print(5, "[A]=RUN TEST | [B]=ABORT | [X]=PARAM | [Y]=MODE");
        pros::lcd::print(6, "[DPAD UP/DN]=ADJUST | [DPAD L/R]=STEP SIZE");
        pros::lcd::print(7, "Terminal: [CSV] time_ms,target,measured,err,out");
    }
};
