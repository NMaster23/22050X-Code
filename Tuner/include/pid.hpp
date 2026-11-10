#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <string>

/**
 * @brief PID Controller Gains and Limits configuration
 */
struct PIDGains {
    double kp = 0.0;              // Proportional gain
    double ki = 0.0;              // Integral gain
    double kd = 0.0;              // Derivative gain
    double windup_range = 0.0;    // Error threshold below which integral accumulates (0 = disabled/always accumulate)
    bool sign_flip_reset = true;  // Reset integral when error crosses zero (prevents overshoot accumulation)
    double slew_rate = 0.0;       // Max allowed change in output per cycle (0 = disabled)
    double min_output = -127.0;   // Minimum clamped controller output
    double max_output = 127.0;    // Maximum clamped controller output
};

/**
 * @brief Criteria for determining when the PID loop has settled
 */
struct SettleSettings {
    double small_error = 1.0;              // Small error threshold (units of sensor)
    uint32_t small_error_timeout_ms = 150; // Required time within small error to declare settled
    double large_error = 3.0;              // Large error threshold (units of sensor)
    uint32_t large_error_timeout_ms = 500; // Required time within large error to declare settled
    uint32_t max_time_ms = 4000;           // Hard timeout for motion
};

/**
 * @brief Performance metrics recorded during a step response test
 */
struct StepMetrics {
    double target = 0.0;
    double initial_value = 0.0;
    double peak_value = 0.0;
    double peak_overshoot = 0.0;       // Maximum overshoot past target (in sensor units)
    double peak_overshoot_pct = 0.0;   // Maximum overshoot percentage
    double steady_state_error = 0.0;   // Final error upon completion
    uint32_t rise_time_ms = 0;         // Time to first reach 90% of target
    uint32_t settling_time_ms = 0;     // Time until settled condition is met
    uint32_t total_time_ms = 0;        // Total elapsed time of the test
    bool settled = false;              // True if settled within criteria, false if timed out
    bool timed_out = false;
};

/**
 * @brief Robust, production-grade PID controller designed for VEX V5 robotics.
 *
 * Supports anti-windup (integration bounds and zero-crossing reset),
 * slew rate limiting, derivative filtering, and dual-threshold settle detection.
 */
class PID {
public:
    PID() = default;

    PID(const PIDGains& gains, const SettleSettings& settle = {})
        : gains_(gains), settle_(settle) {
        reset();
    }

    void set_gains(const PIDGains& gains) {
        gains_ = gains;
    }

    const PIDGains& get_gains() const {
        return gains_;
    }

    PIDGains& gains() {
        return gains_;
    }

    void set_settle(const SettleSettings& settle) {
        settle_ = settle;
    }

    const SettleSettings& get_settle() const {
        return settle_;
    }

    /**
     * @brief Reset internal states (integral, previous error, timers, slew)
     */
    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
        prev_output_ = 0.0;
        prev_target_ = 0.0;
        has_run_ = false;

        small_error_timer_ = 0;
        large_error_timer_ = 0;
        start_time_ms_ = 0;

        metrics_ = StepMetrics{};
    }

    /**
     * @brief Initialize metrics tracking for a new test run starting at start_time_ms
     */
    void start_test(double target, double initial_value, uint32_t start_time_ms) {
        reset();
        start_time_ms_ = start_time_ms;
        prev_target_ = target;
        metrics_.target = target;
        metrics_.initial_value = initial_value;
        metrics_.peak_value = initial_value;
    }

    /**
     * @brief Compute the PID control output given target and measured current value.
     * @param target Desired setpoint
     * @param current Measured feedback value
     * @param dt_sec Time delta in seconds since last computation (default 0.01s = 10ms)
     * @return Clamped controller output
     */
    double compute(double target, double current, double dt_sec = 0.01) {
        if (dt_sec <= 0.0001) dt_sec = 0.01; // Guard against division by zero

        double error = target - current;

        // Proportional term
        double p_out = gains_.kp * error;

        // Anti-windup: zero-crossing integral reset
        if (gains_.sign_flip_reset && has_run_ && ((error > 0 && prev_error_ < 0) || (error < 0 && prev_error_ > 0))) {
            integral_ = 0.0;
        }

        // Integral accumulation with windup threshold
        if (gains_.windup_range <= 0.0 || std::abs(error) < gains_.windup_range) {
            integral_ += error * dt_sec;
        } else {
            integral_ = 0.0;
        }

        double i_out = gains_.ki * integral_;

        // Derivative term
        double derivative = (has_run_) ? ((error - prev_error_) / dt_sec) : 0.0;
        double d_out = gains_.kd * derivative;

        // Unconstrained total output
        double raw_output = p_out + i_out + d_out;

        // Clamping to configured output boundaries
        double clamped_output = std::clamp(raw_output, gains_.min_output, gains_.max_output);

        // Slew rate limiting
        double final_output = clamped_output;
        if (gains_.slew_rate > 0.0 && has_run_) {
            double max_change = gains_.slew_rate * (dt_sec * 100.0); // scaled per 10ms standard tick
            double change = clamped_output - prev_output_;
            if (std::abs(change) > max_change) {
                final_output = prev_output_ + std::copysign(max_change, change);
            }
        }

        prev_error_ = error;
        prev_output_ = final_output;
        has_run_ = true;

        return final_output;
    }

    /**
     * @brief Update settle condition and performance metrics.
     * @param error Current error (target - current)
     * @param current Current feedback position
     * @param now_ms Current timestamp in milliseconds
     * @return True if system has settled or timed out, false otherwise
     */
    bool update_settle(double error, double current, uint32_t now_ms) {
        uint32_t elapsed_ms = (start_time_ms_ == 0) ? 0 : (now_ms - start_time_ms_);
        double abs_err = std::abs(error);
        double total_step = metrics_.target - metrics_.initial_value;

        // Track peak value and overshoot
        if (std::abs(total_step) > 1e-4) {
            bool positive_step = total_step > 0;
            if (positive_step) {
                if (current > metrics_.peak_value) {
                    metrics_.peak_value = current;
                }
                double overshoot = metrics_.peak_value - metrics_.target;
                if (overshoot > 0) {
                    metrics_.peak_overshoot = overshoot;
                    metrics_.peak_overshoot_pct = (overshoot / std::abs(total_step)) * 100.0;
                }
            } else {
                if (current < metrics_.peak_value) {
                    metrics_.peak_value = current;
                }
                double overshoot = metrics_.target - metrics_.peak_value;
                if (overshoot > 0) {
                    metrics_.peak_overshoot = overshoot;
                    metrics_.peak_overshoot_pct = (overshoot / std::abs(total_step)) * 100.0;
                }
            }

            // Track rise time (time to 90% of target)
            if (metrics_.rise_time_ms == 0) {
                double progress = (current - metrics_.initial_value) / total_step;
                if (progress >= 0.90) {
                    metrics_.rise_time_ms = elapsed_ms;
                }
            }
        }

        // Small error settling timer
        if (abs_err < settle_.small_error) {
            small_error_timer_ += 10; // Assuming ~10ms evaluation tick
        } else {
            small_error_timer_ = 0;
        }

        // Large error settling timer
        if (abs_err < settle_.large_error) {
            large_error_timer_ += 10;
        } else {
            large_error_timer_ = 0;
        }

        // Check if settled
        bool settled_by_small = (small_error_timer_ >= settle_.small_error_timeout_ms);
        bool settled_by_large = (large_error_timer_ >= settle_.large_error_timeout_ms);

        if (settled_by_small || settled_by_large) {
            metrics_.settled = true;
            metrics_.settling_time_ms = elapsed_ms;
            metrics_.total_time_ms = elapsed_ms;
            metrics_.steady_state_error = error;
            return true;
        }

        // Check hard timeout
        if (settle_.max_time_ms > 0 && elapsed_ms >= settle_.max_time_ms) {
            metrics_.settled = false;
            metrics_.timed_out = true;
            metrics_.total_time_ms = elapsed_ms;
            metrics_.steady_state_error = error;
            return true;
        }

        return false;
    }

    const StepMetrics& get_metrics() const {
        return metrics_;
    }

    double get_prev_error() const { return prev_error_; }
    double get_integral() const { return integral_; }
    double get_prev_output() const { return prev_output_; }

private:
    PIDGains gains_{};
    SettleSettings settle_{};
    StepMetrics metrics_{};

    double prev_error_ = 0.0;
    double integral_ = 0.0;
    double prev_output_ = 0.0;
    double prev_target_ = 0.0;
    bool has_run_ = false;

    uint32_t small_error_timer_ = 0;
    uint32_t large_error_timer_ = 0;
    uint32_t start_time_ms_ = 0;
};
