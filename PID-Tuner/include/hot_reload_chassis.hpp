#pragma once

#include "lemlib/api.hpp"
#include "robot_config.hpp"
#include "serial_bridge.hpp"
#include <cmath>
#include <new>
#include <algorithm>

namespace tuning_harness {

/**
 * @brief Subclass of lemlib::Chassis providing direct hot-reload access to protected
 * ControllerSettings, Drivetrain, PID, and ExitCondition members without losing OdomSensors state.
 */
class HotReloadableChassis : public lemlib::Chassis {
public:
    HotReloadableChassis(
        lemlib::Drivetrain drivetrain,
        lemlib::ControllerSettings linearSettings,
        lemlib::ControllerSettings angularSettings,
        lemlib::OdomSensors sensors
    ) : lemlib::Chassis(drivetrain, linearSettings, angularSettings, sensors) {}

    void hotReloadLateral(const lemlib::ControllerSettings& settings, bool signFlipReset = false) {
        this->lateralSettings = settings;
        this->lateralPID.~PID();
        new (&this->lateralPID) lemlib::PID(
            settings.kP,
            settings.kI,
            settings.kD,
            settings.windupRange,
            signFlipReset
        );
        this->lateralSmallExit.~ExitCondition();
        new (&this->lateralSmallExit) lemlib::ExitCondition(
            settings.smallError,
            static_cast<int>(settings.smallErrorTimeout)
        );
        this->lateralLargeExit.~ExitCondition();
        new (&this->lateralLargeExit) lemlib::ExitCondition(
            settings.largeError,
            static_cast<int>(settings.largeErrorTimeout)
        );
    }

    void hotReloadAngular(const lemlib::ControllerSettings& settings, bool signFlipReset = true) {
        this->angularSettings = settings;
        this->angularPID.~PID();
        new (&this->angularPID) lemlib::PID(
            settings.kP,
            settings.kI,
            settings.kD,
            settings.windupRange,
            signFlipReset
        );
        this->angularSmallExit.~ExitCondition();
        new (&this->angularSmallExit) lemlib::ExitCondition(
            settings.smallError,
            static_cast<int>(settings.smallErrorTimeout)
        );
        this->angularLargeExit.~ExitCondition();
        new (&this->angularLargeExit) lemlib::ExitCondition(
            settings.largeError,
            static_cast<int>(settings.largeErrorTimeout)
        );
    }

    void setHorizontalDrift(float horizontalDrift) {
        if (horizontalDrift > 0.0f) {
            this->drivetrain.horizontalDrift = horizontalDrift;
        }
    }

    const lemlib::ControllerSettings& getLateralSettings() const {
        return this->lateralSettings;
    }

    const lemlib::ControllerSettings& getAngularSettings() const {
        return this->angularSettings;
    }

    const lemlib::Drivetrain& getDrivetrain() const {
        return this->drivetrain;
    }

    const lemlib::OdomSensors& getSensors() const {
        return this->sensors;
    }
};

/**
 * @brief Dynamic Placement-New Manager for HotReloadableChassis.
 * Allows either in-place gain injection or complete Chassis reconstruction between runs
 * while preserving lemlib::OdomSensors calibration and Pose state.
 */
class DynamicChassisWrapper {
public:
    DynamicChassisWrapper(
        lemlib::Drivetrain drivetrain,
        lemlib::ControllerSettings lateralSettings,
        lemlib::ControllerSettings angularSettings,
        lemlib::OdomSensors sensors
    ) : drivetrain_(drivetrain),
        lateral_settings_(lateralSettings),
        angular_settings_(angularSettings),
        swing_settings_(robot_config::makeSwingSettings()),
        sensors_(sensors),
        boomerang_lead_(robot_config::TUNED_BOOMERANG_LEAD) {
        instance_ = new (storage_) HotReloadableChassis(
            drivetrain_,
            lateral_settings_,
            angular_settings_,
            sensors_
        );
    }

    void calibrateOnce() {
        if (!calibrated_) {
            instance_->calibrate(true);
            calibrated_ = true;
        }
    }

    HotReloadableChassis& get() {
        return *instance_;
    }

    const HotReloadableChassis& get() const {
        return *instance_;
    }

    /**
     * @brief Dynamically reconstructs lemlib::Chassis in-place between runs while
     * preserving OdomSensors calibration and robot Pose state.
     */
    void reconstruct() {
        instance_->cancelAllMotions();
        lemlib::Pose saved_pose = instance_->getPose();
        instance_->~HotReloadableChassis();
        instance_ = new (storage_) HotReloadableChassis(
            drivetrain_,
            lateral_settings_,
            angular_settings_,
            sensors_
        );
        instance_->setPose(saved_pose);
    }

    void applyCommandGains(const serial_bridge::HostCommandPacket& pkt) {
        bool sign_flip = (pkt.trigger_flags & serial_bridge::TriggerFlags::SIGN_FLIP_RESET) != 0;
        bool reconstruct_req = (pkt.trigger_flags & serial_bridge::TriggerFlags::RECONSTRUCT_CHASSIS) != 0;

        lemlib::ControllerSettings updated(
            pkt.kp,
            pkt.ki,
            pkt.kd,
            pkt.windup_range,
            pkt.small_error,
            pkt.small_error_timeout,
            pkt.large_error,
            pkt.large_error_timeout,
            pkt.slew
        );

        if (pkt.horizontal_drift > 0.0f) {
            drivetrain_.horizontalDrift = pkt.horizontal_drift;
            instance_->setHorizontalDrift(pkt.horizontal_drift);
        }
        if (pkt.lead > 0.0f && pkt.lead <= 1.0f) {
            boomerang_lead_ = pkt.lead;
        }

        auto ctrl = static_cast<serial_bridge::ControllerId>(pkt.controller_id);
        switch (ctrl) {
            case serial_bridge::ControllerId::LEMLIB_LATERAL:
                lateral_settings_ = updated;
                if (reconstruct_req) {
                    reconstruct();
                } else {
                    instance_->hotReloadLateral(lateral_settings_, sign_flip);
                }
                break;

            case serial_bridge::ControllerId::LEMLIB_ANGULAR:
                angular_settings_ = updated;
                if (reconstruct_req) {
                    reconstruct();
                } else {
                    instance_->hotReloadAngular(angular_settings_, sign_flip);
                }
                break;

            case serial_bridge::ControllerId::LEMLIB_BOOMERANG:
                lateral_settings_ = updated;
                if (reconstruct_req) {
                    reconstruct();
                } else {
                    instance_->hotReloadLateral(lateral_settings_, sign_flip);
                }
                break;

            case serial_bridge::ControllerId::LEMLIB_SWING:
                swing_settings_ = updated;
                instance_->hotReloadAngular(swing_settings_, sign_flip);
                break;

            case serial_bridge::ControllerId::CASCADE_LIFT:
                break;
        }
    }

    void activateSwingGains() {
        instance_->hotReloadAngular(swing_settings_, true);
    }

    void restoreAngularGains() {
        instance_->hotReloadAngular(angular_settings_, true);
    }

    float getBoomerangLead() const { return boomerang_lead_; }
    float getHorizontalDrift() const { return drivetrain_.horizontalDrift; }

private:
    alignas(HotReloadableChassis) uint8_t storage_[sizeof(HotReloadableChassis)];
    HotReloadableChassis* instance_ = nullptr;
    lemlib::Drivetrain drivetrain_;
    lemlib::ControllerSettings lateral_settings_;
    lemlib::ControllerSettings angular_settings_;
    lemlib::ControllerSettings swing_settings_;
    lemlib::OdomSensors sensors_;
    float boomerang_lead_;
    bool calibrated_ = false;
};

/**
 * @brief Dedicated Cascade Lift Controller using lemlib::PID + Gravity Feedforward Hold
 */
class CascadeLiftController {
public:
    CascadeLiftController(pros::MotorGroup& motors)
        : motors_(motors),
          pid_(
              robot_config::CASCADE_KP,
              robot_config::CASCADE_KI,
              robot_config::CASCADE_KD,
              robot_config::CASCADE_WINDUP_RANGE,
              robot_config::CASCADE_SIGN_FLIP_RESET
          ),
          small_exit_(0.25f, 80),
          large_exit_(0.75f, 350),
          gravity_ff_mv_(robot_config::CASCADE_GRAVITY_FF_MV) {}

    void hotReload(
        float kp,
        float ki,
        float kd,
        float windup_range,
        bool sign_flip_reset,
        float small_error,
        float small_timeout,
        float large_error,
        float large_timeout,
        float gravity_ff_mv
    ) {
        pid_.~PID();
        new (&pid_) lemlib::PID(kp, ki, kd, windup_range, sign_flip_reset);

        float se = (small_error > 0.01f) ? small_error : 0.25f;
        int st = (small_timeout > 10.0f) ? static_cast<int>(small_timeout) : 80;
        float le = (large_error > 0.01f) ? large_error : 0.75f;
        int lt = (large_timeout > 10.0f) ? static_cast<int>(large_timeout) : 350;

        small_exit_.~ExitCondition();
        new (&small_exit_) lemlib::ExitCondition(se, st);

        large_exit_.~ExitCondition();
        new (&large_exit_) lemlib::ExitCondition(le, lt);

        if (gravity_ff_mv >= 0.0f) {
            gravity_ff_mv_ = gravity_ff_mv;
        }
    }

    void setTargetHeightInches(float target_in, uint32_t timeout_ms, uint32_t now_ms) {
        target_height_in_ = std::clamp(target_in, 0.0f, robot_config::CASCADE_MAX_HEIGHT_IN);
        timeout_ms_ = timeout_ms;
        start_time_ms_ = now_ms;
        pid_.reset();
        small_exit_.reset();
        large_exit_.reset();
        active_ = true;
    }

    void resetOrigin() {
        motors_.tare_position();
        target_height_in_ = 0.0f;
        pid_.reset();
        active_ = false;
    }

    float getHeightInches() const {
        double deg = motors_.get_position();
        return static_cast<float>(deg) / robot_config::CASCADE_DEG_PER_INCH;
    }

    void update(uint32_t now_ms) {
        if (coasting_) {
            motors_.move_voltage(0);
            return;
        }

        float current_in = getHeightInches();
        float error_in = target_height_in_ - current_in;

        if (active_) {
            bool settled = small_exit_.update(error_in) || large_exit_.update(error_in);
            bool timed_out = (now_ms - start_time_ms_) >= timeout_ms_;
            if (settled || timed_out) {
                active_ = false;
            }
        }

        float pid_out = pid_.update(error_in);
        // Scale PID output (in LemLib -127..127 scale) to millivolts (-12000..12000 mV) + gravity FF
        float ff_mv = (target_height_in_ > 0.3f || current_in > 0.3f) ? gravity_ff_mv_ : 0.0f;
        float cmd_mv = pid_out * (12000.0f / 127.0f) + ff_mv;
        cmd_mv = std::clamp(cmd_mv, -12000.0f, 12000.0f);

        if (!active_ && std::abs(error_in) < 0.2f && target_height_in_ <= 0.1f) {
            motors_.brake();
        } else {
            motors_.move_voltage(static_cast<int32_t>(cmd_mv));
        }
    }

    void emergencyCoast() {
        coasting_ = true;
        active_ = false;
        motors_.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        motors_.move_voltage(0);
    }

    void clearCoast() {
        if (coasting_) {
            coasting_ = false;
            motors_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
    }

    bool isActive() const { return active_; }

private:
    pros::MotorGroup& motors_;
    lemlib::PID pid_;
    lemlib::ExitCondition small_exit_;
    lemlib::ExitCondition large_exit_;
    float target_height_in_ = 0.0f;
    float gravity_ff_mv_ = 1150.0f;
    uint32_t start_time_ms_ = 0;
    uint32_t timeout_ms_ = 2000;
    bool active_ = false;
    bool coasting_ = false;
};

/**
 * @brief Hardware and Camera Boundary Watchdog
 * Immediately cancels all motions and coasts all motors if:
 * 1. Host vision engine flags robot mask approaching calibrated camera boundary,
 * 2. Motor current draw indicates a physical stall for > 400 ms, or
 * 3. The 100 Hz serial heartbeat drops for > 150 ms during active autonomous tuning.
 */
class HardwareWatchdog {
public:
    uint8_t evaluate(
        uint32_t now_ms,
        uint32_t last_packet_ms,
        bool tuning_session_active,
        bool boundary_abort_flag,
        float left_current_ma,
        float right_current_ma,
        float left_vel_rpm,
        float right_vel_rpm,
        float lift_current_ma,
        float lift_vel_rpm,
        float max_temp_c
    ) {
        uint8_t flags = serial_bridge::WatchdogFlags::NONE;

        // 1. Camera boundary check
        if (boundary_abort_flag) {
            flags |= serial_bridge::WatchdogFlags::BOUNDARY_TRIP;
        }

        // 2. Physical stall check (> 400 ms sustained high current + near-zero velocity)
        bool drive_stalling =
            (left_current_ma > robot_config::WATCHDOG_STALL_CURRENT_MA &&
             std::abs(left_vel_rpm) < robot_config::WATCHDOG_STALL_VEL_RPM) ||
            (right_current_ma > robot_config::WATCHDOG_STALL_CURRENT_MA &&
             std::abs(right_vel_rpm) < robot_config::WATCHDOG_STALL_VEL_RPM);

        if (drive_stalling) {
            if (drive_stall_start_ms_ == 0) {
                drive_stall_start_ms_ = now_ms;
            } else if (now_ms - drive_stall_start_ms_ > robot_config::WATCHDOG_STALL_TIME_MS) {
                flags |= serial_bridge::WatchdogFlags::DRIVETRAIN_STALL;
            }
        } else {
            drive_stall_start_ms_ = 0;
        }

        bool lift_stalling =
            (lift_current_ma > robot_config::WATCHDOG_STALL_CURRENT_MA &&
             std::abs(lift_vel_rpm) < robot_config::WATCHDOG_STALL_VEL_RPM);

        if (lift_stalling) {
            if (lift_stall_start_ms_ == 0) {
                lift_stall_start_ms_ = now_ms;
            } else if (now_ms - lift_stall_start_ms_ > robot_config::WATCHDOG_STALL_TIME_MS) {
                flags |= serial_bridge::WatchdogFlags::LIFT_STALL;
            }
        } else {
            lift_stall_start_ms_ = 0;
        }

        // 3. 100 Hz Serial Heartbeat dropout check
        if (tuning_session_active && last_packet_ms > 0 &&
            (now_ms - last_packet_ms > robot_config::WATCHDOG_HEARTBEAT_TIMEOUT_MS)) {
            flags |= serial_bridge::WatchdogFlags::HEARTBEAT_LOST;
        }

        // 4. Thermal monitoring (55 C PROS threshold)
        if (max_temp_c >= robot_config::PROS_THERMAL_LIMIT_C) {
            flags |= serial_bridge::WatchdogFlags::THERMAL_WARNING;
        }

        last_flags_ = flags;
        return flags;
    }

    bool isTripped() const {
        constexpr uint8_t trip_mask =
            serial_bridge::WatchdogFlags::BOUNDARY_TRIP |
            serial_bridge::WatchdogFlags::DRIVETRAIN_STALL |
            serial_bridge::WatchdogFlags::LIFT_STALL |
            serial_bridge::WatchdogFlags::HEARTBEAT_LOST;
        return (last_flags_ & trip_mask) != 0;
    }

    uint8_t getFlags() const { return last_flags_; }

    void reset() {
        drive_stall_start_ms_ = 0;
        lift_stall_start_ms_ = 0;
        last_flags_ = serial_bridge::WatchdogFlags::NONE;
    }

private:
    uint32_t drive_stall_start_ms_ = 0;
    uint32_t lift_stall_start_ms_ = 0;
    uint8_t last_flags_ = serial_bridge::WatchdogFlags::NONE;
};

} // namespace tuning_harness
