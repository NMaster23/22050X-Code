use anyhow::Result;
use plotters::prelude::*;
use std::fs;
use std::path::Path;

use crate::optimizer::metrics::StepResponseSample;
use crate::optimizer::pipeline::FullTuningResults;
use crate::repo_inspector::RobotHardwareSpec;
use crate::stress_test::StressTestSuiteResults;
use crate::vision::{FieldHomography, SensorDiagnostics};

pub struct ReportAndPatchGenerator;

impl ReportAndPatchGenerator {
    /// Patches the verified `lemlib::ControllerSettings`, `horizontalDrift`, and cascade lift `lemlib::PID`
    /// constants directly into `./include/robot_config.hpp` and `./src/main.cpp`.
    pub fn patch_cpp_sources(
        project_root: &Path,
        tuned: &FullTuningResults,
    ) -> Result<Vec<String>> {
        let mut patched_files = Vec::new();

        // 1. Patch include/robot_config.hpp between [BEGIN_TUNED_GAINS] and [END_TUNED_GAINS]
        let cfg_path = project_root.join("include/robot_config.hpp");
        if cfg_path.exists() {
            let content = fs::read_to_string(&cfg_path)?;
            let replacement_block = format!(
                "// [BEGIN_TUNED_GAINS]\n\
                 // Automatically patched by vision_tuner (--mode full-suite)\n\
                 // ============================================================================\n\n\
                 // LemLib Drivetrain Horizontal Drift\n\
                 constexpr float TUNED_HORIZONTAL_DRIFT = {:.4}f;\n\
                 constexpr float TUNED_BOOMERANG_LEAD = {:.4}f;\n\n\
                 // LemLib Lateral Controller Settings (moveToPoint / moveToPose lateral)\n\
                 constexpr float LATERAL_KP = {:.4}f;\n\
                 constexpr float LATERAL_KI = {:.4}f;\n\
                 constexpr float LATERAL_KD = {:.4}f;\n\
                 constexpr float LATERAL_WINDUP_RANGE = {:.4}f;\n\
                 constexpr float LATERAL_SMALL_ERROR = {:.4}f;\n\
                 constexpr float LATERAL_SMALL_ERROR_TIMEOUT = {:.1}f;\n\
                 constexpr float LATERAL_LARGE_ERROR = {:.4}f;\n\
                 constexpr float LATERAL_LARGE_ERROR_TIMEOUT = {:.1}f;\n\
                 constexpr float LATERAL_SLEW = {:.4}f;\n\n\
                 // LemLib Angular Controller Settings (turnToHeading / turnToPoint)\n\
                 constexpr float ANGULAR_KP = {:.4}f;\n\
                 constexpr float ANGULAR_KI = {:.4}f;\n\
                 constexpr float ANGULAR_KD = {:.4}f;\n\
                 constexpr float ANGULAR_WINDUP_RANGE = {:.4}f;\n\
                 constexpr float ANGULAR_SMALL_ERROR = {:.4}f;\n\
                 constexpr float ANGULAR_SMALL_ERROR_TIMEOUT = {:.1}f;\n\
                 constexpr float ANGULAR_LARGE_ERROR = {:.4}f;\n\
                 constexpr float ANGULAR_LARGE_ERROR_TIMEOUT = {:.1}f;\n\
                 constexpr float ANGULAR_SLEW = {:.4}f;\n\n\
                 // LemLib Swing Controller Settings (swingToHeading / swingToPoint)\n\
                 constexpr float SWING_KP = {:.4}f;\n\
                 constexpr float SWING_KI = {:.4}f;\n\
                 constexpr float SWING_KD = {:.4}f;\n\
                 constexpr float SWING_WINDUP_RANGE = {:.4}f;\n\
                 constexpr float SWING_SMALL_ERROR = {:.4}f;\n\
                 constexpr float SWING_SMALL_ERROR_TIMEOUT = {:.1}f;\n\
                 constexpr float SWING_LARGE_ERROR = {:.4}f;\n\
                 constexpr float SWING_LARGE_ERROR_TIMEOUT = {:.1}f;\n\
                 constexpr float SWING_SLEW = {:.4}f;\n\n\
                 // Cascade Lift lemlib::PID Settings (2-Motor 6:1 Blue Cartridge)\n\
                 constexpr float CASCADE_KP = {:.4}f;\n\
                 constexpr float CASCADE_KI = {:.4}f;\n\
                 constexpr float CASCADE_KD = {:.4}f;\n\
                 constexpr float CASCADE_WINDUP_RANGE = {:.4}f;\n\
                 constexpr bool CASCADE_SIGN_FLIP_RESET = {};\n\
                 constexpr float CASCADE_GRAVITY_FF_MV = {:.1}f;\n\n\
                 // ============================================================================\n\
                 // [END_TUNED_GAINS]",
                tuned.horizontal_drift,
                tuned.boomerang_lead,
                tuned.lateral_gains.kp,
                tuned.lateral_gains.ki,
                tuned.lateral_gains.kd,
                tuned.lateral_gains.windup_range,
                tuned.lateral_gains.small_error,
                tuned.lateral_gains.small_error_timeout,
                tuned.lateral_gains.large_error,
                tuned.lateral_gains.large_error_timeout,
                tuned.lateral_gains.slew,
                tuned.angular_gains.kp,
                tuned.angular_gains.ki,
                tuned.angular_gains.kd,
                tuned.angular_gains.windup_range,
                tuned.angular_gains.small_error,
                tuned.angular_gains.small_error_timeout,
                tuned.angular_gains.large_error,
                tuned.angular_gains.large_error_timeout,
                tuned.angular_gains.slew,
                tuned.swing_gains.kp,
                tuned.swing_gains.ki,
                tuned.swing_gains.kd,
                tuned.swing_gains.windup_range,
                tuned.swing_gains.small_error,
                tuned.swing_gains.small_error_timeout,
                tuned.swing_gains.large_error,
                tuned.swing_gains.large_error_timeout,
                tuned.swing_gains.slew,
                tuned.cascade_gains.kp,
                tuned.cascade_gains.ki,
                tuned.cascade_gains.kd,
                tuned.cascade_gains.windup_range,
                if tuned.cascade_gains.sign_flip_reset {
                    "true"
                } else {
                    "false"
                },
                tuned.cascade_gains.gravity_ff_mv,
            );

            if let Some(updated) = replace_between_markers(
                &content,
                "// [BEGIN_TUNED_GAINS]",
                "// [END_TUNED_GAINS]",
                &replacement_block,
            ) {
                fs::write(&cfg_path, updated)?;
                patched_files.push(cfg_path.display().to_string());
            }
        }

        // 2. Patch src/main.cpp between [BEGIN_MAIN_CONTROLLER_SETTINGS] and [END_MAIN_CONTROLLER_SETTINGS]
        let main_path = project_root.join("src/main.cpp");
        if main_path.exists() {
            let content = fs::read_to_string(&main_path)?;
            let main_block = format!(
                "// [BEGIN_MAIN_CONTROLLER_SETTINGS]\n\
                 lemlib::OdomSensors sensors(\n\
                 \x20   nullptr,\n\
                 \x20   nullptr,\n\
                 \x20   nullptr,\n\
                 \x20   nullptr,\n\
                 \x20   &imu\n\
                 );\n\n\
                 lemlib::ControllerSettings lat_controller(\n\
                 \x20   {:.4}f, // kP (Camera-in-the-Loop Tuned)\n\
                 \x20   {:.4}f, // kI\n\
                 \x20   {:.4}f, // kD\n\
                 \x20   {:.4}f, // windupRange\n\
                 \x20   {:.4}f, // smallError (in)\n\
                 \x20   {:.1}f, // smallErrorTimeout (ms)\n\
                 \x20   {:.4}f, // largeError (in)\n\
                 \x20   {:.1}f, // largeErrorTimeout (ms)\n\
                 \x20   {:.4}f  // slew\n\
                 );\n\n\
                 lemlib::ControllerSettings angular_controller(\n\
                 \x20   {:.4}f, // kP (Camera-in-the-Loop Tuned)\n\
                 \x20   {:.4}f, // kI\n\
                 \x20   {:.4}f, // kD\n\
                 \x20   {:.4}f, // windupRange\n\
                 \x20   {:.4}f, // smallError (deg)\n\
                 \x20   {:.1}f, // smallErrorTimeout (ms)\n\
                 \x20   {:.4}f, // largeError (deg)\n\
                 \x20   {:.1}f, // largeErrorTimeout (ms)\n\
                 \x20   {:.4}f  // slew\n\
                 );\n\n\
                 lemlib::Drivetrain drivetrain(\n\
                 \x20   &left_drive,\n\
                 \x20   &right_drive,\n\
                 \x20   robot_config::DRIVETRAIN_TRACK_WIDTH_IN,\n\
                 \x20   robot_config::DRIVETRAIN_WHEEL_DIAMETER_IN,\n\
                 \x20   robot_config::DRIVETRAIN_RPM,\n\
                 \x20   {:.4}f // horizontalDrift (Camera-in-the-Loop Tuned)\n\
                 );\n\
                 // [END_MAIN_CONTROLLER_SETTINGS]",
                tuned.lateral_gains.kp,
                tuned.lateral_gains.ki,
                tuned.lateral_gains.kd,
                tuned.lateral_gains.windup_range,
                tuned.lateral_gains.small_error,
                tuned.lateral_gains.small_error_timeout,
                tuned.lateral_gains.large_error,
                tuned.lateral_gains.large_error_timeout,
                tuned.lateral_gains.slew,
                tuned.angular_gains.kp,
                tuned.angular_gains.ki,
                tuned.angular_gains.kd,
                tuned.angular_gains.windup_range,
                tuned.angular_gains.small_error,
                tuned.angular_gains.small_error_timeout,
                tuned.angular_gains.large_error,
                tuned.angular_gains.large_error_timeout,
                tuned.angular_gains.slew,
                tuned.horizontal_drift,
            );

            if let Some(updated) = replace_between_markers(
                &content,
                "// [BEGIN_MAIN_CONTROLLER_SETTINGS]",
                "// [END_MAIN_CONTROLLER_SETTINGS]",
                &main_block,
            ) {
                fs::write(&main_path, updated)?;
                patched_files.push(main_path.display().to_string());
            }
        }

        Ok(patched_files)
    }

    /// Generates all high-resolution SVG plots via `plotters` and writes `TUNING_REPORT.md`.
    pub fn generate_report_and_svgs(
        project_root: &Path,
        hw: &RobotHardwareSpec,
        sensor_diag: &SensorDiagnostics,
        homography: &FieldHomography,
        tuned: &FullTuningResults,
        stress: &StressTestSuiteResults,
        patched_files: &[String],
    ) -> Result<()> {
        let plots_dir = project_root.join("reports/plots");
        fs::create_dir_all(&plots_dir)?;

        let lat_svg = plots_dir.join("lateral_step_response.svg");
        Self::render_step_comparison_svg(
            &lat_svg,
            "22050X Lateral Controller Step Response (24.0 in MoveToPoint)",
            "Position (inches)",
            24.0,
            &tuned.lateral_step_before,
            &tuned.lateral_step_after,
        )?;

        let ang_svg = plots_dir.join("angular_step_response.svg");
        Self::render_step_comparison_svg(
            &ang_svg,
            "22050X Angular Controller Step Response (90.0 deg TurnToHeading)",
            "Heading (degrees CW)",
            90.0,
            &tuned.angular_step_before,
            &tuned.angular_step_after,
        )?;

        let boom_svg = plots_dir.join("boomerang_trajectory_overlay.svg");
        Self::render_boomerang_trajectory_svg(
            &boom_svg,
            &tuned.boomerang_traj_cam,
            &tuned.boomerang_traj_odom,
        )?;

        let lift_svg = plots_dir.join("cascade_lift_step.svg");
        Self::render_step_comparison_svg(
            &lift_svg,
            "22050X 2-Motor 6:1 Cascade Lift Step Response (18.0 in + Gravity FF)",
            "Lift Height (inches)",
            18.0,
            &tuned.cascade_step_samples,
            &tuned.cascade_step_samples,
        )?;

        let end_svg = plots_dir.join("endurance_thermal_drift.svg");
        Self::render_endurance_svg(&end_svg, stress)?;

        let imp_svg = plots_dir.join("impulse_disturbance_rejection.svg");
        Self::render_impulse_svg(&imp_svg, stress)?;

        // Build TUNING_REPORT.md
        let mut md = String::with_capacity(16384);
        md.push_str("# 22050X Markerless Camera-in-the-Loop LemLib PID Tuning & Stress-Test Engineering Report\n\n");
        md.push_str("## 1. Executive Summary & System Architecture\n\n");
        md.push_str(&format!(
            "- **Target Robot Platform:** VEX V5 `{}` (PROS Kernel `{}`, LemLib `{}`)\n",
            hw.project_name, hw.pros_kernel_version, hw.lemlib_version
        ));
        md.push_str(&format!(
            "- **Vision Sensor Suite:** `{}` (`{}`) + `{}` (`{}`)\n",
            sensor_diag.ir_group_name,
            sensor_diag.ir_source_id,
            sensor_diag.rgb_group_name,
            sensor_diag.rgb_source_id
        ));
        md.push_str(&format!(
            "- **Active NIR Torch Illumination:** Supported = `{}`, Enabled = `{}` (Nokhwa MSMF Fallback = `{}`)\n",
            sensor_diag.active_nir_torch_supported,
            sensor_diag.active_nir_torch_enabled,
            sensor_diag.nokhwa_fallback_available
        ));
        md.push_str(&format!(
            "- **24\"x24\" Foam Tile Homography Calibration:** Detected Seam Lines = `{}`, Grid Intersections = `{}`, Metric Reprojection RMSE = `{:.4} in`\n",
            homography.detected_seams.len(),
            homography.grid_intersections_px.len(),
            homography.reprojection_rmse_in
        ));
        md.push_str(&format!(
            "- **Overall Stress-Test Suite Verdict:** **{}**\n\n",
            if stress.all_passed { "ALL TESTS PASSED" } else { "ATTENTION REQUIRED" }
        ));

        md.push_str("## 2. Extracted 22050X Hardware & Subsystem Topology\n\n");
        md.push_str("| Subsystem | Motor / Sensor Name | Port | Direction / Polarity | Cartridge / Spec |\n");
        md.push_str("| :--- | :--- | :---: | :---: | :--- |\n");
        for m in &hw.drivetrain_motors {
            md.push_str(&format!(
                "| Drivetrain | `{}` | `{}` | `{}` | {:.0} RPM (6:1 Blue, 4.0\" Omni, {:.1}\" Track) |\n",
                m.name,
                m.port,
                if m.reversed { "Reversed (-)" } else { "Forward (+)" },
                m.cartridge_rpm,
                hw.track_width_in
            ));
        }
        for m in &hw.cascade_motors {
            md.push_str(&format!(
                "| Cascade Lift | `{}` | `{}` | `{}` | {:.0} RPM (6:1 Blue Cartridge, Hold Brake) |\n",
                m.name,
                m.port,
                if m.reversed { "Reversed (-)" } else { "Forward (+)" },
                m.cartridge_rpm
            ));
        }
        for m in &hw.auxiliary_motors {
            md.push_str(&format!(
                "| Auxiliary / End-Effector | `{}` | `{}` | `{}` | {:.0} RPM |\n",
                m.name,
                m.port,
                if m.reversed { "Reversed (-)" } else { "Forward (+)" },
                m.cartridge_rpm
            ));
        }
        md.push_str(&format!(
            "| Inertial Sensor | `imu` | `{}` | Clockwise (+) | V5 Inertial Sensor |\n",
            hw.imu_port
        ));
        for tw in &hw.tracking_wheels {
            md.push_str(&format!(
                "| Odometry Wheel | `{}` | `{}` | Offset `{:.2} in` | {:.2}\" Omniwheel (`NEW_2`) |\n",
                tw.name, tw.port, tw.offset_in, tw.diameter_in
            ));
        }
        md.push('\n');

        md.push_str("## 3. Verified & Locked LemLib `ControllerSettings` and Subsystem PID Gains\n\n");
        md.push_str("| Controller | `kP` | `kI` | `kD` | `windupRange` | `smallError` | `smallTimeout` (ms) | `largeError` | `largeTimeout` (ms) | `slew` | Extra Parameters |\n");
        md.push_str("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |\n");
        md.push_str(&format!(
            "| `LEMLIB_LATERAL` | `{:.4}` | `{:.4}` | `{:.4}` | `{:.2}` | `{:.2} in` | `{:.0}` | `{:.2} in` | `{:.0}` | `{:.2}` | `signFlipReset = true` |\n",
            tuned.lateral_gains.kp,
            tuned.lateral_gains.ki,
            tuned.lateral_gains.kd,
            tuned.lateral_gains.windup_range,
            tuned.lateral_gains.small_error,
            tuned.lateral_gains.small_error_timeout,
            tuned.lateral_gains.large_error,
            tuned.lateral_gains.large_error_timeout,
            tuned.lateral_gains.slew
        ));
        md.push_str(&format!(
            "| `LEMLIB_ANGULAR` | `{:.4}` | `{:.4}` | `{:.4}` | `{:.2}` | `{:.2}°` | `{:.0}` | `{:.2}°` | `{:.0}` | `{:.2}` | `signFlipReset = true` |\n",
            tuned.angular_gains.kp,
            tuned.angular_gains.ki,
            tuned.angular_gains.kd,
            tuned.angular_gains.windup_range,
            tuned.angular_gains.small_error,
            tuned.angular_gains.small_error_timeout,
            tuned.angular_gains.large_error,
            tuned.angular_gains.large_error_timeout,
            tuned.angular_gains.slew
        ));
        md.push_str(&format!(
            "| `LEMLIB_BOOMERANG` | `{:.4}` | `{:.4}` | `{:.4}` | `{:.2}` | `{:.2} in` | `{:.0}` | `{:.2} in` | `{:.0}` | `{:.2}` | `horizontalDrift = {:.3}`, `lead = {:.3}` |\n",
            tuned.lateral_gains.kp,
            tuned.lateral_gains.ki,
            tuned.lateral_gains.kd,
            tuned.lateral_gains.windup_range,
            tuned.lateral_gains.small_error,
            tuned.lateral_gains.small_error_timeout,
            tuned.lateral_gains.large_error,
            tuned.lateral_gains.large_error_timeout,
            tuned.lateral_gains.slew,
            tuned.horizontal_drift,
            tuned.boomerang_lead
        ));
        md.push_str(&format!(
            "| `LEMLIB_SWING` | `{:.4}` | `{:.4}` | `{:.4}` | `{:.2}` | `{:.2}°` | `{:.0}` | `{:.2}°` | `{:.0}` | `{:.2}` | Dual-side `DriveSide::LEFT/RIGHT` |\n",
            tuned.swing_gains.kp,
            tuned.swing_gains.ki,
            tuned.swing_gains.kd,
            tuned.swing_gains.windup_range,
            tuned.swing_gains.small_error,
            tuned.swing_gains.small_error_timeout,
            tuned.swing_gains.large_error,
            tuned.swing_gains.large_error_timeout,
            tuned.swing_gains.slew
        ));
        md.push_str(&format!(
            "| `CASCADE_LIFT` | `{:.4}` | `{:.4}` | `{:.4}` | `{:.2}` | `0.20 in` | `70` | `0.70 in` | `240` | `0.00` | `gravityFF = {:.0} mV`, `signFlip = {}` |\n\n",
            tuned.cascade_gains.kp,
            tuned.cascade_gains.ki,
            tuned.cascade_gains.kd,
            tuned.cascade_gains.windup_range,
            tuned.cascade_gains.gravity_ff_mv,
            tuned.cascade_gains.sign_flip_reset
        ));

        md.push_str("## 4. Five-Stage `argmin` Optimization Convergence Log\n\n");
        md.push_str("| Stage | Pipeline Stage Description | Target Controller | Evaluations | Initial Cost $J_0$ | Final Cost $J^*$ | Rise Time $t_r$ (s) | Overshoot $M_p$ (%) | Settling Time $t_s$ (s) | Steady Error $e_{ss}$ |\n");
        md.push_str("| :---: | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n");
        for sr in &tuned.stage_records {
            md.push_str(&format!(
                "| {} | {} | `{}` | {} | `{:.2}` | **`{:.2}`** | `{:.3}` | `{:.2}%` | `{:.3}` | `{:.4}` |\n",
                sr.stage_number,
                sr.stage_name,
                sr.controller_name,
                sr.evaluations,
                sr.initial_cost_j,
                sr.final_cost_j,
                sr.final_metrics.rise_time_s,
                sr.final_metrics.peak_overshoot_pct,
                sr.final_metrics.settling_time_s,
                sr.final_metrics.steady_state_error
            ));
        }
        md.push('\n');

        md.push_str("## 5. Odometry vs. Vision Ground-Truth Slip Analysis\n\n");
        md.push_str(&format!(
            "- **Motion-Onset Cross-Correlation Clock Offset:** `{:.1} ms` (Correlation Peak = `{:.4}`)\n",
            tuned.slip_summary.estimated_clock_offset_ms,
            tuned.slip_summary.cross_correlation_peak
        ));
        md.push_str(&format!(
            "- **RMS Odometry vs. NIR Vision Pose Divergence ($E_{{\\text{{odom}}}}$):** `{:.4} in` (Peak = `{:.4} in`)\n",
            tuned.slip_summary.rms_position_divergence_in,
            tuned.slip_summary.peak_position_divergence_in
        ));
        md.push_str(&format!(
            "- **Initial 200 ms Longitudinal Wheel Slip:** `{:.4} in`\n",
            tuned.slip_summary.startup_200ms_wheel_slip_in
        ));
        md.push_str(&format!(
            "- **Maximum Lateral Omni-Wheel Scrub:** `{:.4} in`\n",
            tuned.slip_summary.max_lateral_omni_scrub_in
        ));
        md.push_str(&format!(
            "- **IMU Heading Scale Drift:** `{:.4}°`\n\n",
            tuned.slip_summary.imu_heading_drift_deg
        ));

        md.push_str("## 6. High-Resolution Step-Response & Trajectory Plots\n\n");
        md.push_str("### 6.1 Lateral Controller Step Response (Before vs. After Tuning)\n");
        md.push_str("![Lateral Step Response](reports/plots/lateral_step_response.svg)\n\n");
        md.push_str("### 6.2 Angular Controller Step Response (Before vs. After Tuning)\n");
        md.push_str("![Angular Step Response](reports/plots/angular_step_response.svg)\n\n");
        md.push_str("### 6.3 Boomerang `moveToPose(24, 24, 90°)` Homography Grid Trajectory Overlay\n");
        md.push_str("![Boomerang Trajectory Overlay](reports/plots/boomerang_trajectory_overlay.svg)\n\n");
        md.push_str("### 6.4 Cascade Lift 6:1 PID + Gravity Feedforward Step Response\n");
        md.push_str("![Cascade Lift Step Response](reports/plots/cascade_lift_step.svg)\n\n");
        md.push_str("### 6.5 15-Lap Chained High-Speed Endurance Thermal & Odometry Drift Profile\n");
        md.push_str("![Endurance Thermal and Drift](reports/plots/endurance_thermal_drift.svg)\n\n");
        md.push_str("### 6.6 Impulse Disturbance Rejection at `(24, 24, 90°)`\n");
        md.push_str("![Impulse Disturbance Rejection](reports/plots/impulse_disturbance_rejection.svg)\n\n");

        md.push_str("## 7. Exhaustive Multi-Axis Stress-Testing Matrices\n\n");
        md.push_str("### 7.1 Scale Invariance Matrix ($2\\text{ in} \\dots 72\\text{ in}$ Lateral, $10^\\circ \\dots 180^\\circ$ Point & Swing Turns)\n\n");
        md.push_str("| Category | Command | Target | Rise Time $t_r$ (s) | Overshoot $M_p$ (%) | Settling Time $t_s$ (s) | Steady Error $e_{ss}$ | Odom Divergence $E_{\\text{odom}}$ | Status |\n");
        md.push_str("| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n");
        for r in &stress.scale_invariance {
            md.push_str(&format!(
                "| {} | `{}` | `{:.0} {}` | `{:.3}` | `{:.2}%` | `{:.3}` | `{:.4} {}` | `{:.4}` | **{}** |\n",
                r.category,
                r.command_label,
                r.target_magnitude,
                r.unit,
                r.rise_time_s,
                r.peak_overshoot_pct,
                r.settling_time_s,
                r.steady_state_error,
                r.unit,
                r.odom_divergence,
                if r.passed { "PASS" } else { "FAIL" }
            ));
        }
        md.push('\n');

        md.push_str("### 7.2 Cascade Lift Center-of-Gravity (CG) Perturbation Matrix\n\n");
        md.push_str("| Cascade Lift Extension | Lift Height (in) | Drivetrain Motion | Rise Time $t_r$ (s) | Overshoot $M_p$ (%) | Settling Time $t_s$ (s) | Steady Error $e_{ss}$ | Oscillations $N_{\\text{osc}}$ | Status |\n");
        md.push_str("| :--- | :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |\n");
        for r in &stress.cg_perturbation {
            md.push_str(&format!(
                "| {} | `{:.1}` | `{}` | `{:.3}` | `{:.2}%` | `{:.3}` | `{:.4}` | `{}` | **{}** |\n",
                r.lift_state_label,
                r.lift_height_in,
                r.motion_label,
                r.rise_time_s,
                r.peak_overshoot_pct,
                r.settling_time_s,
                r.steady_state_error,
                r.oscillation_count,
                if r.passed { "PASS" } else { "FAIL" }
            ));
        }
        md.push('\n');

        md.push_str("### 7.3 Chained High-Speed 15-Lap Endurance & Thermal Torture Test (`minSpeed = 48`, `earlyExitRange = 2.5 in`)\n\n");
        md.push_str("| Lap # | Elapsed Time (s) | Cumulative Odom Drift (in) | Heading Drift (°) | Left Drive Temp (°C) | Right Drive Temp (°C) | Cascade Lift Temp (°C) | Margin to 55°C Throttle | Status |\n");
        md.push_str("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n");
        for r in &stress.endurance_laps {
            md.push_str(&format!(
                "| {} | `{:.2}` | `{:.4} in` | `{:.3}°` | `{:.2}°C` | `{:.2}°C` | `{:.2}°C` | `+{:.2}°C` | **{}** |\n",
                r.lap_number,
                r.elapsed_s,
                r.cumulative_odom_drift_in,
                r.heading_drift_deg,
                r.left_motor_temp_c,
                r.right_motor_temp_c,
                r.lift_motor_temp_c,
                r.thermal_margin_c,
                if r.passed { "PASS" } else { "FAIL" }
            ));
        }
        md.push('\n');

        md.push_str("### 7.4 External Impulse Disturbance Rejection at `(24, 24, 90°)`\n\n");
        md.push_str("| Disturbance Scenario | Applied External Impulse | Peak NIR Displacement (in) | Peak Heading Deflection (°) | Recovery Time into Band (ms) | Final Pos Error (in) | Final Heading Error (°) | Status |\n");
        md.push_str("| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |\n");
        for r in &stress.impulse_disturbances {
            md.push_str(&format!(
                "| {} | {} | `{:.3} in` | `{:.2}°` | `{:.0} ms` | `{:.4} in` | `{:.4}°` | **{}** |\n",
                r.disturbance_label,
                r.applied_impulse_desc,
                r.peak_displacement_in,
                r.peak_heading_deflection_deg,
                r.recovery_time_ms,
                r.final_settled_pos_err_in,
                r.final_settled_ang_err_deg,
                if r.passed { "PASS" } else { "FAIL" }
            ));
        }
        md.push('\n');

        md.push_str("## 8. Automated C++ Firmware Patch Manifest\n\n");
        for pf in patched_files {
            md.push_str(&format!("- Patched `{}`\n", pf));
        }

        let report_path = project_root.join("TUNING_REPORT.md");
        fs::write(&report_path, md)?;
        Ok(())
    }

    fn render_step_comparison_svg(
        out_path: &Path,
        title: &str,
        y_label: &str,
        target_val: f64,
        before: &[StepResponseSample],
        after: &[StepResponseSample],
    ) -> Result<()> {
        let root = SVGBackend::new(out_path, (960, 520)).into_drawing_area();
        root.fill(&WHITE)?;

        let max_t = before
            .last()
            .map(|s| s.time_s)
            .unwrap_or(1.2)
            .max(after.last().map(|s| s.time_s).unwrap_or(1.2))
            .max(0.5);
        let max_y = (target_val * 1.25).max(5.0);

        let mut chart = ChartBuilder::on(&root)
            .caption(title, ("sans-serif", 22).into_font())
            .margin(20)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(0.0..max_t, -1.0..max_y)?;

        chart
            .configure_mesh()
            .x_desc("Time (seconds)")
            .y_desc(y_label)
            .draw()?;

        // Target reference line
        chart
            .draw_series(LineSeries::new(
                vec![(0.0, target_val), (max_t, target_val)],
                ShapeStyle::from(&BLACK.mix(0.6)).stroke_width(2),
            ))?
            .label("Step Reference Target")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLACK));

        // Untuned (Before)
        chart
            .draw_series(LineSeries::new(
                before.iter().map(|s| (s.time_s, s.measured_cam)),
                ShapeStyle::from(&RED.mix(0.75)).stroke_width(2),
            ))?
            .label("Baseline Untuned (NIR Ground-Truth)")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

        // Tuned NIR Camera Ground-Truth
        chart
            .draw_series(LineSeries::new(
                after.iter().map(|s| (s.time_s, s.measured_cam)),
                ShapeStyle::from(&BLUE).stroke_width(3),
            ))?
            .label("Locked Tuned (NIR Camera Ground-Truth)")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

        // Tuned LemLib Odometry
        chart
            .draw_series(LineSeries::new(
                after.iter().map(|s| (s.time_s, s.measured_odom)),
                ShapeStyle::from(&GREEN.mix(0.85)).stroke_width(2),
            ))?
            .label("Locked Tuned (LemLib chassis.getPose())")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], GREEN));

        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.85))
            .border_style(BLACK)
            .draw()?;

        root.present()?;
        Ok(())
    }

    fn render_boomerang_trajectory_svg(
        out_path: &Path,
        cam_pts: &[(f64, f64, f64)],
        odom_pts: &[(f64, f64, f64)],
    ) -> Result<()> {
        let root = SVGBackend::new(out_path, (820, 620)).into_drawing_area();
        root.fill(&WHITE)?;

        let mut chart = ChartBuilder::on(&root)
            .caption(
                "22050X Boomerang moveToPose(24, 24, 90°) Homography Field Trajectory Overlay",
                ("sans-serif", 20).into_font(),
            )
            .margin(20)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(-6.0..32.0, -4.0..30.0)?;

        chart
            .configure_mesh()
            .x_desc("Field X (inches)")
            .y_desc("Field Y (inches)")
            .draw()?;

        // Draw 24"x24" foam tile seam lines
        chart.draw_series(LineSeries::new(
            vec![(0.0, -4.0), (0.0, 30.0)],
            ShapeStyle::from(&BLACK.mix(0.25)).stroke_width(1),
        ))?;
        chart.draw_series(LineSeries::new(
            vec![(24.0, -4.0), (24.0, 30.0)],
            ShapeStyle::from(&BLACK.mix(0.25)).stroke_width(1),
        ))?;
        chart.draw_series(LineSeries::new(
            vec![(-6.0, 24.0), (32.0, 24.0)],
            ShapeStyle::from(&BLACK.mix(0.25)).stroke_width(1),
        ))?;

        chart
            .draw_series(LineSeries::new(
                cam_pts.iter().map(|&(x, y, _)| (x, y)),
                ShapeStyle::from(&BLUE).stroke_width(3),
            ))?
            .label("Active NIR + RGB Camera Ground-Truth Trajectory")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

        chart
            .draw_series(LineSeries::new(
                odom_pts.iter().map(|&(x, y, _)| (x, y)),
                ShapeStyle::from(&RED.mix(0.8)).stroke_width(2),
            ))?
            .label("LemLib Internal Odometry (chassis.getPose())")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.85))
            .border_style(BLACK)
            .draw()?;

        root.present()?;
        Ok(())
    }

    fn render_endurance_svg(out_path: &Path, stress: &StressTestSuiteResults) -> Result<()> {
        let root = SVGBackend::new(out_path, (960, 500)).into_drawing_area();
        root.fill(&WHITE)?;

        let mut chart = ChartBuilder::on(&root)
            .caption(
                "15-Lap Chained High-Speed Endurance Motor Temperatures vs. 55°C PROS Limit",
                ("sans-serif", 20).into_font(),
            )
            .margin(20)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(1.0..15.0, 20.0..60.0)?;

        chart
            .configure_mesh()
            .x_desc("Endurance Circuit Lap Number")
            .y_desc("Temperature (°C)")
            .draw()?;

        chart
            .draw_series(LineSeries::new(
                vec![(1.0, 55.0), (15.0, 55.0)],
                ShapeStyle::from(&RED).stroke_width(2),
            ))?
            .label("PROS Thermal Throttling Threshold (55.0 °C)")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

        chart
            .draw_series(LineSeries::new(
                stress
                    .endurance_laps
                    .iter()
                    .map(|r| (r.lap_number as f64, r.left_motor_temp_c)),
                ShapeStyle::from(&BLUE).stroke_width(3),
            ))?
            .label("Left Drivetrain Motor Group Temp (°C)")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

        chart
            .draw_series(LineSeries::new(
                stress
                    .endurance_laps
                    .iter()
                    .map(|r| (r.lap_number as f64, r.cascade_motor_temp_c())),
                ShapeStyle::from(&GREEN).stroke_width(2),
            ))?
            .label("Cascade Lift Motor Group Temp (°C)")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], GREEN));

        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.85))
            .border_style(BLACK)
            .draw()?;

        root.present()?;
        Ok(())
    }

    fn render_impulse_svg(out_path: &Path, stress: &StressTestSuiteResults) -> Result<()> {
        let root = SVGBackend::new(out_path, (960, 500)).into_drawing_area();
        root.fill(&WHITE)?;

        let mut chart = ChartBuilder::on(&root)
            .caption(
                "Stationary Pose Hold (24, 24, 90°) External Impulse Disturbance Rejection",
                ("sans-serif", 20).into_font(),
            )
            .margin(20)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(0.0..0.95, 0.0..2.5)?;

        chart
            .configure_mesh()
            .x_desc("Time (seconds)")
            .y_desc("NIR Camera Displacement from (24, 24) (inches)")
            .draw()?;

        chart
            .draw_series(LineSeries::new(
                vec![(0.0, 0.20), (0.95, 0.20)],
                ShapeStyle::from(&BLACK.mix(0.6)).stroke_width(2),
            ))?
            .label("Settling Band Target (±0.20 in)")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLACK));

        if let Some(first) = stress.impulse_disturbances.first() {
            chart
                .draw_series(LineSeries::new(
                    first.trajectory_samples.iter().map(|&(t, p, _)| (t, p)),
                    ShapeStyle::from(&BLUE).stroke_width(3),
                ))?
                .label(&first.disturbance_label)
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));
        }

        if let Some(second) = stress.impulse_disturbances.get(1) {
            chart
                .draw_series(LineSeries::new(
                    second.trajectory_samples.iter().map(|&(t, p, _)| (t, p)),
                    ShapeStyle::from(&MAGENTA).stroke_width(3),
                ))?
                .label(&second.disturbance_label)
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], MAGENTA));
        }

        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.85))
            .border_style(BLACK)
            .draw()?;

        root.present()?;
        Ok(())
    }
}

impl crate::stress_test::EnduranceLapRow {
    pub fn cascade_motor_temp_c(&self) -> f64 {
        self.lift_motor_temp_c
    }
}

fn replace_between_markers(
    source: &str,
    start_marker: &str,
    end_marker: &str,
    replacement: &str,
) -> Option<String> {
    let start_idx = source.find(start_marker)?;
    let end_idx = source.find(end_marker)?;
    if end_idx < start_idx {
        return None;
    }
    let after_end = end_idx + end_marker.len();
    let mut out = String::with_capacity(source.len() + 512);
    out.push_str(&source[..start_idx]);
    out.push_str(replacement);
    out.push_str(&source[after_end..]);
    Some(out)
}
