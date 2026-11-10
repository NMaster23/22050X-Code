use anyhow::Result;
use bytemuck::Zeroable;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

use crate::optimizer::metrics::TransientMetrics;
use crate::optimizer::pipeline::{ClosedLoopHarness, FullTuningResults};
use crate::serial_bridge::{
    trigger_flags, ControllerId, HostCommandPacket, MotionCommandType,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScaleInvarianceRow {
    pub category: String,
    pub command_label: String,
    pub target_magnitude: f64,
    pub unit: String,
    pub rise_time_s: f64,
    pub peak_overshoot_pct: f64,
    pub settling_time_s: f64,
    pub steady_state_error: f64,
    pub odom_divergence: f64,
    pub passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CgPerturbationRow {
    pub lift_state_label: String,
    pub lift_height_in: f64,
    pub motion_label: String,
    pub rise_time_s: f64,
    pub peak_overshoot_pct: f64,
    pub settling_time_s: f64,
    pub steady_state_error: f64,
    pub oscillation_count: u32,
    pub passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnduranceLapRow {
    pub lap_number: usize,
    pub elapsed_s: f64,
    pub cumulative_odom_drift_in: f64,
    pub heading_drift_deg: f64,
    pub left_motor_temp_c: f64,
    pub right_motor_temp_c: f64,
    pub lift_motor_temp_c: f64,
    pub thermal_margin_c: f64,
    pub passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImpulseDisturbanceRecord {
    pub disturbance_label: String,
    pub applied_impulse_desc: String,
    pub peak_displacement_in: f64,
    pub peak_heading_deflection_deg: f64,
    pub recovery_time_ms: f64,
    pub final_settled_pos_err_in: f64,
    pub final_settled_ang_err_deg: f64,
    pub trajectory_samples: Vec<(f64, f64, f64)>, // (time_s, pos_err_in, ang_err_deg)
    pub passed: bool,
}

#[derive(Debug, Clone)]
pub struct StressTestSuiteResults {
    pub scale_invariance: Vec<ScaleInvarianceRow>,
    pub cg_perturbation: Vec<CgPerturbationRow>,
    pub endurance_laps: Vec<EnduranceLapRow>,
    pub impulse_disturbances: Vec<ImpulseDisturbanceRecord>,
    pub all_passed: bool,
}

pub struct StressTestRunner;

impl StressTestRunner {
    pub fn execute_all(
        harness: &Arc<Mutex<ClosedLoopHarness>>,
        tuned: &FullTuningResults,
    ) -> Result<StressTestSuiteResults> {
        let mut scale_rows = Vec::new();
        let mut cg_rows = Vec::new();
        let mut lap_rows = Vec::new();
        let mut impulse_rows = Vec::new();

        // ====================================================================
        // 1. SCALE INVARIANCE MATRIX
        // Lateral: 2 in, 6 in, 24 in, 48 in, 72 in (Forward & Reverse)
        // Angular: 10 deg, 45 deg, 90 deg, 135 deg, 180 deg (turnToHeading & swingToHeading)
        // ====================================================================
        for &dist_in in &[2.0f32, 6.0, 24.0, 48.0, 72.0] {
            let max_steps = ((dist_in * 2.2) as usize).clamp(75, 220);
            let (samples, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibLateral,
                MotionCommandType::MoveToPointFwd,
                &tuned.lateral_gains,
                tuned.horizontal_drift,
                tuned.boomerang_lead,
                true,
                0.0,
                dist_in,
                0.0,
                0.0,
                tuned.cascade_gains.gravity_ff_mv,
                max_steps,
                true,
            )?;
            let m = TransientMetrics::extract(&samples, dist_in as f64, 0.25);
            let passed = m.peak_overshoot_pct < 1.5 && m.steady_state_error < 0.15;
            scale_rows.push(ScaleInvarianceRow {
                category: "Lateral (moveToPoint)".to_string(),
                command_label: format!("moveToPoint(0, {:.0})", dist_in),
                target_magnitude: dist_in as f64,
                unit: "in".to_string(),
                rise_time_s: m.rise_time_s,
                peak_overshoot_pct: m.peak_overshoot_pct,
                settling_time_s: m.settling_time_s,
                steady_state_error: m.steady_state_error,
                odom_divergence: m.odom_slip_divergence,
                passed,
            });
        }

        for &angle_deg in &[10.0f32, 45.0, 90.0, 135.0, 180.0] {
            let max_steps = ((angle_deg * 0.9) as usize).clamp(70, 180);
            // Point Turn (turnToHeading)
            let (samples_pt, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibAngular,
                MotionCommandType::TurnToHeading,
                &tuned.angular_gains,
                tuned.horizontal_drift,
                tuned.boomerang_lead,
                true,
                0.0,
                0.0,
                angle_deg,
                0.0,
                tuned.cascade_gains.gravity_ff_mv,
                max_steps,
                true,
            )?;
            let m_pt = TransientMetrics::extract(&samples_pt, angle_deg as f64, 0.45);
            let passed_pt = m_pt.peak_overshoot_pct < 1.5 && m_pt.steady_state_error < 0.30;
            scale_rows.push(ScaleInvarianceRow {
                category: "Angular (turnToHeading)".to_string(),
                command_label: format!("turnToHeading({:.0}°)", angle_deg),
                target_magnitude: angle_deg as f64,
                unit: "deg".to_string(),
                rise_time_s: m_pt.rise_time_s,
                peak_overshoot_pct: m_pt.peak_overshoot_pct,
                settling_time_s: m_pt.settling_time_s,
                steady_state_error: m_pt.steady_state_error,
                odom_divergence: m_pt.odom_slip_divergence,
                passed: passed_pt,
            });

            // Swing Turn (swingToHeading)
            let (samples_sw, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibSwing,
                MotionCommandType::SwingToHeadingLeft,
                &tuned.swing_gains,
                tuned.horizontal_drift,
                tuned.boomerang_lead,
                true,
                0.0,
                0.0,
                angle_deg,
                0.0,
                tuned.cascade_gains.gravity_ff_mv,
                max_steps + 15,
                true,
            )?;
            let m_sw = TransientMetrics::extract(&samples_sw, angle_deg as f64, 0.50);
            let passed_sw = m_sw.peak_overshoot_pct < 1.5 && m_sw.steady_state_error < 0.30;
            scale_rows.push(ScaleInvarianceRow {
                category: "Swing (swingToHeading)".to_string(),
                command_label: format!("swingToHeading({:.0}°, LEFT)", angle_deg),
                target_magnitude: angle_deg as f64,
                unit: "deg".to_string(),
                rise_time_s: m_sw.rise_time_s,
                peak_overshoot_pct: m_sw.peak_overshoot_pct,
                settling_time_s: m_sw.settling_time_s,
                steady_state_error: m_sw.steady_state_error,
                odom_divergence: m_sw.odom_slip_divergence,
                passed: passed_sw,
            });
        }

        // ====================================================================
        // 2. CASCADE LIFT CG PERTURBATION MATRIX
        // Retracted (0 in), Mid-Extended (14 in), Fully Raised (28 in)
        // across Lateral (24 in), Angular (90 deg), and Boomerang (24, 24, 90 deg)
        // ====================================================================
        for &(lift_label, lift_h) in &[
            ("Fully Retracted (0 in)", 0.0f32),
            ("Mid-Extended (14 in)", 14.0f32),
            ("Fully Raised (28 in)", 28.0f32),
        ] {
            harness.lock().unwrap().transceiver.twin.lift_height_in = lift_h;

            // Lateral 24 in under CG shift
            let (s_lat, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibLateral,
                MotionCommandType::MoveToPointFwd,
                &tuned.lateral_gains,
                tuned.horizontal_drift,
                tuned.boomerang_lead,
                true,
                0.0,
                24.0,
                0.0,
                lift_h,
                tuned.cascade_gains.gravity_ff_mv,
                135,
                true,
            )?;
            let m_lat = TransientMetrics::extract(&s_lat, 24.0, 0.25);
            cg_rows.push(CgPerturbationRow {
                lift_state_label: lift_label.to_string(),
                lift_height_in: lift_h as f64,
                motion_label: "Lateral 24 in (moveToPoint)".to_string(),
                rise_time_s: m_lat.rise_time_s,
                peak_overshoot_pct: m_lat.peak_overshoot_pct,
                settling_time_s: m_lat.settling_time_s,
                steady_state_error: m_lat.steady_state_error,
                oscillation_count: m_lat.oscillation_count,
                passed: m_lat.peak_overshoot_pct < 1.8 && m_lat.steady_state_error < 0.15,
            });

            // Angular 90 deg under CG shift
            let (s_ang, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibAngular,
                MotionCommandType::TurnToHeading,
                &tuned.angular_gains,
                tuned.horizontal_drift,
                tuned.boomerang_lead,
                true,
                0.0,
                0.0,
                90.0,
                lift_h,
                tuned.cascade_gains.gravity_ff_mv,
                125,
                true,
            )?;
            let m_ang = TransientMetrics::extract(&s_ang, 90.0, 0.45);
            cg_rows.push(CgPerturbationRow {
                lift_state_label: lift_label.to_string(),
                lift_height_in: lift_h as f64,
                motion_label: "Angular 90° (turnToHeading)".to_string(),
                rise_time_s: m_ang.rise_time_s,
                peak_overshoot_pct: m_ang.peak_overshoot_pct,
                settling_time_s: m_ang.settling_time_s,
                steady_state_error: m_ang.steady_state_error,
                oscillation_count: m_ang.oscillation_count,
                passed: m_ang.peak_overshoot_pct < 1.8 && m_ang.steady_state_error < 0.30,
            });

            // Boomerang (24, 24, 90 deg) under CG shift
            let (s_boom, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibBoomerang,
                MotionCommandType::MoveToPose,
                &tuned.lateral_gains,
                tuned.horizontal_drift,
                tuned.boomerang_lead,
                true,
                24.0,
                24.0,
                90.0,
                lift_h,
                tuned.cascade_gains.gravity_ff_mv,
                150,
                true,
            )?;
            let m_boom = TransientMetrics::extract(&s_boom, 33.94, 0.35);
            cg_rows.push(CgPerturbationRow {
                lift_state_label: lift_label.to_string(),
                lift_height_in: lift_h as f64,
                motion_label: "Boomerang (24, 24, 90°)".to_string(),
                rise_time_s: m_boom.rise_time_s,
                peak_overshoot_pct: m_boom.peak_overshoot_pct,
                settling_time_s: m_boom.settling_time_s,
                steady_state_error: m_boom.steady_state_error,
                oscillation_count: m_boom.oscillation_count,
                passed: m_boom.peak_overshoot_pct < 2.0 && m_boom.steady_state_error < 0.25,
            });
        }

        // ====================================================================
        // 3. CHAINED HIGH-SPEED 15-LAP ENDURANCE & THERMAL TORTURE TEST
        // Uses LemLib .minSpeed = 48 and .earlyExitRange = 2.5 across 15 continuous laps
        // ====================================================================
        {
            let mut h = harness.lock().unwrap();
            h.transceiver.twin.true_x = 0.0;
            h.transceiver.twin.true_y = 0.0;
            h.transceiver.twin.true_theta_deg = 0.0;
            h.transceiver.twin.odom_x = 0.0;
            h.transceiver.twin.odom_y = 0.0;
            h.transceiver.twin.odom_theta_deg = 0.0;
            h.transceiver.twin.lift_height_in = 0.0;

            let waypoints: [(f32, f32, f32); 4] = [
                (0.0, 36.0, 90.0),
                (36.0, 36.0, 180.0),
                (36.0, 0.0, 270.0),
                (0.0, 0.0, 0.0),
            ];

            let mut total_elapsed_s = 0.0f64;

            for lap in 1..=15 {
                for &(wx, wy, wth) in &waypoints {
                    let mut cmd = HostCommandPacket::zeroed();
                    cmd.seq = h.seq_counter;
                    h.seq_counter = h.seq_counter.wrapping_add(1);
                    cmd.controller_id = ControllerId::LemlibBoomerang as u8;
                    cmd.motion_type = MotionCommandType::ChainedEnduranceLap as u8;
                    cmd.trigger_flags =
                        trigger_flags::EXECUTE_MOTION | trigger_flags::HOT_RELOAD_GAINS;
                    cmd.kp = tuned.lateral_gains.kp;
                    cmd.ki = tuned.lateral_gains.ki;
                    cmd.kd = tuned.lateral_gains.kd;
                    cmd.windup_range = tuned.lateral_gains.windup_range;
                    cmd.small_error = tuned.lateral_gains.small_error;
                    cmd.small_error_timeout = tuned.lateral_gains.small_error_timeout;
                    cmd.large_error = tuned.lateral_gains.large_error;
                    cmd.large_error_timeout = tuned.lateral_gains.large_error_timeout;
                    cmd.slew = tuned.lateral_gains.slew;
                    cmd.horizontal_drift = tuned.horizontal_drift;
                    cmd.lead = tuned.boomerang_lead;
                    cmd.max_speed = 120.0;
                    cmd.min_speed = 48.0;
                    cmd.early_exit_range = 2.5;
                    cmd.target_x = wx;
                    cmd.target_y = wy;
                    cmd.target_theta = wth;
                    cmd.timeout_ms = 650;

                    for step in 0..42 {
                        if step > 0 {
                            cmd.motion_type = MotionCommandType::Heartbeat as u8;
                            cmd.trigger_flags = 0;
                            cmd.seq = h.seq_counter;
                            h.seq_counter = h.seq_counter.wrapping_add(1);
                        }
                        let telem = h.transceiver.transceive_100hz(&mut cmd, 0.010)?;
                        total_elapsed_s += 0.010;
                        if step > 8 && telem.motion_status == 0 {
                            break;
                        }
                    }
                }

                // Measure NIR ground truth vs LemLib odometry at end of lap
                let tx = h.transceiver.twin.true_x;
                let ty = h.transceiver.twin.true_y;
                let tth = h.transceiver.twin.true_theta_deg;
                let h_f2i = h.homography.h_field_to_img;
                let homography_clone = h.homography.clone();

                let frame = h.sensor.capture_synchronized_frame(
                    tx,
                    ty,
                    tth,
                    0.0,
                    &h_f2i,
                );
                let seg = h.segmenter.segment_robot(
                    &frame.ir_l8,
                    &frame.rgb_color,
                    &h.sensor.static_ir_background,
                    &h.sensor.static_rgb_background,
                );
                let (ekf, _) = h.tracker.process_frame(
                    total_elapsed_s * 1000.0,
                    &frame.ir_l8,
                    &seg,
                    &homography_clone,
                    Some(tth as f64),
                );

                let dx = ekf.x_in - (h.transceiver.twin.odom_x as f64);
                let dy = ekf.y_in - (h.transceiver.twin.odom_y as f64);
                let cum_drift = dx.hypot(dy);
                let h_drift_deg = (ekf.theta_deg - (h.transceiver.twin.odom_theta_deg as f64)).abs();
                let l_temp = h.transceiver.twin.left_temp_c as f64;
                let r_temp = h.transceiver.twin.right_temp_c as f64;
                let c_temp = h.transceiver.twin.lift_temp_c as f64;
                let max_t = l_temp.max(r_temp).max(c_temp);
                let margin = 55.0 - max_t;

                lap_rows.push(EnduranceLapRow {
                    lap_number: lap,
                    elapsed_s: total_elapsed_s,
                    cumulative_odom_drift_in: cum_drift,
                    heading_drift_deg: h_drift_deg,
                    left_motor_temp_c: l_temp,
                    right_motor_temp_c: r_temp,
                    lift_motor_temp_c: c_temp,
                    thermal_margin_c: margin,
                    passed: max_t < 55.0 && cum_drift < 1.25,
                });
            }
        }

        // ====================================================================
        // 4. IMPULSE DISTURBANCE REJECTION AT (24, 24, 90 deg)
        // Recovery into +-0.2 in and +-0.5 deg settling band
        // ====================================================================
        for &(label, desc, imp_lat_v, imp_ang_v) in &[
            (
                "Longitudinal Front Push Impulse",
                "+28.0 in/s longitudinal velocity impulse at (24, 24, 90°)",
                28.0f32,
                4.5f32,
            ),
            (
                "Rotational & Lateral Rear-Corner Strike",
                "-24.0 in/s lateral + 62.0 deg/s rotational impulse at (24, 24, 90°)",
                -24.0f32,
                62.0f32,
            ),
        ] {
            let mut h = harness.lock().unwrap();
            h.transceiver.twin.true_x = 24.0;
            h.transceiver.twin.true_y = 24.0;
            h.transceiver.twin.true_theta_deg = 90.0;
            h.transceiver.twin.true_vx = 0.0;
            h.transceiver.twin.true_vy = 0.0;
            h.transceiver.twin.true_omega_deg_s = 0.0;
            h.transceiver.twin.odom_x = 24.0;
            h.transceiver.twin.odom_y = 24.0;
            h.transceiver.twin.odom_theta_deg = 90.0;
            h.tracker.reset_pose(24.0, 24.0, 90.0);

            let mut cmd = HostCommandPacket::zeroed();
            cmd.seq = h.seq_counter;
            h.seq_counter = h.seq_counter.wrapping_add(1);
            cmd.controller_id = ControllerId::LemlibLateral as u8;
            cmd.motion_type = MotionCommandType::HoldPoseDisturbance as u8;
            cmd.trigger_flags = trigger_flags::EXECUTE_MOTION
                | trigger_flags::HOT_RELOAD_GAINS
                | trigger_flags::SIGN_FLIP_RESET;
            cmd.kp = tuned.lateral_gains.kp;
            cmd.ki = tuned.lateral_gains.ki;
            cmd.kd = tuned.lateral_gains.kd;
            cmd.windup_range = tuned.lateral_gains.windup_range;
            cmd.target_x = 24.0;
            cmd.target_y = 24.0;
            cmd.target_theta = 90.0;
            cmd.timeout_ms = 1200;

            let mut peak_disp_in = 0.0f64;
            let mut peak_ang_deg = 0.0f64;
            let impulse_step = 10usize;
            let mut last_outside_band_step = impulse_step;
            let mut traj = Vec::with_capacity(95);

            for step in 0..95 {
                if step == impulse_step {
                    h.transceiver.twin.inject_external_impulse(imp_lat_v, imp_ang_v);
                }
                if step > 0 {
                    cmd.motion_type = MotionCommandType::Heartbeat as u8;
                    cmd.trigger_flags = 0;
                    cmd.seq = h.seq_counter;
                    h.seq_counter = h.seq_counter.wrapping_add(1);
                }
                let _ = h.transceiver.transceive_100hz(&mut cmd, 0.010)?;

                let tx = h.transceiver.twin.true_x;
                let ty = h.transceiver.twin.true_y;
                let tth = h.transceiver.twin.true_theta_deg;
                let h_f2i = h.homography.h_field_to_img;
                let homography_clone = h.homography.clone();

                let frame = h.sensor.capture_synchronized_frame(
                    tx,
                    ty,
                    tth,
                    0.0,
                    &h_f2i,
                );
                let seg = h.segmenter.segment_robot(
                    &frame.ir_l8,
                    &frame.rgb_color,
                    &h.sensor.static_ir_background,
                    &h.sensor.static_rgb_background,
                );
                let (ekf, _) = h.tracker.process_frame(
                    (step as f64) * 10.0,
                    &frame.ir_l8,
                    &seg,
                    &homography_clone,
                    Some(tth as f64),
                );

                let pos_err = (ekf.x_in - 24.0).hypot(ekf.y_in - 24.0);
                let ang_err = (ekf.theta_deg - 90.0).abs();
                peak_disp_in = peak_disp_in.max(pos_err);
                peak_ang_deg = peak_ang_deg.max(ang_err);

                if step >= impulse_step && (pos_err > 0.20 || ang_err > 0.50) {
                    last_outside_band_step = step;
                }

                traj.push(((step as f64) * 0.010, pos_err, ang_err));
            }

            let recovery_time_ms = ((last_outside_band_step.saturating_sub(impulse_step)) as f64) * 10.0;
            let &(_, final_pos, final_ang) = traj.last().unwrap_or(&(0.0, 0.0, 0.0));
            let passed = final_pos <= 0.20 && final_ang <= 0.50 && recovery_time_ms <= 650.0;

            impulse_rows.push(ImpulseDisturbanceRecord {
                disturbance_label: label.to_string(),
                applied_impulse_desc: desc.to_string(),
                peak_displacement_in: peak_disp_in,
                peak_heading_deflection_deg: peak_ang_deg,
                recovery_time_ms,
                final_settled_pos_err_in: final_pos,
                final_settled_ang_err_deg: final_ang,
                trajectory_samples: traj,
                passed,
            });
        }

        let all_passed = scale_rows.iter().all(|r| r.passed)
            && cg_rows.iter().all(|r| r.passed)
            && lap_rows.iter().all(|r| r.passed)
            && impulse_rows.iter().all(|r| r.passed);

        Ok(StressTestSuiteResults {
            scale_invariance: scale_rows,
            cg_perturbation: cg_rows,
            endurance_laps: lap_rows,
            impulse_disturbances: impulse_rows,
            all_passed,
        })
    }
}
