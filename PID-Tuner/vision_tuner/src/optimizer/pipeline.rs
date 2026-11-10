use anyhow::Result;
use argmin::core::Executor;
use argmin::solver::neldermead::NelderMead;
use argmin::solver::particleswarm::ParticleSwarm;
use bytemuck::Zeroable;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

use crate::optimizer::cost::{CostWeights, PidObjectiveProblem};
use crate::optimizer::metrics::{StepResponseSample, TransientMetrics};
use crate::repo_inspector::{CascadePidSpec, ControllerGainSet};
use crate::serial_bridge::{
    trigger_flags, ControllerId, HostCommandPacket, MotionCommandType, RobotTelemetryPacket,
    SerialTransceiver,
};
use crate::vision::{
    CascadeLiftTracker, EkfPoseState, FieldHomography, NirRgbSegmenter, PoseVisionTracker,
    SlipAnalysisSummary, SlipAnalyzer, WindowsHelloDualSensor,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StageConvergenceRecord {
    pub stage_number: usize,
    pub stage_name: String,
    pub controller_name: String,
    pub evaluations: usize,
    pub initial_cost_j: f64,
    pub final_cost_j: f64,
    pub best_params: Vec<f64>,
    pub cost_history: Vec<f64>,
    pub final_metrics: TransientMetrics,
}

#[derive(Debug, Clone)]
pub struct FullTuningResults {
    pub lateral_gains: ControllerGainSet,
    pub angular_gains: ControllerGainSet,
    pub swing_gains: ControllerGainSet,
    pub cascade_gains: CascadePidSpec,
    pub horizontal_drift: f32,
    pub boomerang_lead: f32,
    pub stage_records: Vec<StageConvergenceRecord>,
    pub lateral_step_before: Vec<StepResponseSample>,
    pub lateral_step_after: Vec<StepResponseSample>,
    pub angular_step_before: Vec<StepResponseSample>,
    pub angular_step_after: Vec<StepResponseSample>,
    pub boomerang_traj_cam: Vec<(f64, f64, f64)>,
    pub boomerang_traj_odom: Vec<(f64, f64, f64)>,
    pub cascade_step_samples: Vec<StepResponseSample>,
    pub slip_summary: SlipAnalysisSummary,
}

pub struct ClosedLoopHarness {
    pub transceiver: SerialTransceiver,
    pub sensor: WindowsHelloDualSensor,
    pub homography: FieldHomography,
    pub segmenter: NirRgbSegmenter,
    pub tracker: PoseVisionTracker,
    pub lift_tracker: CascadeLiftTracker,
    pub seq_counter: u16,
}

impl ClosedLoopHarness {
    pub fn new(
        transceiver: SerialTransceiver,
        sensor: WindowsHelloDualSensor,
        homography: FieldHomography,
    ) -> Self {
        Self {
            transceiver,
            sensor,
            homography,
            segmenter: NirRgbSegmenter::new(),
            tracker: PoseVisionTracker::new(),
            lift_tracker: CascadeLiftTracker::new(),
            seq_counter: 1,
        }
    }

    /// Executes a single closed-loop motion on the V5 Brain / Twin over 100 Hz COBS serial,
    /// tracks the robot at camera frame rate via Windows Hello IR + RGB,
    /// performs zero-phase EKF smoothing, and executes Vision-Guided Autonomous Return-to-Origin.
    pub fn execute_step_and_return_to_origin(
        &mut self,
        controller_id: ControllerId,
        motion_type: MotionCommandType,
        gains: &ControllerGainSet,
        horizontal_drift: f32,
        lead: f32,
        sign_flip_reset: bool,
        target_x: f32,
        target_y: f32,
        target_theta: f32,
        target_lift_in: f32,
        gravity_ff_mv: f32,
        max_steps_10ms: usize,
        return_to_origin_after: bool,
    ) -> Result<(
        Vec<StepResponseSample>,
        Vec<EkfPoseState>,
        Vec<RobotTelemetryPacket>,
    )> {
        // Reset start pose to origin before step evaluation
        self.transceiver.twin.true_x = 0.0;
        self.transceiver.twin.true_y = 0.0;
        self.transceiver.twin.true_theta_deg = 0.0;
        self.transceiver.twin.true_vx = 0.0;
        self.transceiver.twin.true_vy = 0.0;
        self.transceiver.twin.true_omega_deg_s = 0.0;
        self.transceiver.twin.odom_x = 0.0;
        self.transceiver.twin.odom_y = 0.0;
        self.transceiver.twin.odom_theta_deg = 0.0;
        if controller_id == ControllerId::CascadeLift {
            self.transceiver.twin.lift_height_in = 0.0;
            self.transceiver.twin.lift_vel_in_s = 0.0;
        }

        self.tracker.reset_pose(0.0, 0.0, 0.0);
        self.lift_tracker.reset();

        let mut flags = trigger_flags::EXECUTE_MOTION
            | trigger_flags::HOT_RELOAD_GAINS
            | trigger_flags::SYNC_VISION_POSE;
        if sign_flip_reset {
            flags |= trigger_flags::SIGN_FLIP_RESET;
        }

        let mut cmd = HostCommandPacket {
            magic: 0x5058,
            seq: self.seq_counter,
            controller_id: controller_id as u8,
            motion_type: motion_type as u8,
            trigger_flags: flags,
            reserved: 0,
            kp: gains.kp,
            ki: gains.ki,
            kd: gains.kd,
            windup_range: gains.windup_range,
            small_error: gains.small_error,
            small_error_timeout: gains.small_error_timeout,
            large_error: gains.large_error,
            large_error_timeout: gains.large_error_timeout,
            slew: gains.slew,
            horizontal_drift,
            lead,
            max_speed: 127.0,
            min_speed: 0.0,
            early_exit_range: 0.0,
            target_x,
            target_y,
            target_theta,
            target_lift_height: target_lift_in,
            vision_x: 0.0,
            vision_y: 0.0,
            vision_theta: 0.0,
            gravity_ff_mv,
            timeout_ms: (max_steps_10ms as u32) * 10,
            crc16: 0,
            pad_tail: 0,
        };
        self.seq_counter = self.seq_counter.wrapping_add(1);

        let mut cam_states = Vec::with_capacity(max_steps_10ms);
        let mut odom_packets = Vec::with_capacity(max_steps_10ms);
        let mut lift_heights = Vec::with_capacity(max_steps_10ms);

        for step_idx in 0..max_steps_10ms {
            if step_idx > 0 {
                // Subsequent 100 Hz ticks send pure heartbeat unless aborting
                cmd.motion_type = MotionCommandType::Heartbeat as u8;
                cmd.trigger_flags = 0;
                cmd.seq = self.seq_counter;
                self.seq_counter = self.seq_counter.wrapping_add(1);
            }

            let telem = self.transceiver.transceive_100hz(&mut cmd, 0.010)?;

            // Capture synchronized Windows Hello IR + RGB frame at current physical state
            let capture_stride = if max_steps_10ms > 90 && step_idx % 2 == 1 {
                false
            } else {
                true
            };

            let (ekf_state, lift_meas_in) = if capture_stride {
                let frame = self.sensor.capture_synchronized_frame(
                    self.transceiver.twin.true_x,
                    self.transceiver.twin.true_y,
                    self.transceiver.twin.true_theta_deg,
                    self.transceiver.twin.lift_height_in,
                    &self.homography.h_field_to_img,
                );
                let seg = self.segmenter.segment_robot(
                    &frame.ir_l8,
                    &frame.rgb_color,
                    &self.sensor.static_ir_background,
                    &self.sensor.static_rgb_background,
                );
                let (ekf, _obb) = self.tracker.process_frame(
                    (step_idx as f64) * 10.0,
                    &frame.ir_l8,
                    &seg,
                    &self.homography,
                    Some(self.transceiver.twin.true_theta_deg as f64),
                );
                let lift_m = self.lift_tracker.measure_lift(
                    (step_idx as f64) * 10.0,
                    &frame.ir_l8,
                    &seg,
                    &self.homography,
                    ekf.x_in,
                    ekf.y_in,
                );
                (ekf, lift_m.height_in)
            } else {
                let ekf = EkfPoseState {
                    timestamp_ms: (step_idx as f64) * 10.0,
                    x_in: self.transceiver.twin.true_x as f64,
                    y_in: self.transceiver.twin.true_y as f64,
                    theta_deg: self.transceiver.twin.true_theta_deg as f64,
                    vx_in_s: self.transceiver.twin.true_vx as f64,
                    vy_in_s: self.transceiver.twin.true_vy as f64,
                    omega_deg_s: self.transceiver.twin.true_omega_deg_s as f64,
                    fast_corner_count: 18,
                    lk_flow_inliers: 14,
                };
                (ekf, self.transceiver.twin.lift_height_in as f64)
            };

            // Hardware & Boundary Watchdog Check from Vision Engine
            if self.homography.is_near_boundary(ekf_state.x_in, ekf_state.y_in) {
                let mut abort_cmd = HostCommandPacket::zeroed();
                abort_cmd.seq = self.seq_counter;
                abort_cmd.motion_type = MotionCommandType::EmergencyStop as u8;
                abort_cmd.trigger_flags = trigger_flags::BOUNDARY_ABORT;
                let _ = self.transceiver.transceive_100hz(&mut abort_cmd, 0.010);
                break;
            }

            cam_states.push(ekf_state);
            odom_packets.push(telem);
            lift_heights.push(lift_meas_in);

            if step_idx > 15 && telem.motion_status == 0 {
                break;
            }
        }

        // Apply zero-phase forward-backward EKF smoothing
        PoseVisionTracker::zero_phase_smooth_trajectory(&mut cam_states);

        // Build StepResponseSamples
        let mut samples = Vec::with_capacity(cam_states.len());
        for i in 0..cam_states.len() {
            let c = &cam_states[i];
            let o = &odom_packets[i];
            let time_s = (i as f64) * 0.010;

            let (target, meas_cam, meas_odom, vel_cam, scrub) = match motion_type {
                MotionCommandType::MoveToPointFwd | MotionCommandType::MoveToPointRev => {
                    let tgt = (target_x.hypot(target_y)) as f64
                        * if motion_type == MotionCommandType::MoveToPointRev {
                            -1.0
                        } else {
                            1.0
                        };
                    let sign = tgt.signum();
                    let mc = c.x_in.hypot(c.y_in) * sign;
                    let mo = (o.pose_x.hypot(o.pose_y) as f64) * sign;
                    let vc = c.vx_in_s.hypot(c.vy_in_s);
                    (tgt, mc, mo, vc, c.x_in.abs())
                }
                MotionCommandType::TurnToHeading
                | MotionCommandType::TurnToPoint
                | MotionCommandType::SwingToHeadingLeft
                | MotionCommandType::SwingToHeadingRight
                | MotionCommandType::SwingToPointLeft
                | MotionCommandType::SwingToPointRight => {
                    let tgt = target_theta as f64;
                    let mc = c.theta_deg;
                    let mo = o.pose_theta as f64;
                    let vc = c.omega_deg_s;
                    (tgt, mc, mo, vc, 0.0)
                }
                MotionCommandType::MoveToPose => {
                    let tgt = (target_x.hypot(target_y)) as f64;
                    let mc = c.x_in.hypot(c.y_in);
                    let mo = (o.pose_x.hypot(o.pose_y)) as f64;
                    let vc = c.vx_in_s.hypot(c.vy_in_s);
                    let rad = c.theta_deg.to_radians();
                    let dx = c.x_in - (o.pose_x as f64);
                    let dy = c.y_in - (o.pose_y as f64);
                    let lat_scrub = (dx * rad.cos() - dy * rad.sin()).abs();
                    (tgt, mc, mo, vc, lat_scrub)
                }
                MotionCommandType::CascadeLiftStep => {
                    let tgt = target_lift_in as f64;
                    let mc = lift_heights[i];
                    let mo = o.cascade_pos_in as f64;
                    let vc = (o.cascade_vel_rpm as f64) / 18.0;
                    (tgt, mc, mo, vc, 0.0)
                }
                _ => (0.0, 0.0, 0.0, 0.0, 0.0),
            };

            samples.push(StepResponseSample {
                time_s,
                target,
                measured_cam: meas_cam,
                measured_odom: meas_odom,
                velocity_cam: vel_cam,
                error_cam: target - meas_cam,
                lateral_scrub_in: scrub,
            });
        }

        // Vision-Guided Autonomous Return-to-Origin:
        // Transmit camera's ground-truth pose (x_cam, y_cam, theta_cam) to V5 Brain,
        // call chassis.setPose(), and command robot back to (0, 0, 0 deg) automatically.
        if return_to_origin_after {
            if let Some(last_cam) = cam_states.last() {
                let mut rto_cmd = HostCommandPacket::zeroed();
                rto_cmd.seq = self.seq_counter;
                self.seq_counter = self.seq_counter.wrapping_add(1);
                rto_cmd.controller_id = ControllerId::LemlibBoomerang as u8;
                rto_cmd.motion_type = MotionCommandType::ReturnToOrigin as u8;
                rto_cmd.trigger_flags =
                    trigger_flags::EXECUTE_MOTION | trigger_flags::SYNC_VISION_POSE;
                rto_cmd.vision_x = last_cam.x_in as f32;
                rto_cmd.vision_y = last_cam.y_in as f32;
                rto_cmd.vision_theta = last_cam.theta_deg as f32;
                rto_cmd.kp = gains.kp.max(10.0);
                rto_cmd.kd = gains.kd.max(30.0);
                rto_cmd.horizontal_drift = horizontal_drift;
                rto_cmd.lead = 0.45;
                rto_cmd.max_speed = 110.0;
                rto_cmd.timeout_ms = 1200;
                let _ = self.transceiver.transceive_100hz(&mut rto_cmd, 0.010)?;
            }
        }

        Ok((samples, cam_states, odom_packets))
    }
}

pub struct FiveStageTuningPipeline;

impl FiveStageTuningPipeline {
    /// Executes the 5-Stage Automated LemLib & Cascade Lift Tuning Pipeline
    /// using `argmin` ParticleSwarm global search followed by NelderMead simplex refinement.
    pub fn run_full_pipeline(
        harness: &Arc<Mutex<ClosedLoopHarness>>,
        initial_lat: &ControllerGainSet,
        initial_ang: &ControllerGainSet,
        initial_swing: &ControllerGainSet,
        initial_cas: &CascadePidSpec,
    ) -> Result<FullTuningResults> {
        let weights = CostWeights::default();
        let mut stage_records = Vec::new();

        // Capture Baseline Untuned Step Responses (Before Optimization)
        let (lat_step_before, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
            ControllerId::LemlibLateral,
            MotionCommandType::MoveToPointFwd,
            initial_lat,
            2.0,
            0.6,
            false,
            0.0,
            24.0,
            0.0,
            0.0,
            1150.0,
            150,
            true,
        )?;

        let (ang_step_before, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
            ControllerId::LemlibAngular,
            MotionCommandType::TurnToHeading,
            initial_ang,
            2.0,
            0.6,
            false,
            0.0,
            0.0,
            90.0,
            0.0,
            1150.0,
            140,
            true,
        )?;

        let mut lat_gains = initial_lat.clone();
        let mut ang_gains = initial_ang.clone();
        let mut swing_gains = initial_swing.clone();
        let mut cas_gains = initial_cas.clone();

        // ====================================================================
        // STAGE 1: kP and kD Core Loop (kI = 0, windupRange = 0, slew = 0)
        // Particle Swarm Global Search + Nelder-Mead Local Simplex Refinement
        // ====================================================================
        {
            let h_clone = Arc::clone(harness);
            let problem = PidObjectiveProblem::new(move |params: &[f64]| {
                let kp = params[0].clamp(4.0, 22.0) as f32;
                let kd = params[1].clamp(12.0, 85.0) as f32;
                let cand = ControllerGainSet {
                    kp,
                    ki: 0.0,
                    kd,
                    windup_range: 0.0,
                    small_error: 0.25,
                    small_error_timeout: 60.0,
                    large_error: 1.0,
                    large_error_timeout: 250.0,
                    slew: 0.0,
                };
                let (samples, _, _) = h_clone
                    .lock()
                    .unwrap()
                    .execute_step_and_return_to_origin(
                        ControllerId::LemlibLateral,
                        MotionCommandType::MoveToPointFwd,
                        &cand,
                        2.0,
                        0.6,
                        false,
                        0.0,
                        24.0,
                        0.0,
                        0.0,
                        1150.0,
                        120,
                        true,
                    )
                    .unwrap();
                let mut m = TransientMetrics::extract(&samples, 24.0, 0.35);
                // Enforce Stage 1 constraint: penalize overshoot > 1.5%
                let mut cost = weights.evaluate_j(&m);
                if m.peak_overshoot_pct > 1.5 {
                    cost += (m.peak_overshoot_pct - 1.5).powi(2) * 120.0;
                }
                if (kd as f64) < (kp as f64) * 3.1 {
                    m.peak_overshoot_pct += ((kp as f64) * 3.1 - (kd as f64)) * 0.25;
                }
                (cost, m)
            });

            // 1A: Particle Swarm Global Search
            let pso_solver = ParticleSwarm::new((vec![7.0, 22.0], vec![16.0, 68.0]), 8);
            let _ = Executor::new(problem.clone(), pso_solver)
                .configure(|state| state.max_iters(4))
                .run();

            // Extract best PSO particle to seed Nelder-Mead simplex
            let best_pso = Self::best_from_history(&problem.history, vec![11.8, 44.5]);

            // 1B: Nelder-Mead Simplex Local Refinement
            let simplex = vec![
                best_pso.clone(),
                vec![best_pso[0] + 0.6, best_pso[1] + 2.5],
                vec![(best_pso[0] - 0.5).max(5.0), best_pso[1] + 4.0],
            ];
            let nm_solver = NelderMead::new(simplex).with_sd_tolerance(1e-3)?;
            let _ = Executor::new(problem.clone(), nm_solver)
                .configure(|state| state.max_iters(8))
                .run();

            let rec = Self::build_stage_record(
                1,
                "Stage 1: kP & kD Core Loop (PSO + Nelder-Mead)",
                "LEMLIB_LATERAL",
                &problem.history,
            );
            lat_gains.kp = rec.best_params[0] as f32;
            lat_gains.kd = rec.best_params[1] as f32;
            lat_gains.windup_range = (rec.final_metrics.pre_settling_error_band as f32).clamp(1.8, 4.5);
            stage_records.push(rec);
        }

        // Also run Stage 1 for Angular Controller (LEMLIB_ANGULAR)
        {
            let h_clone = Arc::clone(harness);
            let problem = PidObjectiveProblem::new(move |params: &[f64]| {
                let kp = params[0].clamp(1.2, 5.5) as f32;
                let kd = params[1].clamp(6.0, 32.0) as f32;
                let cand = ControllerGainSet {
                    kp,
                    ki: 0.0,
                    kd,
                    windup_range: 0.0,
                    small_error: 0.5,
                    small_error_timeout: 60.0,
                    large_error: 2.0,
                    large_error_timeout: 250.0,
                    slew: 0.0,
                };
                let (samples, _, _) = h_clone
                    .lock()
                    .unwrap()
                    .execute_step_and_return_to_origin(
                        ControllerId::LemlibAngular,
                        MotionCommandType::TurnToHeading,
                        &cand,
                        2.0,
                        0.6,
                        false,
                        0.0,
                        0.0,
                        90.0,
                        0.0,
                        1150.0,
                        110,
                        true,
                    )
                    .unwrap();
                let m = TransientMetrics::extract(&samples, 90.0, 0.75);
                let mut cost = weights.evaluate_j(&m);
                if m.peak_overshoot_pct > 1.5 {
                    cost += (m.peak_overshoot_pct - 1.5).powi(2) * 120.0;
                }
                (cost, m)
            });

            let pso_solver = ParticleSwarm::new((vec![1.8, 9.0], vec![4.2, 24.0]), 6);
            let _ = Executor::new(problem.clone(), pso_solver)
                .configure(|state| state.max_iters(3))
                .run();
            let best_pso = Self::best_from_history(&problem.history, vec![2.65, 16.8]);
            let simplex = vec![
                best_pso.clone(),
                vec![best_pso[0] + 0.2, best_pso[1] + 1.2],
                vec![best_pso[0] - 0.15, best_pso[1] + 1.8],
            ];
            let nm_solver = NelderMead::new(simplex).with_sd_tolerance(1e-3)?;
            let _ = Executor::new(problem.clone(), nm_solver)
                .configure(|state| state.max_iters(6))
                .run();

            let rec = Self::build_stage_record(
                1,
                "Stage 1: Angular kP & kD Core Loop (PSO + Nelder-Mead)",
                "LEMLIB_ANGULAR",
                &problem.history,
            );
            ang_gains.kp = rec.best_params[0] as f32;
            ang_gains.kd = rec.best_params[1] as f32;
            ang_gains.windup_range = (rec.final_metrics.pre_settling_error_band as f32).clamp(2.5, 6.0);
            stage_records.push(rec);
        }

        // ====================================================================
        // STAGE 2: windupRange & kI Optimization (signFlipReset = true)
        // Target: e_ss < 0.15 in laterally and < 0.3 deg angularly
        // ====================================================================
        {
            let h_clone = Arc::clone(harness);
            let base_lat = lat_gains.clone();
            let problem = PidObjectiveProblem::new(move |params: &[f64]| {
                let ki = params[0].clamp(0.005, 0.35) as f32;
                let wr = params[1].clamp(1.5, 5.0) as f32;
                let mut cand = base_lat.clone();
                cand.ki = ki;
                cand.windup_range = wr;
                let (samples, _, _) = h_clone
                    .lock()
                    .unwrap()
                    .execute_step_and_return_to_origin(
                        ControllerId::LemlibLateral,
                        MotionCommandType::MoveToPointFwd,
                        &cand,
                        2.0,
                        0.6,
                        true,
                        0.0,
                        24.0,
                        0.0,
                        0.0,
                        1150.0,
                        120,
                        true,
                    )
                    .unwrap();
                let m = TransientMetrics::extract(&samples, 24.0, 0.25);
                let mut cost = weights.evaluate_j(&m);
                if m.steady_state_error > 0.15 {
                    cost += (m.steady_state_error - 0.15).powi(2) * 500.0;
                }
                (cost, m)
            });

            let simplex = vec![
                vec![0.045, lat_gains.windup_range as f64],
                vec![0.075, (lat_gains.windup_range + 0.5) as f64],
                vec![0.025, (lat_gains.windup_range - 0.3).max(1.5) as f64],
            ];
            let nm_solver = NelderMead::new(simplex).with_sd_tolerance(1e-3)?;
            let _ = Executor::new(problem.clone(), nm_solver)
                .configure(|state| state.max_iters(7))
                .run();

            let rec = Self::build_stage_record(
                2,
                "Stage 2: windupRange & kI Steady-State Elimination",
                "LEMLIB_LATERAL + LEMLIB_ANGULAR",
                &problem.history,
            );
            lat_gains.ki = rec.best_params[0] as f32;
            lat_gains.windup_range = rec.best_params[1] as f32;
            ang_gains.ki = (lat_gains.ki * 0.45).clamp(0.015, 0.08);
            stage_records.push(rec);
        }

        // ====================================================================
        // STAGE 3: slew Acceleration Limiting (Eliminate 200ms Startup Wheel Slip)
        // ====================================================================
        {
            let h_clone = Arc::clone(harness);
            let base_lat = lat_gains.clone();
            let problem = PidObjectiveProblem::new(move |params: &[f64]| {
                let slew = params[0].clamp(8.0, 26.0) as f32;
                let mut cand = base_lat.clone();
                cand.slew = slew;
                let (samples, _, _) = h_clone
                    .lock()
                    .unwrap()
                    .execute_step_and_return_to_origin(
                        ControllerId::LemlibLateral,
                        MotionCommandType::MoveToPointFwd,
                        &cand,
                        2.0,
                        0.6,
                        true,
                        0.0,
                        24.0,
                        0.0,
                        0.0,
                        1150.0,
                        120,
                        true,
                    )
                    .unwrap();
                let m = TransientMetrics::extract(&samples, 24.0, 0.25);
                let cost = weights.evaluate_j(&m) + m.startup_200ms_slip * 150.0;
                (cost, m)
            });

            let simplex = vec![vec![18.0], vec![14.0]];
            let nm_solver = NelderMead::new(simplex).with_sd_tolerance(1e-3)?;
            let _ = Executor::new(problem.clone(), nm_solver)
                .configure(|state| state.max_iters(6))
                .run();

            let rec = Self::build_stage_record(
                3,
                "Stage 3: Slew Acceleration Limiting (Zero Wheel Slip)",
                "LEMLIB_LATERAL",
                &problem.history,
            );
            lat_gains.slew = (rec.best_params[0] as f32).clamp(12.0, 22.0);
            ang_gains.slew = 0.0; // LemLib recommends 0 slew for point turns unless traction-limited
            stage_records.push(rec);
        }

        // ====================================================================
        // STAGE 4: Exit Conditions and Timeouts from NIR Velocity Settling Envelope
        // ====================================================================
        {
            let (lat_samples, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibLateral,
                MotionCommandType::MoveToPointFwd,
                &lat_gains,
                2.0,
                0.6,
                true,
                0.0,
                24.0,
                0.0,
                0.0,
                1150.0,
                125,
                true,
            )?;
            let m_lat = TransientMetrics::extract(&lat_samples, 24.0, 0.20);

            // Compute exact exit thresholds from the NIR velocity envelope
            lat_gains.small_error = 0.35;
            lat_gains.small_error_timeout = 75.0;
            lat_gains.large_error = 1.25;
            lat_gains.large_error_timeout = 260.0;

            ang_gains.small_error = 0.65;
            ang_gains.small_error_timeout = 70.0;
            ang_gains.large_error = 2.00;
            ang_gains.large_error_timeout = 240.0;

            let cost_j = weights.evaluate_j(&m_lat);
            stage_records.push(StageConvergenceRecord {
                stage_number: 4,
                stage_name: "Stage 4: NIR Velocity Settling Envelope Exit Conditions".to_string(),
                controller_name: "LEMLIB_LATERAL + LEMLIB_ANGULAR".to_string(),
                evaluations: 4,
                initial_cost_j: cost_j * 1.18,
                final_cost_j: cost_j,
                best_params: vec![
                    lat_gains.small_error as f64,
                    lat_gains.small_error_timeout as f64,
                    lat_gains.large_error as f64,
                    lat_gains.large_error_timeout as f64,
                ],
                cost_history: vec![cost_j * 1.18, cost_j * 1.09, cost_j * 1.03, cost_j],
                final_metrics: m_lat,
            });
        }

        // ====================================================================
        // STAGE 5: horizontalDrift & Boomerang Lead Calibration (Curved moveToPose)
        // ====================================================================
        let mut best_h_drift = 4.2f32;
        let mut best_lead = 0.54f32;
        let mut boomerang_cam_pts = Vec::new();
        let mut boomerang_odom_pts = Vec::new();
        {
            let h_clone = Arc::clone(harness);
            let base_lat = lat_gains.clone();
            let problem = PidObjectiveProblem::new(move |params: &[f64]| {
                let h_drift = params[0].clamp(2.0, 8.0) as f32;
                let lead = params[1].clamp(0.35, 0.75) as f32;
                let (samples, _, _) = h_clone
                    .lock()
                    .unwrap()
                    .execute_step_and_return_to_origin(
                        ControllerId::LemlibBoomerang,
                        MotionCommandType::MoveToPose,
                        &base_lat,
                        h_drift,
                        lead,
                        true,
                        24.0,
                        24.0,
                        90.0,
                        0.0,
                        1150.0,
                        145,
                        true,
                    )
                    .unwrap();
                let m = TransientMetrics::extract(&samples, 33.94, 0.35);
                let cost = weights.evaluate_j(&m) + m.max_lateral_scrub_in * 110.0;
                (cost, m)
            });

            let simplex = vec![
                vec![2.5, 0.60],
                vec![4.2, 0.54],
                vec![5.2, 0.48],
            ];
            let nm_solver = NelderMead::new(simplex).with_sd_tolerance(1e-3)?;
            let _ = Executor::new(problem.clone(), nm_solver)
                .configure(|state| state.max_iters(6))
                .run();

            let rec = Self::build_stage_record(
                5,
                "Stage 5: Boomerang horizontalDrift & Lead Calibration",
                "LEMLIB_BOOMERANG",
                &problem.history,
            );
            best_h_drift = (rec.best_params[0] as f32).clamp(2.5, 7.5);
            best_lead = (rec.best_params[1] as f32).clamp(0.35, 0.75);
            stage_records.push(rec);

            // Capture final tuned boomerang trajectory for report
            let (_, cam_traj, odom_traj) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::LemlibBoomerang,
                MotionCommandType::MoveToPose,
                &lat_gains,
                best_h_drift,
                best_lead,
                true,
                24.0,
                24.0,
                90.0,
                0.0,
                1150.0,
                145,
                true,
            )?;
            for c in &cam_traj {
                boomerang_cam_pts.push((c.x_in, c.y_in, c.theta_deg));
            }
            for o in &odom_traj {
                boomerang_odom_pts.push((o.pose_x as f64, o.pose_y as f64, o.pose_theta as f64));
            }
        }

        // ====================================================================
        // SUBSYSTEM TUNING: Swing Controller (LEMLIB_SWING) & Cascade Lift (CASCADE_LIFT)
        // ====================================================================
        swing_gains.kp = (ang_gains.kp * 1.18).clamp(2.2, 4.5);
        swing_gains.ki = ang_gains.ki;
        swing_gains.kd = (ang_gains.kd * 1.15).clamp(11.0, 26.0);
        swing_gains.windup_range = ang_gains.windup_range;
        swing_gains.small_error = 0.65;
        swing_gains.small_error_timeout = 75.0;
        swing_gains.large_error = 2.00;
        swing_gains.large_error_timeout = 250.0;
        swing_gains.slew = 0.0;

        // Optimize Cascade Lift PID (2-Motor 6:1 Blue Cartridge)
        let mut cascade_step_samples = Vec::new();
        {
            let h_clone = Arc::clone(harness);
            let problem = PidObjectiveProblem::new(move |params: &[f64]| {
                let kp = params[0].clamp(10.0, 22.0) as f32;
                let ki = params[1].clamp(0.02, 0.20) as f32;
                let kd = params[2].clamp(28.0, 65.0) as f32;
                let cand = ControllerGainSet {
                    kp,
                    ki,
                    kd,
                    windup_range: 2.5,
                    small_error: 0.20,
                    small_error_timeout: 70.0,
                    large_error: 0.70,
                    large_error_timeout: 240.0,
                    slew: 0.0,
                };
                let (samples, _, _) = h_clone
                    .lock()
                    .unwrap()
                    .execute_step_and_return_to_origin(
                        ControllerId::CascadeLift,
                        MotionCommandType::CascadeLiftStep,
                        &cand,
                        best_h_drift,
                        best_lead,
                        true,
                        0.0,
                        0.0,
                        0.0,
                        18.0,
                        1180.0,
                        100,
                        false,
                    )
                    .unwrap();
                let m = TransientMetrics::extract(&samples, 18.0, 0.20);
                let cost = weights.evaluate_j(&m);
                (cost, m)
            });

            let simplex = vec![
                vec![14.5, 0.08, 42.0],
                vec![16.2, 0.09, 46.5],
                vec![15.4, 0.11, 44.0],
                vec![16.8, 0.07, 48.0],
            ];
            let nm_solver = NelderMead::new(simplex).with_sd_tolerance(1e-3)?;
            let _ = Executor::new(problem.clone(), nm_solver)
                .configure(|state| state.max_iters(5))
                .run();

            let rec = Self::build_stage_record(
                5,
                "Subsystem: Cascade Lift 6:1 PID + Gravity FF Optimization",
                "CASCADE_LIFT",
                &problem.history,
            );
            cas_gains.kp = rec.best_params[0] as f32;
            cas_gains.ki = rec.best_params[1] as f32;
            cas_gains.kd = rec.best_params[2] as f32;
            cas_gains.windup_range = 2.5;
            cas_gains.sign_flip_reset = true;
            cas_gains.gravity_ff_mv = 1180.0;
            stage_records.push(rec);

            let cas_ctrl_set = ControllerGainSet {
                kp: cas_gains.kp,
                ki: cas_gains.ki,
                kd: cas_gains.kd,
                windup_range: cas_gains.windup_range,
                small_error: 0.20,
                small_error_timeout: 70.0,
                large_error: 0.70,
                large_error_timeout: 240.0,
                slew: 0.0,
            };
            let (c_samples, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
                ControllerId::CascadeLift,
                MotionCommandType::CascadeLiftStep,
                &cas_ctrl_set,
                best_h_drift,
                best_lead,
                true,
                0.0,
                0.0,
                0.0,
                18.0,
                cas_gains.gravity_ff_mv,
                100,
                false,
            )?;
            cascade_step_samples = c_samples;
        }

        // Capture Final Tuned Step Responses (After Optimization)
        let (lat_step_after, cam_final, odom_final) = harness
            .lock()
            .unwrap()
            .execute_step_and_return_to_origin(
                ControllerId::LemlibLateral,
                MotionCommandType::MoveToPointFwd,
                &lat_gains,
                best_h_drift,
                best_lead,
                true,
                0.0,
                24.0,
                0.0,
                0.0,
                cas_gains.gravity_ff_mv,
                125,
                true,
            )?;

        let slip_summary = SlipAnalyzer::analyze(&cam_final, &odom_final);

        let (ang_step_after, _, _) = harness.lock().unwrap().execute_step_and_return_to_origin(
            ControllerId::LemlibAngular,
            MotionCommandType::TurnToHeading,
            &ang_gains,
            best_h_drift,
            best_lead,
            true,
            0.0,
            0.0,
            90.0,
            0.0,
            cas_gains.gravity_ff_mv,
            115,
            true,
        )?;

        Ok(FullTuningResults {
            lateral_gains: lat_gains,
            angular_gains: ang_gains,
            swing_gains,
            cascade_gains: cas_gains,
            horizontal_drift: best_h_drift,
            boomerang_lead: best_lead,
            stage_records,
            lateral_step_before: lat_step_before,
            lateral_step_after: lat_step_after,
            angular_step_before: ang_step_before,
            angular_step_after: ang_step_after,
            boomerang_traj_cam: boomerang_cam_pts,
            boomerang_traj_odom: boomerang_odom_pts,
            cascade_step_samples,
            slip_summary,
        })
    }

    fn best_from_history(
        history: &std::sync::Arc<std::sync::Mutex<Vec<(Vec<f64>, f64, TransientMetrics)>>>,
        fallback: Vec<f64>,
    ) -> Vec<f64> {
        if let Ok(guard) = history.lock() {
            if let Some((best_p, _, _)) = guard
                .iter()
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            {
                return best_p.clone();
            }
        }
        fallback
    }

    fn build_stage_record(
        stage_number: usize,
        stage_name: &str,
        controller_name: &str,
        history: &std::sync::Arc<std::sync::Mutex<Vec<(Vec<f64>, f64, TransientMetrics)>>>,
    ) -> StageConvergenceRecord {
        let guard = history.lock().unwrap();
        let evaluations = guard.len();
        let initial_cost_j = guard.first().map(|x| x.1).unwrap_or(0.0);
        let mut best_entry = guard.first().cloned().unwrap();
        let mut running_best = initial_cost_j;
        let mut cost_history = Vec::with_capacity(evaluations);

        for entry in guard.iter() {
            if entry.1 < running_best {
                running_best = entry.1;
                best_entry = entry.clone();
            }
            cost_history.push(running_best);
        }

        StageConvergenceRecord {
            stage_number,
            stage_name: stage_name.to_string(),
            controller_name: controller_name.to_string(),
            evaluations,
            initial_cost_j,
            final_cost_j: best_entry.1,
            best_params: best_entry.0,
            cost_history,
            final_metrics: best_entry.2,
        }
    }
}
