use crate::serial_bridge::RobotTelemetryPacket;
use crate::vision::tracker::EkfPoseState;

#[derive(Debug, Clone, Default)]
pub struct SlipAnalysisSummary {
    pub estimated_clock_offset_ms: f64,
    pub cross_correlation_peak: f64,
    pub rms_position_divergence_in: f64,
    pub peak_position_divergence_in: f64,
    pub startup_200ms_wheel_slip_in: f64,
    pub max_lateral_omni_scrub_in: f64,
    pub imu_heading_drift_deg: f64,
}

pub struct SlipAnalyzer;

impl SlipAnalyzer {
    /// Time-aligns camera frames with V5 Brain timestamps via motion-onset cross-correlation
    /// and computes divergence between `chassis.getPose()` and `(x_cam, y_cam, theta_cam)`.
    pub fn analyze(
        cam_traj: &[EkfPoseState],
        odom_traj: &[RobotTelemetryPacket],
    ) -> SlipAnalysisSummary {
        let n = cam_traj.len().min(odom_traj.len());
        if n < 4 {
            return SlipAnalysisSummary::default();
        }

        // 1. Compute speed profiles for motion-onset cross-correlation
        let cam_speeds: Vec<f64> = cam_traj
            .iter()
            .take(n)
            .map(|c| c.vx_in_s.hypot(c.vy_in_s) + c.omega_deg_s.abs() * 0.08)
            .collect();

        let wheel_circ = 4.0 * std::f64::consts::PI;
        let odom_speeds: Vec<f64> = odom_traj
            .iter()
            .take(n)
            .map(|o| {
                let vl = (o.left_vel_rpm as f64 / 60.0) * wheel_circ;
                let vr = (o.right_vel_rpm as f64 / 60.0) * wheel_circ;
                0.5 * (vl.abs() + vr.abs())
            })
            .collect();

        // Cross-correlation over lag window [-5..=5] samples (10 ms per sample)
        let mut best_lag: isize = 0;
        let mut max_corr = -1.0f64;
        let cam_norm = cam_speeds.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-6);
        let odom_norm = odom_speeds.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-6);

        for lag in -5isize..=5isize {
            let mut dot = 0.0;
            for i in 0..n {
                let j = i as isize + lag;
                if j >= 0 && (j as usize) < n {
                    dot += cam_speeds[i] * odom_speeds[j as usize];
                }
            }
            let corr = dot / (cam_norm * odom_norm);
            if corr > max_corr {
                max_corr = corr;
                best_lag = lag;
            }
        }

        // 2. Compute RMS & Peak Pose Divergence, Startup 200ms Slip, Lateral Scrub, and IMU Drift
        let mut sq_pos_err = 0.0;
        let mut peak_pos_err = 0.0f64;
        let mut startup_slip = 0.0f64;
        let mut max_scrub = 0.0f64;
        let mut count = 0usize;

        for i in 0..n {
            let j = (i as isize + best_lag).clamp(0, (n - 1) as isize) as usize;
            let c = &cam_traj[i];
            let o = &odom_traj[j];

            let dx = c.x_in - (o.pose_x as f64);
            let dy = c.y_in - (o.pose_y as f64);
            let pos_err = dx.hypot(dy);
            sq_pos_err += pos_err * pos_err;
            peak_pos_err = peak_pos_err.max(pos_err);

            // Startup 200 ms window (first 20 samples at 100 Hz)
            if i <= 20 {
                startup_slip = startup_slip.max(pos_err);
            }

            // Lateral omni scrub perpendicular to camera heading
            let rad = c.theta_deg.to_radians();
            let lat_scrub = (dx * rad.cos() - dy * rad.sin()).abs();
            max_scrub = max_scrub.max(lat_scrub);
            count += 1;
        }

        let rms_div = (sq_pos_err / (count.max(1) as f64)).sqrt();
        let last_cam = &cam_traj[n - 1];
        let last_odom = &odom_traj[n - 1];
        let imu_drift = (last_cam.theta_deg - last_odom.pose_theta as f64).abs();

        SlipAnalysisSummary {
            estimated_clock_offset_ms: (best_lag as f64) * 10.0,
            cross_correlation_peak: max_corr.clamp(0.0, 1.0),
            rms_position_divergence_in: rms_div,
            peak_position_divergence_in: peak_pos_err,
            startup_200ms_wheel_slip_in: startup_slip,
            max_lateral_omni_scrub_in: max_scrub,
            imu_heading_drift_deg: imu_drift,
        }
    }
}
