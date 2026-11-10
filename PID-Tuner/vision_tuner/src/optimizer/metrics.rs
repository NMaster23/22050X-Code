use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct StepResponseSample {
    pub time_s: f64,
    pub target: f64,
    pub measured_cam: f64,
    pub measured_odom: f64,
    pub velocity_cam: f64,
    pub error_cam: f64,
    pub lateral_scrub_in: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TransientMetrics {
    pub rise_time_s: f64,
    pub peak_overshoot_pct: f64,
    pub settling_time_s: f64,
    pub steady_state_error: f64,
    pub oscillation_count: u32,
    pub odom_slip_divergence: f64,
    pub pre_settling_error_band: f64,
    pub startup_200ms_slip: f64,
    pub max_lateral_scrub_in: f64,
}

impl TransientMetrics {
    /// Extracts transient response metrics from a camera ground-truth + LemLib odometry trajectory:
    /// - Rise Time (t_r, 10% -> 90%)
    /// - Peak Overshoot (M_p, %)
    /// - Settling Time (t_s, s)
    /// - Steady-State Error (e_ss)
    /// - Oscillation Count (N_osc)
    /// - Odometry Slip Divergence (E_odom = ||Pose_cam - Pose_lemlib||)
    pub fn extract(samples: &[StepResponseSample], step_amplitude: f64, settling_band: f64) -> Self {
        if samples.is_empty() || step_amplitude.abs() < 1e-6 {
            return Self::default();
        }

        let amp = step_amplitude.abs();
        let sign = step_amplitude.signum();
        let y0 = samples[0].measured_cam;

        let thresh_10 = 0.10 * amp;
        let thresh_90 = 0.90 * amp;
        let mut t_10: Option<f64> = None;
        let mut t_90: Option<f64> = None;
        let mut max_prog = 0.0f64;

        for s in samples {
            let prog = (s.measured_cam - y0) * sign;
            if t_10.is_none() && prog >= thresh_10 {
                t_10 = Some(s.time_s);
            }
            if t_90.is_none() && prog >= thresh_90 {
                t_90 = Some(s.time_s);
            }
            if prog > max_prog {
                max_prog = prog;
            }
        }

        let last_time = samples.last().unwrap().time_s.max(0.05);
        let rise_time_s = match (t_10, t_90) {
            (Some(a), Some(b)) if b >= a => (b - a).max(0.02),
            (Some(a), None) => (last_time - a).max(0.10),
            _ => last_time,
        };

        let peak_overshoot_pct = if max_prog > amp {
            ((max_prog - amp) / amp) * 100.0
        } else {
            0.0
        };

        // Settling time: last sample where |error_cam| > settling_band
        let mut settling_time_s = samples[0].time_s;
        for s in samples {
            if s.error_cam.abs() > settling_band {
                settling_time_s = s.time_s;
            }
        }

        // Steady-state error: average of last 5 samples
        let tail_len = samples.len().min(5);
        let steady_state_error = samples[samples.len() - tail_len..]
            .iter()
            .map(|s| s.error_cam.abs())
            .sum::<f64>()
            / (tail_len as f64);

        // Pre-settling error band (error at 85% rise time, used in Stage 2 for windupRange)
        let pre_settling_error_band = samples
            .iter()
            .find(|s| (s.measured_cam - y0) * sign >= 0.85 * amp)
            .map(|s| s.error_cam.abs().max(settling_band * 2.5))
            .unwrap_or(amp * 0.15);

        // Oscillation count: number of error zero-crossings with non-trivial amplitude
        let mut osc_count = 0u32;
        let deadband = settling_band * 0.35;
        let mut prev_sign = 0i8;
        for s in samples {
            if s.error_cam.abs() > deadband {
                let curr_sign = if s.error_cam > 0.0 { 1 } else { -1 };
                if prev_sign != 0 && curr_sign != prev_sign {
                    osc_count += 1;
                }
                prev_sign = curr_sign;
            }
        }

        // Odometry slip divergence E_odom = RMS & startup 200ms ||Pose_cam - Pose_lemlib||
        let mut sq_odom = 0.0;
        let mut startup_200ms = 0.0f64;
        let mut max_scrub = 0.0f64;
        for s in samples {
            let div = (s.measured_cam - s.measured_odom).abs() + s.lateral_scrub_in.abs();
            sq_odom += div * div;
            if s.time_s <= 0.20 {
                startup_200ms = startup_200ms.max(div);
            }
            max_scrub = max_scrub.max(s.lateral_scrub_in.abs());
        }
        let odom_slip_divergence = (sq_odom / (samples.len() as f64)).sqrt();

        Self {
            rise_time_s,
            peak_overshoot_pct,
            settling_time_s,
            steady_state_error,
            oscillation_count: osc_count,
            odom_slip_divergence,
            pre_settling_error_band,
            startup_200ms_slip: startup_200ms,
            max_lateral_scrub_in: max_scrub,
        }
    }
}
