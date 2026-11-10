use argmin::core::{CostFunction, Error};
use std::sync::{Arc, Mutex};

use crate::optimizer::metrics::TransientMetrics;

/// Objective Cost Function Weights:
/// J = w_1 * t_r + w_2 * M_p^2 + w_3 * t_s + w_4 * e_ss^2 + w_5 * N_osc + w_6 * E_odom
#[derive(Debug, Clone, Copy)]
pub struct CostWeights {
    pub w1_rise_time: f64,
    pub w2_overshoot_sq: f64,
    pub w3_settling_time: f64,
    pub w4_steady_error_sq: f64,
    pub w5_oscillation: f64,
    pub w6_odom_divergence: f64,
}

impl Default for CostWeights {
    fn default() -> Self {
        Self {
            w1_rise_time: 12.0,
            w2_overshoot_sq: 18.0,
            w3_settling_time: 10.0,
            w4_steady_error_sq: 85.0,
            w5_oscillation: 25.0,
            w6_odom_divergence: 30.0,
        }
    }
}

impl CostWeights {
    pub fn evaluate_j(&self, m: &TransientMetrics) -> f64 {
        self.w1_rise_time * m.rise_time_s
            + self.w2_overshoot_sq * (m.peak_overshoot_pct * m.peak_overshoot_pct)
            + self.w3_settling_time * m.settling_time_s
            + self.w4_steady_error_sq * (m.steady_state_error * m.steady_state_error)
            + self.w5_oscillation * (m.oscillation_count as f64)
            + self.w6_odom_divergence * m.odom_slip_divergence
    }
}

/// Generic argmin::core::CostFunction wrapper for Closed-Loop Camera-in-the-Loop PID Evaluation
#[derive(Clone)]
pub struct PidObjectiveProblem<F>
where
    F: Fn(&[f64]) -> (f64, TransientMetrics) + Send + Sync,
{
    pub evaluator: Arc<F>,
    pub history: Arc<Mutex<Vec<(Vec<f64>, f64, TransientMetrics)>>>,
}

impl<F> PidObjectiveProblem<F>
where
    F: Fn(&[f64]) -> (f64, TransientMetrics) + Send + Sync,
{
    pub fn new(evaluator: F) -> Self {
        Self {
            evaluator: Arc::new(evaluator),
            history: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

impl<F> CostFunction for PidObjectiveProblem<F>
where
    F: Fn(&[f64]) -> (f64, TransientMetrics) + Send + Sync,
{
    type Param = Vec<f64>;
    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, Error> {
        let (cost, metrics) = (self.evaluator)(param);
        if let Ok(mut guard) = self.history.lock() {
            guard.push((param.clone(), cost, metrics));
        }
        Ok(cost)
    }
}
