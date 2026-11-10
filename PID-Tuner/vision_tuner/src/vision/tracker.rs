use image::GrayImage;
use imageproc::corners::corners_fast9;
use nalgebra::{Matrix2, SMatrix, SVector, Vector2};

use crate::vision::homography::FieldHomography;
use crate::vision::segmentation::SegmentationResult;

#[derive(Debug, Clone)]
pub struct OrientedBoundingBox {
    pub center_in: (f64, f64),
    pub width_in: f64,
    pub length_in: f64,
    pub heading_deg_cw: f64,
    pub corners_in: [(f64, f64); 4],
    pub corners_px: [(f64, f64); 4],
}

#[derive(Debug, Clone, Copy)]
pub struct EkfPoseState {
    pub timestamp_ms: f64,
    pub x_in: f64,
    pub y_in: f64,
    pub theta_deg: f64,
    pub vx_in_s: f64,
    pub vy_in_s: f64,
    pub omega_deg_s: f64,
    pub fast_corner_count: usize,
    pub lk_flow_inliers: usize,
}

pub struct PoseVisionTracker {
    /// 6-state EKF vector: [x, y, theta_deg, vx, vy, omega_deg_s]^T
    pub state: SVector<f64, 6>,
    pub cov: SMatrix<f64, 6, 6>,
    pub q_process: SMatrix<f64, 6, 6>,
    pub r_meas: SMatrix<f64, 3, 3>,
    prev_ir_frame: Option<GrayImage>,
    prev_fast_corners: Vec<(f64, f64)>,
    prev_timestamp_ms: Option<f64>,
}

impl PoseVisionTracker {
    pub fn new() -> Self {
        let mut q = SMatrix::<f64, 6, 6>::identity() * 0.015;
        q[(3, 3)] = 0.45;
        q[(4, 4)] = 0.45;
        q[(5, 5)] = 1.20;

        let mut r = SMatrix::<f64, 3, 3>::identity();
        r[(0, 0)] = 0.008; // x measurement variance (in^2)
        r[(1, 1)] = 0.008; // y measurement variance (in^2)
        r[(2, 2)] = 0.045; // theta measurement variance (deg^2)

        Self {
            state: SVector::<f64, 6>::zeros(),
            cov: SMatrix::<f64, 6, 6>::identity() * 0.25,
            q_process: q,
            r_meas: r,
            prev_ir_frame: None,
            prev_fast_corners: Vec::new(),
            prev_timestamp_ms: None,
        }
    }

    pub fn reset_pose(&mut self, x_in: f64, y_in: f64, theta_deg: f64) {
        self.state = SVector::<f64, 6>::new(x_in, y_in, theta_deg, 0.0, 0.0, 0.0);
        self.cov = SMatrix::<f64, 6, 6>::identity() * 0.05;
        self.prev_ir_frame = None;
        self.prev_fast_corners.clear();
        self.prev_timestamp_ms = None;
    }

    /// Extracts sub-pixel pose, heading, and velocity by fusing:
    /// 1. Minimum-area Oriented Bounding Box on the NIR/RGB silhouette mapped to metric field space,
    /// 2. FAST9 corner features + ORB-style front-bezel intensity moment disambiguation,
    /// 3. Lucas-Kanade windowed optical flow on rigid chassis corners,
    /// 4. 6-state Extended Kalman Filter (nalgebra).
    pub fn process_frame(
        &mut self,
        timestamp_ms: f64,
        ir_frame: &GrayImage,
        seg: &SegmentationResult,
        homography: &FieldHomography,
        hint_theta_deg: Option<f64>,
    ) -> (EkfPoseState, OrientedBoundingBox) {
        let dt_sec = self
            .prev_timestamp_ms
            .map(|prev| ((timestamp_ms - prev) / 1000.0).clamp(0.002, 0.10))
            .unwrap_or(0.010);
        self.prev_timestamp_ms = Some(timestamp_ms);

        // 1. Project segmented robot silhouette pixels into metric field coordinates (inches)
        let mut field_pts: Vec<(f64, f64, f64)> = Vec::with_capacity(seg.foreground_pixels.len());
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_w = 0.0;

        // Also track brightest specular bezel region (NIR >= 242) to disambiguate front (+Y) vs back (-Y)
        let mut bezel_x = 0.0;
        let mut bezel_y = 0.0;
        let mut bezel_w = 0.0;

        for &(u, v, nir) in &seg.foreground_pixels {
            let fp = homography.pixel_to_field(u as f64, v as f64);
            let w = nir as f64;
            field_pts.push((fp.x, fp.y, w));
            sum_x += fp.x * w;
            sum_y += fp.y * w;
            sum_w += w;
            if nir >= 242 {
                bezel_x += fp.x * w;
                bezel_y += fp.y * w;
                bezel_w += w;
            }
        }

        let (meas_x, meas_y) = if sum_w > 0.0 {
            (sum_x / sum_w, sum_y / sum_w)
        } else {
            (self.state[0], self.state[1])
        };

        // 2. Detect FAST9 corners inside the segmented robot bounding box
        let raw_corners = corners_fast9(&seg.binary_mask, 40);
        let (u_min, v_min, u_max, v_max) = seg.bbox_px;
        let curr_fast_corners: Vec<(f64, f64)> = raw_corners
            .into_iter()
            .filter(|c| c.x >= u_min && c.x <= u_max && c.y >= v_min && c.y <= v_max)
            .take(64)
            .map(|c| (c.x as f64, c.y as f64))
            .collect();

        // 3. Lucas-Kanade Optical Flow on FAST corners between consecutive NIR frames
        let mut lk_inliers = 0usize;
        let mut flow_dx_in = 0.0;
        let mut flow_dy_in = 0.0;

        if let Some(ref prev_ir) = self.prev_ir_frame {
            if !self.prev_fast_corners.is_empty() {
                for &(pu, pv) in &self.prev_fast_corners {
                    if let Some((du, dv)) = Self::lucas_kanade_patch_flow(prev_ir, ir_frame, pu, pv) {
                        let p0 = homography.pixel_to_field(pu, pv);
                        let p1 = homography.pixel_to_field(pu + du, pv + dv);
                        let step_x = p1.x - p0.x;
                        let step_y = p1.y - p0.y;
                        if step_x.hypot(step_y) < 4.5 {
                            flow_dx_in += step_x;
                            flow_dy_in += step_y;
                            lk_inliers += 1;
                        }
                    }
                }
            }
        }
        if lk_inliers > 0 {
            flow_dx_in /= lk_inliers as f64;
            flow_dy_in /= lk_inliers as f64;
        }

        self.prev_ir_frame = Some(ir_frame.clone());
        self.prev_fast_corners = curr_fast_corners.clone();

        // 4. Fit Minimum-Area Oriented Bounding Box across candidate orientations
        let prior_theta = hint_theta_deg.unwrap_or(self.state[2]);
        let mut meas_theta = Self::estimate_obb_heading(
            &field_pts,
            meas_x,
            meas_y,
            bezel_x,
            bezel_y,
            bezel_w,
            prior_theta,
        );
        if let Some(h_th) = hint_theta_deg {
            // Fuse high-SNR bezel/OBB orientation with sub-degree optical alignment
            let diff = Self::wrap_deg(meas_theta - h_th);
            if diff.abs() < 8.0 {
                meas_theta = h_th + 0.15 * diff;
            } else {
                meas_theta = h_th;
            }
        }

        // 5. 6-State Extended Kalman Filter Predict + Update
        // State transition F: constant-velocity model on [x, y, theta, vx, vy, omega]
        let mut f_mat = SMatrix::<f64, 6, 6>::identity();
        f_mat[(0, 3)] = dt_sec;
        f_mat[(1, 4)] = dt_sec;
        f_mat[(2, 5)] = dt_sec;

        let mut pred_state = f_mat * self.state;
        if lk_inliers >= 4 {
            pred_state[0] = 0.65 * pred_state[0] + 0.35 * (self.state[0] + flow_dx_in);
            pred_state[1] = 0.65 * pred_state[1] + 0.35 * (self.state[1] + flow_dy_in);
        }
        let pred_cov = f_mat * self.cov * f_mat.transpose() + self.q_process;

        // Measurement H maps [x, y, theta, vx, vy, omega] -> [x, y, theta]
        let mut h_mat = SMatrix::<f64, 3, 6>::zeros();
        h_mat[(0, 0)] = 1.0;
        h_mat[(1, 1)] = 1.0;
        h_mat[(2, 2)] = 1.0;

        let z = SVector::<f64, 3>::new(meas_x, meas_y, meas_theta);
        let mut y_innov = z - (h_mat * pred_state);
        y_innov[2] = Self::wrap_deg(y_innov[2]);

        let s_cov = h_mat * pred_cov * h_mat.transpose() + self.r_meas;
        let s_inv = s_cov.try_inverse().unwrap_or_else(SMatrix::<f64, 3, 3>::identity);
        let k_gain = pred_cov * h_mat.transpose() * s_inv;

        self.state = pred_state + k_gain * y_innov;
        let i_mat = SMatrix::<f64, 6, 6>::identity();
        self.cov = (i_mat - k_gain * h_mat) * pred_cov;

        let obb = Self::build_obb(self.state[0], self.state[1], self.state[2], homography);

        let ekf_out = EkfPoseState {
            timestamp_ms,
            x_in: self.state[0],
            y_in: self.state[1],
            theta_deg: self.state[2],
            vx_in_s: self.state[3],
            vy_in_s: self.state[4],
            omega_deg_s: self.state[5],
            fast_corner_count: curr_fast_corners.len(),
            lk_flow_inliers: lk_inliers,
        };

        (ekf_out, obb)
    }

    /// Zero-Phase Forward-Backward (Rauch-Tung-Striebel style) Trajectory Velocity Refiner
    pub fn zero_phase_smooth_trajectory(samples: &mut [EkfPoseState]) {
        let n = samples.len();
        if n < 5 {
            return;
        }
        let mut xs: Vec<f64> = samples.iter().map(|s| s.x_in).collect();
        let mut ys: Vec<f64> = samples.iter().map(|s| s.y_in).collect();
        let mut ths: Vec<f64> = samples.iter().map(|s| s.theta_deg).collect();

        // Forward pass
        let alpha = 0.42;
        for i in 1..n {
            xs[i] = alpha * xs[i] + (1.0 - alpha) * xs[i - 1];
            ys[i] = alpha * ys[i] + (1.0 - alpha) * ys[i - 1];
            ths[i] = ths[i - 1] + alpha * Self::wrap_deg(ths[i] - ths[i - 1]);
        }
        // Backward pass (cancels phase lag completely)
        for i in (0..(n - 1)).rev() {
            xs[i] = alpha * xs[i] + (1.0 - alpha) * xs[i + 1];
            ys[i] = alpha * ys[i] + (1.0 - alpha) * ys[i + 1];
            ths[i] = ths[i + 1] + alpha * Self::wrap_deg(ths[i] - ths[i + 1]);
        }

        for i in 0..n {
            samples[i].x_in = xs[i];
            samples[i].y_in = ys[i];
            samples[i].theta_deg = ths[i];
            if i > 0 && i + 1 < n {
                let dt = ((samples[i + 1].timestamp_ms - samples[i - 1].timestamp_ms) / 1000.0).max(0.002);
                samples[i].vx_in_s = (xs[i + 1] - xs[i - 1]) / dt;
                samples[i].vy_in_s = (ys[i + 1] - ys[i - 1]) / dt;
                samples[i].omega_deg_s = Self::wrap_deg(ths[i + 1] - ths[i - 1]) / dt;
            }
        }
    }

    /// 5x5 Patch Lucas-Kanade Optical Flow solver using 2x2 structure tensor
    fn lucas_kanade_patch_flow(
        prev: &GrayImage,
        curr: &GrayImage,
        u: f64,
        v: f64,
    ) -> Option<(f64, f64)> {
        let ui = u.round() as i32;
        let vi = v.round() as i32;
        let w = prev.width() as i32;
        let h = prev.height() as i32;
        if ui < 4 || ui >= w - 4 || vi < 4 || vi >= h - 4 {
            return None;
        }

        let mut ixx = 0.0f64;
        let mut ixy = 0.0f64;
        let mut iyy = 0.0f64;
        let mut ixt = 0.0f64;
        let mut iyt = 0.0f64;

        for dy in -2..=2 {
            for dx in -2..=2 {
                let x = (ui + dx) as u32;
                let y = (vi + dy) as u32;
                let ix = (prev.get_pixel(x + 1, y)[0] as f64 - prev.get_pixel(x - 1, y)[0] as f64) * 0.5;
                let iy = (prev.get_pixel(x, y + 1)[0] as f64 - prev.get_pixel(x, y - 1)[0] as f64) * 0.5;
                let it = curr.get_pixel(x, y)[0] as f64 - prev.get_pixel(x, y)[0] as f64;

                ixx += ix * ix;
                ixy += ix * iy;
                iyy += iy * iy;
                ixt += ix * it;
                iyt += iy * it;
            }
        }

        let g = Matrix2::new(ixx, ixy, ixy, iyy);
        let det = g.determinant();
        if det.abs() < 120.0 {
            return None;
        }
        let g_inv = g.try_inverse()?;
        let b = Vector2::new(-ixt, -iyt);
        let flow = g_inv * b;
        if flow.norm() <= 8.0 {
            Some((flow.x, flow.y))
        } else {
            None
        }
    }

    fn estimate_obb_heading(
        pts: &[(f64, f64, f64)],
        cx: f64,
        cy: f64,
        bezel_x: f64,
        bezel_y: f64,
        bezel_w: f64,
        prior_theta_deg: f64,
    ) -> f64 {
        if pts.len() < 12 {
            return prior_theta_deg;
        }

        // First check front specular bezel vector relative to centroid
        if bezel_w > 0.0 {
            let bx = bezel_x / bezel_w - cx;
            let by = bezel_y / bezel_w - cy;
            if bx.hypot(by) > 1.5 {
                let bezel_heading = bx.atan2(by).to_degrees();
                // Refine via minimum-area bounding box search in +-10 deg window around bezel_heading
                let mut best_angle = bezel_heading;
                let mut min_area = f64::MAX;
                for step in -20..=20 {
                    let cand = bezel_heading + (step as f64) * 0.5;
                    let rad = cand.to_radians();
                    let (s, c) = (rad.sin(), rad.cos());
                    let mut min_lx = f64::MAX;
                    let mut max_lx = f64::MIN;
                    let mut min_ly = f64::MAX;
                    let mut max_ly = f64::MIN;
                    for &(px, py, _) in pts.iter().step_by(3) {
                        let dx = px - cx;
                        let dy = py - cy;
                        let lx = dx * c - dy * s;
                        let ly = dx * s + dy * c;
                        min_lx = min_lx.min(lx);
                        max_lx = max_lx.max(lx);
                        min_ly = min_ly.min(ly);
                        max_ly = max_ly.max(ly);
                    }
                    let area = (max_lx - min_lx) * (max_ly - min_ly);
                    if area < min_area {
                        min_area = area;
                        best_angle = cand;
                    }
                }
                return prior_theta_deg + Self::wrap_deg(best_angle - prior_theta_deg);
            }
        }

        prior_theta_deg
    }

    fn build_obb(
        cx_in: f64,
        cy_in: f64,
        theta_deg_cw: f64,
        homography: &FieldHomography,
    ) -> OrientedBoundingBox {
        let width_in = 13.5;
        let length_in = 15.0;
        let hw = width_in * 0.5;
        let hl = length_in * 0.5;
        let rad = theta_deg_cw.to_radians();
        let (s, c) = (rad.sin(), rad.cos());

        let local = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)];
        let mut corners_in = [(0.0, 0.0); 4];
        let mut corners_px = [(0.0, 0.0); 4];

        for (i, &(lx, ly)) in local.iter().enumerate() {
            let fx = cx_in + lx * c + ly * s;
            let fy = cy_in - lx * s + ly * c;
            let px = homography.field_to_pixel(fx, fy);
            corners_in[i] = (fx, fy);
            corners_px[i] = (px.x, px.y);
        }

        OrientedBoundingBox {
            center_in: (cx_in, cy_in),
            width_in,
            length_in,
            heading_deg_cw: theta_deg_cw,
            corners_in,
            corners_px,
        }
    }

    fn wrap_deg(angle: f64) -> f64 {
        let mut a = (angle + 180.0).rem_euclid(360.0) - 180.0;
        if a <= -180.0 {
            a += 360.0;
        }
        a
    }
}
