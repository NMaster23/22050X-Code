use anyhow::Result;
use image::{GrayImage, Luma, RgbImage};
use imageproc::edges::canny;
use nalgebra::{Matrix3, SMatrix, Vector2, Vector3};

use crate::vision::sensor::WindowsHelloDualSensor;

#[derive(Debug, Clone)]
pub struct TileSeamLine {
    pub rho: f64,
    pub theta_rad: f64,
    pub votes: u32,
    pub p1: (f64, f64),
    pub p2: (f64, f64),
}

#[derive(Debug, Clone)]
pub struct FieldHomography {
    /// Maps image pixel (u, v, 1)^T -> metric VEX field coordinates (x_in, y_in, w)^T
    pub h_img_to_field: Matrix3<f64>,
    /// Maps metric VEX field coordinates (x_in, y_in, 1)^T -> image pixel (u, v, w)^T
    pub h_field_to_img: Matrix3<f64>,
    pub detected_seams: Vec<TileSeamLine>,
    pub grid_intersections_px: Vec<(f64, f64)>,
    pub reprojection_rmse_in: f64,
    pub field_bounds_in: (f64, f64, f64, f64), // (min_x, max_x, min_y, max_y)
}

impl FieldHomography {
    /// Calibrate 24"x24" VEX foam tile metric homography using Canny edge detection
    /// and Hough line transforms across the fused IR + RGB static field frame.
    pub fn calibrate_from_static_field(ir_frame: &GrayImage, rgb_frame: &RgbImage) -> Result<Self> {
        let width = ir_frame.width();
        let height = ir_frame.height();

        // 1. Fuse IR luminance (L8) with RGB green-channel luminance to enhance 24"x24" foam tile seams
        let mut fused = GrayImage::new(width, height);
        for y in 0..height {
            for x in 0..width {
                let ir_v = ir_frame.get_pixel(x, y)[0] as u16;
                let rgb_px = rgb_frame.get_pixel(x, y);
                let luma_rgb = ((rgb_px[0] as u16 * 77 + rgb_px[1] as u16 * 150 + rgb_px[2] as u16 * 29) >> 8) as u16;
                let val = ((ir_v * 3 + luma_rgb * 2) / 5).min(255) as u8;
                fused.put_pixel(x, y, Luma([val]));
            }
        }

        // 2. Run Canny Edge Detection on the fused static field image
        let edges = canny(&fused, 12.0, 32.0);

        // 3. Probabilistic / Accumulator Hough Transform to extract orthogonal tile seam lines
        let seams = Self::extract_hough_seam_lines(&edges, 35);

        // 4. Separate into horizontal-like and vertical-like seam families & compute grid intersections
        let mut vert_seams: Vec<&TileSeamLine> = Vec::new();
        let mut horiz_seams: Vec<&TileSeamLine> = Vec::new();
        for s in &seams {
            let deg = s.theta_rad.to_degrees().rem_euclid(180.0);
            if !(45.0..=135.0).contains(&deg) {
                vert_seams.push(s);
            } else {
                horiz_seams.push(s);
            }
        }
        vert_seams.sort_by(|a, b| a.rho.partial_cmp(&b.rho).unwrap());
        horiz_seams.sort_by(|a, b| a.rho.partial_cmp(&b.rho).unwrap());

        let mut intersections = Vec::new();
        for vs in &vert_seams {
            for hs in &horiz_seams {
                if let Some(pt) = Self::intersect_lines(vs, hs) {
                    if pt.0 >= 0.0 && pt.0 < width as f64 && pt.1 >= 0.0 && pt.1 < height as f64 {
                        intersections.push(pt);
                    }
                }
            }
        }

        // 5. Solve Direct Linear Transform (DLT) Homography from 24"x24" foam tile grid correspondences
        let nominal_f2i = WindowsHelloDualSensor::nominal_field_to_pixel_matrix(width as f64, height as f64);
        let mut src_pixels = Vec::new();
        let mut dst_field_in = Vec::new();

        for gx in (-1..=3).map(|k| (k * 24) as f64) {
            for gy in (0..=3).map(|k| (k * 24) as f64) {
                let p_px = nominal_f2i * Vector3::new(gx, gy, 1.0);
                let u = p_px.x / p_px.z;
                let v = p_px.y / p_px.z;
                // Snap to nearest detected Hough intersection if within 6 px
                let mut matched = (u, v);
                let mut best_d = 6.0;
                for &(ix, iy) in &intersections {
                    let d = (ix - u).hypot(iy - v);
                    if d < best_d {
                        best_d = d;
                        matched = (ix, iy);
                    }
                }
                src_pixels.push(Vector2::new(matched.0, matched.1));
                dst_field_in.push(Vector2::new(gx, gy));
            }
        }

        let h_img_to_field = Self::solve_dlt_homography(&src_pixels, &dst_field_in)
            .unwrap_or_else(|| nominal_f2i.try_inverse().unwrap_or_else(Matrix3::identity));
        let h_field_to_img = h_img_to_field
            .try_inverse()
            .unwrap_or(nominal_f2i);

        // Compute reprojection RMSE in inches across all 24"x24" tile vertices
        let mut sq_err_sum = 0.0;
        for (px, true_f) in src_pixels.iter().zip(dst_field_in.iter()) {
            let proj = Self::pixel_to_field_Static(&h_img_to_field, px.x, px.y);
            let err = (proj.x - true_f.x).hypot(proj.y - true_f.y);
            sq_err_sum += err * err;
        }
        let rmse = (sq_err_sum / (src_pixels.len().max(1) as f64)).sqrt();

        Ok(Self {
            h_img_to_field,
            h_field_to_img,
            detected_seams: seams,
            grid_intersections_px: intersections,
            reprojection_rmse_in: rmse,
            field_bounds_in: (-34.0, 82.0, -22.0, 94.0),
        })
    }

    pub fn pixel_to_field(&self, u: f64, v: f64) -> Vector2<f64> {
        Self::pixel_to_field_Static(&self.h_img_to_field, u, v)
    }

    pub fn field_to_pixel(&self, x_in: f64, y_in: f64) -> Vector2<f64> {
        let p = self.h_field_to_img * Vector3::new(x_in, y_in, 1.0);
        Vector2::new(p.x / p.z, p.y / p.z)
    }

    pub fn is_near_boundary(&self, x_in: f64, y_in: f64) -> bool {
        let (min_x, max_x, min_y, max_y) = self.field_bounds_in;
        x_in < min_x || x_in > max_x || y_in < min_y || y_in > max_y
    }

    fn pixel_to_field_Static(h: &Matrix3<f64>, u: f64, v: f64) -> Vector2<f64> {
        let p = h * Vector3::new(u, v, 1.0);
        Vector2::new(p.x / p.z, p.y / p.z)
    }

    fn extract_hough_seam_lines(edges: &GrayImage, min_votes: u32) -> Vec<TileSeamLine> {
        let w = edges.width() as f64;
        let h = edges.height() as f64;
        let max_rho = w.hypot(h).ceil() as usize;
        let num_theta = 180usize;
        let num_rho = max_rho * 2 + 1;
        let mut acc = vec![0u32; num_theta * num_rho];

        let sin_cos: Vec<(f64, f64)> = (0..num_theta)
            .map(|t| {
                let rad = (t as f64).to_radians();
                (rad.sin(), rad.cos())
            })
            .collect();

        for y in (0..edges.height()).step_by(2) {
            for x in (0..edges.width()).step_by(2) {
                if edges.get_pixel(x, y)[0] > 0 {
                    let xf = x as f64;
                    let yf = y as f64;
                    for (t_idx, &(s, c)) in sin_cos.iter().enumerate() {
                        let rho = xf * c + yf * s;
                        let r_idx = (rho.round() as isize + max_rho as isize) as usize;
                        if r_idx < num_rho {
                            acc[t_idx * num_rho + r_idx] += 1;
                        }
                    }
                }
            }
        }

        let mut lines: Vec<TileSeamLine> = Vec::new();
        for t_idx in 0..num_theta {
            for r_idx in 0..num_rho {
                let votes = acc[t_idx * num_rho + r_idx];
                if votes >= min_votes {
                    let rho = (r_idx as isize - max_rho as isize) as f64;
                    let theta_rad = (t_idx as f64).to_radians();

                    // Non-maximum suppression against already accepted lines
                    let duplicate = lines.iter().any(|l: &TileSeamLine| {
                        (l.rho - rho).abs() < 14.0 && (l.theta_rad - theta_rad).abs() < 0.15
                    });
                    if !duplicate {
                        let c = theta_rad.cos();
                        let s = theta_rad.sin();
                        let x0 = c * rho;
                        let y0 = s * rho;
                        let p1 = (x0 + 1000.0 * (-s), y0 + 1000.0 * c);
                        let p2 = (x0 - 1000.0 * (-s), y0 - 1000.0 * c);
                        lines.push(TileSeamLine {
                            rho,
                            theta_rad,
                            votes,
                            p1,
                            p2,
                        });
                    }
                }
            }
        }

        lines
    }

    fn intersect_lines(l1: &TileSeamLine, l2: &TileSeamLine) -> Option<(f64, f64)> {
        let c1 = l1.theta_rad.cos();
        let s1 = l1.theta_rad.sin();
        let c2 = l2.theta_rad.cos();
        let s2 = l2.theta_rad.sin();
        let det = c1 * s2 - s1 * c2;
        if det.abs() < 0.25 {
            return None;
        }
        let x = (s2 * l1.rho - s1 * l2.rho) / det;
        let y = (-c2 * l1.rho + c1 * l2.rho) / det;
        Some((x, y))
    }

    fn solve_dlt_homography(src: &[Vector2<f64>], dst: &[Vector2<f64>]) -> Option<Matrix3<f64>> {
        if src.len() < 4 || src.len() != dst.len() {
            return None;
        }
        // Normal equations (A^T A) h = 0 solved via 9x9 symmetric eigendecomposition / SVD
        let mut ata = SMatrix::<f64, 9, 9>::zeros();
        for (p, q) in src.iter().zip(dst.iter()) {
            let (u, v) = (p.x, p.y);
            let (x, y) = (q.x, q.y);
            let r1 = [
                -u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x,
            ];
            let r2 = [
                0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y,
            ];
            for i in 0..9 {
                for j in 0..9 {
                    ata[(i, j)] += r1[i] * r1[j] + r2[i] * r2[j];
                }
            }
        }

        let svd = ata.svd(true, true);
        let v_t = svd.v_t?;
        let last_row = v_t.row(8);
        let scale = if last_row[8].abs() > 1e-12 {
            last_row[8]
        } else {
            1.0
        };
        Some(Matrix3::new(
            last_row[0] / scale,
            last_row[1] / scale,
            last_row[2] / scale,
            last_row[3] / scale,
            last_row[4] / scale,
            last_row[5] / scale,
            last_row[6] / scale,
            last_row[7] / scale,
            1.0,
        ))
    }
}
