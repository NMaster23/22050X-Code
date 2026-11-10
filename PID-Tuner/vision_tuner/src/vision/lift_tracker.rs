use image::GrayImage;

use crate::vision::homography::FieldHomography;
use crate::vision::segmentation::SegmentationResult;

#[derive(Debug, Clone, Copy)]
pub struct CascadeLiftMeasurement {
    pub timestamp_ms: f64,
    pub height_in: f64,
    pub velocity_in_s: f64,
    pub upper_stage_edge_v_px: f64,
    pub chassis_base_v_px: f64,
}

/// Markerless Cascade Lift Tracker
/// Tracks vertical bounding-box expansion and high-contrast NIR specular edges
/// of the upper cascade lift stage relative to the 22050X chassis base.
pub struct CascadeLiftTracker {
    pub px_per_inch_vertical: f64,
    prev_height_in: f64,
    prev_vel_in_s: f64,
    prev_timestamp_ms: Option<f64>,
}

impl CascadeLiftTracker {
    pub fn new() -> Self {
        Self {
            px_per_inch_vertical: 1.85,
            prev_height_in: 0.0,
            prev_vel_in_s: 0.0,
            prev_timestamp_ms: None,
        }
    }

    pub fn reset(&mut self) {
        self.prev_height_in = 0.0;
        self.prev_vel_in_s = 0.0;
        self.prev_timestamp_ms = None;
    }

    pub fn measure_lift(
        &mut self,
        timestamp_ms: f64,
        ir_frame: &GrayImage,
        seg: &SegmentationResult,
        homography: &FieldHomography,
        chassis_x_in: f64,
        chassis_y_in: f64,
    ) -> CascadeLiftMeasurement {
        let dt_sec = self
            .prev_timestamp_ms
            .map(|p| ((timestamp_ms - p) / 1000.0).clamp(0.002, 0.10))
            .unwrap_or(0.010);
        self.prev_timestamp_ms = Some(timestamp_ms);

        let center_px = homography.field_to_pixel(chassis_x_in, chassis_y_in);
        let cu = center_px.x.round() as i32;
        let cv = center_px.y.round() as i32;

        // Scan vertically upward from chassis center for the highest high-contrast NIR specular edge (>= 250)
        let mut top_specular_v = cv as f64;
        let w = ir_frame.width() as i32;
        let h = ir_frame.height() as i32;

        let search_top = (cv - 65).max(2);
        let search_bot = (cv - 1).max(2);

        for v in search_top..=search_bot {
            let mut specular_hits = 0;
            for du in -12..=12 {
                let u = cu + du;
                if u >= 0 && u < w && v >= 0 && v < h {
                    if ir_frame.get_pixel(u as u32, v as u32)[0] >= 250 {
                        specular_hits += 1;
                    }
                }
            }
            if specular_hits >= 8 {
                top_specular_v = v as f64;
                break;
            }
        }

        // Also check bounding box vertical expansion
        let (_, v_min, _, _) = seg.bbox_px;
        let vertical_span_px = (cv as f64 - top_specular_v).max(0.0);
        let raw_height_in = if vertical_span_px > 1.0 {
            (vertical_span_px / self.px_per_inch_vertical).clamp(0.0, 28.5)
        } else {
            let _ = v_min;
            0.0
        };

        let smoothed_h = 0.68 * raw_height_in + 0.32 * self.prev_height_in;
        let raw_vel = (smoothed_h - self.prev_height_in) / dt_sec;
        let smoothed_vel = 0.55 * raw_vel + 0.45 * self.prev_vel_in_s;

        self.prev_height_in = smoothed_h;
        self.prev_vel_in_s = smoothed_vel;

        CascadeLiftMeasurement {
            timestamp_ms,
            height_in: smoothed_h,
            velocity_in_s: smoothed_vel,
            upper_stage_edge_v_px: top_specular_v,
            chassis_base_v_px: cv as f64,
        }
    }
}
