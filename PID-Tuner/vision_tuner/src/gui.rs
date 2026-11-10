use eframe::egui;
use egui::{Color32, ColorImage, Pos2, Rect, Stroke, TextureHandle, TextureOptions, Vec2};
use egui_plot::{Line, Plot, PlotPoints};

use crate::optimizer::pipeline::FullTuningResults;
use crate::stress_test::StressTestSuiteResults;
use crate::vision::{FieldHomography, WindowsHelloDualSensor};

pub struct VisionTunerHudApp {
    pub sensor: WindowsHelloDualSensor,
    pub homography: FieldHomography,
    pub tuned: FullTuningResults,
    pub stress: StressTestSuiteResults,
    pub ir_texture: Option<TextureHandle>,
    pub rgb_texture: Option<TextureHandle>,
    pub playback_idx: usize,
}

impl VisionTunerHudApp {
    pub fn new(
        sensor: WindowsHelloDualSensor,
        homography: FieldHomography,
        tuned: FullTuningResults,
        stress: StressTestSuiteResults,
    ) -> Self {
        Self {
            sensor,
            homography,
            tuned,
            stress,
            ir_texture: None,
            rgb_texture: None,
            playback_idx: 0,
        }
    }

    fn update_stream_textures(&mut self, ctx: &egui::Context) {
        let n = self.tuned.boomerang_traj_cam.len().max(1);
        let (rx, ry, rth) = self
            .tuned
            .boomerang_traj_cam
            .get(self.playback_idx % n)
            .copied()
            .unwrap_or((0.0, 0.0, 0.0));
        self.playback_idx = (self.playback_idx + 1) % n;

        let frame = self.sensor.capture_synchronized_frame(
            rx as f32,
            ry as f32,
            rth as f32,
            8.0,
            &self.homography.h_field_to_img,
        );

        let w = frame.ir_l8.width() as usize;
        let h = frame.ir_l8.height() as usize;

        // Convert L8 IR frame to ColorImage
        let mut ir_rgba = Vec::with_capacity(w * h * 4);
        for &luma in frame.ir_l8.as_raw() {
            ir_rgba.push(luma);
            ir_rgba.push(luma);
            ir_rgba.push(luma);
            ir_rgba.push(255);
        }
        let ir_cimg = ColorImage::from_rgba_unmultiplied([w, h], &ir_rgba);
        self.ir_texture = Some(ctx.load_texture("ir_stream", ir_cimg, TextureOptions::LINEAR));

        // Convert RGB24 frame to ColorImage
        let rgb_cimg = ColorImage::from_rgb([w, h], frame.rgb_color.as_raw());
        self.rgb_texture = Some(ctx.load_texture("rgb_stream", rgb_cimg, TextureOptions::LINEAR));
    }
}

impl eframe::App for VisionTunerHudApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.update_stream_textures(ctx);

        egui::TopBottomPanel::top("hud_top_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("22050X Windows Hello Active NIR + RGB Camera-in-the-Loop PID Tuner HUD");
                ui.separator();
                ui.label(format!(
                    "IR: {} | RGB: {} | Torch: ON",
                    self.sensor.diagnostics.ir_group_name, self.sensor.diagnostics.rgb_group_name
                ));
            });
        });

        egui::SidePanel::right("telemetry_gauges_panel")
            .min_width(320.0)
            .show(ctx, |ui| {
                ui.heading("Motor Thermal & Telemetry Gauges");
                ui.separator();
                if let Some(last_lap) = self.stress.endurance_laps.last() {
                    let l_ratio = (last_lap.left_motor_temp_c / 55.0).clamp(0.0, 1.0) as f32;
                    let r_ratio = (last_lap.right_motor_temp_c / 55.0).clamp(0.0, 1.0) as f32;
                    let c_ratio = (last_lap.lift_motor_temp_c / 55.0).clamp(0.0, 1.0) as f32;

                    ui.label(format!(
                        "Left Drive Group (Ports 20, -10): {:.1} °C / 55.0 °C",
                        last_lap.left_motor_temp_c
                    ));
                    ui.add(egui::ProgressBar::new(l_ratio).fill(Color32::from_rgb(40, 160, 240)));

                    ui.label(format!(
                        "Right Drive Group (Ports -7, 9): {:.1} °C / 55.0 °C",
                        last_lap.right_motor_temp_c
                    ));
                    ui.add(egui::ProgressBar::new(r_ratio).fill(Color32::from_rgb(40, 160, 240)));

                    ui.label(format!(
                        "Cascade Lift 6:1 Group (Ports 1, -2): {:.1} °C / 55.0 °C",
                        last_lap.lift_motor_temp_c
                    ));
                    ui.add(egui::ProgressBar::new(c_ratio).fill(Color32::from_rgb(60, 200, 120)));
                }

                ui.add_space(12.0);
                ui.heading("Locked LemLib Gains");
                egui::Grid::new("gains_grid").striped(true).show(ui, |ui| {
                    ui.label("Controller");
                    ui.label("kP");
                    ui.label("kI");
                    ui.label("kD");
                    ui.end_row();

                    ui.label("Lateral");
                    ui.label(format!("{:.3}", self.tuned.lateral_gains.kp));
                    ui.label(format!("{:.4}", self.tuned.lateral_gains.ki));
                    ui.label(format!("{:.3}", self.tuned.lateral_gains.kd));
                    ui.end_row();

                    ui.label("Angular");
                    ui.label(format!("{:.3}", self.tuned.angular_gains.kp));
                    ui.label(format!("{:.4}", self.tuned.angular_gains.ki));
                    ui.label(format!("{:.3}", self.tuned.angular_gains.kd));
                    ui.end_row();

                    ui.label("Swing");
                    ui.label(format!("{:.3}", self.tuned.swing_gains.kp));
                    ui.label(format!("{:.4}", self.tuned.swing_gains.ki));
                    ui.label(format!("{:.3}", self.tuned.swing_gains.kd));
                    ui.end_row();

                    ui.label("Cascade Lift");
                    ui.label(format!("{:.3}", self.tuned.cascade_gains.kp));
                    ui.label(format!("{:.4}", self.tuned.cascade_gains.ki));
                    ui.label(format!("{:.3}", self.tuned.cascade_gains.kd));
                    ui.end_row();
                });

                ui.add_space(12.0);
                ui.heading("Phase Portrait (e vs. de/dt)");
                let phase_pts: PlotPoints = self
                    .tuned
                    .lateral_step_after
                    .iter()
                    .map(|s| [s.error_cam, -s.velocity_cam])
                    .collect();
                Plot::new("phase_portrait_plot")
                    .height(180.0)
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new(phase_pts).name("Lateral Phase Portrait"));
                    });

                ui.add_space(8.0);
                ui.heading("Optimizer Convergence Curve (J)");
                let mut conv = Vec::new();
                let mut step_x = 0.0;
                for sr in &self.tuned.stage_records {
                    for &c in &sr.cost_history {
                        conv.push([step_x, c]);
                        step_x += 1.0;
                    }
                }
                Plot::new("convergence_plot")
                    .height(160.0)
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new(PlotPoints::from(conv)).name("Objective Cost J"));
                    });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.columns(2, |cols| {
                cols[0].heading("Active NIR Reflectance Stream (L8)");
                if let Some(ref tex) = self.ir_texture {
                    cols[0].image((tex.id(), Vec2::new(360.0, 240.0)));
                }

                cols[1].heading("RGB Stream + 24\"x24\" Homography Grid & OBB Overlay");
                if let Some(ref tex) = self.rgb_texture {
                    let resp = cols[1].image((tex.id(), Vec2::new(360.0, 240.0)));
                    let rect: Rect = resp.rect;
                    let painter = cols[1].painter_at(rect);

                    // Overlay 24"x24" tile grid lines & oriented chassis bounding box
                    for gx in (0..=2).map(|k| (k * 24) as f64) {
                        let p0 = self.homography.field_to_pixel(gx, 0.0);
                        let p1 = self.homography.field_to_pixel(gx, 48.0);
                        let sx = rect.min.x + (p0.x as f32 / 320.0) * rect.width();
                        let sy = rect.min.y + (p0.y as f32 / 240.0) * rect.height();
                        let ex = rect.min.x + (p1.x as f32 / 320.0) * rect.width();
                        let ey = rect.min.y + (p1.y as f32 / 240.0) * rect.height();
                        painter.line_segment(
                            [Pos2::new(sx, sy), Pos2::new(ex, ey)],
                            Stroke::new(1.5_f32, Color32::from_rgb(0, 220, 120)),
                        );
                    }
                }
            });

            ui.separator();
            ui.heading("Target vs. Actual Trajectory Trails (NIR Ground-Truth vs. LemLib Odometry)");
            let cam_trail: PlotPoints = self
                .tuned
                .boomerang_traj_cam
                .iter()
                .map(|&(x, y, _)| [x, y])
                .collect();
            let odom_trail: PlotPoints = self
                .tuned
                .boomerang_traj_odom
                .iter()
                .map(|&(x, y, _)| [x, y])
                .collect();

            Plot::new("trajectory_trail_plot")
                .height(260.0)
                .data_aspect(1.0)
                .show(ui, |plot_ui| {
                    plot_ui.line(Line::new(cam_trail).name("NIR Camera Ground-Truth"));
                    plot_ui.line(Line::new(odom_trail).name("LemLib Odometry"));
                });
        });

        ctx.request_repaint();
    }
}
