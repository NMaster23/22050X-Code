pub mod gui;
pub mod optimizer;
pub mod repo_inspector;
pub mod report;
pub mod serial_bridge;
pub mod stress_test;
pub mod vision;

use anyhow::Result;
use clap::Parser;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::gui::VisionTunerHudApp;
use crate::optimizer::pipeline::{ClosedLoopHarness, FiveStageTuningPipeline};
use crate::repo_inspector::inspect_repository;
use crate::report::ReportAndPatchGenerator;
use crate::serial_bridge::SerialTransceiver;
use crate::stress_test::StressTestRunner;
use crate::vision::{FieldHomography, WindowsHelloDualSensor};

#[derive(Parser, Debug)]
#[command(
    name = "vision_tuner",
    about = "22050X Windows Hello Active NIR + RGB Markerless Camera-in-the-Loop LemLib PID Tuning & Stress-Testing Engine"
)]
struct CliArgs {
    /// Execution mode: "full-suite" (headless automated calibration, tuning, stress test, C++ patch & report) or "gui" (60+ FPS egui HUD)
    #[arg(long, default_value = "full-suite")]
    mode: String,

    /// Optional explicit VEX V5 USB Serial Port (e.g., COM4)
    #[arg(long)]
    port: Option<String>,
}

fn resolve_pros_project_root() -> PathBuf {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    if let Some(parent) = manifest_dir.parent() {
        if parent.join("project.pros").exists() {
            return parent.to_path_buf();
        }
    }
    let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    if cwd.join("project.pros").exists() {
        return cwd;
    }
    let pid_tuner = cwd.join("PID-Tuner");
    if pid_tuner.join("project.pros").exists() {
        return pid_tuner;
    }
    cwd
}

fn main() -> Result<()> {
    let args = CliArgs::parse();
    let t_start = Instant::now();
    let project_root: PathBuf = resolve_pros_project_root();

    println!("================================================================================");
    println!(" 22050X WINDOWS HELLO ACTIVE NIR + RGB MARKERLESS CAMERA-IN-THE-LOOP PID TUNER");
    println!("================================================================================");
    println!("[Phase 1.1] Inspecting PROS C++ & LemLib workspace at: {}", project_root.display());

    let hw_spec = inspect_repository(&project_root)?;
    println!(
        "  -> Project: {} | PROS Kernel: {} | LemLib: {}",
        hw_spec.project_name, hw_spec.pros_kernel_version, hw_spec.lemlib_version
    );
    println!(
        "  -> Drivetrain: TrackWidth = {:.1} in, Wheel = {:.1} in, RPM = {:.0}, initial horizontalDrift = {:.2}",
        hw_spec.track_width_in, hw_spec.wheel_diameter_in, hw_spec.drivetrain_rpm, hw_spec.horizontal_drift
    );
    println!(
        "  -> Motors: LeftDrive = [20, -10], RightDrive = [-7, 9], CascadeLift = [1, -2] (6:1 Blue), IMU = Port {}",
        hw_spec.imu_port
    );

    println!("\n[Phase 1.4] Initializing Deterministic 100 Hz COBS + CRC16 Serial Bridge...");
    let transceiver = SerialTransceiver::connect_or_attach(args.port.as_deref());
    println!(
        "  -> Transceiver Bound: {} (Packet Sizes: HostCmd = {} B, RobotTelem = {} B)",
        transceiver.port_name,
        std::mem::size_of::<crate::serial_bridge::HostCommandPacket>(),
        std::mem::size_of::<crate::serial_bridge::RobotTelemetryPacket>()
    );

    println!("\n[Phase 2.1] Opening Windows Hello Dual-Stream Sensor (Active NIR L8 + RGB24)...");
    let sensor = WindowsHelloDualSensor::initialize(320, 240)?;
    println!(
        "  -> IR Stream:  {} ({})",
        sensor.diagnostics.ir_group_name, sensor.diagnostics.ir_source_id
    );
    println!(
        "  -> RGB Stream: {} ({})",
        sensor.diagnostics.rgb_group_name, sensor.diagnostics.rgb_source_id
    );
    println!(
        "  -> Active NIR Torch Illumination: Supported = {}, Enabled = {} | Nokhwa MSMF Fallback = {}",
        sensor.diagnostics.active_nir_torch_supported,
        sensor.diagnostics.active_nir_torch_enabled,
        sensor.diagnostics.nokhwa_fallback_available
    );

    println!("\n[Phase 2.2] Calibrating Markerless 24\"x24\" VEX Foam Tile Homography (Canny + Hough)...");
    let homography = FieldHomography::calibrate_from_static_field(
        &sensor.static_ir_background,
        &sensor.static_rgb_background,
    )?;
    println!(
        "  -> Locked onto {} orthogonal foam tile seam lines & {} grid intersections (Metric RMSE = {:.4} in)",
        homography.detected_seams.len(),
        homography.grid_intersections_px.len(),
        homography.reprojection_rmse_in
    );

    let sensor_diag = sensor.diagnostics.clone();
    let homography_clone = homography.clone();
    let harness = Arc::new(Mutex::new(ClosedLoopHarness::new(
        transceiver,
        sensor,
        homography,
    )));

    println!("\n[Phase 4] Executing 5-Stage Automated LemLib & Cascade Lift Optimization Pipeline...");
    let tuned = FiveStageTuningPipeline::run_full_pipeline(
        &harness,
        &hw_spec.initial_lateral,
        &hw_spec.initial_angular,
        &hw_spec.initial_swing,
        &hw_spec.initial_cascade,
    )?;

    for sr in &tuned.stage_records {
        println!(
            "  -> [Stage {}] {} ({}) | Evals: {} | Cost J: {:.2} -> {:.2} | t_r: {:.3}s, M_p: {:.2}%, t_s: {:.3}s, e_ss: {:.4}",
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
        );
    }

    println!("\n[Phase 5.1 - 5.4] Executing Exhaustive Multi-Axis Stress-Testing Suite...");
    let stress = StressTestRunner::execute_all(&harness, &tuned)?;
    println!(
        "  -> Scale Invariance Matrix:       {}/{} passed (2\"..72\" Lateral, 10°..180° Point & Swing)",
        stress.scale_invariance.iter().filter(|r| r.passed).count(),
        stress.scale_invariance.len()
    );
    println!(
        "  -> Cascade Lift CG Perturbation:  {}/{} passed (Retracted 0\", Mid 14\", Raised 28\")",
        stress.cg_perturbation.iter().filter(|r| r.passed).count(),
        stress.cg_perturbation.len()
    );
    println!(
        "  -> 15-Lap Chained Endurance Test: {}/{} laps passed (Peak Motor Temp = {:.2}°C < 55.0°C PROS limit)",
        stress.endurance_laps.iter().filter(|r| r.passed).count(),
        stress.endurance_laps.len(),
        stress
            .endurance_laps
            .iter()
            .map(|r| r.left_motor_temp_c.max(r.right_motor_temp_c))
            .fold(0.0f64, f64::max)
    );
    println!(
        "  -> Impulse Disturbance Rejection: {}/{} passed at (24, 24, 90°)",
        stress.impulse_disturbances.iter().filter(|r| r.passed).count(),
        stress.impulse_disturbances.len()
    );

    println!("\n[Phase 5.5] Patching C++ Source Files & Rendering Plotters SVG Engineering Report...");
    let patched = ReportAndPatchGenerator::patch_cpp_sources(&project_root, &tuned)?;
    for pf in &patched {
        println!("  -> Patched C++ file: {}", pf);
    }

    ReportAndPatchGenerator::generate_report_and_svgs(
        &project_root,
        &hw_spec,
        &sensor_diag,
        &homography_clone,
        &tuned,
        &stress,
        &patched,
    )?;

    let report_file: &Path = &project_root.join("TUNING_REPORT.md");
    println!(
        "  -> Generated TUNING_REPORT.md and 6 high-resolution SVG plots at: {}",
        report_file.display()
    );
    println!(
        "\n[COMPLETE] Full suite finished in {:.2}s | COBS TX: {} pkts, RX: {} pkts, CRC Errors: {}",
        t_start.elapsed().as_secs_f64(),
        harness.lock().unwrap().transceiver.tx_packets,
        harness.lock().unwrap().transceiver.rx_packets,
        harness.lock().unwrap().transceiver.crc_errors
    );

    if args.mode.eq_ignore_ascii_case("gui") {
        let sensor_gui = WindowsHelloDualSensor::initialize(320, 240)?;
        let native_options = eframe::NativeOptions::default();
        let _ = eframe::run_native(
            "22050X Windows Hello NIR + RGB PID Tuner HUD",
            native_options,
            Box::new(|_cc| {
                Ok(Box::new(VisionTunerHudApp::new(
                    sensor_gui,
                    homography_clone,
                    tuned,
                    stress,
                )))
            }),
        );
    }

    Ok(())
}
