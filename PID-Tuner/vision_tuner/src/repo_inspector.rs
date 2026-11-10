use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorPortSpec {
    pub name: String,
    pub port: i8,
    pub reversed: bool,
    pub cartridge_rpm: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackingWheelSpec {
    pub name: String,
    pub port: u8,
    pub diameter_in: f32,
    pub offset_in: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerGainSet {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub windup_range: f32,
    pub small_error: f32,
    pub small_error_timeout: f32,
    pub large_error: f32,
    pub large_error_timeout: f32,
    pub slew: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CascadePidSpec {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub windup_range: f32,
    pub sign_flip_reset: bool,
    pub gravity_ff_mv: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotHardwareSpec {
    pub project_name: String,
    pub pros_kernel_version: String,
    pub lemlib_version: String,
    pub drivetrain_motors: Vec<MotorPortSpec>,
    pub cascade_motors: Vec<MotorPortSpec>,
    pub auxiliary_motors: Vec<MotorPortSpec>,
    pub imu_port: u8,
    pub tracking_wheels: Vec<TrackingWheelSpec>,
    pub track_width_in: f32,
    pub wheel_diameter_in: f32,
    pub drivetrain_rpm: f32,
    pub horizontal_drift: f32,
    pub boomerang_lead: f32,
    pub initial_lateral: ControllerGainSet,
    pub initial_angular: ControllerGainSet,
    pub initial_swing: ControllerGainSet,
    pub initial_cascade: CascadePidSpec,
}

pub fn inspect_repository(repo_root: &Path) -> Result<RobotHardwareSpec> {
    let pros_path = find_first_existing(
        repo_root,
        &["project.pros", "PID-Tuner/project.pros", "22050X-Autonomous/project.pros"],
    )
    .context("Could not locate project.pros in workspace")?;

    let pros_text = fs::read_to_string(&pros_path)?;
    let pros_json: serde_json::Value = serde_json::from_str(&pros_text)?;

    let state = &pros_json["py/state"];
    let project_name = state["project_name"]
        .as_str()
        .unwrap_or("22050X-PID-Tuner")
        .to_string();
    let pros_kernel_version = state["templates"]["kernel"]["version"]
        .as_str()
        .unwrap_or("4.2.3")
        .to_string();
    let lemlib_version = state["templates"]["LemLib"]["version"]
        .as_str()
        .unwrap_or("0.5.6")
        .to_string();

    // Parse include/robot_config.hpp, src/main.cpp, 22050X-Driver/src/main.cpp, and 22050X-Autonomous/src/main.cpp
    let mut combined_sources = String::new();
    for rel in &[
        "include/robot_config.hpp",
        "src/main.cpp",
        "22050X-Driver/src/main.cpp",
        "22050X-Autonomous/src/main.cpp",
        "PID-Tuner/src/main.cpp",
    ] {
        let p = repo_root.join(rel);
        if p.exists() {
            if let Ok(content) = fs::read_to_string(&p) {
                combined_sources.push_str(&content);
                combined_sources.push('\n');
            }
        }
    }

    let lf_port = extract_int_constant(&combined_sources, "MOTOR_LEFT_FRONT_PORT").unwrap_or(20) as i8;
    let lb_port = extract_int_constant(&combined_sources, "MOTOR_LEFT_BACK_PORT").unwrap_or(-10) as i8;
    let rf_port = extract_int_constant(&combined_sources, "MOTOR_RIGHT_FRONT_PORT").unwrap_or(-7) as i8;
    let rb_port = extract_int_constant(&combined_sources, "MOTOR_RIGHT_BACK_PORT").unwrap_or(9) as i8;

    let cas_l_port = extract_int_constant(&combined_sources, "MOTOR_CASCADE_LEFT_PORT").unwrap_or(1) as i8;
    let cas_r_port = extract_int_constant(&combined_sources, "MOTOR_CASCADE_RIGHT_PORT").unwrap_or(-2) as i8;

    let intake_port = extract_int_constant(&combined_sources, "MOTOR_INTAKE_PORT").unwrap_or(19) as i8;
    let ee_rot_port = extract_int_constant(&combined_sources, "END_EFFECTOR_ROT_PORT").unwrap_or(18) as i8;
    let ee_sense_port = extract_int_constant(&combined_sources, "END_EFFECTOR_ROT_SENSE_PORT").unwrap_or(13) as i8;
    let ee_port = extract_int_constant(&combined_sources, "END_EFFECTOR_PORT").unwrap_or(14) as i8;

    let imu_port = extract_int_constant(&combined_sources, "IMU_PORT").unwrap_or(3) as u8;
    let odom_x_port = extract_int_constant(&combined_sources, "ODOM_X_PORT").unwrap_or(3) as u8;
    let odom_y_port = extract_int_constant(&combined_sources, "ODOM_Y_PORT").unwrap_or(0) as u8;

    let track_width_in = extract_float_constant(&combined_sources, "DRIVETRAIN_TRACK_WIDTH_IN").unwrap_or(10.0);
    let drivetrain_rpm = extract_float_constant(&combined_sources, "DRIVETRAIN_RPM").unwrap_or(600.0);
    let horizontal_drift = extract_float_constant(&combined_sources, "TUNED_HORIZONTAL_DRIFT").unwrap_or(2.0);
    let boomerang_lead = extract_float_constant(&combined_sources, "TUNED_BOOMERANG_LEAD").unwrap_or(0.6);

    let drivetrain_motors = vec![
        MotorPortSpec {
            name: "left_front".to_string(),
            port: lf_port.abs(),
            reversed: lf_port < 0,
            cartridge_rpm: 600.0,
        },
        MotorPortSpec {
            name: "left_back".to_string(),
            port: lb_port.abs(),
            reversed: lb_port < 0,
            cartridge_rpm: 600.0,
        },
        MotorPortSpec {
            name: "right_front".to_string(),
            port: rf_port.abs(),
            reversed: rf_port < 0,
            cartridge_rpm: 600.0,
        },
        MotorPortSpec {
            name: "right_back".to_string(),
            port: rb_port.abs(),
            reversed: rb_port < 0,
            cartridge_rpm: 600.0,
        },
    ];

    let cascade_motors = vec![
        MotorPortSpec {
            name: "cascade_left".to_string(),
            port: cas_l_port.abs(),
            reversed: cas_l_port < 0,
            cartridge_rpm: 600.0,
        },
        MotorPortSpec {
            name: "cascade_right".to_string(),
            port: cas_r_port.abs(),
            reversed: cas_r_port < 0,
            cartridge_rpm: 600.0,
        },
    ];

    let auxiliary_motors = vec![
        MotorPortSpec {
            name: "intake".to_string(),
            port: intake_port.abs(),
            reversed: intake_port < 0,
            cartridge_rpm: 600.0,
        },
        MotorPortSpec {
            name: "end_effector_rot".to_string(),
            port: ee_rot_port.abs(),
            reversed: ee_rot_port < 0,
            cartridge_rpm: 200.0,
        },
        MotorPortSpec {
            name: "end_effector_rot_sense".to_string(),
            port: ee_sense_port.abs(),
            reversed: ee_sense_port < 0,
            cartridge_rpm: 200.0,
        },
        MotorPortSpec {
            name: "end_effector".to_string(),
            port: ee_port.abs(),
            reversed: ee_port < 0,
            cartridge_rpm: 200.0,
        },
    ];

    let tracking_wheels = vec![
        TrackingWheelSpec {
            name: "horizontal_tracking (odom_x)".to_string(),
            port: odom_x_port,
            diameter_in: 2.0,
            offset_in: -5.75,
        },
        TrackingWheelSpec {
            name: "vertical_tracking (odom_y)".to_string(),
            port: odom_y_port,
            diameter_in: 2.0,
            offset_in: -2.50,
        },
    ];

    let initial_lateral = ControllerGainSet {
        kp: extract_float_constant(&combined_sources, "LATERAL_KP").unwrap_or(10.0),
        ki: extract_float_constant(&combined_sources, "LATERAL_KI").unwrap_or(0.0),
        kd: extract_float_constant(&combined_sources, "LATERAL_KD").unwrap_or(30.0),
        windup_range: extract_float_constant(&combined_sources, "LATERAL_WINDUP_RANGE").unwrap_or(3.0),
        small_error: extract_float_constant(&combined_sources, "LATERAL_SMALL_ERROR").unwrap_or(1.0),
        small_error_timeout: extract_float_constant(&combined_sources, "LATERAL_SMALL_ERROR_TIMEOUT").unwrap_or(100.0),
        large_error: extract_float_constant(&combined_sources, "LATERAL_LARGE_ERROR").unwrap_or(3.0),
        large_error_timeout: extract_float_constant(&combined_sources, "LATERAL_LARGE_ERROR_TIMEOUT").unwrap_or(500.0),
        slew: extract_float_constant(&combined_sources, "LATERAL_SLEW").unwrap_or(20.0),
    };

    let initial_angular = ControllerGainSet {
        kp: extract_float_constant(&combined_sources, "ANGULAR_KP").unwrap_or(2.0),
        ki: extract_float_constant(&combined_sources, "ANGULAR_KI").unwrap_or(0.0),
        kd: extract_float_constant(&combined_sources, "ANGULAR_KD").unwrap_or(10.0),
        windup_range: extract_float_constant(&combined_sources, "ANGULAR_WINDUP_RANGE").unwrap_or(3.0),
        small_error: extract_float_constant(&combined_sources, "ANGULAR_SMALL_ERROR").unwrap_or(1.0),
        small_error_timeout: extract_float_constant(&combined_sources, "ANGULAR_SMALL_ERROR_TIMEOUT").unwrap_or(100.0),
        large_error: extract_float_constant(&combined_sources, "ANGULAR_LARGE_ERROR").unwrap_or(3.0),
        large_error_timeout: extract_float_constant(&combined_sources, "ANGULAR_LARGE_ERROR_TIMEOUT").unwrap_or(500.0),
        slew: extract_float_constant(&combined_sources, "ANGULAR_SLEW").unwrap_or(0.0),
    };

    let initial_swing = ControllerGainSet {
        kp: extract_float_constant(&combined_sources, "SWING_KP").unwrap_or(2.4),
        ki: extract_float_constant(&combined_sources, "SWING_KI").unwrap_or(0.0),
        kd: extract_float_constant(&combined_sources, "SWING_KD").unwrap_or(12.0),
        windup_range: extract_float_constant(&combined_sources, "SWING_WINDUP_RANGE").unwrap_or(3.0),
        small_error: extract_float_constant(&combined_sources, "SWING_SMALL_ERROR").unwrap_or(1.0),
        small_error_timeout: extract_float_constant(&combined_sources, "SWING_SMALL_ERROR_TIMEOUT").unwrap_or(100.0),
        large_error: extract_float_constant(&combined_sources, "SWING_LARGE_ERROR").unwrap_or(3.0),
        large_error_timeout: extract_float_constant(&combined_sources, "SWING_LARGE_ERROR_TIMEOUT").unwrap_or(500.0),
        slew: extract_float_constant(&combined_sources, "SWING_SLEW").unwrap_or(0.0),
    };

    let initial_cascade = CascadePidSpec {
        kp: extract_float_constant(&combined_sources, "CASCADE_KP").unwrap_or(14.5),
        ki: extract_float_constant(&combined_sources, "CASCADE_KI").unwrap_or(0.08),
        kd: extract_float_constant(&combined_sources, "CASCADE_KD").unwrap_or(42.0),
        windup_range: extract_float_constant(&combined_sources, "CASCADE_WINDUP_RANGE").unwrap_or(2.5),
        sign_flip_reset: true,
        gravity_ff_mv: extract_float_constant(&combined_sources, "CASCADE_GRAVITY_FF_MV").unwrap_or(1150.0),
    };

    Ok(RobotHardwareSpec {
        project_name,
        pros_kernel_version,
        lemlib_version,
        drivetrain_motors,
        cascade_motors,
        auxiliary_motors,
        imu_port,
        tracking_wheels,
        track_width_in,
        wheel_diameter_in: 4.0,
        drivetrain_rpm,
        horizontal_drift,
        boomerang_lead,
        initial_lateral,
        initial_angular,
        initial_swing,
        initial_cascade,
    })
}

fn find_first_existing(root: &Path, candidates: &[&str]) -> Option<PathBuf> {
    for c in candidates {
        let p = root.join(c);
        if p.exists() {
            return Some(p);
        }
    }
    None
}

fn extract_int_constant(source: &str, ident: &str) -> Option<i32> {
    for line in source.lines() {
        if line.contains(ident) && line.contains('=') {
            let rhs = line.split('=').nth(1)?;
            let cleaned = rhs
                .trim()
                .trim_end_matches(';')
                .trim();
            if let Ok(val) = cleaned.parse::<i32>() {
                return Some(val);
            }
        }
    }
    None
}

fn extract_float_constant(source: &str, ident: &str) -> Option<f32> {
    for line in source.lines() {
        if line.contains(ident) && line.contains('=') {
            let rhs = line.split('=').nth(1)?;
            let cleaned = rhs
                .trim()
                .trim_end_matches(';')
                .trim()
                .trim_end_matches('f');
            if let Ok(val) = cleaned.parse::<f32>() {
                return Some(val);
            }
        }
    }
    None
}
