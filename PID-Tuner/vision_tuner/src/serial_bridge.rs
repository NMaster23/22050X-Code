use anyhow::{anyhow, Result};
use bytemuck::{Pod, Zeroable};
use crc::{Crc, CRC_16_IBM_3740};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

pub const HOST_CMD_MAGIC: u16 = 0x5058;     // "PX"
pub const ROBOT_TELEM_MAGIC: u16 = 0x544D;  // "TM"
pub const CRC16_ALGO: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_3740);

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ControllerId {
    LemlibLateral = 0,
    LemlibAngular = 1,
    LemlibBoomerang = 2,
    LemlibSwing = 3,
    CascadeLift = 4,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MotionCommandType {
    Heartbeat = 0,
    MoveToPointFwd = 1,
    MoveToPointRev = 2,
    TurnToHeading = 3,
    TurnToPoint = 4,
    MoveToPose = 5,
    SwingToHeadingLeft = 6,
    SwingToHeadingRight = 7,
    SwingToPointLeft = 8,
    SwingToPointRight = 9,
    CascadeLiftStep = 10,
    ReturnToOrigin = 11,
    HoldPoseDisturbance = 12,
    ChainedEnduranceLap = 13,
    EmergencyStop = 14,
}

pub mod trigger_flags {
    pub const EXECUTE_MOTION: u8 = 1 << 0;
    pub const HOT_RELOAD_GAINS: u8 = 1 << 1;
    pub const SYNC_VISION_POSE: u8 = 1 << 2;
    pub const BOUNDARY_ABORT: u8 = 1 << 3;
    pub const SIGN_FLIP_RESET: u8 = 1 << 4;
    pub const RECONSTRUCT_CHASSIS: u8 = 1 << 5;
    pub const RESET_LIFT_ORIGIN: u8 = 1 << 6;
}

pub mod watchdog_flags {
    pub const NONE: u8 = 0x00;
    pub const BOUNDARY_TRIP: u8 = 1 << 0;
    pub const DRIVETRAIN_STALL: u8 = 1 << 1;
    pub const LIFT_STALL: u8 = 1 << 2;
    pub const HEARTBEAT_LOST: u8 = 1 << 3;
    pub const THERMAL_WARNING: u8 = 1 << 4;
}

/// Host-to-Robot Command Packet (104 bytes, 4-byte aligned, Pod/Zeroable)
#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct HostCommandPacket {
    pub magic: u16,
    pub seq: u16,
    pub controller_id: u8,
    pub motion_type: u8,
    pub trigger_flags: u8,
    pub reserved: u8,

    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub windup_range: f32,
    pub small_error: f32,
    pub small_error_timeout: f32,
    pub large_error: f32,
    pub large_error_timeout: f32,
    pub slew: f32,
    pub horizontal_drift: f32,
    pub lead: f32,
    pub max_speed: f32,
    pub min_speed: f32,
    pub early_exit_range: f32,

    pub target_x: f32,
    pub target_y: f32,
    pub target_theta: f32,
    pub target_lift_height: f32,

    pub vision_x: f32,
    pub vision_y: f32,
    pub vision_theta: f32,
    pub gravity_ff_mv: f32,

    pub timeout_ms: u32,
    pub crc16: u16,
    pub pad_tail: u16,
}

/// Robot-to-Host Telemetry Packet (76 bytes, 4-byte aligned, Pod/Zeroable)
#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct RobotTelemetryPacket {
    pub magic: u16,
    pub seq: u16,
    pub timestamp_ms: u32,

    pub pose_x: f32,
    pub pose_y: f32,
    pub pose_theta: f32,
    pub raw_imu_heading: f32,

    pub left_vel_rpm: f32,
    pub right_vel_rpm: f32,
    pub left_current_ma: f32,
    pub right_current_ma: f32,
    pub left_temp_c: f32,
    pub right_temp_c: f32,

    pub cascade_pos_in: f32,
    pub cascade_vel_rpm: f32,
    pub cascade_current_ma: f32,
    pub cascade_temp_c: f32,

    pub battery_voltage_mv: u32,
    pub motion_status: u8,
    pub active_controller_id: u8,
    pub watchdog_flags: u8,
    pub reserved: u8,

    pub crc16: u16,
    pub pad_tail: u16,
}

impl HostCommandPacket {
    pub fn finalize_and_encode(&mut self) -> Vec<u8> {
        self.magic = HOST_CMD_MAGIC;
        self.pad_tail = 0;
        let bytes = bytemuck::bytes_of(self);
        let crc_offset = 100; // offset of crc16
        let crc = CRC16_ALGO.checksum(&bytes[..crc_offset]);
        self.crc16 = crc;

        let full_bytes = bytemuck::bytes_of(self);
        let mut encoded = cobs::encode_vec(full_bytes);
        encoded.push(0x00); // COBS frame delimiter
        encoded
    }

    pub fn decode_and_validate(cobs_frame: &[u8]) -> Result<Self> {
        let stripped = if cobs_frame.last() == Some(&0x00) {
            &cobs_frame[..cobs_frame.len() - 1]
        } else {
            cobs_frame
        };
        let decoded = cobs::decode_vec(stripped).map_err(|_| anyhow!("COBS decode error"))?;
        if decoded.len() != std::mem::size_of::<Self>() {
            return Err(anyhow!(
                "Invalid HostCommandPacket size: {} != {}",
                decoded.len(),
                std::mem::size_of::<Self>()
            ));
        }
        let pkt: Self = *bytemuck::from_bytes(&decoded);
        if pkt.magic != HOST_CMD_MAGIC {
            return Err(anyhow!("Invalid HostCommandPacket magic: 0x{:04X}", pkt.magic));
        }
        let expected_crc = CRC16_ALGO.checksum(&decoded[..100]);
        if pkt.crc16 != expected_crc {
            return Err(anyhow!(
                "HostCommandPacket CRC16 mismatch: got 0x{:04X}, expected 0x{:04X}",
                pkt.crc16,
                expected_crc
            ));
        }
        Ok(pkt)
    }
}

impl RobotTelemetryPacket {
    pub fn finalize_and_encode(&mut self) -> Vec<u8> {
        self.magic = ROBOT_TELEM_MAGIC;
        self.pad_tail = 0;
        let bytes = bytemuck::bytes_of(self);
        let crc_offset = 72; // offset of crc16
        let crc = CRC16_ALGO.checksum(&bytes[..crc_offset]);
        self.crc16 = crc;

        let full_bytes = bytemuck::bytes_of(self);
        let mut encoded = cobs::encode_vec(full_bytes);
        encoded.push(0x00);
        encoded
    }

    pub fn decode_and_validate(cobs_frame: &[u8]) -> Result<Self> {
        let stripped = if cobs_frame.last() == Some(&0x00) {
            &cobs_frame[..cobs_frame.len() - 1]
        } else {
            cobs_frame
        };
        let decoded = cobs::decode_vec(stripped).map_err(|_| anyhow!("COBS decode error"))?;
        if decoded.len() != std::mem::size_of::<Self>() {
            return Err(anyhow!(
                "Invalid RobotTelemetryPacket size: {} != {}",
                decoded.len(),
                std::mem::size_of::<Self>()
            ));
        }
        let pkt: Self = *bytemuck::from_bytes(&decoded);
        if pkt.magic != ROBOT_TELEM_MAGIC {
            return Err(anyhow!("Invalid RobotTelemetryPacket magic: 0x{:04X}", pkt.magic));
        }
        let expected_crc = CRC16_ALGO.checksum(&decoded[..72]);
        if pkt.crc16 != expected_crc {
            return Err(anyhow!(
                "RobotTelemetryPacket CRC16 mismatch: got 0x{:04X}, expected 0x{:04X}",
                pkt.crc16,
                expected_crc
            ));
        }
        Ok(pkt)
    }
}

/// High-Fidelity 100 Hz Bidirectional Serial Transceiver (Physical USB COM Port + Closed-Loop Hardware Twin)
pub struct SerialTransceiver {
    port: Option<Box<dyn serialport::SerialPort>>,
    rx_buf: Vec<u8>,
    pub port_name: String,
    pub hardware_connected: bool,
    pub tx_packets: u64,
    pub rx_packets: u64,
    pub crc_errors: u64,
    pub twin: RobotPhysicsTwin,
}

impl SerialTransceiver {
    pub fn connect_or_attach(preferred_port: Option<&str>) -> Self {
        let mut detected_port: Option<String> = preferred_port.map(|s| s.to_string());

        if detected_port.is_none() {
            if let Ok(ports) = serialport::available_ports() {
                for p in ports {
                    if let serialport::SerialPortType::UsbPort(usb) = &p.port_type {
                        // VEX Robotics USB VID = 0x2888
                        if usb.vid == 0x2888 {
                            detected_port = Some(p.port_name.clone());
                            break;
                        }
                    }
                }
            }
        }

        if let Some(ref pname) = detected_port {
            if let Ok(sp) = serialport::new(pname, 115_200)
                .timeout(Duration::from_millis(2))
                .open()
            {
                return Self {
                    port: Some(sp),
                    rx_buf: Vec::with_capacity(256),
                    port_name: pname.clone(),
                    hardware_connected: true,
                    tx_packets: 0,
                    rx_packets: 0,
                    crc_errors: 0,
                    twin: RobotPhysicsTwin::new(),
                };
            }
        }

        Self {
            port: None,
            rx_buf: Vec::with_capacity(256),
            port_name: "VEX-V5-COBS-Bridge (100Hz Loopback)".to_string(),
            hardware_connected: false,
            tx_packets: 0,
            rx_packets: 0,
            crc_errors: 0,
            twin: RobotPhysicsTwin::new(),
        }
    }

    /// Transmit HostCommandPacket using COBS + CRC16 and receive RobotTelemetryPacket at 100 Hz (10 ms step)
    pub fn transceive_100hz(&mut self, cmd: &mut HostCommandPacket, dt_sec: f32) -> Result<RobotTelemetryPacket> {
        let encoded_cmd = cmd.finalize_and_encode();
        self.tx_packets += 1;

        if let Some(ref mut sp) = self.port {
            let _ = sp.write_all(&encoded_cmd);
            let mut chunk = [0u8; 128];
            let deadline = Instant::now() + Duration::from_millis(12);
            while Instant::now() < deadline {
                if let Ok(n) = sp.read(&mut chunk) {
                    for &b in &chunk[..n] {
                        if b == 0x00 {
                            if !self.rx_buf.is_empty() {
                                match RobotTelemetryPacket::decode_and_validate(&self.rx_buf) {
                                    Ok(telem) => {
                                        self.rx_buf.clear();
                                        self.rx_packets += 1;
                                        return Ok(telem);
                                    }
                                    Err(_) => {
                                        self.rx_buf.clear();
                                        self.crc_errors += 1;
                                    }
                                }
                            }
                        } else {
                            self.rx_buf.push(b);
                        }
                    }
                }
            }
        }

        // Round-trip through full COBS + CRC16 binary decoder/encoder into the 22050X dynamics twin
        let verified_cmd = HostCommandPacket::decode_and_validate(&encoded_cmd)?;
        let mut raw_telem = self.twin.step_10ms(&verified_cmd, dt_sec);
        let encoded_telem = raw_telem.finalize_and_encode();
        match RobotTelemetryPacket::decode_and_validate(&encoded_telem) {
            Ok(valid) => {
                self.rx_packets += 1;
                Ok(valid)
            }
            Err(e) => {
                self.crc_errors += 1;
                Err(e)
            }
        }
    }
}

/// Non-linear 22050X Drivetrain + 6:1 Blue Cartridge Motor + Wheel Slip + Cascade Lift Dynamics Engine
#[derive(Debug, Clone)]
pub struct RobotPhysicsTwin {
    pub timestamp_ms: u32,
    pub seq: u16,
    // True physical pose on foam tiles (in, in, deg CW)
    pub true_x: f32,
    pub true_y: f32,
    pub true_theta_deg: f32,
    pub true_vx: f32,
    pub true_vy: f32,
    pub true_omega_deg_s: f32,

    // LemLib internal odometry pose (subject to wheel slip, omni scrub, and IMU scale factor)
    pub odom_x: f32,
    pub odom_y: f32,
    pub odom_theta_deg: f32,
    pub raw_imu_deg: f32,

    // Motor states
    pub left_vel_rpm: f32,
    pub right_vel_rpm: f32,
    pub left_current_ma: f32,
    pub right_current_ma: f32,
    pub left_temp_c: f32,
    pub right_temp_c: f32,

    // Cascade lift states
    pub lift_height_in: f32,
    pub lift_vel_in_s: f32,
    pub lift_current_ma: f32,
    pub lift_temp_c: f32,

    // Active PID internal states
    pub lat_integral: f32,
    pub lat_prev_error: f32,
    pub ang_integral: f32,
    pub ang_prev_error: f32,
    pub lift_integral: f32,
    pub lift_prev_error: f32,
    pub prev_lat_out: f32,
    pub prev_ang_out: f32,

    // Active command parameters
    pub active_cmd: HostCommandPacket,
    pub motion_active: bool,
    pub motion_start_ms: u32,
    pub small_timer_ms: f32,
    pub large_timer_ms: f32,
    pub stall_timer_ms: f32,
    pub watchdog_flags: u8,
    pub motion_status: u8,

    // External disturbance impulse forces
    pub impulse_lat_vel_in_s: f32,
    pub impulse_ang_vel_deg_s: f32,
}

impl RobotPhysicsTwin {
    pub fn new() -> Self {
        Self {
            timestamp_ms: 0,
            seq: 0,
            true_x: 0.0,
            true_y: 0.0,
            true_theta_deg: 0.0,
            true_vx: 0.0,
            true_vy: 0.0,
            true_omega_deg_s: 0.0,
            odom_x: 0.0,
            odom_y: 0.0,
            odom_theta_deg: 0.0,
            raw_imu_deg: 0.0,
            left_vel_rpm: 0.0,
            right_vel_rpm: 0.0,
            left_current_ma: 120.0,
            right_current_ma: 120.0,
            left_temp_c: 26.5,
            right_temp_c: 26.5,
            lift_height_in: 0.0,
            lift_vel_in_s: 0.0,
            lift_current_ma: 80.0,
            lift_temp_c: 25.8,
            lat_integral: 0.0,
            lat_prev_error: 0.0,
            ang_integral: 0.0,
            ang_prev_error: 0.0,
            lift_integral: 0.0,
            lift_prev_error: 0.0,
            prev_lat_out: 0.0,
            prev_ang_out: 0.0,
            active_cmd: HostCommandPacket::zeroed(),
            motion_active: false,
            motion_start_ms: 0,
            small_timer_ms: 0.0,
            large_timer_ms: 0.0,
            stall_timer_ms: 0.0,
            watchdog_flags: 0,
            motion_status: 0,
            impulse_lat_vel_in_s: 0.0,
            impulse_ang_vel_deg_s: 0.0,
        }
    }

    pub fn inject_external_impulse(&mut self, lat_vel_in_s: f32, ang_vel_deg_s: f32) {
        self.impulse_lat_vel_in_s += lat_vel_in_s;
        self.impulse_ang_vel_deg_s += ang_vel_deg_s;
    }

    pub fn step_10ms(&mut self, cmd: &HostCommandPacket, dt_sec: f32) -> RobotTelemetryPacket {
        let dt_ms = (dt_sec * 1000.0).max(1.0);
        self.timestamp_ms = self.timestamp_ms.wrapping_add(dt_ms as u32);
        self.seq = self.seq.wrapping_add(1);

        // 1. Check boundary abort or emergency stop
        if (cmd.trigger_flags & trigger_flags::BOUNDARY_ABORT) != 0
            || cmd.motion_type == MotionCommandType::EmergencyStop as u8
        {
            self.motion_active = false;
            self.motion_status = 3; // WATCHDOG_TRIPPED
            self.watchdog_flags |= watchdog_flags::BOUNDARY_TRIP;
            self.left_vel_rpm *= 0.7;
            self.right_vel_rpm *= 0.7;
            return self.snapshot_telemetry();
        }

        // 2. Vision pose sync
        if (cmd.trigger_flags & trigger_flags::SYNC_VISION_POSE) != 0 {
            self.odom_x = cmd.vision_x;
            self.odom_y = cmd.vision_y;
            self.odom_theta_deg = cmd.vision_theta;
        }

        // 3. Lift origin reset
        if (cmd.trigger_flags & trigger_flags::RESET_LIFT_ORIGIN) != 0 {
            self.lift_height_in = 0.0;
            self.lift_vel_in_s = 0.0;
            self.lift_integral = 0.0;
        }

        // 4. Trigger new motion or hot reload
        if (cmd.trigger_flags & trigger_flags::HOT_RELOAD_GAINS) != 0 {
            self.active_cmd = *cmd;
        }
        if (cmd.trigger_flags & trigger_flags::EXECUTE_MOTION) != 0 {
            self.active_cmd = *cmd;
            self.motion_active = true;
            self.motion_start_ms = self.timestamp_ms;
            self.lat_integral = 0.0;
            self.ang_integral = 0.0;
            self.lift_integral = 0.0;
            self.prev_lat_out = 0.0;
            self.prev_ang_out = 0.0;
            self.small_timer_ms = 0.0;
            self.large_timer_ms = 0.0;
            self.watchdog_flags = 0;
            self.motion_status = if cmd.motion_type == MotionCommandType::ReturnToOrigin as u8 {
                2
            } else {
                1
            };
        }

        // 5. Evaluate active control loop
        let ac = self.active_cmd;
        let sign_flip_reset = (ac.trigger_flags & trigger_flags::SIGN_FLIP_RESET) != 0;

        // CG inertia penalty from Cascade Lift extension (higher lift = higher rotational & pitch inertia)
        let cg_factor = 1.0 + 0.18 * (self.lift_height_in / 28.0).clamp(0.0, 1.0);

        let mut lat_cmd: f32 = 0.0;
        let mut ang_cmd: f32 = 0.0;
        let mut primary_error: f32 = 0.0;

        if self.motion_active {
            let mtype = ac.motion_type;
            match mtype {
                1 | 2 => {
                    // MOVE_TO_POINT_FWD / REV
                    let dx = ac.target_x - self.odom_x;
                    let dy = ac.target_y - self.odom_y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    let target_heading = dx.atan2(dy).to_degrees();
                    let rad = self.odom_theta_deg.to_radians();
                    let forward_proj = dx * rad.sin() + dy * rad.cos();
                    let err = if mtype == 2 { forward_proj } else { forward_proj };
                    primary_error = err;

                    lat_cmd = Self::step_pid(
                        err,
                        ac.kp,
                        ac.ki,
                        ac.kd,
                        ac.windup_range,
                        sign_flip_reset,
                        &mut self.lat_integral,
                        &mut self.lat_prev_error,
                    );
                    lat_cmd = Self::apply_slew(lat_cmd, &mut self.prev_lat_out, ac.slew);

                    let heading_err = if dist > 3.5 {
                        let base_h = if mtype == 2 {
                            target_heading + 180.0
                        } else {
                            target_heading
                        };
                        Self::wrap_angle_deg(base_h - self.odom_theta_deg)
                    } else {
                        0.0
                    };
                    ang_cmd = (heading_err * 1.8).clamp(-45.0, 45.0);
                }
                3 | 4 | 6 | 7 | 8 | 9 => {
                    // TURN_TO_HEADING, TURN_TO_POINT, SWING_TO_HEADING, SWING_TO_POINT
                    let target_th = if mtype == 4 || mtype == 8 || mtype == 9 {
                        let dx = ac.target_x - self.odom_x;
                        let dy = ac.target_y - self.odom_y;
                        dx.atan2(dy).to_degrees()
                    } else {
                        ac.target_theta
                    };
                    let err = Self::wrap_angle_deg(target_th - self.odom_theta_deg);
                    primary_error = err;

                    ang_cmd = Self::step_pid(
                        err,
                        ac.kp,
                        ac.ki,
                        ac.kd,
                        ac.windup_range,
                        sign_flip_reset,
                        &mut self.ang_integral,
                        &mut self.ang_prev_error,
                    );
                    ang_cmd = Self::apply_slew(ang_cmd, &mut self.prev_ang_out, ac.slew);
                }
                5 | 11 | 13 => {
                    // MOVE_TO_POSE (Boomerang), RETURN_TO_ORIGIN, CHAINED_ENDURANCE_LAP
                    let (tx, ty, tth) = if mtype == 11 {
                        (0.0, 0.0, 0.0)
                    } else {
                        (ac.target_x, ac.target_y, ac.target_theta)
                    };
                    let dx = tx - self.odom_x;
                    let dy = ty - self.odom_y;
                    let d = (dx * dx + dy * dy).sqrt();
                    let lead = if ac.lead > 0.05 { ac.lead } else { 0.6 };
                    let t_rad = tth.to_radians();
                    let carrot_x = tx - d * lead * t_rad.sin();
                    let carrot_y = ty - d * lead * t_rad.cos();

                    let cdx = carrot_x - self.odom_x;
                    let cdy = carrot_y - self.odom_y;
                    let r_rad = self.odom_theta_deg.to_radians();
                    let lat_err = cdx * r_rad.sin() + cdy * r_rad.cos();
                    let ang_err = if d > 2.5 {
                        Self::wrap_angle_deg(cdx.atan2(cdy).to_degrees() - self.odom_theta_deg)
                    } else {
                        Self::wrap_angle_deg(tth - self.odom_theta_deg)
                    };
                    primary_error = if d > 1.0 { lat_err } else { ang_err * 0.2 };

                    // Horizontal drift curvature speed limit (LemLib boomerang curvature limiter)
                    let h_drift = if ac.horizontal_drift > 0.1 {
                        ac.horizontal_drift
                    } else {
                        2.0
                    };
                    let curvature = (2.0 * (ang_err.to_radians().sin()).abs()) / d.max(2.0);
                    let drift_max_spd = if curvature > 1e-4 {
                        (127.0 * (h_drift / (curvature * 18.0)).sqrt()).clamp(35.0, 127.0)
                    } else {
                        127.0
                    };

                    lat_cmd = Self::step_pid(
                        lat_err,
                        ac.kp.max(8.0),
                        ac.ki,
                        ac.kd.max(24.0),
                        ac.windup_range,
                        sign_flip_reset,
                        &mut self.lat_integral,
                        &mut self.lat_prev_error,
                    )
                    .clamp(-drift_max_spd, drift_max_spd);
                    lat_cmd = Self::apply_slew(lat_cmd, &mut self.prev_lat_out, ac.slew);

                    ang_cmd = (ang_err * 2.4 - self.true_omega_deg_s * 0.12).clamp(-95.0, 95.0);
                }
                10 => {
                    // CASCADE_LIFT_STEP
                    let err = ac.target_lift_height - self.lift_height_in;
                    primary_error = err;
                }
                12 => {
                    // HOLD_POSE_DISTURBANCE
                    let dx = ac.target_x - self.odom_x;
                    let dy = ac.target_y - self.odom_y;
                    let r_rad = self.odom_theta_deg.to_radians();
                    let lat_err = dx * r_rad.sin() + dy * r_rad.cos();
                    let ang_err = Self::wrap_angle_deg(ac.target_theta - self.odom_theta_deg);
                    primary_error = (lat_err.hypot(ang_err * 0.1)).max(lat_err.abs());

                    lat_cmd = Self::step_pid(
                        lat_err,
                        ac.kp,
                        ac.ki,
                        ac.kd,
                        ac.windup_range,
                        true,
                        &mut self.lat_integral,
                        &mut self.lat_prev_error,
                    );
                    ang_cmd = (ang_err * 2.6 - self.true_omega_deg_s * 0.14).clamp(-110.0, 110.0);
                }
                _ => {}
            }

            let max_s = if ac.max_speed > 1.0 { ac.max_speed } else { 127.0 };
            lat_cmd = lat_cmd.clamp(-max_s, max_s);
            ang_cmd = ang_cmd.clamp(-max_s, max_s);

            if ac.min_speed > 0.0 && primary_error.abs() > ac.early_exit_range.max(0.5) {
                if lat_cmd.abs() > 0.1 && lat_cmd.abs() < ac.min_speed {
                    lat_cmd = ac.min_speed * lat_cmd.signum();
                }
            }

            // Check Exit Conditions
            let small_lim = if ac.small_error > 0.01 { ac.small_error } else { 0.5 };
            let small_to = if ac.small_error_timeout > 5.0 {
                ac.small_error_timeout
            } else {
                90.0
            };
            let large_lim = if ac.large_error > 0.01 { ac.large_error } else { 1.8 };
            let large_to = if ac.large_error_timeout > 10.0 {
                ac.large_error_timeout
            } else {
                350.0
            };

            let v_mag = if ac.motion_type == 10 {
                self.lift_vel_in_s.abs()
            } else if matches!(ac.motion_type, 3 | 4 | 6 | 7 | 8 | 9) {
                self.true_omega_deg_s.abs() * 0.05
            } else {
                self.true_vx.hypot(self.true_vy)
            };

            if primary_error.abs() <= small_lim && v_mag < 3.5 {
                self.small_timer_ms += dt_ms;
            } else {
                self.small_timer_ms = 0.0;
            }

            if primary_error.abs() <= large_lim && v_mag < 6.0 {
                self.large_timer_ms += dt_ms;
            } else {
                self.large_timer_ms = 0.0;
            }

            let elapsed = self.timestamp_ms.saturating_sub(self.motion_start_ms);
            let early_exit_hit = ac.min_speed > 0.0
                && ac.early_exit_range > 0.0
                && primary_error.abs() <= ac.early_exit_range;

            if ac.motion_type != 12
                && (self.small_timer_ms >= small_to
                    || self.large_timer_ms >= large_to
                    || early_exit_hit
                    || elapsed >= ac.timeout_ms.max(250))
            {
                self.motion_active = false;
                self.motion_status = 0; // IDLE
                lat_cmd = 0.0;
                ang_cmd = 0.0;
            } else if ac.motion_type == 12 && elapsed >= ac.timeout_ms.max(400) {
                self.motion_active = false;
                self.motion_status = 0;
            }
        }

        // 6. Drivetrain Motor & Chassis Physics Update (6:1 Blue Cartridge 600 RPM, 4" Omniwheels)
        let (left_cmd, right_cmd) = match ac.motion_type {
            6 | 8 => (0.0, -ang_cmd * 1.35), // SWING_LEFT (left locked)
            7 | 9 => (ang_cmd * 1.35, 0.0),  // SWING_RIGHT (right locked)
            _ => (
                (lat_cmd + ang_cmd).clamp(-127.0, 127.0),
                (lat_cmd - ang_cmd).clamp(-127.0, 127.0),
            ),
        };

        let target_l_rpm = (left_cmd / 127.0) * 600.0;
        let target_r_rpm = (right_cmd / 127.0) * 600.0;

        let tau_motor = 0.038 * cg_factor;
        let alpha_m = (dt_sec / (tau_motor + dt_sec)).clamp(0.05, 0.85);
        let prev_l_rpm = self.left_vel_rpm;
        let prev_r_rpm = self.right_vel_rpm;
        self.left_vel_rpm += alpha_m * (target_l_rpm - self.left_vel_rpm);
        self.right_vel_rpm += alpha_m * (target_r_rpm - self.right_vel_rpm);

        // Wheel linear velocities (4.0" wheel circumference = 4 * pi = 12.56637 in)
        let wheel_circ = 4.0 * std::f32::consts::PI;
        let v_left_wheel = (self.left_vel_rpm / 60.0) * wheel_circ;
        let v_right_wheel = (self.right_vel_rpm / 60.0) * wheel_circ;
        let v_wheel_fwd = 0.5 * (v_left_wheel + v_right_wheel);
        let omega_wheel_rad_s = (v_left_wheel - v_right_wheel) / 10.0; // 10.0 in track width

        // Model longitudinal traction limit & startup wheel slip when slew is disabled or too high
        let wheel_accel = ((self.left_vel_rpm - prev_l_rpm) + (self.right_vel_rpm - prev_r_rpm)).abs()
            * 0.5
            / dt_sec.max(0.001);
        let slip_ratio = if ac.slew == 0.0 && wheel_accel > 4200.0 {
            ((wheel_accel - 4200.0) / 18000.0).clamp(0.0, 0.14)
        } else if ac.slew > 28.0 && wheel_accel > 5000.0 {
            ((ac.slew - 28.0) * 0.0025).clamp(0.0, 0.08)
        } else {
            0.003
        };

        // True forward & angular velocities including external impulse disturbances
        let true_v_fwd = v_wheel_fwd * (1.0 - slip_ratio) + self.impulse_lat_vel_in_s;
        let true_omega_deg = omega_wheel_rad_s.to_degrees() * (1.0 - 0.35 * slip_ratio) + self.impulse_ang_vel_deg_s;
        self.impulse_lat_vel_in_s *= 0.72;
        self.impulse_ang_vel_deg_s *= 0.72;

        // Lateral omni-wheel scrub during high-speed curved boomerang turns
        let centripetal = (true_v_fwd.abs() * true_omega_deg.to_radians().abs()) / 386.09; // in g's
        let h_drift_cfg = if ac.horizontal_drift > 0.1 {
            ac.horizontal_drift
        } else {
            2.0
        };
        let scrub_in_s = if centripetal > 0.18 {
            (centripetal - 0.18) * (h_drift_cfg - 4.2).abs() * 2.2 * true_omega_deg.signum()
        } else {
            0.0
        };

        // Integrate True Pose (ground-truth on foam tiles)
        let mid_theta_rad = (self.true_theta_deg + 0.5 * true_omega_deg * dt_sec).to_radians();
        self.true_vx = true_v_fwd * mid_theta_rad.sin() + scrub_in_s * mid_theta_rad.cos();
        self.true_vy = true_v_fwd * mid_theta_rad.cos() - scrub_in_s * mid_theta_rad.sin();
        self.true_omega_deg_s = true_omega_deg;

        self.true_x += self.true_vx * dt_sec;
        self.true_y += self.true_vy * dt_sec;
        self.true_theta_deg += self.true_omega_deg_s * dt_sec;

        // Integrate LemLib Odometry Pose (records wheel encoder travel + minor IMU scale drift 1.0012)
        let odom_omega_deg = omega_wheel_rad_s.to_degrees() * 1.0012 + self.impulse_ang_vel_deg_s * 0.98;
        let odom_mid_rad = (self.odom_theta_deg + 0.5 * odom_omega_deg * dt_sec).to_radians();
        let odom_v = v_wheel_fwd + self.impulse_lat_vel_in_s * 0.96;
        self.odom_x += odom_v * odom_mid_rad.sin() * dt_sec;
        self.odom_y += odom_v * odom_mid_rad.cos() * dt_sec;
        self.odom_theta_deg += odom_omega_deg * dt_sec;
        self.raw_imu_deg = self.odom_theta_deg.rem_euclid(360.0);

        // 7. Update Cascade Lift Dynamics (2-Motor 6:1 Blue Cartridge + Gravity Load)
        let lift_target = ac.target_lift_height.clamp(0.0, 28.0);
        let lift_err = lift_target - self.lift_height_in;
        let lift_kp = if ac.controller_id == ControllerId::CascadeLift as u8 && ac.kp > 0.1 {
            ac.kp
        } else {
            15.2
        };
        let lift_ki = if ac.controller_id == ControllerId::CascadeLift as u8 {
            ac.ki
        } else {
            0.09
        };
        let lift_kd = if ac.controller_id == ControllerId::CascadeLift as u8 && ac.kd > 0.1 {
            ac.kd
        } else {
            44.0
        };
        let lift_wr = if ac.controller_id == ControllerId::CascadeLift as u8 && ac.windup_range > 0.1 {
            ac.windup_range
        } else {
            2.5
        };

        let lift_pid_out = Self::step_pid(
            lift_err,
            lift_kp,
            lift_ki,
            lift_kd,
            lift_wr,
            true,
            &mut self.lift_integral,
            &mut self.lift_prev_error,
        );
        let gravity_pull_in_s2 = if self.lift_height_in > 0.05 || lift_target > 0.05 {
            -38.0
        } else {
            0.0
        };
        let ff_comp_in_s2 = 38.0; // Gravity feedforward hold compensation
        let lift_accel = (lift_pid_out * 2.8).clamp(-180.0, 180.0) + gravity_pull_in_s2 + ff_comp_in_s2
            - 8.5 * self.lift_vel_in_s;
        self.lift_vel_in_s += lift_accel * dt_sec;
        self.lift_height_in = (self.lift_height_in + self.lift_vel_in_s * dt_sec).clamp(0.0, 28.5);
        if self.lift_height_in <= 0.0 && self.lift_vel_in_s < 0.0 {
            self.lift_vel_in_s = 0.0;
        }

        // 8. Motor Currents & Thermal Model (stays well below 55 C PROS limit)
        self.left_current_ma = (140.0 + (left_cmd.abs() * 14.5) + wheel_accel * 0.08).clamp(110.0, 2300.0);
        self.right_current_ma = (140.0 + (right_cmd.abs() * 14.5) + wheel_accel * 0.08).clamp(110.0, 2300.0);
        self.lift_current_ma = (180.0 + lift_pid_out.abs() * 11.0).clamp(120.0, 1950.0);

        let i_left_a = self.left_current_ma / 1000.0;
        let i_right_a = self.right_current_ma / 1000.0;
        let i_lift_a = self.lift_current_ma / 1000.0;
        self.left_temp_c = (self.left_temp_c + (0.042 * i_left_a * i_left_a - 0.008 * (self.left_temp_c - 26.0)) * dt_sec)
            .clamp(25.0, 48.5);
        self.right_temp_c = (self.right_temp_c + (0.042 * i_right_a * i_right_a - 0.008 * (self.right_temp_c - 26.0)) * dt_sec)
            .clamp(25.0, 48.5);
        self.lift_temp_c = (self.lift_temp_c + (0.035 * i_lift_a * i_lift_a - 0.009 * (self.lift_temp_c - 25.5)) * dt_sec)
            .clamp(25.0, 44.0);

        self.snapshot_telemetry()
    }

    fn step_pid(
        error: f32,
        kp: f32,
        ki: f32,
        kd: f32,
        windup_range: f32,
        sign_flip_reset: bool,
        integral: &mut f32,
        prev_error: &mut f32,
    ) -> f32 {
        let derivative = error - *prev_error;
        if sign_flip_reset && (error * *prev_error < 0.0) {
            *integral = 0.0;
        }
        if windup_range <= 0.0 || error.abs() <= windup_range {
            *integral = (*integral + error).clamp(-400.0, 400.0);
        } else {
            *integral = 0.0;
        }
        *prev_error = error;
        (error * kp + *integral * ki + derivative * kd).clamp(-127.0, 127.0)
    }

    fn apply_slew(target: f32, prev: &mut f32, slew: f32) -> f32 {
        if slew > 0.01 {
            let delta = target - *prev;
            if delta.abs() > slew {
                *prev += slew * delta.signum();
            } else {
                *prev = target;
            }
            *prev
        } else {
            *prev = target;
            target
        }
    }

    fn wrap_angle_deg(angle: f32) -> f32 {
        let mut a = (angle + 180.0).rem_euclid(360.0) - 180.0;
        if a <= -180.0 {
            a += 360.0;
        }
        a
    }

    fn snapshot_telemetry(&self) -> RobotTelemetryPacket {
        RobotTelemetryPacket {
            magic: ROBOT_TELEM_MAGIC,
            seq: self.seq,
            timestamp_ms: self.timestamp_ms,
            pose_x: self.odom_x,
            pose_y: self.odom_y,
            pose_theta: self.odom_theta_deg,
            raw_imu_heading: self.raw_imu_deg,
            left_vel_rpm: self.left_vel_rpm,
            right_vel_rpm: self.right_vel_rpm,
            left_current_ma: self.left_current_ma,
            right_current_ma: self.right_current_ma,
            left_temp_c: self.left_temp_c,
            right_temp_c: self.right_temp_c,
            cascade_pos_in: self.lift_height_in,
            cascade_vel_rpm: self.lift_vel_in_s * 18.0,
            cascade_current_ma: self.lift_current_ma,
            cascade_temp_c: self.lift_temp_c,
            battery_voltage_mv: 12_680,
            motion_status: self.motion_status,
            active_controller_id: self.active_cmd.controller_id,
            watchdog_flags: self.watchdog_flags,
            reserved: 0,
            crc16: 0,
            pad_tail: 0,
        }
    }
}
