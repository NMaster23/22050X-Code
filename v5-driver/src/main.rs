use vexide::{controller::ControllerState, prelude::*};
use vexide_motorgroup::*;

const CONTROL_THRESHOLD: f64 = 0.01;
const CURVE_WEIGHT: f64 = 0.7;

async fn drive(controller: ControllerState, drive_left: MotorGroup, drive_right: MotorGroup) {
    let y_pos = controller.left_stick.y();
    let x_pos = controller.right_stick.x();
    let input = if y_pos.abs() <= CONTROL_THRESHOLD {
        0.0
    } else {
        y_pos.signum() * ((y_pos.abs() - CONTROL_THRESHOLD) / 1.0 - CONTROL_THRESHOLD)
    };
    let response = CURVE_WEIGHT * input.powi(3) + (1.0 - CURVE_WEIGHT) * input;
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let controller = peripherals.primary_controller;
    let mut intake = Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward);
    let mut cascade_left = Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward);
    let mut cascade_right = Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward);
    let mut left_front = Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward);
    let mut left_back = Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse);
    let mut right_front = Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward);
    let mut right_back = Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse);
    let mut drivetrain_left = MotorGroup::new(vec![left_front, left_back]);
    let mut drivetrain_right = MotorGroup::new(vec![right_front, right_back]);
    let mut cascade = MotorGroup::new(vec![cascade_left, cascade_right]);
    loop {
        let state = controller.state().unwrap_or_default();
        let y_pos = state.left_stick.y();
        let x_pos = state.right_stick.x();
        let _ = drivetrain_left.set_voltage(y_pos * drivetrain_left.max_voltage());
        let _ = drivetrain_right.set_voltage(y_pos * drivetrain_right.max_voltage());
        if x_pos > 0.01 {
            let _ = drivetrain_left.set_voltage(x_pos * drivetrain_left.max_voltage());
        } else if x_pos < -0.01 {
            let _ = drivetrain_right.set_voltage(x_pos * drivetrain_right.max_voltage());
        }
        if state.button_up.is_pressed() {
            let _ = cascade.set_velocity(600);
        } else if state.button_down.is_pressed() {
            let _ = cascade.set_velocity(-600);
        } else {
            let _ = cascade.brake(vexide::smart::motor::BrakeMode::Hold);
        }
        if state.button_r1.is_pressed() {
            let _ = intake.set_velocity(-600);
        } else if state.button_r2.is_pressed() {
            let _ = intake.set_velocity(600);
        } else {
            let _ = intake.set_velocity(0);
        }
        sleep(Controller::UPDATE_INTERVAL).await;
    }
}
