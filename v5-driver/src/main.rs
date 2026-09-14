use vexide::{controller::ControllerState, prelude::*};
use vexide_motorgroup::*;

const CONTROL_THRESHOLD: f64 = 0.01;
const CURVE_WEIGHT: f64 = 0.7;

fn drive(
    controller: &ControllerState,
    mut drive_left: &mut MotorGroup,
    mut drive_right: &mut MotorGroup,
) {
    let y_pos = controller.left_stick.y();
    let x_pos = controller.right_stick.x();
    let input_y = if y_pos.abs() <= CONTROL_THRESHOLD {
        0.0
    } else {
        y_pos.signum() * ((y_pos.abs() - CONTROL_THRESHOLD) / (1.0 - CONTROL_THRESHOLD))
    };
    let input_x = if x_pos.abs() <= CONTROL_THRESHOLD {
        0.0
    } else {
        x_pos.signum() * ((x_pos.abs() - CONTROL_THRESHOLD) / (1.0 - CONTROL_THRESHOLD))
    };
    let response_y = CURVE_WEIGHT * input_y.powi(3) + (1.0 - CURVE_WEIGHT) * input_y;
    let response_x = CURVE_WEIGHT * input_x.powi(3) + (1.0 - CURVE_WEIGHT) * input_x;
    let quickturn = controller.button_right.is_pressed() || response_y == 0.0;
    let turn = if quickturn {
        response_x
    } else {
        response_y.abs() * response_x
    };
    let mut left = response_y + turn;
    let mut right = response_y - turn;
    let magnitude = left.abs().max(right.abs());
    if magnitude > 1.0 {
        left /= magnitude;
        right /= magnitude;
    }
    let _ = drive_left.set_voltage(left * drive_left.max_voltage());
    let _ = drive_right.set_voltage(right * drive_right.max_voltage());
}

fn cascade_control(controller: &ControllerState, mut cascade: &mut MotorGroup) {
    if controller.button_up.is_pressed() {
        let _ = cascade.set_velocity(600);
    } else if controller.button_down.is_pressed() {
        let _ = cascade.set_velocity(-600);
    } else {
        let _ = cascade.brake(vexide::smart::motor::BrakeMode::Hold);
    }
}

fn intake_control(controller: &ControllerState, mut intake: &mut Motor) {
    if controller.button_r1.is_pressed() {
        let _ = intake.set_velocity(-600);
    } else if controller.button_r2.is_pressed() {
        let _ = intake.set_velocity(600);
    } else {
        let _ = intake.set_velocity(0);
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let controller = peripherals.primary_controller;
    let mut intake = Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward);
    let mut cascade_left = Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward);
    let mut cascade_right = Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse);
    let mut left_front = Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward);
    let mut left_back = Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse);
    let mut right_front = Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse);
    let mut right_back = Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward);
    let mut drivetrain_left = MotorGroup::new(vec![left_front, left_back]);
    let mut drivetrain_right = MotorGroup::new(vec![right_front, right_back]);
    let mut cascade = MotorGroup::new(vec![cascade_left, cascade_right]);
    loop {
        let state = &controller.state().unwrap_or_default();
        drive(state, &mut drivetrain_left, &mut drivetrain_right);
        cascade_control(state, &mut cascade);
        intake_control(state, &mut intake);
        sleep(Controller::UPDATE_INTERVAL).await;
    }
}
