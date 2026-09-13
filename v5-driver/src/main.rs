use vexide::prelude::*;
use vexide_motorgroup::*;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let controller = peripherals.primary_controller;
    let mut left_front = Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward);
    let mut left_back = Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward);
    let mut right_front = Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward);
    let mut right_back = Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward);
    let mut drivetrain_left = MotorGroup::new(vec![left_front, left_back]);
    let mut drivetrain_right = MotorGroup::new(vec![right_front, right_back]);
    loop {
        let state = controller.state().unwrap_or_default();
        let y_pos = state.left_stick.y();
        let x_pos = state.left_stick.x();
        drivetrain_left.set_voltage(y_pos * drivetrain_left.max_voltage());
        drivetrain_right.set_voltage(y_pos * drivetrain_right.max_voltage());
        if x_pos > 1.0 {
            drivetrain_left.set_voltage(x_pos * drivetrain_left.max_voltage());
        } else if x_pos < 1.0 {
            drivetrain_right.set_voltage(x_pos * drivetrain_right.max_voltage());
        }
        sleep(Controller::UPDATE_INTERVAL).await;
    }
}
