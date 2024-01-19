extern crate ev3dev_lang_rust;

use ev3dev_lang_rust::Ev3Result;
use ev3dev_lang_rust::motors::{LargeMotor, MotorPort};
use ev3dev_lang_rust::sensors::{ColorSensor, SensorPort};

const TARGET_REFLECTIVITY: i32 = 50;
const KP: f32 = 1.5;
const KI: f32 = 0.01;
const KD: f32 = 0.1;

struct PIDController {
    integral: f32,
    last_error: f32,
}

impl PIDController {
    fn new() -> Self {
        PIDController {
            integral: 0.0,
            last_error: 0.0,
        }
    }

    fn update(&mut self, error: f32) -> f32 {
        self.integral += error;
        let derivative = error - self.last_error;
        self.last_error = error;

        KP * error + KI * self.integral + KD * derivative
    }
}

fn main() -> Ev3Result<()> {
    let left_motor = LargeMotor::get(MotorPort::OutA)?;
    let right_motor = LargeMotor::get(MotorPort::OutD)?;
    let left_color_sensor = ColorSensor::get(SensorPort::In2)?;
    let right_color_sensor = ColorSensor::get(SensorPort::In3)?;

    left_color_sensor.set_mode_col_reflect()?;
    right_color_sensor.set_mode_col_reflect()?;

    let mut pid_left = PIDController::new();
    let mut pid_right = PIDController::new();

    left_motor.set_duty_cycle_sp(50)?;
    right_motor.set_duty_cycle_sp(50)?;

    loop {
        let reflect_left = left_color_sensor.get_color()? as f32;
        let reflect_right = right_color_sensor.get_color()? as f32;
        let error_left = TARGET_REFLECTIVITY as f32 - reflect_left;
        let error_right = TARGET_REFLECTIVITY as f32 - reflect_right;
        let correction_left = pid_left.update(error_left);
        let correction_right = pid_right.update(error_right);
        let left_speed = 50.0 - correction_left;
        let right_speed = 50.0 + correction_right;

        left_motor.set_duty_cycle_sp(left_speed as i32)?;
        right_motor.set_duty_cycle_sp(right_speed as i32)?;
    }
}
