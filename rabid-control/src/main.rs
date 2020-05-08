use std::thread;
use std::time::Duration;

mod md23;
use md23::{MD23Driver, State};

mod path;


fn output_state(states: &Vec<State>)
{
    for state in states.iter() {
        match state {
            State::Normal{voltage} => println!("voltage: {}", voltage),
            State::Error => panic!("Error in I2C communication"),
            State::LowVoltage => panic!("Robot running low on battery")
        }
    }
}

fn main() {
    let mut md23 = MD23Driver::new(3);
    loop {
        let res = md23.drive(0.5);
        output_state(&res);
        thread::sleep(Duration::from_millis(2000));
        let res = md23.stop();
        output_state(&res);
        thread::sleep(Duration::from_millis(2000));
        md23.drive(-0.5);
        thread::sleep(Duration::from_millis(2000));
        md23.stop();
        thread::sleep(Duration::from_millis(2000));
    }
}
