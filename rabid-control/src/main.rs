use std::time::Duration;
use crossbeam_channel::{bounded, tick, Receiver, select};

mod md23;
use md23::{MD23Driver, State};

mod path;


fn output_state(states: &Vec<State>)
{
    for state in states.iter() {
        match state {
            State::Normal{voltage, enc1, enc2, when, speed1, speed2, ..} => println!("when: {:?}: voltage: {}, enc1: {}, enc2: {} speed1: {} speed2: {}", when, voltage, enc1, enc2, speed1, speed2),
            State::Error => panic!("Error in I2C communication"),
            State::LowVoltage => panic!("Robot running low on battery"),
            _ => {}
        }
    }
}

fn ctrl_channel() -> Result<Receiver<()>, ctrlc::Error> {
    let (sender, receiver) = bounded(100);
    ctrlc::set_handler(move || {
        let _ = sender.send(());
    })?;

    Ok(receiver)
}

fn main()
{
    let ctrl_c_events = ctrl_channel().expect("SIGINT handler error");
    let mut md23 = MD23Driver::new(3);
    md23.drive(1.0);
    let ticks = tick(Duration::from_millis(100));

    loop {
        select! {
            recv(ticks) -> _ => {
                let res = md23.state();
                output_state(&res);
            }
            recv(ctrl_c_events) -> _ => {
                println!("Got SIGINT - goodbye!");
                md23.shutdown();
                break;
            }
        }
    }
}
