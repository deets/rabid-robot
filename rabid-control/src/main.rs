use std::thread;
use std::time::Duration;
use crossbeam_channel::{bounded, tick, Receiver, select};
use serde::{Serialize, Deserialize};
use nanomsg::{Socket, Protocol, Error};
use std::io::{Read};

mod md23;
use md23::{MD23Driver, State};

mod path;

#[derive(Serialize, Deserialize, Debug)]
struct AxisMovement {
    axis: u8,
    value: i16,
}


fn open_socket(addr: &str) -> Result<Receiver<AxisMovement>, Error> {
    let mut socket = Socket::new(Protocol::Pair)?;
    socket.bind(addr)?;

    let (sender, receiver) = bounded(100);

    thread::spawn(move || {
        loop {
            let mut buffer = Vec::new();
            socket.read_to_end(&mut buffer).expect("Nanomsg Socket Error");
            let json = String::from_utf8(buffer).unwrap();
            let axis_movment: AxisMovement = serde_json::from_str(&json).unwrap();
            sender.send(axis_movment).expect("sending failed");
        }
    });
    Ok(receiver)
}


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
    let ticks = tick(Duration::from_millis(100));
    let axis_receiver = open_socket("tcp://0.0.0.0:5000").expect("Socket error");
    let dead_zone = 10_000;
    let mut speed = 0.0;
    let mut turn = 0.0;
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
            },
            recv(axis_receiver) -> message =>
            {
                let AxisMovement{ axis, value} = message.expect("no axis message");
                if axis == 1 {
                    if value > dead_zone || value < -dead_zone {
                        speed = -(value as f32 / 32768.0);
                    } else {
                        speed = 0.0;
                    }
                }
                if axis == 0 {
                    if value > dead_zone || value < -dead_zone {
                        turn = (value as f32 / 32768.0) / 2.5;
                    } else {
                        turn = 0.0;
                    }
                }
                md23.drive(speed, turn);
                println!("axis {} moves {}", axis, value);
            }
        }
    }
}
