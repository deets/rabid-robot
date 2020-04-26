use std::thread;
use std::sync::mpsc;
use std::time::Duration;

use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

const MD23_ADDR: u16 = 0x58;

enum Message
{
    Drive{speed: f32},
}

enum Mode
{
    Normal,
    LowVoltage
}

enum State {
    Battery{voltage: f32}
}

use State::Battery;

struct MD23Driver {
    outgoing: std::sync::mpsc::Sender<Message>,
    incoming: std::sync::mpsc::Receiver<State>,
}

impl MD23Driver {

    fn start_thread(
        rx: std::sync::mpsc::Receiver<Message>,
        tx: std::sync::mpsc::Sender<State>,
        addr: u16
    )
    {
        thread::spawn(move || {
            let mut mode = Mode::Normal;
            let mut dev = match LinuxI2CDevice::new("/dev/i2c-1", addr)
            {
                Ok(dev) => dev,
                Err(err) => panic!("oh no {}", err)
            };

            loop {
                for message in rx.try_iter()
                {
                    match message {
                        Message::Drive{speed} => {
                            let speed = (speed * 127.0 + 128.0) as u8;
                            dev.smbus_write_byte_data(0, speed);
                            dev.smbus_write_byte_data(1, speed);
                        }
                    }
                }
                let voltage = match dev.smbus_read_byte_data(10) {
                    Ok(voltage) => voltage as f32,
                    Err(err) => panic!("error reading battery: {}", err)
                } / 10.0;
                tx.send(Battery{voltage: voltage});
                thread::sleep(Duration::from_millis(100));
            }
        });
    }

    pub fn new(addr: u16) -> MD23Driver
    {
        let (tx, rx) = mpsc::channel();
        let (tx_incoming, rx_incoming) = mpsc::channel();
        MD23Driver::start_thread(rx, tx_incoming, addr);
        MD23Driver{
            outgoing: tx,
            incoming: rx_incoming
        }
    }

    fn gather_state_messages(self: &mut MD23Driver) -> Vec<State>
    {
        let mut result = Vec::new();
        result.extend(self.incoming.try_iter());
        return result;
    }

    pub fn drive(self: &mut MD23Driver, speed: f32) -> Vec<State>
    {
        self.outgoing.send(Message::Drive{speed: speed});
        self.gather_state_messages()
    }

    pub fn stop(self: &mut MD23Driver) -> Vec<State>
    {
        self.outgoing.send(Message::Drive{speed: 0.0});
        self.gather_state_messages()
    }
}

fn output_state(states: &Vec<State>)
{
    for state in states.iter() {
        match state {
            Battery{voltage} => println!("voltage: {}", voltage)
        }
    }
}
fn main() {
    let mut md23 = MD23Driver::new(MD23_ADDR);
    loop {
        thread::sleep(Duration::from_millis(2000));
        let res = md23.drive(0.5);
        output_state(&res);
        thread::sleep(Duration::from_millis(2000));
        let res = md23.stop();
        output_state(&res);
        thread::sleep(Duration::from_millis(2000));
        md23.drive(-0.5);
        thread::sleep(Duration::from_millis(2000));
        md23.stop();
    }
}
