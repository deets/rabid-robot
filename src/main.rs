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

enum State
{
    Normal,
    LowVoltage
}

struct MD23Driver {
    outgoing: std::sync::mpsc::Sender<Message>,
}

impl MD23Driver {

    fn start_thread(rx: std::sync::mpsc::Receiver<Message>, addr: u16)
    {
        thread::spawn(move || {
            let mut state = State::Normal;
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
                            println!("speed: {}", speed);
                            dev.smbus_write_byte_data(0, speed);
                            dev.smbus_write_byte_data(1, speed);
                        }
                    }
                }
            }
        });
    }

    pub fn new(addr: u16) -> MD23Driver
    {
        let (tx, rx) = mpsc::channel();
        MD23Driver::start_thread(rx, addr);
        MD23Driver{
            outgoing: tx
        }
    }

    pub fn drive(self: &mut MD23Driver, speed: f32)
    {
        self.outgoing.send(Message::Drive{speed: speed});
    }

    pub fn stop(self: &mut MD23Driver)
    {
        self.outgoing.send(Message::Drive{speed: 0.0});
    }

}
fn main() {
    let mut md23 = MD23Driver::new(MD23_ADDR);
    loop {
        thread::sleep(Duration::from_millis(1000));
        md23.drive(0.5);
        thread::sleep(Duration::from_millis(1000));
        md23.stop();
        thread::sleep(Duration::from_millis(1000));
        md23.drive(-0.5);
        thread::sleep(Duration::from_millis(1000));
        md23.stop();
    }
}
