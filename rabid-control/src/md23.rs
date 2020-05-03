// A driver for the MD23 robot kit
//
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
    LowVoltage,
    Error
}

pub enum State {
    Battery{voltage: f32},
    Error,
    LowVoltage
}

pub struct MD23Driver {
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
                match mode {
                    Mode::Normal => {
                        for message in rx.try_iter()
                        {
                            match message {
                                Message::Drive{speed} => {
                                    let speed = (speed * 127.0 + 128.0) as u8;
                                    let mut foo = || -> Result<(), LinuxI2CError>
                                    {
                                        dev.smbus_write_byte_data(0, speed)?;
                                        dev.smbus_write_byte_data(1, speed)?;
                                        Ok(())
                                    };
                                    match foo()
                                    {
                                        Ok(_) => {}
                                        Err(_) => { mode = Mode::Error; }
                                    }
                                }
                            }
                        }
                        let voltage = match dev.smbus_read_byte_data(10) {
                            Ok(voltage) => voltage as f32,
                            Err(err) => panic!("error reading battery: {}", err)
                        } / 10.0;
                        tx.send(State::Battery{voltage: voltage}).expect("thread error");
                        if voltage < 3.3 {
                            mode = Mode::LowVoltage;
                        }
                    },
                    Mode::LowVoltage =>
                    {
                        tx.send(State::LowVoltage{}).expect("thread error");
                    }
                    Mode::Error => {
                        tx.send(State::Error{}).expect("thread error");
                    }

                }
                thread::sleep(Duration::from_millis(100));
            }
        });
    }

    pub fn new() -> MD23Driver
    {
        let addr = MD23_ADDR;
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
        self.outgoing.send(Message::Drive{speed: speed}).expect("thread error");
        self.gather_state_messages()
    }

    pub fn stop(self: &mut MD23Driver) -> Vec<State>
    {
        self.outgoing.send(Message::Drive{speed: 0.0}).expect("thread error");
        self.gather_state_messages()
    }
}
