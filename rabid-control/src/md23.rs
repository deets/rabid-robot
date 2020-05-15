// A driver for the MD23 robot kit
//
use std::thread;
use std::sync::mpsc;
use std::time::{Duration, Instant};
use byteorder::{ByteOrder, BigEndian};

use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};


const MD23_ADDR: u16 = 0x58;
const MD23_SPEED1: u8 = 0;
const MD23_SPEED2: u8 = 1;
const MD23_MODE: u8 = 15;
const MD23_ENC1: u8 = 2;
const MD23_ENC2: u8 = 6;
const MD23_VOLTAGE: u8 = 10;
const MD23_ENCODER_STEPS_PER_REVOLUTION: f32 = 360.0;

enum Message
{
    Drive{speed: f32, turn: f32},
    Shutdown
}

#[derive(Clone, Copy)]
pub enum State {
    Normal
    {
        when: Instant,
        voltage: f32,
        enc1: u32,
        enc2: u32,
        diff1: i32,
        diff2: i32,
        speed1: f32,    // Given in revolutions/second, sign indicates direction
        speed2: f32,
    },
    LowVoltage,
    Error,
    Shutdown,
}

pub struct MD23Driver {
    outgoing: std::sync::mpsc::Sender<Message>,
    incoming: std::sync::mpsc::Receiver<State>,
}

impl MD23Driver {

    fn read_encoder(dev: &mut LinuxI2CDevice, address: u8) -> Result<u32, LinuxI2CError>
    {
        let mut vec = Vec::new();
        for i in 0..4 {
            vec.push(dev.smbus_read_byte_data(address + i)?);
        }
        Ok(BigEndian::read_u32(&vec))
    }

    fn compute_state(dev: &mut LinuxI2CDevice, battery_cell_count: u8, previous_state: &State) -> Result<State, LinuxI2CError>
    {
        let now = Instant::now();
        let new_enc1 = MD23Driver::read_encoder(dev, MD23_ENC1)?;
        let new_enc2 = MD23Driver::read_encoder(dev, MD23_ENC2)?;
        let mut diff1 = 0;
        let mut diff2 = 0;
        let mut speed1 = 0.0;
        let mut speed2 = 0.0;

        if let State::Normal{enc1, enc2, when, ..} = previous_state {
             let time_delta = now.duration_since(*when).as_secs_f32();
             diff1 = encoder_diff(&new_enc1, enc1);
             diff2 = encoder_diff(&new_enc2, enc2);
             speed1 = diff1 as f32 / (time_delta * MD23_ENCODER_STEPS_PER_REVOLUTION);
             speed2 = diff2 as f32 / (time_delta * MD23_ENCODER_STEPS_PER_REVOLUTION);
        }

        let voltage = dev.smbus_read_byte_data(MD23_VOLTAGE)?;
        let voltage = voltage as f32 / 10.0;
        if voltage < 3.3 * battery_cell_count as f32 {
            return Ok(State::LowVoltage);
        } else {
            return Ok(State::Normal
                      {
                          when: now,
                          voltage: voltage,
                          enc1: new_enc1,
                          enc2: new_enc2,
                          diff1: diff1,
                          diff2: diff2,
                          speed1: speed1,
                          speed2: speed2,
                      }
            )
        }
    }

    fn start_thread(
        rx: std::sync::mpsc::Receiver<Message>,
        tx: std::sync::mpsc::Sender<State>,
        addr: u16,
        battery_cell_count: u8
    )
    {
        thread::spawn(move || {
            let mut dev = LinuxI2CDevice::new("/dev/i2c-1", addr).expect("MD23 I2C error");
            dev.smbus_write_byte_data(MD23_MODE, 2).expect("setting mode failed");
            let mut state = State::Normal{
                when: Instant::now(),
                voltage: -1.0,
                enc1: 0,
                enc2: 0,
                diff1: 0,
                diff2: 0,
                speed1: 0.0,
                speed2: 0.0,

            };
            loop {
                state = match MD23Driver::compute_state(&mut dev, battery_cell_count, &state)
                {
                    Ok(state) => state,
                    Err(_) => State::Error
                };

                match state {
                    State::Normal{..} => {
                        for message in rx.try_iter()
                        {
                            match message {
                                Message::Drive{speed, turn} => {
                                    let speed = (speed * 127.0 + 128.0) as u8;
                                    let turn = (turn * 127.0 + 128.0) as u8;
                                    let mut foo = || -> Result<(), LinuxI2CError>
                                    {
                                        dev.smbus_write_byte_data(MD23_SPEED1, speed)?;
                                        dev.smbus_write_byte_data(MD23_SPEED2, turn)?;
                                        Ok(())
                                    };
                                    match foo()
                                    {
                                        Ok(_) => {}
                                        Err(_) => { state = State::Error; }
                                    }
                                },
                                Message::Shutdown => {
                                    state = State::Shutdown;
                                }
                            }
                        }
                    },
                    State::LowVoltage =>
                    {
                    }
                    State::Error => {
                    }
                    State::Shutdown => {
                        break;
                    }
                }
                tx.send(state).expect("thread error");
                thread::sleep(Duration::from_millis(100));
            }
        });
    }

    pub fn new(battery_cell_count: u8) -> MD23Driver
    {
        let addr = MD23_ADDR;
        let (tx, rx) = mpsc::channel();
        let (tx_incoming, rx_incoming) = mpsc::channel();
        MD23Driver::start_thread(rx, tx_incoming, addr, battery_cell_count);
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

    pub fn drive(self: &mut MD23Driver, speed: f32, turn: f32) -> Vec<State>
    {
        self.outgoing.send(Message::Drive{speed, turn}).expect("thread error");
        self.gather_state_messages()
    }

    pub fn stop(self: &mut MD23Driver) -> Vec<State>
    {
        self.outgoing.send(Message::Drive{speed: 0.0, turn: 0.0}).expect("thread error");
        self.gather_state_messages()
    }

    pub fn state(self: &mut MD23Driver) -> Vec<State>
    {
        self.gather_state_messages()
    }

    pub fn shutdown(self: &mut MD23Driver)
    {
        self.stop();
        self.outgoing.send(Message::Shutdown).expect("thread error");
        loop {
            for message in self.incoming.iter() {
                if let State::Shutdown = message {
                    return;
                }
            }
        }
    }

}

fn encoder_diff(a: &u32, b: &u32) -> i32
{
    let a = *a as i64;
    let b = *b as i64;
    (a - b) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encoder_diff_simple() {
        let a = 26858;
        let b = 26980;
        assert_eq!(encoder_diff(&b, &a), 26980 - 26858);
        assert_eq!(encoder_diff(&a, &b), 26858 - 26980);
    }

    #[test]
    fn encoder_diff_with_overrun() {
        let a = 0;
        let b = 4294967295;
        assert_eq!(encoder_diff(&a, &b), 1);
        assert_eq!(encoder_diff(&b, &a), -1);
    }

}
