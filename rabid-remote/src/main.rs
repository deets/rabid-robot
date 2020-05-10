extern crate sdl2;
extern crate serde;

use sdl2::pixels::Color;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::Duration;
use nanomsg::{Socket, Protocol, Error};

use serde::{Serialize, Deserialize};

fn connect_to_robot(addr: &str) -> Result<nanomsg::Socket, Error> {
    let mut socket = Socket::new(Protocol::Pair)?;
    socket.connect(addr)?;
    Ok(socket)
}


#[derive(Serialize, Deserialize, Debug)]
struct AxisMovement {
    axis: u8,
    value: i16,
}


fn open_joystick(sdl_context: &sdl2::Sdl) -> Result<sdl2::joystick::Joystick, String>
{
    let joystick_subsystem = sdl_context.joystick()?;

    let available = joystick_subsystem.num_joysticks()
        .map_err(|e| format!("can't enumerate joysticks: {}", e))?;

    println!("{} joysticks available", available);

    // Iterate over all available joysticks and stop once we manage to open one.
    let mut joystick = (0..available).find_map(|id| match joystick_subsystem.open(id) {
        Ok(c) => {
            println!("Success: opened \"{}\"", c.name());
            Some(c)
        },
        Err(e) => {
            println!("failed: {:?}", e);
            None
        },
    }).expect("Couldn't open any joystick");
    Ok(joystick)
}

fn send_axis_value(socket: &mut Socket, axis: u8, value: i16)
{
    let msg = AxisMovement{axis, value};
    let serialized = serde_json::to_string(&msg).unwrap();
    socket.nb_write(serialized.as_bytes());
}

pub fn main() {
    let uri = "tcp://fpv-laptimer.local:5000";
    let mut socket = connect_to_robot(uri).unwrap();

    let sdl_context = sdl2::init().unwrap();
    let _joystick = open_joystick(&sdl_context).unwrap();

    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem.window("rust-sdl2 demo", 800, 600)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    canvas.set_draw_color(Color::RGB(0, 255, 255));
    canvas.clear();
    canvas.present();
    let mut event_pump = sdl_context.event_pump().unwrap();
    let mut i = 0;
    'running: loop {
        i = (i + 1) % 255;
        canvas.set_draw_color(Color::RGB(i, 64, 255 - i));
        canvas.clear();
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                Event::JoyAxisMotion{ axis_idx, value: val, .. } => {
                    // Axis motion is an absolute value in the range
                    // [-32768, 32767]. Let's simulate a very rough dead
                    // zone to ignore spurious events.
                    send_axis_value(&mut socket, axis_idx, val);
                },
                Event::JoyHatMotion{ hat_idx, state, .. } =>
                    println!("Hat {} moved to {:?}", hat_idx, state),
                Event::JoyButtonDown{ button_idx, .. } => {
                    println!("Button {} down", button_idx);
                },
                _ => {}
            }
        }
        // The rest of the game loop goes here...

        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}
