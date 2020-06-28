use std::f64::consts::PI;
use ::rr::path::{LinearSegment, CircleSegment};

fn main()
{
    let straight = LinearSegment::new(10.0);
    println!("{}", serde_json::to_string(&straight).expect("json"));
    let curve = CircleSegment::new(10.0, PI / 2.0);
    println!("{}", serde_json::to_string(&curve).expect("json"));
}
