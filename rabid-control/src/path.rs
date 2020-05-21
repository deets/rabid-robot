extern crate nalgebra as na;
use na::{Vector2, Rotation2};
use std::fmt;
use std::time::Duration;
use libm::fmin;

pub type Vector = Vector2<f64>;
pub type Rotation = Rotation2<f64>;

fn signum(n: f64) -> f64
{
    if n > 0.0 {
        1.0
    }
    else
    {
        -1.0
    }
}

pub trait PathSegment {
    fn length(&self) -> f64;
    fn at(&self, position: f64) -> (Vector, Rotation);
}

pub struct LinearSegment
{
    length: f64
}

impl LinearSegment {
    fn new(length: f64) -> LinearSegment
    {
        LinearSegment{length: length}
    }
}

impl PathSegment for LinearSegment
{
    fn length(&self) -> f64
    {
        return self.length;
    }

    fn at(&self, position: f64) -> (Vector, Rotation)
    {
        (Vector::new(position * self.length, 0.0), Rotation::new(0.0))
    }
}


pub struct CircleSegment
{
    radius: f64,
    arc: f64,
}

impl CircleSegment {
    pub fn new(radius: f64, arc: f64) -> CircleSegment
    {
        CircleSegment{radius, arc}
    }
}

impl PathSegment for CircleSegment
{
    fn length(&self) -> f64
    {
        return self.arc.abs() * self.radius;
    }

    fn at(&self, position: f64) -> (Vector, Rotation)
    {
        // to perform the rotation, we take a
        // vector of length radius pointing down or up,
        // rotate it by position * arc, and
        // move the result up/down by a vector pointing
        // up the same amount
        let v = Vector::new(0.0, -self.radius * signum(self.arc));
        let r = Rotation::new(self.arc * position);
        let v = r.transform_vector(&v) - v;
        (v, r)
    }
}

struct CompoundPathSegment
{
    segment: Box<dyn PathSegment>,
    // where this segment starts, concatenated
    // to the previous one
    pos: Vector,
    // the rotation we start off
    rot: Rotation,
    // The end in relative units.
    relative_start: f64,
    // The length in relative units
    relative_length: f64
}

impl fmt::Debug for CompoundPathSegment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CompoundPathSegment")
         .field("pos", &(self.pos[0], self.pos[1]))
         .field("rot", &self.rot.angle())
         .field("relative_start", &self.relative_start)
         .field("relative_length", &self.relative_length)
         .finish()
    }
}
#[derive(Debug)]
pub struct CompoundPath
{
    segments: Vec<CompoundPathSegment>
}

impl CompoundPath {

    fn new() -> CompoundPath
    {
        CompoundPath{segments: Vec::new()}
    }

    fn _length(&self) -> f64
    {
        self.segments.iter().fold(0.0, |acc, x| acc + x.segment.length())
    }


    fn push(&mut self, segment: Box<dyn PathSegment>)
    {
        self.segments.push(CompoundPathSegment{
            segment: segment,
            relative_length: 0.0,
            relative_start: 0.0,
            pos: Vector::new(0.0, 0.0),
            rot: Rotation::new(0.0),
        });
        let mut pos = Vector::new(0.0, 0.0);
        let mut rot = Rotation::new(0.0);
        let mut relative_start = 0.0;
        let total_length = self._length();

        for segment in self.segments.iter_mut()
        {
            segment.relative_length = segment.segment.length() / total_length;
            segment.relative_start = relative_start;
            relative_start += segment.relative_length;

            segment.pos = pos;
            segment.rot = rot;
            let (rpos, rrot) = segment.segment.at(1.0);
            // our new pos for the next segment is
            // placed at the end of the current one,
            // properly rotated
            pos += rot.transform_vector(&rpos);
            rot = Rotation::new(rot.angle() + rrot.angle());
        }
    }
}

impl PathSegment for CompoundPath
{
    fn at(&self, position: f64) -> (Vector, Rotation)
    {
        let index = match self.segments.binary_search_by(
            |segment| segment.relative_start.partial_cmp(&position).expect("Nan"))
        {
            Ok(index) => index,
            Err(index) => index - 1
        };
        let segment = &self.segments[index];
        // adjust the position relative to the
        // start and length
        let position = (position - segment.relative_start) / segment.relative_length;
        let (rpos, rrot) = segment.segment.at(position);

        let pos = segment.rot.transform_vector(&rpos) + segment.pos;
        let rot = Rotation::new(segment.rot.angle() + rrot.angle());
        (pos, rot)
    }

    fn length(&self) -> f64
    {
        self._length()
    }
}

// The main purpose of the Ramp is to map
// a Duration and result in the distance
// covered during this time. This while
// maintaining an acceleration phase,
// possibly a steady phase, and a
// deceleration phase.
//
// Length is given in cm
// Speed is given in cm/s
// Acceleration in cm/s^2
pub struct Ramp
{
    length: f64,
    max_velocity: f64,
    max_acceleration: f64,
}

impl Ramp
{

    fn segment_duration(&self)-> (f64, f64)
    {
        let mut full_speed_time = 0.0;
        // this is the total time spent
        // linearily accelerating until the inflection point
        // and then linarily breaking until stop.
        let mut ramp_time = (self.length / self.max_acceleration).sqrt() * 2.0;
        // if we accelerate for one ramp time, and the resulting
        // speed is higher than allowed, we must compose our
        // ramp from three sections!
        if (ramp_time / 2.0) * self.max_acceleration > self.max_velocity
        {
            // for starters, we assume that
            // we can fully accelerate, meaning
            // we spend
            //   ramp_time = max_velocity / max_acceleration
            // seconds accelerating, and the same
            // amount decelerating. And we cover
            //   ramp_length = ramp_time * max_velocity / 2.0
            // centimeters in that time, because it's two
            // triangles folded to make one square
            // that is half as wide as the two ramps.
            //     .           ____
            //    /|\          |  /|
            //   / | \    ->   | / |
            //  /  |  \        |/  |
            //  -------        -----
            //
            // so the rest must be spend at full speed.
            ramp_time = (self.max_velocity / self.max_acceleration) * 2.0;
            let ramp_length = ramp_time / 2.0 * self.max_velocity;
            full_speed_time = (self.length - ramp_length) / self.max_velocity;
        }
        (ramp_time, full_speed_time)
    }

    fn total_duration(&self) -> Duration
    {
        let (ramp_time, full_speed_time) = self.segment_duration();
        Duration::from_secs_f64(ramp_time + full_speed_time)
    }

    fn position_at_duration(&self, when: Duration) -> f64
    {
        let mut when = when.as_secs_f64();
        let (ramp_time, full_speed_time) = self.segment_duration();
        let duration = ramp_time + full_speed_time;
        let first_ramp_time = ramp_time / 2.0;
        match when {
            when if duration <= when => {
                self.length
            },
            when if when <= first_ramp_time => {
                0.5 * self.max_acceleration * when.powf(2.0)
            }
            _ => {
                // we first have to travel the whole acceleration ramp
                let total_ramp_length = 0.5 * self.max_acceleration * first_ramp_time.powf(2.0);
                let mut position = total_ramp_length;
                when -= first_ramp_time;
                // We drive for on full speed for a while.
                // This can even be 0 if we don't have a ramp but
                // just acc/decc
                let full_speed_time = fmin(full_speed_time, when);
                position += full_speed_time * self.max_velocity;
                // if when was bigger than full_speed_time
                // we are left with decelerating
                when -=  full_speed_time;
                if when > 0.0 {
                    position += total_ramp_length - 0.5 * self.max_acceleration * (first_ramp_time - when).powf(2.0);
                }
                position
            }
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use std::f64::consts::PI;

    fn equal_eps(a: &Vector, b: &Vector, e: f64) -> bool
    {
        let d = (b - a).norm();
        d <= e
    }

    #[test]
    fn linear_segment_length() {
        let segment = LinearSegment::new(10.0);
        assert_eq!(segment.length(), 10.0);
    }

    #[test]
    fn linear_segment_at() {
        let length = 10.0;
        let segment = LinearSegment::new(length);
        let beginning = (Vector::new(0.0, 0.0), Rotation::new(0.0));
        let end = (Vector::new(length, 0.0), Rotation::new(0.0));

        assert_eq!(segment.at(0.0), beginning);
        assert_eq!(segment.at(1.0), end);
    }

    #[test]
    fn circle_segment_length() {
        let radius = 4.0;
        let arc = PI / 2.0;
        let segment = CircleSegment::new(radius, arc); // 90 deg left turn
        assert_eq!(segment.length(), arc * radius);

        let segment = CircleSegment::new(radius, -arc); // 90 deg right(!) turn
        assert_eq!(segment.length(), arc * radius);
    }


    #[test]
    fn circle_segment_at() {
        let radius = 4.0;
        let arc = PI / 2.0;
        let segment = CircleSegment::new(radius, arc); // 90 deg left turn
        let (pos, rot) = segment.at(1.0);
        assert!(equal_eps(&pos, &Vector::new(radius, radius), 0.0001));
        assert_eq!(Rotation::new(arc), rot);

        let segment = CircleSegment::new(radius, -arc); // 90 deg right(!) turn
        let (pos, rot) = segment.at(1.0);
        assert!(equal_eps(&pos, &Vector::new(radius, -radius), 0.0001));
        assert_eq!(Rotation::new(-arc), rot);
    }

   #[test]
    fn compound_path_length() {
        // This is how the path looks:
        //
        //              +   <- (14, 5), (pi/2)
        //              |
        //              +   <- (14, 4), (pi/2)
        //              |
        //              /
        // +--------++-/

        let radius = 4.0;
        let arc = PI / 2.0;
        let straight = LinearSegment::new(10.0);
        let curve = CircleSegment::new(radius, arc);
        let straight2 = LinearSegment::new(1.0);
        let total = straight.length() + curve.length() + straight2.length();
        let mut compound_path = CompoundPath::new();
        compound_path.push(Box::new(straight));
        compound_path.push(Box::new(curve));
        compound_path.push(Box::new(straight2));
        assert_eq!(total, compound_path.length());
        let (pos, rot) = compound_path.at(1.0);
        assert!(equal_eps(&Vector::new(14.0, 5.0), &pos, 0.0001));
        assert_eq!(Rotation::new(PI / 2.0), rot);
    }

    #[test]
    fn ramp_duration()
    {
        // The robot turns 3 rps with 10cm
        // wheel diameter. So a realistic speed
        // is 90cm/s. I take a conservative third there.
        let speed = 30.0;
        // if we want to reach full speed withn 3 seconds,
        // this means that we have 10cm/s^2 acceleration
        let acceleration = 10.0;
        let length = 500.0;
        // this means we spend a total of six seconds
        // accelerading/decelerating, and cover
        // 6 * 30.0 / 2.0 -> 90.0 centimeters in this time.
        // (two triangles make one rectangle!)
        // This leaves us with 500.0 - 90.0 -> 410.0cm spent
        // at 30.0 cm/s, for 13.6666 seconds.
        // So total should 19.6666 seconds
        let expectation = 19.666666666;
        let ramp = Ramp{ length, max_velocity: speed, max_acceleration: acceleration };
        assert_eq!(Duration::from_secs_f64(expectation), ramp.total_duration());
    }

    #[test]
    fn ramp_duration_without_plateau()
    {
        // The robot turns 3 rps with 10cm
        // wheel diameter. So a realistic speed
        // is 90cm/s. I take a conservative third there.
        let speed = 30.0;
        // if we want to reach full speed withn 3 seconds,
        // this means that we have 10cm/s^2 acceleration
        let acceleration = 10.0;
        let length = 90.0;
        // this means we spend a total of six seconds
        // accelerading/decelerating, and cover
        // 6 * 30.0 / 2.0 -> 90.0 centimeters in this time.
        // (two triangles make one rectangle!)
        let expectation = 6.0;
        let ramp = Ramp{ length, max_velocity: speed, max_acceleration: acceleration };
        assert_eq!(Duration::from_secs_f64(expectation), ramp.total_duration());
    }

    #[test]
    fn ramp_duration_with_acceleration_cut_short()
    {
        // The robot turns 3 rps with 10cm
        // wheel diameter. So a realistic speed
        // is 90cm/s. I take a conservative third there.
        let speed = 30.0 as f64;
        // if we want to reach full speed withn 3 seconds,
        // this means that we have 10cm/s^2 acceleration
        let acceleration = 10.0 as f64;
        let length = 50.0;
        // The area under the triangular acceleration
        // is equal to
        //
        // f(t) = 0.5*at^2
        //
        // and we have two of these ramps, meaning that
        // we need to solve
        // 2*f(t * 2) = distance
        //
        // This results in
        //
        // t = sqrt(distance/acceleration) * 2
        //
        // as the duration of the ramp.
        let expectation = (length / acceleration).sqrt() * 2.0;
        let ramp = Ramp{ length, max_velocity: speed, max_acceleration: acceleration };
        assert_eq!(Duration::from_secs_f64(expectation), ramp.total_duration());
    }

    #[test]
    fn ramp_position_at_duration_zero()
    {
        let speed = 30.0;
        let acceleration = 10.0;
        let length = 180.0;
        let ramp = Ramp{ length, max_velocity: speed, max_acceleration: acceleration };

        assert_eq!(0.0, ramp.position_at_duration(Duration::from_secs_f64(0.0)));
    }

    #[test]
    fn ramp_position_at_and_over_full_duration()
    {
        let speed = 30.0;
        let acceleration = 10.0;
        let length = 180.0;
        let ramp = Ramp{ length, max_velocity: speed, max_acceleration: acceleration };
        assert_eq!(length, ramp.position_at_duration(ramp.total_duration()));
        assert_eq!(length, ramp.position_at_duration(ramp.total_duration().mul_f64(2.0)));
    }

    #[test]
    fn ramp_position_at_half_duration()
    {
        let speed = 30.0;
        let acceleration = 10.0;
        let length = 180.0;
        let ramp = Ramp{ length, max_velocity: speed, max_acceleration: acceleration };
        assert_eq!(length / 2.0, ramp.position_at_duration(ramp.total_duration().mul_f64(0.5)));
    }

    #[test]
    fn ramp_position_shortly_before_end()
    {
        let speed = 30.0;
        let acceleration = 10.0;
        let length = 180.0;
        let decl_size = 0.5 * acceleration * (1.0_f64).powf(2.0);
        let ramp = Ramp{ length, max_velocity: speed, max_acceleration: acceleration };
        assert_eq!(length - decl_size, ramp.position_at_duration(ramp.total_duration() - Duration::from_secs_f64(1.0)));
    }
}
