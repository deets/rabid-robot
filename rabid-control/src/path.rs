extern crate nalgebra as na;
use na::{Vector2, Rotation2};
use std::fmt;

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
    fn new(radius: f64, arc: f64) -> CircleSegment
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
        println!("{:?}", compound_path);
        assert_eq!(total, compound_path.length());
        let (pos, rot) = compound_path.at(1.0);
        assert!(equal_eps(&Vector::new(14.0, 5.0), &pos, 0.0001));
        assert_eq!(Rotation::new(PI / 2.0), rot);
    }

}
