extern crate nalgebra as na;
use na::{Vector2, Rotation2};
use std::f64::consts::PI;

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

#[cfg(test)]
mod tests {
    use super::*;

    fn equal_eps(a: &Vector, b: &Vector, e: f64) -> bool
    {
        let d = (b - a).norm();
        (d <= e)
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

}
