extern crate nalgebra as na;
use na::{Vector2, Rotation2};

pub type Vector = Vector2<f64>;
pub type Rotation = Rotation2<f64>;

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

#[cfg(test)]
mod tests {
    use super::*;

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
}
