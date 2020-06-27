use crate::path::{PathSegment, Vector};

#[derive(Debug)]
pub struct WheelPositions
{
    left: Vector,
    right: Vector,
}

pub struct TwoWheelRobot
{
    wheelbase: f64,
    wheeldiameter: f64,
}


impl TwoWheelRobot
{
    pub fn wheel_position_at(&self, path: &dyn PathSegment, position: f64) -> WheelPositions
    {
        let left = Vector::new(0.0, -self.wheelbase / 2.0);
        let right = -left;
        let (base, rot) = path.at(position);
        let left = rot.transform_vector(&left);
        let right = rot.transform_vector(&right);
        WheelPositions{left: base + left, right: base + right}
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    use crate::path::{CircleSegment};

    fn equal_eps(a: &Vector, b: &Vector, e: f64) -> bool
    {
        let d = (b - a).norm();
        d <= e
    }

    impl PartialEq for WheelPositions
    {
        fn eq(&self, other: &Self) -> bool {
            equal_eps(&self.left, &other.left, 0.01) &&
                equal_eps(&self.right, &other.right, 0.01)
        }
    }

    #[test]
    fn points_for_path()
    {
        let wheelbase = 23.5;
        let radius = 100.0;
        let robot = TwoWheelRobot{wheelbase: wheelbase, wheeldiameter: 10.0};
        let path = CircleSegment::new(radius, PI * 2.0);
        let left_offset = Vector::new(0.0, -wheelbase / 2.0);
        let right_offset = -left_offset;

        let expected = WheelPositions{left: left_offset, right: right_offset};
        let positions = robot.wheel_position_at(&path, 0.0);
        assert_eq!(expected, positions);

        // Turned one half circle up means our
        // robot base sits at (0, diameter), and
        // the wheels are opposite from that
        let base = Vector::new(0.0, radius * 2.0);
        let expected = WheelPositions{
            left: base + right_offset,
            right: base + left_offset};
        let positions = robot.wheel_position_at(&path, 0.5);
        assert_eq!(expected, positions);
    }
}
