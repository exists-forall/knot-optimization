use nalgebra::{Vector2, Isometry3, Point3};
use joint::JointSpec;

#[derive(Clone, Copy, Debug)]
enum PointsState {
    FirstPoint,
    MainPoints(Option<Isometry3<f64>>),
    Done,
}

#[derive(Clone, Copy, Debug)]
pub struct Points<I> {
    state: PointsState,
    spec: JointSpec,
    joints: I,
}

impl<I: Iterator<Item = Isometry3<f64>>> Iterator for Points<I> {
    type Item = Point3<f64>;

    fn next(&mut self) -> Option<Point3<f64>> {
        match self.state {
            PointsState::FirstPoint => {
                self.state = PointsState::MainPoints(None);
                Some(Point3::origin())
            }
            PointsState::MainPoints(prev_trans) => {
                match self.joints.next() {
                    Some(curr_trans) => {
                        self.state = PointsState::MainPoints(Some(curr_trans));
                        Some(Point3 { coords: curr_trans.translation.vector })
                    }
                    None => {
                        self.state = PointsState::Done;
                        Some(Point3 {
                            coords: (prev_trans.expect("Should have at least one joint") *
                                         self.spec.origin_to_out())
                                .translation
                                .vector,
                        })
                    }
                }
            }
            PointsState::Done => None,
        }
    }
}

pub fn points<I: Iterator<Item = Isometry3<f64>>>(spec: JointSpec, joints: I) -> Points<I> {
    Points {
        state: PointsState::FirstPoint,
        spec,
        joints,
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WindingAngles {
    prev_vec: Option<Vector2<f64>>,
    total: f64,
}

impl WindingAngles {
    pub fn new() -> Self {
        WindingAngles {
            prev_vec: None,
            total: 0.0,
        }
    }

    pub fn next_point(&mut self, point: Point3<f64>) {
        let vec = Vector2::new(point.x, point.y);
        if let Some(prev_vec) = self.prev_vec {
            // These "cosine" and "sine" values are in fact scaled by the product of the lengths of
            // the two vectors, but atan2 doesn't care about scaling.
            let rel_cos = prev_vec.dot(&vec);
            let rel_sin = prev_vec.perp(&vec);
            self.total += rel_sin.atan2(rel_cos);
        }
        self.prev_vec = Some(vec);
    }

    pub fn total(&self) -> f64 {
        self.total
    }
}

#[cfg(test)]
mod test {
    use filter::*;
    use std::f64::consts::PI;

    #[test]
    fn single_joint_points() {
        let spec = JointSpec::new(1.0, 1.0, PI / 4.0, 0.5);
        let joints = [spec.in_to_origin()];
        let points_vec: Vec<_> = points(spec, joints.iter().cloned()).collect();
        assert_relative_eq!(points_vec[0], Point3::origin());
        assert_relative_eq!(points_vec[1], Point3::new(0.0, 1.0, 0.0));
        assert_relative_eq!(
            points_vec[2],
            Point3::new(-(0.5f64).sqrt(), 1.0 + (0.5f64).sqrt(), 0.0)
        );
    }

    #[test]
    fn triplet_winding_angle() {
        let mut winding_angles = WindingAngles::new();

        winding_angles.next_point(Point3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(winding_angles.total(), 0.0);

        winding_angles.next_point(Point3::new(1.0, 0.5, 100.0));
        assert_relative_eq!(winding_angles.total(), (0.5f64).atan());

        winding_angles.next_point(Point3::new(2.0, 2.0, -70.0));
        assert_relative_eq!(winding_angles.total(), PI / 4.0);
    }
}
