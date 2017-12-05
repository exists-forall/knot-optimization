use std::f64::consts::PI;

use nalgebra::{Isometry3, Vector3, UnitQuaternion, Quaternion};
use alga::general::SubsetOf;

fn align_and_flip(trans: &Isometry3<f64>) -> Isometry3<f64> {
    trans * UnitQuaternion::new_unchecked(Quaternion::new(0.0, 0.0, 0.0, 1.0))
}

pub fn locking_angle_aligned(
    num_angles: u32,
    trans_0: &Isometry3<f64>,
    trans_1: &Isometry3<f64>,
) -> f64 {
    let axis_0 = trans_0 * Vector3::y_axis().to_superset();
    let axis_1 = trans_1 * Vector3::y_axis().to_superset();

    let align = UnitQuaternion::rotation_between(&axis_1, &axis_0)
        .unwrap_or(UnitQuaternion::identity());

    let aligned_rel_rotation = trans_0.rotation.inverse() * align * trans_1.rotation;

    aligned_rel_rotation.angle() / (2.0 * PI) * (num_angles as f64)
}

pub fn locking_angle_opposing(
    num_angles: u32,
    trans_0: &Isometry3<f64>,
    trans_1: &Isometry3<f64>,
) -> f64 {
    locking_angle_aligned(num_angles, trans_0, &align_and_flip(trans_1))
}

#[cfg(test)]
mod test {
    use approx_locking_angle::*;

    #[test]
    fn perfectly_aligned_y() {
        let trans_0 = Isometry3::identity();
        let trans_1 = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 2.0 * PI / 3.0)
            .to_superset();

        assert_relative_eq!(locking_angle_aligned(1, &trans_0, &trans_1), 1.0 / 3.0);
        assert_relative_eq!(locking_angle_aligned(2, &trans_0, &trans_1), 2.0 / 3.0);
    }

    #[test]
    fn perfect_aligned_x() {
        let trans_0 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0).to_superset();
        let trans_1 =
            (UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0) *
                 UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 2.0 * PI / 3.0)).to_superset();

        assert_relative_eq!(locking_angle_aligned(1, &trans_0, &trans_1), 1.0 / 3.0);
    }

    #[test]
    fn trans_0_rotated() {
        let trans_0 = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -2.0 * PI / 3.0)
            .to_superset();
        let trans_1 = Isometry3::identity();

        assert_relative_eq!(locking_angle_aligned(1, &trans_0, &trans_1), 1.0 / 3.0);
    }

    #[test]
    fn misaligned_y() {
        let trans_0 = Isometry3::identity();
        let trans_1 =
            (UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 8.0) *
                 UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 2.0 * PI / 3.0)).to_superset();

        assert_relative_eq!(locking_angle_aligned(1, &trans_0, &trans_1), 1.0 / 3.0);
    }
}
