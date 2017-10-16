use std::f64::consts::PI;

use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use alga::general::SubsetOf;

/// Return a rotation by pi radians about the x-axis. Faster than, but equivalent to,
/// `UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI)`.
fn quaternion_x_pi() -> UnitQuaternion<f64> {
    UnitQuaternion::new_unchecked(Quaternion::new(0.0, 1.0, 0.0, 0.0))
}

/// Enumerate dihedral-n symmetries in three dimensions.
pub fn symmetries(count: u32) -> impl Iterator<Item = UnitQuaternion<f64>> {
    let step = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 2.0 * PI / (count as f64));
    let mut frame_up = UnitQuaternion::identity();
    let mut is_frame_down = false;
    (0..(2 * count)).map(move |_| if is_frame_down {
        // In principle, maintaining a boolean variable for "flips" rather than repeatedly composing
        // rotations by pi should be faster and more numerically stable, although it is admittedly
        // less readable.
        let curr_frame = frame_up * quaternion_x_pi();
        frame_up = step * frame_up;
        is_frame_down = false;
        curr_frame
    } else {
        is_frame_down = true;
        frame_up
    })
}

/// Give the "first" non-identity dihedral-n symmetry in three dimensions. If you think of each
/// symmetry as a slice of a 2n-slice pizza, this gives the transformation from the first slice
/// to its counterclockwise neighbor. (Perhaps counterintuitively, this corresponds to the *fourth*
/// symmetry yielded by `symmetries`.)
pub fn adjacent_symmetry(count: u32) -> UnitQuaternion<f64> {
    (UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 2.0 * PI / (count as f64)) *
         quaternion_x_pi()).to_superset()
}

#[cfg(test)]
mod test {
    use symmetry::*;

    #[test]
    fn d3() {
        let d3_symmetries = symmetries(3).collect::<Vec<_>>();

        assert_eq!(d3_symmetries.len(), 6);

        // "up" symmetries
        assert_relative_eq!(d3_symmetries[0] * Vector3::x_axis(), Vector3::x_axis());
        assert_relative_eq!(
            d3_symmetries[2] * Vector3::x_axis().to_superset(),
            Vector3::new((2.0 * PI / 3.0).cos(), (2.0 * PI / 3.0).sin(), 0.0)
        );
        assert_relative_eq!(
            d3_symmetries[4] * Vector3::x_axis().to_superset(),
            Vector3::new((4.0 * PI / 3.0).cos(), (4.0 * PI / 3.0).sin(), 0.0)
        );

        assert_relative_eq!(d3_symmetries[0] * Vector3::z_axis(), Vector3::z_axis());
        assert_relative_eq!(d3_symmetries[2] * Vector3::z_axis(), Vector3::z_axis());
        assert_relative_eq!(d3_symmetries[4] * Vector3::z_axis(), Vector3::z_axis());

        // "down" symmetries
        assert_relative_eq!(d3_symmetries[1] * Vector3::x_axis(), Vector3::x_axis());
        assert_relative_eq!(
            d3_symmetries[3] * Vector3::x_axis().to_superset(),
            Vector3::new((2.0 * PI / 3.0).cos(), (2.0 * PI / 3.0).sin(), 0.0)
        );
        assert_relative_eq!(
            d3_symmetries[5] * Vector3::x_axis().to_superset(),
            Vector3::new((4.0 * PI / 3.0).cos(), (4.0 * PI / 3.0).sin(), 0.0)
        );

        assert_relative_eq!(d3_symmetries[1] * Vector3::z_axis(), -Vector3::z_axis());
        assert_relative_eq!(d3_symmetries[3] * Vector3::z_axis(), -Vector3::z_axis());
        assert_relative_eq!(d3_symmetries[5] * Vector3::z_axis(), -Vector3::z_axis());
    }

    #[test]
    fn d3_adjacent() {
        let d3_adjacent = adjacent_symmetry(3);

        assert_relative_eq!(
            d3_adjacent * Vector3::x_axis().to_superset(),
            Vector3::new((2.0 * PI / 3.0).cos(), (2.0 * PI / 3.0).sin(), 0.0)
        );
        assert_relative_eq!(d3_adjacent * Vector3::z_axis(), -Vector3::z_axis());
    }
}
