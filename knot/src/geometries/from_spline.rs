use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion};
use joint::Point;
extern crate bspline;


pub fn from_spline<F: Fn(f64) -> Point3<f64>>(
    arc_len_step: f32,
    spline: &bspline::BSpline<Point>,
    symmetry: u32,
    scale: f32,
) -> (usize, impl Iterator<Item = Isometry3<f64>>) {

    // Create iterator of points on one "leg" of knot that are arc length (in + out) away from
    // each other.
    let dt = 0.01;
    let t_range = spline.knot_domain();
    let mut arc_len_since_last = arc_len_step;
    let mut t = t_range.0;
    let mut end = (t_range.1 / (symmetry as f32)) as usize;
    let iso_iterator = (0..)
        .map(move |_| {
            while t <= end as f32 {
                let f_t_spline = spline.point(t)*scale;
                let f_plus_spline = spline.point(t + dt)*scale;
                let f_minus_spline = spline.point(t - dt)*scale;

                let f_t = f_t_spline.convert();
                let f_plus = f_plus_spline.convert();
                let f_minus = f_minus_spline.convert();

                let vel = (f_plus - f_t) / dt as f64;
                let accel = ((f_minus - f_t) + (f_plus - f_t)) / (dt as f64 * dt as f64);
                let frame_y = vel.normalize();
                let frame_x = -(accel - accel.dot(&vel) / (vel.dot(&vel)) * vel).normalize();
                let frame_z = frame_x.cross(&frame_y);
                let frame = Matrix3::from_columns(&[frame_x, frame_y, frame_z]);
                let frame_rot =
                    UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(frame));
                let frame_trans = Translation3 { vector: f_t.coords };
                let frame_iso = Isometry3::from_parts(frame_trans, frame_rot);
                let d_arc_len = vel.norm() as f32 * dt;

                arc_len_since_last += d_arc_len;
                t += dt;
                if arc_len_since_last > arc_len_step {
                    arc_len_since_last -= arc_len_step;
                    return Some(frame_iso);
                }
            }
            None
        }).take_while(|p| p.is_some())
        .map(|p| p.unwrap());
        (end, iso_iterator)
}
