use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion};
extern crate bspline;

pub fn from_spline(
    spline: &bspline::BSpline<Point>,
    joints: usize,
    symmetry: usize,

) -> impl Iterator<Item = Isometry3<f64>> {
    let t_range = spline.knot_domain();
    let step_size = (t_range.1 - t_range.0) / (joints as f32);
    let mut arc_len_since_last = arc_len_step;
    let mut t = start;
    (0..)
        .map(move |_| {
            while t <= end {
                let f_t = f(t);
                let f_plus = f(t + dt);
                let f_minus = f(t - dt);
                let vel = (f_plus - f_t) / dt;
                let accel = ((f_minus - f_t) + (f_plus - f_t)) / (dt * dt);
                let frame_y = vel.normalize();
                let frame_x = -(accel - accel.dot(&vel) / (vel.dot(&vel)) * vel).normalize();
                let frame_z = frame_x.cross(&frame_y);
                let frame = Matrix3::from_columns(&[frame_x, frame_y, frame_z]);
                let frame_rot =
                    UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(frame));
                let frame_trans = Translation3 { vector: f_t.coords };
                let frame_iso = Isometry3::from_parts(frame_trans, frame_rot);
                let d_arc_len = vel.norm() * dt;

                arc_len_since_last += d_arc_len;
                t += dt;
                if arc_len_since_last > arc_len_step {
                    arc_len_since_last -= arc_len_step;
                    return Some(frame_iso);
                }
            }
            None
        }).take_while(|p| p.is_some())
        .map(|p| p.unwrap())
}
