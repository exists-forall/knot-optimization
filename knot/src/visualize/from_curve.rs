fn from_curve<F: Fn(f64) -> Point3<f64>>(
    count: usize,
    start: f64,
    end: f64,
    f: F,
) -> impl Iterator<Item = Isometry3<f64>> {
    let step = (end - start) / ((count - 1) as f64);
    (0..count).map(move |i| {
        let t = (i as f64) * step + start;
        let dt = 0.01;
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
        frame_iso
    })
}
