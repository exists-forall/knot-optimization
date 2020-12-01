extern crate bspline;
use std::f64::consts::PI;
use alga::general::SubsetOf;
use defaults;
use cost::CostParams;
use isometry_adjust as iso_adj;
use joint::JointSpec;
use optimize_tools::{Chain, Leg, PhantomJoint};
use symmetry::adjacent_symmetry;

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3,
    UnitQuaternion, Vector3};
use joint::Point;


pub fn from_spline<F: Fn() -> bspline::BSpline<Point>> (
    arc_len_step: f32,
    spline_gen: F,
    symmetry: u32,
    scale: f32,
) -> (usize, impl Iterator<Item = Isometry3<f64>>) {

    // Create iterator of points on one "leg" of knot that are arc length (in + out) away from
    // each other.
    let dt = 0.01;
    let spline = spline_gen();
    let t_range = spline.knot_domain();
    let mut arc_len_since_last = arc_len_step;
    let mut t = t_range.0 + dt;
    let end = t_range.1 / (symmetry as f32);
    let iso_iterator = (0..)
        .map(move |_| {
            while t + dt <= end as f32 {
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
        let iterator_clone = iso_iterator.clone();
        let joints = iterator_clone.count();
        (joints, iso_iterator)
}

pub fn generic_chain<F: Fn() -> bspline::BSpline<Point>>(
    scale: f32,
    cost_params: CostParams,
    return_to_initial_weight: f64,
    descent_rate: f64,
    spec: JointSpec,
    spline_gen: F,
    knot_sym: u32
) -> Chain {
    let arclen = 1.1*(spec.dist_in() + spec.dist_out());
    let spline_iter = from_spline(
        arclen as f32, // arc length step
        spline_gen, // bspline generator
        knot_sym, // symmetry
        scale,  // scale
    );
    let chain_size = spline_iter.0;

    Chain::new(
        // spec
        spec,
        // num angles
        defaults::NUM_ANGLES,
        // pre-phantom
        PhantomJoint {
            symmetry: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI).to_superset(),
            index: 0,
            leg: Leg::Incoming,
        },
        // post-phantom
        PhantomJoint {
            symmetry: adjacent_symmetry(3, 4).to_superset(),
            index: chain_size - 1,
            leg: Leg::Outgoing,
        },
        // cost params
        cost_params,
        // 'return to initial' weight
        return_to_initial_weight,
        // descent rate
        descent_rate,
        // steps
        iso_adj::Steps::new_uniform(0.000001),
        // joints
        spline_iter.1.collect(),
    )
}
