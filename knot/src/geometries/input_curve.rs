extern crate bspline;
use std::f64::consts::PI;

use alga::general::SubsetOf;
use nalgebra::{UnitQuaternion, Vector3};

use optimize_tools::{Chain, Leg, PhantomJoint};
use cost::CostParams;
use defaults;
use isometry_adjust as iso_adj;
use joint::{Point, JointSpec};
use symmetry::adjacent_symmetry;

use geometries::from_spline::from_spline;

pub fn chain<F: Fn() -> bspline::BSpline<Point>>(
    scale: f32,
    cost_params: CostParams,
    return_to_initial_weight: f64,
    descent_rate: f64,
    bspline_generator: F,
    sym_number: u32,
    spec: JointSpec,
) -> Chain {
    let arclen = 1.1*(spec.dist_in() + spec.dist_out());
    let spline_iter = from_spline(
        arclen as f32, // arc length step
        bspline_generator, // bspline geneator
        sym_number, // symmetry
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
