use std::f64::consts::PI;

use alga::general::SubsetOf;
use nalgebra::{UnitQuaternion, Vector3};
use nalgebra::Point3;

use optimize_tools::{Chain, Leg, PhantomJoint};
use cost::CostParams;
use defaults;
use isometry_adjust as iso_adj;
use symmetry::adjacent_symmetry;

use geometries::from_curve::from_curve_natural_parameterize;

const TAU: f64 = 2.0 * PI;
const HEIGHT: f64 = 1.0;

pub fn chain(
    chain_size: usize,
    scale: f64,
    cost_params: CostParams,
    return_to_initial_weight: f64,
    descent_rate: f64,
) -> Chain {
    Chain::new(
        // spec
        defaults::joint_spec(),
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
            symmetry: adjacent_symmetry(3, 2).to_superset(),
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
        from_curve_natural_parameterize(
            2.7,  // arc length step
            0.005, // dt
            0.0,  // start
            TAU / 6.0,  // end
            |t| {
                Point3::new(
                    scale * (t.sin() + 2.0 * (2.0 * t).sin()),
                    scale * (t.cos() - 2.0 * (2.0 * t).cos()),
                    -scale * HEIGHT * (3.0 * t).sin(),
                )
            },
        ).collect(),
    )
}
