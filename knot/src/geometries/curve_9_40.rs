use std::f64::consts::PI;

use alga::general::SubsetOf;
use nalgebra::{UnitQuaternion, Vector3};

use optimize_tools::{Chain, Leg, PhantomJoint};
use cost::CostParams;
use defaults;
use isometry_adjust as iso_adj;
use symmetry::adjacent_symmetry;

use geometries::from_curve::from_curve_natural_parameterize;
use geometries::spherical::spherical;

const TAU: f64 = 2.0 * PI;

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
        // pre-phatom
        PhantomJoint {
            symmetry: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI).to_superset(),
            index: 0,
            leg: Leg::Incoming,
        },
        // post-phantom
        PhantomJoint {
            symmetry: adjacent_symmetry(3, 1).to_superset(),
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
            0.01, // dt
            0.0,  // start
            1.0,  // end
            |t| {
                let theta = 2.0 * TAU / 3.0 * t;
                let phi = 1.0 * (TAU / 2.0 * t).sin();
                let rho = 7.0 + 2.5 * (TAU / 2.0 * t).cos();
                spherical(theta, phi, scale * rho)
            },
        ).collect(),
    )
}
