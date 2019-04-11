use std::f64::consts::PI;
use std::f64::INFINITY;
use std::iter::once;

use cost::{CostParams, Thresholds};
use joint::JointSpec;
use symmetry_adjust::{OptimizationParams, Vars};

pub const COST_PARAMS: CostParams = CostParams {
    dist_weight: 5.0,
    axis_weight: 1.0,
    locking_weight: 1.0,
    thresholds: Thresholds {
        dist_for_axis: 4.0,
        axis_for_locking: 0.2,
    },
};

pub const OPTIMIZATION_PARAMS: OptimizationParams = OptimizationParams {
    radius_step: 0.01,
    radial_angle_step: 0.01,
    descent_rate: 1.0 / 32.0,
};

pub const INITIAL_SYMMETRY_ADJUST: Vars = Vars {
    radius: 10.0,
    radial_angle: PI / 2.0,
};

/// Iterator of multiple initial conditions from which gradient descent should be performed to
/// increase the chances of finding a global minimum.  Empirical investigation suggests that
/// rotating a half-revolution usually escapes the basin of a local minimum.
pub fn initial_symmetry_adjusts() -> impl Iterator<Item = Vars> {
    once(Vars {
        radius: 10.0,
        radial_angle: PI / 2.0,
    }).chain(once(Vars {
        radius: 10.0,
        radial_angle: -PI / 2.0,
    }))
}

pub const NUM_ANGLES: u32 = 16;

pub const SYMMETRY_COUNT: u32 = 3;

pub fn joint_spec() -> JointSpec {
    JointSpec::new(1.0, 1.0, PI / 6.0, 1.25)
}

pub mod continuous_optimization {
    use super::*;

    /// Cost parameters for gradient descent, different than cost parameters for exhaustive search!
    pub const COST_PARAMS: CostParams = CostParams {
        dist_weight: 1.0,
        axis_weight: 3.0,
        locking_weight: 0.17,
        thresholds: Thresholds {
            dist_for_axis: INFINITY,
            axis_for_locking: INFINITY,
        },
    };

    pub const RATE: f64 = 0.02;

    /// Empirically-determined number of steps for gradient descent to stabilize
    pub const STEPS: u32 = 5_000;

    pub const REPULSION: bool = true;

    pub const REPULSION_EXPONENT: i32 = 2;

    pub const REPULSION_STRENGTH: f64 = 0.005;

    pub const MAX_REPULSION_STRENGTH: f64 = 1.0; // TODO: Empirically determine best value

    pub const CURVE_9_40_CHAIN_SIZE: usize = 8;

    pub const TREFOIL_CHAIN_SIZE: usize = 6;

    pub const RETURN_TO_INITIAL: bool = true;

    pub const RETURN_TO_INITIAL_WEIGHT: f64 = 0.0075;
}
