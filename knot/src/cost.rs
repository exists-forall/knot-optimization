use std::f64::consts::PI;
use std::f64::INFINITY;

use nalgebra::{Isometry3, Vector3, UnitQuaternion, Quaternion};
use alga::general::SubsetOf;

/// Configurable parameters for determinng when certain terms in the cost function should be
/// calculated.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Thresholds {
    /// The minimum unweighted distance cost (i.e. the squared euclidean distance) at which the axis
    /// cost will be calculated.  Above this distance cost, the axis cost is set to the maximum
    /// regardless of its actual value. This prevents the optimizer from spuriously paying attention
    /// to axis alignment when the joints aren't anywhere close to connecting.
    pub dist_for_axis: f64,

    /// The minimum unweighted axis cost (i.e. one minus the dot product of their axes) at which the
    /// locking angle cost will be calculated.  Above this axis cost, the locking cost is set to the
    /// maximum regardless of its actual value.  This prevnets the optimizer from spuriously paying
    /// attention to locking angles when the joints aren't anywhere close to aligned.
    pub axis_for_locking: f64,
}

/// Always compute all terms of the cost function.
pub const NO_THRESHOLD: Thresholds = Thresholds {
    dist_for_axis: INFINITY,
    axis_for_locking: INFINITY,
};

/// Configurable parmaters for measuring how well two joints fit together
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CostParams {
    /// The weight assigned to the squared euclidean distance between the ends of the two joints.
    pub dist_weight: f64,

    /// The weight assigned to one minus the dot product of the incoming and outgoing "axes" of the
    /// two joints (a measure of the extent to which they fail to meet flush in a plane).
    pub axis_weight: f64,

    /// The weight assigned to a (nonlinear) measure of the difference between the angle at which
    /// the two joints lock together (a rotation about the axis that runs through them), and the
    /// nearest valid angle (quantized according to num_angles).
    pub locking_weight: f64,

    /// Used to determine when certain terms in the cost function should be calculated.
    pub thresholds: Thresholds,
}

/// Several measures of how well two joints fit together.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Costs {
    /// The squared euclidean distance between the ends of the two joints.
    pub dist: f64,

    /// One minus the dot product of the incoming and outgoing "axes" of the two joints (a measure
    /// of the extent to which they fail to meet flush in a plane).
    pub axis: f64,

    /// A (nonlinear) measure of the difference between the angle at which the two joints lock
    /// together (a rotation about the axis that runs through them), and the nearest valid angle
    /// (quantized according to num_angles).
    pub locking: f64,
}

/// Compute several measures of how well two joints fit together, given two coordinate systems
/// rooted at each of their ends.  This function assumes the two coordinate systems have y-axes that
/// are supposed to face the **same** direction, so it is suitable for measuring the connection
/// between the "out" point of one joint and the "in" point of another.
pub fn costs_aligned(
    thresholds: &Thresholds,
    num_angles: u32,
    trans_0: &Isometry3<f64>,
    trans_1: &Isometry3<f64>,
) -> Costs {
    // Distance cost
    let dist_cost = (trans_0.translation.vector - trans_1.translation.vector).norm_squared();

    // Axis misalignment cost
    let (axis_cost, locking_cost) = if dist_cost < thresholds.dist_for_axis {
        // TODO: Optimize identically-zero terms in rotation calculation.
        let axis_0 = trans_0 * Vector3::y_axis().to_superset();
        let axis_1 = trans_1 * Vector3::y_axis().to_superset();

        let axis_cost = 1.0 - axis_0.dot(&axis_1);

        // Locking angle cost
        let locking_cost = if axis_cost < thresholds.axis_for_locking {
            let align = UnitQuaternion::rotation_between(&axis_1, &axis_0)
                .unwrap_or(UnitQuaternion::identity());
            let aligned_rel_rotation = trans_0.rotation.inverse() * align * trans_1.rotation;

            let locking_angle = aligned_rel_rotation.angle();
            let locking_interval_frac = (locking_angle / (2.0 * PI) * (num_angles as f64)) % 1.0;

            // From the observation that x^2 * (1 - x)^2 has local minima at x = 0 and x = 1 and a
            // local maximum at x = 1/2, at which it attains a maximum value of 1/16, and is
            // continuous and differentiable everywhere even when it is extended periodically mod 1.
            16.0 * locking_interval_frac * locking_interval_frac * (1.0 - locking_interval_frac) *
                (1.0 - locking_interval_frac)
        } else {
            1.0
        };

        (axis_cost, locking_cost)
    } else {
        (2.0, 1.0)
    };

    Costs {
        dist: dist_cost,
        axis: axis_cost,
        locking: locking_cost,
    }
}

/// Measure how well two joints fit together, given two coordinate systems rooted at each of their
/// ends.  This function assumes the two coordinate systems have y-axes that are supposed to face
/// the **same** direction, so it is suitable for measuring the connection between the "out" point
/// of one joint and the "in" point of another.
pub fn cost_aligned(
    cost_params: &CostParams,
    num_angles: u32,
    trans_0: &Isometry3<f64>,
    trans_1: &Isometry3<f64>,
) -> f64 {
    let costs = costs_aligned(&cost_params.thresholds, num_angles, trans_0, trans_1);

    costs.dist * cost_params.dist_weight + costs.axis * cost_params.axis_weight +
        costs.locking * cost_params.locking_weight
}

fn align(trans: &Isometry3<f64>) -> Isometry3<f64> {
    trans * UnitQuaternion::new_unchecked(Quaternion::new(0.0, 1.0, 0.0, 0.0))
}

/// Compute several measures of how well two joints fit together, given two coordinate systems
/// rooted at each of their ends.  This function assumes the two coordinate systems have y-axes that
/// are supposed to face in **opposite** directions, so it is suitable for measuring the connection
/// between the "out" points of two joints.
pub fn costs_opposing(
    thresholds: &Thresholds,
    num_angles: u32,
    trans_0: &Isometry3<f64>,
    trans_1: &Isometry3<f64>,
) -> Costs {
    costs_aligned(thresholds, num_angles, trans_0, &align(trans_1))
}

/// Measure how well two joints fit together, given two coordinate systems rooted at each of their
/// ends.  This function assumes the two coordinate systems have y-axes that are supposed to face in
/// **opposite** directions, so it is suitable for measuring the connection between the "out" points
/// of two joints.
pub fn cost_opposing(
    cost_params: &CostParams,
    num_angles: u32,
    trans_0: &Isometry3<f64>,
    trans_1: &Isometry3<f64>,
) -> f64 {
    cost_aligned(cost_params, num_angles, trans_0, &align(trans_1))
}

#[cfg(test)]
mod test {
    use cost::*;
    use nalgebra::Translation3;

    fn assert_symmetry(
        thresholds: &Thresholds,
        num_angles: u32,
        trans_0: &Isometry3<f64>,
        trans_1: &Isometry3<f64>,
    ) {
        let costs_0 = costs_aligned(thresholds, num_angles, trans_0, trans_1);
        let costs_1 = costs_aligned(thresholds, num_angles, trans_1, trans_0);

        assert_relative_eq!(costs_0.dist, costs_1.dist);
        assert_relative_eq!(costs_0.axis, costs_0.axis);
        assert_relative_eq!(costs_0.locking, costs_1.locking);
    }

    #[test]
    fn perfect() {
        let costs = costs_aligned(
            &NO_THRESHOLD,
            1,
            &Isometry3::identity(),
            &Isometry3::identity(),
        );
        assert_relative_eq!(costs.dist, 0.0);
        assert_relative_eq!(costs.axis, 0.0);
        assert_relative_eq!(costs.locking, 0.0);
    }

    #[test]
    fn perfect_except_dist() {
        let trans_0 = Isometry3::identity();
        let trans_1 = Translation3::new(2.0, 0.0, 0.0).to_superset();

        let cost = costs_aligned(&NO_THRESHOLD, 1, &trans_0, &trans_1);

        assert_relative_eq!(cost.dist, 4.0); // *squared* distance
        assert_relative_eq!(cost.axis, 0.0);
        assert_relative_eq!(cost.locking, 0.0);

        assert_symmetry(&NO_THRESHOLD, 1, &trans_0, &trans_1);
    }

    #[test]
    fn perfect_except_axis() {
        let trans_0 = Isometry3::identity();
        let trans_1 = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 6.0).to_superset();

        let cost = costs_aligned(&NO_THRESHOLD, 1, &trans_0, &trans_1);

        assert_relative_eq!(cost.dist, 0.0);
        assert_relative_eq!(cost.axis, 1.0 - (3.0f64).sqrt() / 2.0);
        assert_relative_eq!(cost.locking, 0.0);

        assert_symmetry(&NO_THRESHOLD, 1, &trans_0, &trans_1);
    }

    #[test]
    fn perfect_except_locking() {
        let trans_0 = Isometry3::identity();
        let trans_1 = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0).to_superset();

        let cost = costs_aligned(&NO_THRESHOLD, 4, &trans_0, &trans_1);

        assert_relative_eq!(cost.dist, 0.0);
        assert_relative_eq!(cost.axis, 0.0);
        assert_relative_eq!(cost.locking, 1.0);

        assert_symmetry(&NO_THRESHOLD, 4, &trans_0, &trans_1);
    }

    #[test]
    fn perfect_nontrivial_locking() {
        let trans_0 = Isometry3::identity();
        let trans_1 = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 2.0).to_superset();

        let cost = costs_aligned(&NO_THRESHOLD, 4, &trans_0, &trans_1);

        assert_relative_eq!(cost.dist, 0.0);
        assert_relative_eq!(cost.axis, 0.0);
        assert_relative_eq!(cost.locking, 0.0);

        assert_symmetry(&NO_THRESHOLD, 4, &trans_0, &trans_1);
    }

    #[test]
    fn all_imperfect() {
        let trans_0 = Isometry3::identity();
        let trans_1 = Isometry3::from_parts(
            Translation3::new(2.0, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 6.0) *
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0),
        );

        let cost = costs_aligned(&NO_THRESHOLD, 4, &trans_0, &trans_1);

        assert_relative_eq!(cost.dist, 4.0);
        assert_relative_eq!(cost.axis, 1.0 - (3.0f64).sqrt() / 2.0);
        assert_relative_eq!(cost.locking, 1.0);

        assert_symmetry(&NO_THRESHOLD, 4, &trans_0, &trans_1);
    }

    #[test]
    fn axis_dist_threshold() {
        let thresholds = Thresholds {
            dist_for_axis: 1.0,
            axis_for_locking: INFINITY,
        };

        let trans_0 = Isometry3::identity();
        let trans_1 = Isometry3::from_parts(
            Translation3::new(0.0, 0.5, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 6.0),
        );
        let trans_2 = Isometry3::from_parts(
            Translation3::new(0.0, 1.5, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 6.0),
        );

        let cost_0_1 = costs_aligned(&thresholds, 1, &trans_0, &trans_1);

        assert_relative_eq!(cost_0_1.dist, 0.25);
        assert_relative_eq!(cost_0_1.axis, 1.0 - (3.0f64).sqrt() / 2.0);
        assert_relative_eq!(cost_0_1.locking, 0.0);

        assert_symmetry(&thresholds, 1, &trans_0, &trans_1);

        let cost_0_2 = costs_aligned(&thresholds, 1, &trans_0, &trans_2);

        assert_relative_eq!(cost_0_2.dist, 2.25);
        assert_relative_eq!(cost_0_2.axis, 2.0);
        assert_relative_eq!(cost_0_2.locking, 1.0);

        assert_symmetry(&thresholds, 1, &trans_0, &trans_2);
    }

    #[test]
    fn locking_axis_threshold() {
        let thresholds = Thresholds {
            dist_for_axis: INFINITY,
            axis_for_locking: 1.0 - (2.0f64).sqrt() / 2.0,
        };

        let trans_0 = Isometry3::identity();
        let trans_1 = (UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 6.0)).to_superset();
        let trans_2 = (UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 3.0)).to_superset();

        let cost_0_1 = costs_aligned(&thresholds, 4, &trans_0, &trans_1);

        assert_relative_eq!(cost_0_1.dist, 0.0);
        assert_relative_eq!(cost_0_1.axis, 1.0 - (3.0f64).sqrt() / 2.0);
        assert_relative_eq!(cost_0_1.locking, 0.0);

        assert_symmetry(&thresholds, 1, &trans_0, &trans_1);

        let cost_0_2 = costs_aligned(&thresholds, 4, &trans_0, &trans_2);

        assert_relative_eq!(cost_0_2.dist, 0.0);
        assert_relative_eq!(cost_0_2.axis, 0.5);
        assert_relative_eq!(cost_0_2.locking, 1.0);

        assert_symmetry(&thresholds, 1, &trans_0, &trans_2);
    }
}
