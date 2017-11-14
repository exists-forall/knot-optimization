use std::f64::consts::PI;

use nalgebra::{Isometry3, Translation3, Vector2, Vector3, UnitQuaternion};

use symmetry;
use cost::{CostParams, cost_opposing};

/// Analog parameters for how a chain of joints can be positioned in space with symmetry.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Vars {
    /// The distance of the origin of the chain from the origin of symmetry.
    pub radius: f64,

    /// The rotation of the chain about an axis from the origin of symmetry to origin of the chain,
    /// i.e. a rotation about the radial vector.
    pub radial_angle: f64,
}

/// A [covector](https://en.wikipedia.org/wiki/Linear_form) in the space of `Vars`. This is a
/// principled way to represent a local linear approximation to a function in a way that does not
/// depend on [metric](https://en.wikipedia.org/wiki/Metric_tensor), and so provides a useful
/// alternative to the traditional concept of gradient vectors.
#[derive(Clone, Copy, Debug)]
struct VarsDifferential {
    /// The coefficient on the `d_radius` basis covector
    d_radius: f64,

    /// The coefficient on the `d_radial_angle` basis covector
    d_radial_angle: f64,
}

impl Vars {
    /// Return the transformation from a chain's local coordinate system to the space in which it is
    /// being positioned, according to the current parameters.
    pub fn transform(&self) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(self.radius, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), self.radial_angle),
        )
    }

    /// Numerically approximate the differential of a given function at the current parameters,
    /// using the naive finite difference method.
    fn differential<F: Fn(&Vars) -> f64>(
        &self,
        radius_step: f64,
        radial_angle_step: f64,
        f: &F,
    ) -> VarsDifferential {
        let f_curr = f(self);

        // TODO: Avoid recomputing radial angle rotation
        let f_radius_step = f(&Vars {
            radius: self.radius + radius_step,
            radial_angle: self.radial_angle,
        });
        let d_f_d_radius = (f_radius_step - f_curr) / radius_step;

        let f_radial_angle_step = f(&Vars {
            radius: self.radius,
            radial_angle: self.radial_angle + radial_angle_step,
        });
        let d_f_d_radial_angle = (f_radial_angle_step - f_curr) / radial_angle_step;

        VarsDifferential {
            d_radius: d_f_d_radius,
            d_radial_angle: d_f_d_radial_angle,
        }
    }
}

impl VarsDifferential {
    fn scale(&self, factor: f64) -> VarsDifferential {
        VarsDifferential {
            d_radius: self.d_radius * factor,
            d_radial_angle: self.d_radial_angle * factor,
        }
    }
}

/// A joint-chain-positioning optimization problem to be solved.
#[derive(Clone, Copy, Debug)]
pub struct Problem {
    cost_params: CostParams,
    last_joint_out: Isometry3<f64>,
    num_angles: u32,
    adjacent_symmetry: UnitQuaternion<f64>,
    radial_angle_normalizer: f64,
    symmetry_count: u32,
    skip: u32,
}

impl Problem {
    /// Construct a new problem.
    ///
    /// * `cost_params`: Parameters configuring how to measure the quality of joint positioning and
    ///   alignment.
    ///
    /// * `last_joint_out`: A transformation from the local coordinate system rooted at the *end* of
    ///   the last joint in the chain to the coordinate system of the joint chain as a whole.  In
    ///   other words, defines the position and orientation of the end of the chain in the chain's
    ///   coordinate system.
    ///
    /// * `num_angles`: The number of discrete locking angles at which this chain is supposed to
    ///   connect with the next, used to compute locking costs.
    ///
    /// * `symmetry_count`: The dihedral symmetry number of the knot.  Determines how a given joint
    ///   chain is positioned relative to its neighbor to which it is supposed to connect.
    pub fn new(
        cost_params: CostParams,
        last_joint_out: Isometry3<f64>,
        num_angles: u32,
        symmetry_count: u32,
        skip: u32,
    ) -> Problem {
        let adjacent_symmetry = symmetry::adjacent_symmetry(symmetry_count, skip);
        let radial_angle_normalizer = Vector2::new(
            last_joint_out.translation.vector.y,
            last_joint_out.translation.vector.z,
        ).norm_squared()
            .recip();

        Problem {
            cost_params: cost_params,
            last_joint_out,
            num_angles,
            adjacent_symmetry,
            radial_angle_normalizer,
            symmetry_count,
            skip,
        }
    }

    /// Compute the cost (a measure of how badly the joint chain links up with the nearest copy of
    /// itself) for the given placement parameters.
    pub fn cost(&self, vars: &Vars) -> f64 {
        let adjust = vars.transform();
        let adjusted_last_joint_out = adjust * self.last_joint_out;
        let opposing_last_joint_out = self.adjacent_symmetry * adjusted_last_joint_out;
        cost_opposing(
            &self.cost_params,
            self.num_angles,
            &adjusted_last_joint_out,
            &opposing_last_joint_out,
        )
    }

    /// Convert the given covector to a vector according to a sensible metric (in which the sizes of
    /// infinitesimal radius and radial_angle steps are both measured according to the infinitesimal
    /// units in space by which they move the end of the chain), and add that vector to the given
    /// point in parameter space.
    fn apply_step(&self, vars: &mut Vars, diff: &VarsDifferential) {
        vars.radius += diff.d_radius;
        vars.radial_angle += diff.d_radial_angle * self.radial_angle_normalizer;
    }

    /// Perform a single gradient-descent optimization step.
    /// Returns the squared magnitude of the gradient at the given vars, which external algorithms
    /// can use to measure convergence.
    pub fn optimize(&self, opt_params: &OptimizationParams, vars: &mut Vars) -> f64 {
        let differential = vars.differential(
            opt_params.radius_step,
            opt_params.radial_angle_step,
            &|vars| self.cost(vars),
        );
        self.apply_step(vars, &differential.scale(-opt_params.descent_rate));
        differential.d_radius * differential.d_radius +
            differential.d_radial_angle * differential.d_radial_angle * self.radial_angle_normalizer
    }

    pub fn solve_direct(&self) -> (Vars, f64) {
        let (x, y, z) = (
            self.last_joint_out.translation.vector.x,
            self.last_joint_out.translation.vector.y,
            self.last_joint_out.translation.vector.z,
        );

        let radial_angle_0 = -z.atan2(y);
        let radial_angle_1 = radial_angle_0 + PI;

        // TODO: Document the reasoning behind these formulae and determine whether or not they ever
        // miss a solution.
        let radius_0 = -x +
            y.hypot(z) / ((self.skip as f64) * PI / (self.symmetry_count as f64)).tan();
        let radius_1 = -x +
            y.hypot(z) / (-(self.skip as f64) * PI / (self.symmetry_count as f64)).tan();

        let vars_0 = Vars {
            radial_angle: radial_angle_0,
            radius: radius_0,
        };
        let vars_1 = Vars {
            radial_angle: radial_angle_1,
            radius: radius_1,
        };

        let cost_0 = self.cost(&vars_0);
        let cost_1 = self.cost(&vars_1);
        if cost_0 <= cost_1 {
            (vars_0, cost_0)
        } else {
            (vars_1, cost_1)
        }
    }
}

/// Configurable parameters for tuning the gradient descent process.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct OptimizationParams {
    /// The finite step size used to approximate partial derivatives with respect to `radius`.
    pub radius_step: f64,

    /// The finite step size used to approximate partial derivatives with respect to `radial_angle`.
    pub radial_angle_step: f64,

    /// A factor controlling the rate at which the optimizer takes finite steps in the (opposite)
    /// direction of the gradient.
    pub descent_rate: f64,
}
