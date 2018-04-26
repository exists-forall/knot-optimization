use std::f64::consts::PI;

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use alga::general::SubsetOf;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use report::JointsParity;

/// A specification of the geometry of a joint. A joint is a geometric figure in 3D space consisting
/// of three points: an "in" point, a "midpoint," and an "out" point. A joint has a standard
/// coordinate system, with the origin at its midpoint, the y-axis aligned with the line from the
/// "in" point to the midpoint, and the line from the midpoint to the "out" point aligned with the
/// xy plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointSpec {
    // Basic parameters
    dist_in: f64,
    dist_out: f64,
    bend_angle: f64,
    radius: f64,

    // Computed values
    in_to_origin: Isometry3<f64>,
    out_to_origin: Isometry3<f64>,
    origin_to_in: Isometry3<f64>,
    origin_to_out: Isometry3<f64>,
    origin_to_symmetric: Isometry3<f64>,
}

impl JointSpec {
    pub fn new(dist_in: f64, dist_out: f64, bend_angle: f64, radius: f64) -> JointSpec {
        let in_to_origin = Translation3::new(0.0, dist_in, 0.0).to_superset();
        let out_to_origin = Translation3::new(0.0, -dist_out, 0.0)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -bend_angle);

        let origin_to_in = Translation3::new(0.0, -dist_in, 0.0).to_superset();
        let origin_to_out = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), bend_angle)
            * Translation3::new(0.0, dist_out, 0.0);

        let origin_to_symmetric =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -bend_angle / 2.0).to_superset();

        JointSpec {
            dist_in,
            dist_out,
            bend_angle,
            radius,
            in_to_origin,
            out_to_origin,
            origin_to_in,
            origin_to_out,
            origin_to_symmetric,
        }
    }

    // All fields exposed through accessors because changing them directly could bring computed
    // parameters out of sync.

    /// The distance from the "in" point of a joint to its midpoint.
    pub fn dist_in(&self) -> f64 {
        self.dist_in
    }

    /// The distance from the midpoint of a joint to its "out" point.
    pub fn dist_out(&self) -> f64 {
        self.dist_out
    }

    /// The angle, in radians, between the "in", "mid", and "out" points of the joint.
    pub fn bend_angle(&self) -> f64 {
        self.bend_angle
    }

    /// The radius of the cylinders of the joint.  A measure of the joint's "thickness".
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// A transformation that maps the "in" point of a joint in its local coordinate system to the
    /// origin of the output coordinate system. The inverse of origin_to_in.
    pub fn in_to_origin(&self) -> Isometry3<f64> {
        self.in_to_origin
    }

    /// A transformation that maps the "out" point of a joint in its local coordinate system to the
    /// origin of the input coordinate system, with the "out" direction of the joint mapping to the
    /// y axis. The inverse of origin_to_out.
    pub fn out_to_origin(&self) -> Isometry3<f64> {
        self.out_to_origin
    }

    /// A transformation that maps the origin point of its input coordinate system to the "in" point
    /// of the joint's local coordinate system, with the y axis mapped to a vector parallel to the
    /// "in" direction of the joint and pointing into it. The inverse of in_to_origin.
    pub fn origin_to_in(&self) -> Isometry3<f64> {
        self.origin_to_in
    }

    /// A transformation that maps the origin point of its input coordinate system to the "out"
    /// point of the joint's local coordinate system, with the y axis mapped to a vector parallel to
    /// the "out" direction of the joint and pointing out of it. The inverse of out_to_origin
    pub fn origin_to_out(&self) -> Isometry3<f64> {
        self.origin_to_out
    }

    /// A transformation that rotates from the joint's default local coordinate system to a
    /// coordinate system where its incoming and outgoing directions both make the same angle with
    /// the y-axis, instead of the incoming direction being aligned with the y axis.
    pub fn origin_to_symmetric(&self) -> Isometry3<f64> {
        self.origin_to_symmetric
    }
}

/// A specification of how to place a joint
#[derive(Clone, Copy, Debug)]
pub struct RelativeJoint {
    /// A specification of the geometry of the joint being placed
    pub spec: JointSpec,

    /// The rotation of the joint being placed, in radians around its "in" axis.
    pub angle: f64,
}

/// Given a sequence of joints, return an iterator giving the positions and orientations that each
/// joint would have in absolute space if all the joints were connected end-to-end in one long
/// chain. Each joint's position and orientation is given in terms of a transformation from its
/// local coordinate system to the output coordinate system.
pub fn at_angles<I: Iterator<Item = RelativeJoint>>(
    joints: I,
    mut trans: Isometry3<f64>,
) -> impl Iterator<Item = Isometry3<f64>> {
    joints.map(move |joint| {
        // TODO: Accept this quaternion from the iterator in some type-safe way, so that discrete
        // angles may be stored in a lookup table.
        trans = trans * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), joint.angle);
        trans = trans * joint.spec.in_to_origin();
        let joint_trans = trans;
        trans = trans * joint.spec.origin_to_out();
        joint_trans
    })
}

/// Produce a sequence of joint placements where each joint's rotation is an integer multiple of
/// some fraction of a revolution (with the denominator given by `num_angles`).  Return an iterator
/// suitable for use with `at_angles`.
pub fn discrete_angles<I: Iterator<Item = i32>>(
    spec: JointSpec,
    num_angles: u32,
    angles: I,
) -> impl Iterator<Item = RelativeJoint> {
    let angle_step = (2.0 * PI) / (num_angles as f64);

    angles.map(move |angle| RelativeJoint {
        spec: spec,
        angle: angle_step * (angle as f64),
    })
}

/// Produce a sequence of joint placements where each joint's rotation is an integer multiple of
/// some fraction of a revolution (with the denominator given by `num_angles`), assuming that the
/// first joint will be attached to a copy of itself with the same rotation applied.  Return an
/// iterator suitable for use with `at_angles`.
pub fn discrete_symmetric_angles<I: Iterator<Item = i32>>(
    spec: JointSpec,
    num_angles: u32,
    parity: JointsParity,
    angles: I,
) -> impl Iterator<Item = RelativeJoint> {
    let angle_step = (2.0 * PI) / (num_angles as f64);

    let mut self_attached = match parity {
        JointsParity::Even => true,
        JointsParity::Odd => false,
    };

    angles.map(move |angle| {
        if self_attached {
            self_attached = false;
            RelativeJoint {
                spec: spec,
                angle: angle_step * (angle as f64) * 0.5,
            }
        } else {
            RelativeJoint {
                spec: spec,
                angle: angle_step * (angle as f64),
            }
        }
    })
}

// Serialization and deserialization

/// A struct with just the data needed to fully define a `JointSpec`, without also containing a
/// cache of the the transformations that can be computed from those parameters.  Used for
/// serialization and deserialization purposes only.
#[derive(Serialize, Deserialize)]
struct MinimalJointSpec {
    dist_in: f64,
    dist_out: f64,
    bend_angle: f64,
    radius: f64,
}

impl Serialize for JointSpec {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let params = MinimalJointSpec {
            dist_in: self.dist_in,
            dist_out: self.dist_out,
            bend_angle: self.bend_angle,
            radius: self.radius,
        };

        params.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for JointSpec {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        MinimalJointSpec::deserialize(deserializer).map(|params| {
            JointSpec::new(
                params.dist_in,
                params.dist_out,
                params.bend_angle,
                params.radius,
            )
        })
    }
}

#[cfg(test)]
mod test {
    use joint::*;

    fn assert_inverse_invariants(spec: &JointSpec) {
        assert_relative_eq!(
            spec.in_to_origin() * spec.origin_to_in(),
            Isometry3::identity()
        );
        assert_relative_eq!(
            spec.out_to_origin() * spec.origin_to_out(),
            Isometry3::identity()
        );
    }

    #[test]
    fn straight_joint() {
        let spec = JointSpec::new(1.0, 2.0, 0.0, 0.5);

        assert_relative_eq!(spec.dist_in(), 1.0);
        assert_relative_eq!(spec.dist_out(), 2.0);
        assert_relative_eq!(spec.bend_angle(), 0.0);

        assert_relative_eq!(
            spec.in_to_origin(),
            Translation3::new(0.0, 1.0, 0.0).to_superset()
        );
        assert_relative_eq!(
            spec.out_to_origin(),
            Translation3::new(0.0, -2.0, 0.0).to_superset()
        );

        assert_inverse_invariants(&spec);
    }

    #[test]
    fn bent_joint() {
        let angle = PI / 6.0;
        let spec = JointSpec::new(1.0, 2.0, angle, 0.5);

        assert_relative_eq!(
            spec.in_to_origin(),
            Translation3::new(0.0, 1.0, 0.0).to_superset()
        );
        assert_relative_eq!(
            spec.origin_to_out().translation,
            Translation3::new(-angle.sin() * 2.0, angle.cos() * 2.0, 0.0)
        );
        assert_relative_eq!(
            spec.origin_to_out() * Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(angle.cos(), angle.sin(), 0.0)
        );
        assert_relative_eq!(
            spec.origin_to_out() * Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0)
        );

        assert_inverse_invariants(&spec);
    }

    #[test]
    fn connect_3() {
        let spec1 = JointSpec::new(2.0, 1.0, PI / 4.0, 0.5);
        let spec2 = JointSpec::new(1.0, 1.0, PI / 4.0, 0.5);
        let spec3 = JointSpec::new(1.0, 1.0, PI / 4.0, 0.5);

        let relative_joints = [
            RelativeJoint {
                spec: spec1,
                angle: 0.0,
            },
            RelativeJoint {
                spec: spec2,
                angle: 0.0,
            },
            RelativeJoint {
                spec: spec3,
                angle: PI,
            },
        ];

        let transforms =
            at_angles(relative_joints.iter().cloned(), Isometry3::identity()).collect::<Vec<_>>();

        assert_eq!(transforms.len(), 3);

        assert_relative_eq!(
            transforms[0],
            Translation3::new(0.0, 2.0, 0.0).to_superset()
        );

        assert_relative_eq!(
            transforms[1].translation,
            Translation3::new(-(2.0f64).sqrt(), (2.0f64).sqrt() + 2.0, 0.0)
        );
        assert_relative_eq!(
            transforms[1] * Vector3::new(1.0, 0.0, 0.0),
            Vector3::new((0.5f64).sqrt(), (0.5f64).sqrt(), 0.0)
        );
        assert_relative_eq!(
            transforms[1] * Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 1.0)
        );

        assert_relative_eq!(
            transforms[2].translation,
            Translation3::new(-(2.0f64).sqrt() - 2.0, (2.0f64).sqrt() + 2.0, 0.0)
        );
        assert_relative_eq!(
            transforms[2] * Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0)
        );
        assert_relative_eq!(
            transforms[2] * Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, -1.0)
        );
    }

    #[test]
    fn discrete() {
        let spec = JointSpec::new(1.0, 1.0, PI / 5.0, 0.5);
        let angles = [0, 1, 2, -1];
        let rel_joints = discrete_angles(spec, 16, angles.iter().cloned()).collect::<Vec<_>>();

        assert_eq!(rel_joints.len(), 4);

        assert_eq!(rel_joints[0].spec, spec);
        assert_eq!(rel_joints[1].spec, spec);
        assert_eq!(rel_joints[2].spec, spec);

        assert_relative_eq!(rel_joints[0].angle, 0.0);
        assert_relative_eq!(rel_joints[1].angle, PI / 8.0);
        assert_relative_eq!(rel_joints[2].angle, PI / 4.0);
        assert_relative_eq!(rel_joints[3].angle, -PI / 8.0);
    }
}
