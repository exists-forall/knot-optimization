use std::ops::{Deref, DerefMut};

use nalgebra::{Isometry3, Vector3};

use joint::JointSpec;
use cost::{CostParams, cost_opposing, cost_aligned};
use isometry_adjust as iso_adj;
use symmetry::symmetries;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Leg {
    Incoming,
    Outgoing,
}

#[derive(Clone, Copy, Debug)]
pub struct PhantomJoint {
    pub symmetry: Isometry3<f64>,
    pub index: usize,
    pub leg: Leg,
}

#[derive(Clone, Debug)]
pub struct Chain {
    pub spec: JointSpec,
    pub num_angles: u32,
    pub pre_phantom: PhantomJoint,
    pub post_phantom: PhantomJoint,
    pub cost_params: CostParams,
    pub descent_rate: f64,
    pub steps: iso_adj::Steps,
    pub joints: Vec<Isometry3<f64>>,
}

fn get_leg(spec: &JointSpec, joint: &Isometry3<f64>, leg: &Leg) -> Isometry3<f64> {
    match leg {
        &Leg::Incoming => joint * spec.origin_to_in(),
        &Leg::Outgoing => joint * spec.origin_to_out(),
    }
}

impl Chain {
    fn get_phantom(&self, phantom: &PhantomJoint) -> Isometry3<f64> {
        phantom.symmetry * self.joints[phantom.index]
    }

    fn cost_between(
        &self,
        joint_0: &Isometry3<f64>,
        leg_0: &Leg,
        joint_1: &Isometry3<f64>,
        leg_1: &Leg,
    ) -> f64 {
        // Schematic representation of different cases:
        // (Incoming, Incoming): [out in]<-- -->[in out]
        // (Incoming, Outgoing): [out in]<-- <--[out in]
        // (Outgoing, Incoming): [in out]--> -->[in out]
        // (Outgoing, Outgoing): [in out]--> <--[out in]
        let cost_fn = if leg_0 == leg_1 {
            cost_opposing
        } else {
            cost_aligned
        };

        cost_fn(
            &self.cost_params,
            self.num_angles,
            &get_leg(&self.spec, &joint_0, &leg_0),
            &get_leg(&self.spec, &joint_1, &leg_1),
        )
    }

    pub fn optimize(&mut self) -> f64 {
        let joint_radius = (self.spec.dist_in() + self.spec.dist_out()) * 0.5;

        let mut pre_joint = self.get_phantom(&self.pre_phantom);
        let mut pre_leg = self.pre_phantom.leg;

        let mut curr_total_cost = 0.0;

        for i in 0..self.joints.len() {
            let joint = self.joints[i];
            let (post_joint, post_leg) = if i + 1 < self.joints.len() {
                (self.joints[i + 1], Leg::Incoming)
            } else {
                (self.get_phantom(&self.post_phantom), self.post_phantom.leg)
            };

            let (curr_joint_cost, diff) = iso_adj::differentiate(&self.steps, joint, |new_joint| {
                let pre_cost = self.cost_between(&pre_joint, &pre_leg, &new_joint, &Leg::Incoming);
                let post_cost =
                    self.cost_between(&new_joint, &Leg::Outgoing, &post_joint, &post_leg);
                pre_cost + post_cost
            });
            iso_adj::apply_step(
                joint_radius,
                &mut self.joints[i],
                &diff.scale(-self.descent_rate),
            );

            curr_total_cost += curr_joint_cost;

            pre_joint = joint;
            pre_leg = Leg::Outgoing;
        }

        curr_total_cost
    }
}

#[derive(Clone, Debug)]
pub struct RepulsionChain {
    pub chain: Chain,
    pub repulsion_exp: i32,
    pub repulsion_strength: f64,

    // Necessary because the chain only contains boundary condition information, not global
    // symmetries.
    pub symmetry: u32,

    // cached workspace to avoid reallocation
    forces: Vec<Vector3<f64>>,

    // TODO: Spatial partition structure to avoid qudadratic-time force calculation (can probably
    // reuse or extend CollisionGrid)
}

impl Deref for RepulsionChain {
    type Target = Chain;

    fn deref(&self) -> &Chain {
        &self.chain
    }
}

impl DerefMut for RepulsionChain {
    fn deref_mut(&mut self) -> &mut Chain {
        &mut self.chain
    }
}

impl RepulsionChain {
    pub fn new(chain: Chain, symmetry: u32, repulsion_exp: i32, repulsion_strength: f64) -> Self {
        RepulsionChain {
            chain,
            repulsion_exp,
            repulsion_strength,
            symmetry,
            forces: Vec::new(),
        }
    }

    pub fn repulse(&mut self) {
        assert_eq!(self.forces.len(), 0);
        self.forces.resize(
            self.chain.joints.len(),
            Vector3::new(0.0, 0.0, 0.0),
        );
        for i in 0..self.chain.joints.len() {
            for (sym_i, sym) in symmetries(self.symmetry).enumerate() {
                for j in 0..self.chain.joints.len() {
                    if !(sym_i == 0 && i == j) {
                        let diff = self.chain.joints[i].translation.vector -
                            (sym * self.chain.joints[j]).translation.vector;
                        self.forces[i] += diff / diff.norm() /
                            (diff.norm() - self.chain.spec.radius() * 2.0).powi(2) *
                            self.repulsion_strength;
                    }
                }
            }
        }

        for (force, joint) in self.forces.iter().zip(self.chain.joints.iter_mut()) {
            joint.translation.vector += force * self.chain.descent_rate;
        }
        self.forces.clear();
    }
}
