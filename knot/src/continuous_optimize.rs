use nalgebra::Isometry3;

use joint::JointSpec;
use cost::{CostParams, cost_opposing, cost_aligned};
use isometry_adjust as iso_adj;

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
