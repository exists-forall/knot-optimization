use std::ops::{Deref, DerefMut};

use nalgebra::{Isometry3, Point3, Translation3, Vector3};

use cost::{cost_aligned, cost_opposing, CostParams};
use isometry_adjust as iso_adj;
use joint::JointSpec;

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
    pub return_to_initial_weight: f64,
    pub descent_rate: f64,
    pub steps: iso_adj::Steps,
    pub joints: Vec<Isometry3<f64>>,
    initial_points: Vec<Point3<f64>>,
}

fn get_leg(spec: &JointSpec, joint: &Isometry3<f64>, leg: &Leg) -> Isometry3<f64> {
    match leg {
        &Leg::Incoming => joint * spec.origin_to_in(),
        &Leg::Outgoing => joint * spec.origin_to_out(),
    }
}

impl Chain {
    pub fn new(
        spec: JointSpec,
        num_angles: u32,
        pre_phantom: PhantomJoint,
        post_phantom: PhantomJoint,
        cost_params: CostParams,
        return_to_initial_weight: f64,
        descent_rate: f64,
        steps: iso_adj::Steps,
        joints: Vec<Isometry3<f64>>,
    ) -> Self {
        let initial_points = joints
            .iter()
            .map(|joint| Point3 {
                coords: joint.translation.vector,
            }).collect();

        Chain {
            spec,
            num_angles,
            pre_phantom,
            post_phantom,
            cost_params,
            return_to_initial_weight,
            descent_rate,
            steps,
            joints,
            initial_points,
        }
    }

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

    pub fn apply_diffs(&mut self, ratio: f64, diffs: &[iso_adj::IsometryDifferential]) {
        let joint_radius = (self.spec.dist_in() + self.spec.dist_out()) * 0.5;
        for (i, &diff) in diffs.iter().enumerate() {
            iso_adj::apply_step(
                joint_radius,
                &mut self.joints[i],
                &diff.scale(-self.descent_rate * ratio),
            );
        }
    }

    pub fn total_cost(&self) -> f64 {
        let mut result = 0.0;
        let mut pre_joint = self.get_phantom(&self.pre_phantom);
        let mut pre_leg = self.pre_phantom.leg;
        for i in 0..self.joints.len() {
            let joint = self.joints[i];
            let (post_joint, post_leg) = if i + 1 < self.joints.len() {
                (self.joints[i + 1], Leg::Incoming)
            } else {
                (self.get_phantom(&self.post_phantom), self.post_phantom.leg)
            };

            let pre_cost = self.cost_between(&pre_joint, &pre_leg, &joint, &Leg::Incoming);
            let post_cost = self.cost_between(&joint, &Leg::Outgoing, &post_joint, &post_leg);
            result += pre_cost + post_cost;
            pre_joint = joint;
            pre_leg = Leg::Outgoing;
        }
        result
    }

    pub fn adaptive_optimize(&mut self, ratios: &[f64], tolerance: f64) -> f64 {
        let joint_radius = (self.spec.dist_in() + self.spec.dist_out()) * 0.5;

        let mut curr_total_cost = 0.0;

        let mut diff_mag_squ = 0.0;
        let diffs = {
            let mut pre_joint = self.get_phantom(&self.pre_phantom);
            let mut pre_leg = self.pre_phantom.leg;
            (0..self.joints.len())
                .map(|i| {
                    let joint = self.joints[i];
                    let (post_joint, post_leg) = if i + 1 < self.joints.len() {
                        (self.joints[i + 1], Leg::Incoming)
                    } else {
                        (self.get_phantom(&self.post_phantom), self.post_phantom.leg)
                    };

                    let (curr_joint_cost, diff) =
                        iso_adj::differentiate(&self.steps, joint, |new_joint| {
                            let pre_cost =
                                self.cost_between(&pre_joint, &pre_leg, &new_joint, &Leg::Incoming);
                            let post_cost = self.cost_between(
                                &new_joint,
                                &Leg::Outgoing,
                                &post_joint,
                                &post_leg,
                            );
                            pre_cost + post_cost
                        });
                    curr_total_cost += curr_joint_cost;
                    diff_mag_squ += diff.magnitude_squ(joint_radius);
                    pre_joint = joint;
                    pre_leg = Leg::Outgoing;
                    diff
                }).collect::<Vec<_>>()
        };

        // Ratios should be in DESCENDING order!
        let old_joints = self.joints.clone();
        for &ratio in ratios {
            self.joints.clone_from_slice(&old_joints);
            self.apply_diffs(ratio, &diffs);
            let new_cost = self.total_cost();
            let expected_delta_cost = -diff_mag_squ * self.descent_rate * ratio;
            let actual_delta_cost = new_cost - curr_total_cost;
            if actual_delta_cost / expected_delta_cost >= tolerance {
                self.descent_rate *= ratio;
                return curr_total_cost;
            }
        }

        // Last ratio still violated tolerance, but it's the best we have, so default to it
        self.descent_rate *= ratios.last().unwrap();
        curr_total_cost
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

    pub fn return_to_initial(&mut self) {
        for (joint, initial) in self.joints.iter_mut().zip(self.initial_points.iter()) {
            let diff = initial - Point3 {
                coords: joint.translation.vector,
            };
            *joint =
                Translation3::from_vector(diff * self.return_to_initial_weight * self.descent_rate)
                    * *joint;
        }
    }
}

#[derive(Clone, Debug)]
pub struct RepulsionChain {
    pub chain: Chain,
    pub repulsion_exp: i32,
    pub repulsion_strength: f64,
    pub max_repulsion_strength: f64,

    // Necessary because the chain only contains boundary condition information, not global
    // symmetries.
    pub symmetries: Vec<Isometry3<f64>>,

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

fn within(a: usize, b: usize, range: usize) -> bool {
    ((a as isize) - (b as isize)).abs() <= (range as isize)
}

fn clamped_inverse_power(x: f64, n: i32, scale: f64, clamp: f64) -> f64 {
    let result = if x <= 0.0 {
        clamp
    } else {
        (scale * x.powi(-n)).min(clamp)
    };
    if result.is_nan() {
        eprintln!("Clamped inverse power returned NaN!");
    }
    result
}

impl RepulsionChain {
    pub fn new(
        chain: Chain,
        symmetries: Vec<Isometry3<f64>>,
        repulsion_exp: i32,
        repulsion_strength: f64,
        max_repulsion_strength: f64,
    ) -> Self {
        RepulsionChain {
            chain,
            repulsion_exp,
            repulsion_strength,
            max_repulsion_strength,
            symmetries,
            forces: Vec::new(),
        }
    }

    pub fn repulse(&mut self) {
        assert_eq!(self.forces.len(), 0);
        self.forces
            .resize(self.chain.joints.len(), Vector3::new(0.0, 0.0, 0.0));
        for i in 0..self.chain.joints.len() {
            for (sym_i, sym) in self.symmetries.iter().enumerate() {
                for j in 0..self.chain.joints.len() {
                    let neighbors_in_same_branch = sym_i == 0 && within(i, j, 1);
                    let neighbors_at_start = sym_i == 1 && i == 0 && j == 0; // DEBUG
                    let neighbors_at_end = sym_i == 2
                        && i == self.chain.joints.len() - 1
                        && j == self.chain.joints.len() - 1;

                    let neighbors =
                        neighbors_in_same_branch || neighbors_at_start || neighbors_at_end;

                    if !neighbors {
                        let diff = self.chain.joints[i].translation.vector
                            - (sym * self.chain.joints[j]).translation.vector;
                        // surface distance
                        let surf_dist = diff.norm() - self.chain.spec.radius() * 2.0;
                        self.forces[i] += diff / diff.norm() * clamped_inverse_power(
                            surf_dist,
                            self.repulsion_exp,
                            self.repulsion_strength,
                            self.max_repulsion_strength,
                        );
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
