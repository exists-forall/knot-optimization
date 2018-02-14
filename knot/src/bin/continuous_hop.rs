extern crate alga;
extern crate nalgebra;

extern crate knot;

use std::f64::consts::PI;
use std::f64::INFINITY;

use nalgebra::{UnitQuaternion, Vector3};

use knot::cost::{CostParams, Thresholds};
use knot::symmetry::symmetries;
use knot::geometries::curve_9_40;
use knot::continuous_optimize::Chain;

const TAU: f64 = 2.0 * PI;

const REPULSION: bool = true;

const RATE: f64 = 0.02;

const STEPS: u32 = 5_000;

const EPOCHS: u32 = 4;

fn optimize(chain: &mut Chain, steps: u32) -> f64 {
    let mut last_cost = INFINITY;
    for _ in 0..steps {
        last_cost = chain.optimize();

        // experimental repulsive force
        if REPULSION {
            let forces = (0..chain.joints.len())
                .map(|i| {
                    let mut force = Vector3::new(0.0, 0.0, 0.0);
                    for (sym_i, sym) in symmetries(3).enumerate() {
                        for j in 0..chain.joints.len() {
                            if !(sym_i == 0 && i == j) {
                                let diff = chain.joints[i].translation.vector -
                                    (sym * chain.joints[j]).translation.vector;
                                force += diff / diff.norm().powi(5);
                            }
                        }
                    }
                    force
                })
                .collect::<Vec<_>>();

            for (force, joint) in forces.iter().zip(chain.joints.iter_mut()) {
                joint.translation.vector += force * RATE * 0.1;
            }
        }
    }
    last_cost
}

fn main() {
    let cost_params = CostParams {
        dist_weight: 1.0,
        axis_weight: 3.0,
        locking_weight: 0.17,
        thresholds: Thresholds {
            dist_for_axis: INFINITY,
            axis_for_locking: INFINITY,
        },
    };

    let chain_size = 8;

    let mut best_chain = curve_9_40::chain(chain_size, 1.0, cost_params, RATE);
    let mut best_cost = optimize(&mut best_chain, STEPS);
    println!("Original cost: {}", best_cost);

    for epoch in 0..EPOCHS {
        let mut new_best = None;
        let mut new_best_cost = best_cost;

        println!("Best cost by epoch {}: {}", epoch, best_cost);

        for i in 0..chain_size {
            for &offset in &[-1.0, 1.0] {
                let mut offset_chain = best_chain.clone();
                offset_chain.joints[i] = offset_chain.joints[i] *
                    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), offset * TAU / 16.0);
                let cost = optimize(&mut offset_chain, STEPS);
                println!("{} {:+}: {}", i, offset, cost);
                if cost < new_best_cost {
                    println!("Best so far!");
                    new_best = Some((offset_chain, i, offset));
                    new_best_cost = cost;
                }
            }
        }

        if let Some((new_best_chain, new_best_i, new_best_offset)) = new_best {
            println!("New best: {} {:+}", new_best_i, new_best_offset);
            best_chain = new_best_chain;
            best_cost = new_best_cost;
        } else {
            println!("Couldn't find a better offset");
            break;
        }
    }
}
