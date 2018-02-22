extern crate alga;
extern crate nalgebra;

extern crate knot;

use std::f64::consts::PI;
use std::f64::INFINITY;

use nalgebra::{UnitQuaternion, Vector3};

use knot::geometries::curve_9_40;
use knot::continuous_optimize::RepulsionChain;
use knot::defaults::continuous_optimization::{COST_PARAMS, RATE, STEPS, REPULSION,
                                              REPULSION_EXPONENT, REPULSION_STRENGTH,
                                              MAX_REPULSION_STRENGTH, CURVE_9_40_CHAIN_SIZE,
                                              RETURN_TO_INITIAL_WEIGHT, RETURN_TO_INITIAL};

const TAU: f64 = 2.0 * PI;

const EPOCHS: u32 = 10;

fn optimize(chain: &mut RepulsionChain, steps: u32) -> f64 {
    let mut last_cost = INFINITY;
    for _ in 0..steps {
        last_cost = chain.optimize();

        if REPULSION {
            chain.repulse();
        }

        if RETURN_TO_INITIAL {
            chain.return_to_initial();
        }
    }
    last_cost
}

fn main() {
    let mut best_chain = RepulsionChain::new(
        curve_9_40::chain(
            CURVE_9_40_CHAIN_SIZE,
            1.0,
            COST_PARAMS,
            RETURN_TO_INITIAL_WEIGHT,
            RATE,
        ),
        3,
        1,
        REPULSION_EXPONENT,
        REPULSION_STRENGTH,
        MAX_REPULSION_STRENGTH,
    );
    let mut best_cost = optimize(&mut best_chain, STEPS);
    println!("Original cost: {}", best_cost);

    let mut steps = Vec::new();

    for epoch in 0..EPOCHS {
        let mut new_best = None;
        let mut new_best_cost = best_cost;

        println!("Best cost by epoch {}: {}", epoch, best_cost);

        for i in 0..best_chain.joints.len() {
            for &offset in &[-2.0, -1.0, 1.0, 2.0] {
                let angle = if i == 0 {
                    0.5 * offset * TAU / 16.0
                } else {
                    offset * TAU / 16.0
                };

                let mut offset_chain = best_chain.clone();
                offset_chain.joints[i] = offset_chain.joints[i] *
                    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
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
            steps.push((new_best_i, new_best_offset));
            best_chain = new_best_chain;
            best_cost = new_best_cost;
        } else {
            println!("Couldn't find a better offset");
            break;
        }
    }

    println!("\nFinal steps:");
    for &(i, offset) in &steps {
        println!("{} {:+}", i, offset);
    }
}
