extern crate alga;
extern crate nalgebra;
extern crate serde_json;

extern crate knot;

use std::env::args;
use std::f64::consts::PI;
use std::f64::INFINITY;
use std::fs::File;
use std::process::exit;

use alga::general::SubsetOf;
use nalgebra::{UnitQuaternion, Vector3};

use knot::optimize_tools::{Chain, Leg, PhantomJoint, RepulsionChain};
use knot::defaults::continuous_optimization::{
    COST_PARAMS, CURVE_9_40_CHAIN_SIZE, MAX_REPULSION_STRENGTH, RATE, REPULSION,
    REPULSION_EXPONENT, REPULSION_STRENGTH, RETURN_TO_INITIAL, RETURN_TO_INITIAL_WEIGHT, STEPS,
};
use knot::geometries::curve_9_40;
use knot::isometry_adjust;
use knot::report::{JointsParity, KnotGeometry, Transform};
use knot::symmetry::{symmetries, symmetries_with_skip};

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
    let (mut best_chain, symms, parity) = match args().nth(1) {
        Some(filename) => {
            let file = File::open(&filename).unwrap_or_else(|_| {
                eprintln!("Could not open file {}", filename);
                exit(1);
            });
            let geometry: KnotGeometry = serde_json::from_reader(file).unwrap_or_else(|_| {
                eprintln!("Could not parse input file");
                exit(1);
            });
            (
                RepulsionChain::new(
                    Chain::new(
                        geometry.joint_spec,
                        geometry.num_angles,
                        PhantomJoint {
                            symmetry: geometry.symmetries[1].to_isometry(),
                            index: 0,
                            leg: Leg::Incoming,
                        },
                        // post-phantom
                        PhantomJoint {
                            symmetry: geometry.symmetries[3].to_isometry(),
                            index: geometry.transforms.len() - 1,
                            leg: Leg::Outgoing,
                        },
                        geometry.cost_params,
                        RETURN_TO_INITIAL_WEIGHT,
                        RATE / 10.0,
                        isometry_adjust::Steps::new_uniform(0.000001),
                        geometry
                            .transforms
                            .iter()
                            .map(Transform::to_isometry)
                            .collect(),
                    ),
                    geometry
                        .symmetries
                        .iter()
                        .map(Transform::to_isometry)
                        .collect(),
                    REPULSION_EXPONENT,
                    REPULSION_STRENGTH,
                    MAX_REPULSION_STRENGTH,
                ),
                geometry.symmetries,
                geometry.parity,
            )
        }
        None => (
            RepulsionChain::new(
                curve_9_40::chain(
                    CURVE_9_40_CHAIN_SIZE,
                    0.7,
                    COST_PARAMS,
                    RETURN_TO_INITIAL_WEIGHT,
                    RATE,
                ),
                symmetries(3).map(|quat| quat.to_superset()).collect(),
                REPULSION_EXPONENT,
                REPULSION_STRENGTH,
                MAX_REPULSION_STRENGTH,
            ),
            symmetries_with_skip(3, 4)
                .map(|iso| Transform::from_isometry(iso.to_superset()))
                .collect(),
            JointsParity::Even,
        ),
    };
    let mut best_cost = optimize(&mut best_chain, STEPS);
    eprintln!("Original cost: {}", best_cost);

    let mut steps = Vec::new();

    for epoch in 0..EPOCHS {
        let mut new_best = None;
        let mut new_best_cost = best_cost;

        eprintln!("Best cost by epoch {}: {}", epoch, best_cost);

        for i in 0..best_chain.joints.len() {
            for &offset in &[-2.0, -1.0, 1.0, 2.0] {
                let angle = if i == 0 {
                    0.5 * offset * TAU / 16.0
                } else {
                    offset * TAU / 16.0
                };

                let mut offset_chain = best_chain.clone();
                offset_chain.joints[i] =
                    offset_chain.joints[i]
                        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
                let cost = optimize(&mut offset_chain, STEPS);
                eprintln!("{} {:+}: {}", i, offset, cost);
                if cost < new_best_cost {
                    eprintln!("Best so far!");
                    new_best = Some((offset_chain, i, offset));
                    new_best_cost = cost;
                }
            }
        }

        if let Some((new_best_chain, new_best_i, new_best_offset)) = new_best {
            eprintln!("New best: {} {:+}", new_best_i, new_best_offset);
            steps.push((new_best_i, new_best_offset));
            best_chain = new_best_chain;
            best_cost = new_best_cost;
        } else {
            eprintln!("Couldn't find a better offset");
            break;
        }
    }

    eprintln!("\nFinal steps:");
    for &(i, offset) in &steps {
        eprintln!("{} {:+}", i, offset);
    }

    let transforms = best_chain
        .joints
        .iter()
        .cloned()
        .map(|iso| Transform::from_isometry(iso))
        .collect::<Vec<_>>();

    let geometry = KnotGeometry {
        joint_spec: best_chain.spec,
        num_angles: best_chain.num_angles,
        cost_params: best_chain.cost_params,
        parity: parity,
        symmetries: symms,
        transforms,
    };

    eprintln!("\nFinal geometry:");
    println!("{}", serde_json::to_string_pretty(&geometry).unwrap());
}
