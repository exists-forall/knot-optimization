extern crate alga;
extern crate nalgebra;
extern crate serde_json;

extern crate knot;
extern crate rand;

use std::env::args;
use std::f64::consts::PI;
use std::f64::INFINITY;
use std::fs::File;
use std::process::exit;

use alga::general::SubsetOf;
use nalgebra::{Isometry3, UnitQuaternion, Vector3};

use knot::optimize_tools::{Chain, Leg, PhantomJoint, RepulsionChain};
use knot::defaults::continuous_optimization::{
    COST_PARAMS, TREFOIL_CHAIN_SIZE, MAX_REPULSION_STRENGTH, RATE, REPULSION,
    REPULSION_EXPONENT, REPULSION_STRENGTH, RETURN_TO_INITIAL, RETURN_TO_INITIAL_WEIGHT, STEPS,
};
use knot::geometries::trefoil_curve;
use knot::isometry_adjust;
use knot::report::{JointsParity, KnotGeometry, Transform};
use knot::symmetry::{symmetries, symmetries_with_skip};

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
    let (mut curr_chain, _symms, _parity) = match args().nth(1) {
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
                trefoil_curve::chain(
                    TREFOIL_CHAIN_SIZE,
                    3.1, // arc length multiplier
                    COST_PARAMS,
                    RETURN_TO_INITIAL_WEIGHT,
                    RATE,
                ),
                symmetries(3).map(|quat| quat.to_superset()).collect(),
                REPULSION_EXPONENT,
                REPULSION_STRENGTH,
                MAX_REPULSION_STRENGTH,
            ),
            symmetries_with_skip(3, 2)
                .map(|iso| Transform::from_isometry(iso.to_superset()))
                .collect(),
            JointsParity::Even,
        ),
    };
    let curr_cost = optimize(&mut curr_chain, STEPS);
    println!("Original cost: {}", curr_cost);
    println!("Approximate original locking angles:");
    let mut prev_trans = Isometry3::identity();
    for &joint in &curr_chain.joints {
        let trans_0 = prev_trans;
        let trans_1 = joint * curr_chain.spec.origin_to_in();

        let axis_0 = trans_0 * Vector3::y_axis().to_superset();
        let axis_1 = trans_1 * Vector3::y_axis().to_superset();

        let align = UnitQuaternion::rotation_between(&axis_1, &axis_0)
            .unwrap_or(UnitQuaternion::identity());
        let aligned_rel_rotation =
            trans_0.rotation.inverse() * align * trans_1.rotation;
        let locking_angle = aligned_rel_rotation.angle();
        let locking_number =
            locking_angle / (2.0 * PI) * (curr_chain.num_angles as f64);
        println!("{}", locking_number);
        prev_trans = joint * curr_chain.spec.origin_to_out();
    }

}
