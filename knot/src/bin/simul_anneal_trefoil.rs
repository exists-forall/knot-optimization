extern crate alga;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra;
extern crate serde;
extern crate serde_json;

extern crate knot;
extern crate rand;

use std::env::args;
use std::f64::consts::PI;
use std::f64::consts::E;
use std::f64::INFINITY;
use std::fs::File;
use std::process::exit;

use kiss3d::light::Light;
use kiss3d::window::Window;
use kiss3d::event::{WindowEvent, Action, Key};

use alga::general::SubsetOf;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use knot::optimize_tools::{Chain, Leg, PhantomJoint, RepulsionChain};
use knot::defaults;
use knot::defaults::continuous_optimization::{
    COST_PARAMS, TREFOIL_CHAIN_SIZE, MAX_REPULSION_STRENGTH, RATE, REPULSION,
    REPULSION_EXPONENT, REPULSION_STRENGTH, RETURN_TO_INITIAL, RETURN_TO_INITIAL_WEIGHT, STEPS,
};
use knot::geometries::trefoil_curve;
use knot::isometry_adjust;
use knot::report::{JointsParity, KnotGeometry, Transform};
use knot::symmetry::{symmetries, symmetries_with_skip};
use knot::visualize::joint_render::{add_joints, Style};
use knot::joint::{RelativeJoint, at_angles};

const TAU: f64 = 2.0 * PI;
const TWO_WEIGHT: f64 = 0.5;
const EPOCHS: i32 = 50;

const DEBUG_ANGLES: bool = false;


/* Adjust the constant value as needed!
fn cooling_schedule(epoch: i32, cost_diff: f64) -> bool {
    if cost_diff > 0.0 {
        true
    } else {
        if rand::random::<f64>() < std::f64::consts::E.powf(cost_diff*(epoch as f64)* (50.0) / (EPOCHS as f64)) {
            true
        } else {
            false
        }
    }

}
*/
fn cooling_schedule(epoch: i32, cost_diff: f64) -> bool {
    if cost_diff > 0.0 {
        true
    } else if cost_diff < -2.0 {
        false
    } else {
        let prob : f64;
        prob = E.powf(cost_diff*50.0*(epoch as f64 / EPOCHS as f64)) - (epoch as f64 / EPOCHS as f64);
        if rand::random::<f64>() < prob {
            eprintln!("Prob: {}", prob);
            true
        } else {
            false
        }
    }
}

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
    let (mut curr_chain, symms, parity) = match args().nth(1) {
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
                    3.5,
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
    let mut curr_cost = optimize(&mut curr_chain, STEPS);
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

    let mut steps: Vec<(usize, f64)> = Vec::new();

    let mut change: [i32; 4] = [0, 0, 0, 0];
    let mut best_cost = curr_cost;
    let mut best_epoch = 0;
    let mut best_chain = curr_chain.clone();

    for epoch in 0..EPOCHS {

        let x: u8 = rand::random();
        let mut angle = if rand::random() {
            TAU / 16.0
        } else {
            -1.0 * TAU / 16.0
        };

        let mut offset_chain = curr_chain.clone();

        let rand_joint: usize;

        if rand::random::<f64>() > TWO_WEIGHT {
            rand_joint = (x as usize) % (curr_chain.joints.len() - 2);
            if rand_joint == 0 {
                angle = angle * 0.5;
            }
            offset_chain.joints[rand_joint] =
                offset_chain.joints[rand_joint]
                    * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
            offset_chain.joints[rand_joint + 1] =
                offset_chain.joints[rand_joint + 1]
                    * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -2.0 * angle);
            offset_chain.joints[rand_joint + 2] =
                offset_chain.joints[rand_joint + 2]
                    * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
        } else {
            rand_joint = x as usize % (curr_chain.joints.len() - 1);
            if rand_joint == 0 {
                angle = angle * 0.5;
            }
            offset_chain.joints[rand_joint] =
                offset_chain.joints[rand_joint]
                    * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
            offset_chain.joints[rand_joint + 1] =
                offset_chain.joints[rand_joint + 1]
                    * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -1.0 * angle);
        }

        let cost = optimize(&mut offset_chain, STEPS);
        let cost_diff = curr_cost - cost;
        if cooling_schedule(epoch, cost_diff) {
            curr_chain = offset_chain;
            curr_cost = cost;
            if best_cost > curr_cost {
                best_cost = curr_cost;
                best_epoch = epoch;
                best_chain = curr_chain.clone();
                eprintln!("New Best: {}", best_cost);
            }
            steps.push((rand_joint, (angle * 16.0 / TAU)));
            if epoch <= EPOCHS / 4 {
                change[0] += 1;
            } else {
                let q: usize = (4.0 * (epoch as f64)/(EPOCHS as f64)).floor() as usize;
                change[q] = change[q] + 1;
            }
            // eprintln!("Changed!");
            // eprintln!("{} {:+} : {} {:+}", rand_joint, (angle * 16.0 / TAU), cost, cost_diff);
            eprintln!("Cost after epoch {}: {}", epoch, curr_cost);
        } else {
            // eprintln!("Unchanged!");
            // eprintln!("{} {:+} : {} {:+}", rand_joint, (angle * 16.0 / TAU), cost, cost_diff);
        }

    }

    // eprintln!("\nFinal steps:");
    // for &(i, offset) in &steps {
    //     eprintln!("{} {:+}", i, offset);
    // }

    // let transforms = curr_chain
    //     .joints
    //     .iter()
    //     .cloned()
    //     .map(|iso| Transform::from_isometry(iso))
    //     .collect::<Vec<_>>();

    // let geometry = KnotGeometry {
    //     joint_spec: curr_chain.spec,
    //     num_angles: curr_chain.num_angles,
    //     cost_params: curr_chain.cost_params,
    //     parity: parity,
    //     symmetries: symms,
    //     transforms,
    // };

    // println!("\nFinal geometry:");
    // println!("{}", serde_json::to_string_pretty(&geometry).unwrap());

    // println!("\nChanges per Quarter of Total Steps");
    // println!("{:?}", change);

    println!("\nFinal Cost");
    println!("{}", curr_cost);

    println!("Approximate locking angles:");
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

    println!("\nBest Found Cost");
    println!("{} at epoch {}", best_cost, best_epoch);

    println!("Approximate locking angles:");
    let mut prev_trans = Isometry3::identity();
    for &joint in &best_chain.joints {
        let trans_0 = prev_trans;
        let trans_1 = joint * best_chain.spec.origin_to_in();

        let axis_0 = trans_0 * Vector3::y_axis().to_superset();
        let axis_1 = trans_1 * Vector3::y_axis().to_superset();

        let align = UnitQuaternion::rotation_between(&axis_1, &axis_0)
            .unwrap_or(UnitQuaternion::identity());
        let aligned_rel_rotation =
            trans_0.rotation.inverse() * align * trans_1.rotation;
        let locking_angle = aligned_rel_rotation.angle();
        let locking_number =
            locking_angle / (2.0 * PI) * (best_chain.num_angles as f64);
        println!("{}", locking_number);
        prev_trans = joint * best_chain.spec.origin_to_out();
    }

    let chain = best_chain;




    let mut window = Window::new("Continuous Optimization");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(
        window.scene_mut(),
        &chain.spec,
        chain.num_angles as u16,
        chain.joints.len() * 6
        // debug:
        + if DEBUG_ANGLES {
            chain.joints.len()
        } else {
            0
        },
        Style::Flat,
    );

    let mut step = 0;

    let mut selected = 0;

    let mut return_to_initial = false;

    while window.render() {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(_, Action::Release, _) => {}
                WindowEvent::Key(code, _, _) => match code {
                    Key::Space => {
                        println!("Approximate locking angles:");
                        let mut prev_trans = Isometry3::identity();
                        for &joint in &chain.joints {
                            let trans_0 = prev_trans;
                            let trans_1 = joint * chain.spec.origin_to_in();

                            let axis_0 = trans_0 * Vector3::y_axis().to_superset();
                            let axis_1 = trans_1 * Vector3::y_axis().to_superset();

                            let align = UnitQuaternion::rotation_between(&axis_1, &axis_0)
                                .unwrap_or(UnitQuaternion::identity());
                            let aligned_rel_rotation =
                                trans_0.rotation.inverse() * align * trans_1.rotation;
                            let locking_angle = aligned_rel_rotation.angle();
                            let locking_number =
                                locking_angle / (2.0 * PI) * (chain.num_angles as f64);
                            println!("{}", locking_number);
                            prev_trans = joint * chain.spec.origin_to_out();
                        }
                    }
                    Key::Right => {
                        chain.cost_params.locking_weight *= 1.5;
                        println!("Locking weight: {}", chain.cost_params.locking_weight);
                    }
                    Key::Left => {
                        chain.cost_params.locking_weight /= 1.5;
                        println!("Locking weight: {}", chain.cost_params.locking_weight);
                    }
                    Key::Backslash => {
                        chain.cost_params.locking_weight = 0.0;
                        println!("Locking weight disabled");
                    }
                    Key::Return => {
                        chain.cost_params.locking_weight = COST_PARAMS.locking_weight;
                        println!(
                            "Locking weight enabled: {}",
                            chain.cost_params.locking_weight
                        );
                    }
                    Key::Up => {
                        selected = (selected + 1) % chain.joints.len();
                        println!("Selected {}", selected);
                    }
                    Key::Down => {
                        selected = (selected + chain.joints.len() - 1) % chain.joints.len();
                        println!("Selected {}", selected);
                    }
                    Key::Equals => {
                        chain.joints[selected] = chain.joints[selected]
                            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), TAU / 16.0);
                    }
                    Key::Minus => {
                        chain.joints[selected] = chain.joints[selected]
                            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -TAU / 16.0);
                    }
                    Key::Q => {
                        let cost = chain.optimize();
                        println!("{{{}, {}}},", step, cost);
                    }
                    Key::R => {
                        return_to_initial = !return_to_initial;
                        if return_to_initial {
                            println!("'Return to initial' enabled");
                        } else {
                            println!("'Return to initial' disabled");
                        }
                    }
                    Key::D => {
                        println!("Descent rate: {}", chain.descent_rate);
                    }

                    Key::N => {
                        println!("Repulsion disabled");
                        chain.repulsion_strength = 0.0;
                    }
                    Key::M => {
                        println!("Repulsion enabled");
                        chain.repulsion_strength = REPULSION_STRENGTH;
                    }
                    Key::K => {
                        chain.repulsion_strength *= 1.5;
                        println!("Repulsion: {}", chain.repulsion_strength);
                    }
                    Key::J => {
                        chain.repulsion_strength /= 1.5;
                        println!("Repulsion: {}", chain.repulsion_strength);
                    }

                    Key::V => {
                        println!("Max repulsion disabled");
                        chain.max_repulsion_strength = 0.0001;
                    }
                    Key::B => {
                        chain.max_repulsion_strength = MAX_REPULSION_STRENGTH;
                        println!("Max repulsion enabled: {}", chain.max_repulsion_strength);
                    }
                    Key::H => {
                        chain.max_repulsion_strength *= 1.5;
                        println!("Max repulsion: {}", chain.max_repulsion_strength);
                    }
                    Key::G => {
                        chain.max_repulsion_strength /= 1.5;
                        println!("Max repulsion: {}", chain.max_repulsion_strength);
                    }

                    Key::F => {
                        chain.descent_rate = RATE / 10.0;
                        println!("Descent rate reset: {}", chain.descent_rate);
                    }

                    _ => {}
                },
                _ => {}
            }
        }

        {
            let mut i = 0;
            let mut first = true;
            for sym in symmetries(3) {
                for &joint in &chain.joints {
                    if !first {
                        nodes[i].set_color(0.5, 0.5, 0.5);
                    }
                    nodes[i].set_local_transformation((sym * joint).to_superset());
                    i += 1;
                }
                first = false;
            }

            if DEBUG_ANGLES {
                let mut prev_trans = Isometry3::identity();
                let locking_angles = chain.joints.iter().map(|&joint| {
                    let trans_0 = prev_trans;
                    let trans_1 = joint * chain.spec.origin_to_in();

                    let axis_0 = trans_0 * Vector3::y_axis().to_superset();
                    let axis_1 = trans_1 * Vector3::y_axis().to_superset();

                    let align = UnitQuaternion::rotation_between(&axis_1, &axis_0)
                        .unwrap_or(UnitQuaternion::identity());
                    let aligned_rel_rotation =
                        trans_0.rotation.inverse() * align * trans_1.rotation;
                    let locking_angle = aligned_rel_rotation.angle();
                    prev_trans = joint * chain.spec.origin_to_out();
                    locking_angle
                });
                let rel_joints = locking_angles.map(|angle| RelativeJoint {
                    spec: defaults::joint_spec(),
                    angle,
                });
                for joint in at_angles(rel_joints, Isometry3::identity()) {
                    let translation: Isometry3<f32> =
                        Translation3::new(15.0, 0.0, 0.0).to_superset();
                    let joint_iso: Isometry3<f32> = joint.to_superset();
                    nodes[i].set_local_transformation(translation * joint_iso);
                    i += 1;
                }
            }
        }
        for _ in 0..1000 {
            chain.adaptive_optimize(&[2.0, 1.0, 0.5], 0.5);
            step += 1;

            if REPULSION {
                chain.repulse();
            }

            if return_to_initial {
                chain.return_to_initial();
            }
        }
    }

}
