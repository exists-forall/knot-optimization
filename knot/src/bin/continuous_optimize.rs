#![feature(type_ascription)]

extern crate kiss3d;
extern crate alga;
extern crate nalgebra;
extern crate glfw;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::f64::consts::PI;
use std::fs::File;
use std::process::exit;
use std::env::args;

use kiss3d::window::Window;
use kiss3d::light::Light;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use alga::general::SubsetOf;

use knot::joint::{at_angles, RelativeJoint};
use knot::defaults;
use knot::defaults::continuous_optimization::{COST_PARAMS, RATE, REPULSION, REPULSION_EXPONENT,
                                              REPULSION_STRENGTH, MAX_REPULSION_STRENGTH,
                                              CURVE_9_40_CHAIN_SIZE, RETURN_TO_INITIAL_WEIGHT};
use knot::visualize::joint_render::{add_joints, Style};
use knot::symmetry::symmetries;
use knot::geometries::curve_9_40;
use knot::continuous_optimize::{Chain, Leg, PhantomJoint, RepulsionChain};
use knot::report::{KnotGeometry, Transform};
use knot::isometry_adjust;

const TAU: f64 = 2.0 * PI;

const DEBUG_ANGLES: bool = false;

fn main() {
    let mut chain = match args().nth(1) {
        Some(filename) => {
            let file = File::open(&filename).unwrap_or_else(|_| {
                eprintln!("Could not open file {}", filename);
                exit(1);
            });
            let geometry: KnotGeometry = serde_json::from_reader(file).unwrap_or_else(|_| {
                eprintln!("Could not parse input file");
                exit(1);
            });
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
            )
        }
        None => {
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
            )
        }
    };

    let mut window = Window::new("Continuous Optimization");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(
        window.scene_mut(),
        &chain.spec,
        chain.num_angles,
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
                WindowEvent::Key(_, _, Action::Release, _) => {}
                WindowEvent::Key(code, _, _, _) => {
                    match code {
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
                                let aligned_rel_rotation = trans_0.rotation.inverse() * align *
                                    trans_1.rotation;
                                let locking_angle = aligned_rel_rotation.angle();
                                let locking_number = locking_angle / (2.0 * PI) *
                                    (chain.num_angles as f64);
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
                        Key::Enter => {
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
                        Key::Equal => {
                            chain.joints[selected] = chain.joints[selected] *
                                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), TAU / 16.0);
                        }
                        Key::Minus => {
                            chain.joints[selected] = chain.joints[selected] *
                                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -TAU / 16.0);
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
                    }
                }
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
                    let aligned_rel_rotation = trans_0.rotation.inverse() * align *
                        trans_1.rotation;
                    let locking_angle = aligned_rel_rotation.angle();
                    prev_trans = joint * chain.spec.origin_to_out();
                    locking_angle
                });
                let rel_joints = locking_angles.map(|angle| {
                    RelativeJoint {
                        spec: defaults::joint_spec(),
                        angle,
                    }
                });
                for joint in at_angles(rel_joints, Isometry3::identity()) {
                    nodes[i].set_local_transformation(
                        (Translation3::new(15.0, 0.0, 0.0).to_superset(): Isometry3<f32>) *
                            (joint.to_superset(): Isometry3<f32>),
                    );
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
