#![feature(conservative_impl_trait)]
#![feature(type_ascription)]

extern crate kiss3d;
extern crate alga;
extern crate nalgebra;
extern crate glfw;

extern crate knot;

use std::f64::consts::PI;

use kiss3d::window::Window;
use kiss3d::light::Light;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use alga::general::SubsetOf;

use knot::joint::{at_angles, RelativeJoint};
use knot::defaults;
use knot::defaults::continuous_optimization::{COST_PARAMS, RATE, REPULSION, REPULSION_EXPONENT,
                                              REPULSION_STRENGTH, CURVE_9_40_CHAIN_SIZE};
use knot::visualize::joint_render::add_joints;
use knot::symmetry::symmetries;
use knot::geometries::curve_9_40;
use knot::continuous_optimize::RepulsionChain;

const TAU: f64 = 2.0 * PI;

const DEBUG_ANGLES: bool = false;

fn main() {
    let mut chain = RepulsionChain::new(
        curve_9_40::chain(CURVE_9_40_CHAIN_SIZE, 1.0, COST_PARAMS, RATE),
        3,
        REPULSION_EXPONENT,
        REPULSION_STRENGTH,
    );

    let mut window = Window::new("Continuous Optimization");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(
        window.scene_mut(),
        &chain.spec,
        chain.joints.len() * 6
        // debug:
        + if DEBUG_ANGLES {
            chain.joints.len()
        } else {
            0
        },
    );

    let mut step = 0;

    let mut selected = 0;

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
            chain.optimize();
            step += 1;

            if REPULSION {
                chain.repulse();
            }
        }
    }
}
