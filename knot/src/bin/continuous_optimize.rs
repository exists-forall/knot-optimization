extern crate alga;
extern crate kiss3d;
extern crate nalgebra;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::env::args;
use std::f64::consts::PI;
use std::fs::File;
use std::process::exit;

use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::window::Window;

use alga::general::SubsetOf;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use knot::optimize_tools::{Chain, Leg, PhantomJoint, RepulsionChain};
use knot::defaults;
use knot::defaults::continuous_optimization::{
    COST_PARAMS, MAX_REPULSION_STRENGTH, RATE, REPULSION,
    REPULSION_EXPONENT, REPULSION_STRENGTH, RETURN_TO_INITIAL_WEIGHT,
    CONTINUOUS_PARAMS
};
use knot::geometries::trefoil_curve;
use knot::geometries::from_spline;
use knot::geometries::chinbut_spline;
use knot::isometry_adjust;
use knot::joint::{at_angles, RelativeJoint};
use knot::report::{KnotGeometry, Transform};
use knot::symmetry::{symmetries, symmetries_with_skip};
use knot::visualize::joint_render::{add_joints, Style};

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
        None => RepulsionChain::new(
            from_spline::generic_chain(
                4.0,
                CONTINUOUS_PARAMS,
                RETURN_TO_INITIAL_WEIGHT,
                RATE,
                defaults::joint_spec(),
                chinbut_spline::generate_chinbutspline,
                6,
            ),
            // trefoil_curve::chain(
            //     3.5, // scale
            //     COST_PARAMS,
            //     RETURN_TO_INITIAL_WEIGHT,
            //     RATE,
            //     defaults::joint_spec(),
            // ),
            symmetries(6).map(|quat| quat.to_superset()).collect(),
            REPULSION_EXPONENT,
            REPULSION_STRENGTH,
            MAX_REPULSION_STRENGTH,
        ),
    };

    let mut window = Window::new("Continuous Optimization");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(
        window.scene_mut(), // Scene
        &chain.spec, // JointSpec
        16 as u16, // Number of angles
        chain.joints.len() * 6 // Number of joints
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
            for sym in symmetries_with_skip(3, 2) {
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
