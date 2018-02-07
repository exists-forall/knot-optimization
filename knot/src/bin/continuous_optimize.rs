#![feature(conservative_impl_trait)]
#![feature(type_ascription)]

extern crate kiss3d;
extern crate alga;
extern crate nalgebra;
extern crate glfw;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::f64::consts::PI;
use std::f64::INFINITY;

use kiss3d::window::Window;
use kiss3d::light::Light;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Point3, Translation3, Matrix3, Rotation3, UnitQuaternion, Vector3};
use alga::general::SubsetOf;

use knot::joint::{at_angles, RelativeJoint};
use knot::defaults;
use knot::isometry_adjust as iso_adj;
use knot::continuous_optimize::{Leg, PhantomJoint, Chain};
use knot::visualize::joint_render::add_joints;
use knot::cost::{CostParams, Thresholds};
use knot::symmetry::{adjacent_symmetry, symmetries};

const TAU: f64 = 2.0 * PI;

const SHOW_COST_STEPS: bool = true;

fn from_curve<F: Fn(f64) -> Point3<f64>>(
    count: usize,
    start: f64,
    end: f64,
    f: F,
) -> impl Iterator<Item = Isometry3<f64>> {
    let step = (end - start) / ((count - 1) as f64);
    (0..count).map(move |i| {
        let t = (i as f64) * step + start;
        let dt = 0.01;
        let f_t = f(t);
        let f_plus = f(t + dt);
        let f_minus = f(t - dt);
        let vel = (f_plus - f_t) / dt;
        let accel = ((f_minus - f_t) + (f_plus - f_t)) / (dt * dt);
        let frame_y = vel.normalize();
        let frame_x = -(accel - accel.dot(&vel) / (vel.dot(&vel)) * vel).normalize();
        let frame_z = frame_x.cross(&frame_y);
        let frame = Matrix3::from_columns(&[frame_x, frame_y, frame_z]);
        let frame_rot =
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(frame));
        let frame_trans = Translation3 { vector: f_t.coords };
        let frame_iso = Isometry3::from_parts(frame_trans, frame_rot);
        frame_iso
    })
}

#[allow(dead_code)]
fn from_discrete_curve<I: Iterator<Item = Point3<f64>>>(
    mut i: I,
    scale: f64,
) -> impl Iterator<Item = Isometry3<f64>> {
    let mut prev = i.next().unwrap();
    let mut curr = i.next().unwrap();
    i.map(move |next| {
        let incoming = curr - prev;
        let outgoing = next - curr;
        let normal = -outgoing.cross(&incoming);

        let frame_x = incoming.cross(&normal).normalize();
        let frame_y = incoming.normalize();
        let frame_z = normal.normalize();

        let frame = Matrix3::from_columns(&[frame_x, frame_y, frame_z]);
        let frame_rot =
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(frame));
        let frame_trans = Translation3 { vector: curr.coords * scale };
        let frame_iso = Isometry3::from_parts(frame_trans, frame_rot);

        prev = curr;
        curr = next;

        frame_iso
    })
}

fn spherical(theta: f64, phi: f64, rho: f64) -> Point3<f64> {
    Point3::new(
        rho * theta.cos() * phi.cos(),
        rho * theta.sin() * phi.cos(),
        rho * phi.sin(),
    )
}

const DEBUG_ANGLES: bool = false;

fn main() {
    let cost_params = CostParams {
        dist_weight: 1.0,
        axis_weight: 3.0,
        locking_weight: 0.01,
        thresholds: Thresholds {
            dist_for_axis: INFINITY,
            axis_for_locking: INFINITY,
        },
    };

    let chain_size = 8; // 9;

    let mut chain = Chain {
        spec: defaults::joint_spec(),
        num_angles: defaults::NUM_ANGLES,
        pre_phantom: PhantomJoint {
            symmetry: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI).to_superset(),
            index: 0,
            leg: Leg::Incoming,
        },
        post_phantom: PhantomJoint {
            symmetry: adjacent_symmetry(3, 1).to_superset(),
            index: chain_size - 1,
            leg: Leg::Outgoing,
        },
        cost_params: cost_params,
        descent_rate: 0.0005,
        steps: iso_adj::Steps::new_uniform(0.000001),
        joints: from_curve(chain_size, 0.0, 1.0, |t| {
            let theta = 2.0 * TAU / 3.0 * t;
            let phi = 1.0 * (TAU / 2.0 * t).sin();
            let rho = 7.0 + 2.5 * (TAU / 2.0 * t).cos();
            spherical(theta, phi, rho)
        }).collect(),
    };

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
        for _ in 0..30 {
            let cost = chain.optimize();
            if SHOW_COST_STEPS {
                println!("{{{}, {}}},", step, cost);
            }
            step += 1;

            // experimental repulsive force
            let forces = (0..chain.joints.len())
                .map(|i| {
                    let mut force = Vector3::new(0.0, 0.0, 0.0);
                    for (sym_i, sym) in symmetries(3).enumerate() {
                        for j in 0..chain.joints.len() {
                            if !(sym_i == 0 && i == j) {
                                let diff = chain.joints[i].translation.vector -
                                    (sym * chain.joints[j]).translation.vector;
                                force += diff / diff.norm().powi(4);
                            }
                        }
                    }
                    force
                })
                .collect::<Vec<_>>();

            for (force, joint) in forces.iter().zip(chain.joints.iter_mut()) {
                joint.translation.vector += force * 0.00001;
            }
        }
    }
}
