#![feature(conservative_impl_trait)]

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
use kiss3d::camera::ArcBall;
use kiss3d::scene::SceneNode;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Point3, Translation3, Matrix3, Rotation3, UnitQuaternion, Vector3};
use alga::general::SubsetOf;

use knot::joint::JointSpec;
use knot::defaults;
use knot::isometry_adjust as iso_adj;
use knot::continuous_optimize::{Leg, PhantomJoint, Chain};
use knot::visualize::joint_render::add_joints;
use knot::cost::{CostParams, Thresholds};
use knot::symmetry::{adjacent_symmetry, symmetries};

fn from_curve<F: Fn(f64) -> Point3<f64>>(
    count: usize,
    start: f64,
    end: f64,
    f: F,
) -> impl Iterator<Item = Isometry3<f64>> {
    let step = (end - start) / ((count - 1) as f64);
    return (0..count).map(move |i| {
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
    });
}

fn main() {
    let cost_params = CostParams {
        dist_weight: 1.0,
        axis_weight: 1.0,
        locking_weight: 0.01, // 0.05, // 1.0,
        thresholds: Thresholds {
            dist_for_axis: INFINITY, // 4.0,
            axis_for_locking: INFINITY, // 0.2,
        },
    };

    let chain_size = 5;
    let mut chain = Chain {
        spec: defaults::joint_spec(),
        num_angles: defaults::NUM_ANGLES,
        pre_phantom: PhantomJoint {
            symmetry: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI).to_superset(), // Isometry3::identity(),
            index: 0,
            leg: Leg::Incoming,
        },
        post_phantom: PhantomJoint {
            symmetry: adjacent_symmetry(3, 2).to_superset(),
            index: chain_size - 1,
            leg: Leg::Outgoing,
        },
        cost_params: cost_params,
        descent_rate: 0.05, // 0.01,
        steps: iso_adj::Steps::new_uniform(0.00001 /* 0.01 */),
        joints: from_curve(chain_size, 0.0, 2.0 * PI / 6.0, |t| {
            Point3::new(
                4.0 * (t.sin() + 2.0 * (2.0 * t).sin()),
                4.0 * (t.cos() - 2.0 * (2.0 * t).cos()),
                4.0 * (-(3.0 * t).sin()),
            )
        }).collect(),
    };

    let mut window = Window::new("Continuous Optimization");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(window.scene_mut(), &chain.spec, chain.joints.len() * 6);

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
                        }
                        Key::Left => {
                            chain.cost_params.locking_weight /= 1.5;
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
        }
        for _ in 0..10 {
            chain.optimize();
        }
    }
}
