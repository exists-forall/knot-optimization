#![feature(conservative_impl_trait)]

extern crate kiss3d;
extern crate alga;
extern crate nalgebra;
extern crate glfw;

extern crate knot;
extern crate knot_visualize;

use std::f64::consts::PI;

use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::camera::ArcBall;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Point3};
use alga::general::SubsetOf;

use knot::joint::{JointSpec, discrete_angles, at_angles};
use knot::cost::{CostParams, Thresholds};
use knot::symmetry::symmetries;
use knot::symmetry_adjust;
use knot_visualize::joint_render::add_joints;

fn main() {
    let spec = JointSpec::new(1.0, 1.0, PI / 6.0);
    let symmetry_count = 3;
    let num_angles = 16;
    let rel_joint_angles = [0, 2, 0, 0, -2];

    let joint_transforms = at_angles(
        discrete_angles(spec, num_angles, rel_joint_angles.iter().cloned()),
        Isometry3::identity(),
    ).collect::<Vec<_>>();

    let symmetry_transforms = symmetries(symmetry_count).collect::<Vec<_>>();

    let last_joint_out = joint_transforms.last().unwrap() * spec.origin_to_out();

    let problem = symmetry_adjust::Problem::new(
        CostParams {
            dist_weight: 5.0,
            axis_weight: 1.0,
            locking_weight: 1.0,
            thresholds: Thresholds {
                dist_for_axis: 4.0,
                axis_for_locking: 0.2,
            },
        },
        last_joint_out,
        num_angles,
        symmetry_count,
    );

    let mut window = Window::new("Trefoil");
    window.set_light(Light::StickToCamera);
    let mut camera = ArcBall::new(Point3::new(0.0, 0.0, 20.0), Point3::origin());

    let mut nodes = add_joints(
        window.scene_mut(),
        &spec,
        0.5,
        rel_joint_angles.len() * (symmetry_count as usize) * 2,
    );

    let cost_joint_0_idx = rel_joint_angles.len() - 1;
    let cost_joint_1_idx = rel_joint_angles.len() * 4 - 1;

    nodes[cost_joint_0_idx].set_color(0.0, 1.0, 0.0);
    nodes[cost_joint_1_idx].set_color(1.0, 0.0, 1.0);

    let mut adjust = symmetry_adjust::Vars {
        radius: 10.0,
        radial_angle: PI / 2.0,
    };

    let opt_params = symmetry_adjust::OptimizationParams {
        radius_step: 0.01,
        radial_angle_step: 0.01,
        descent_rate: 0.001,
    };

    loop {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(_, _, Action::Release, _) => {}
                WindowEvent::Key(code, _, _, _) => {
                    match code {
                        Key::Up => adjust.radius += 0.1,
                        Key::Down => adjust.radius -= 0.1,
                        Key::Left => adjust.radial_angle -= 0.1,
                        Key::Right => adjust.radial_angle += 0.1,
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        problem.optimize(&opt_params, &mut adjust);

        {
            let mut node_i = 0;
            let adjust_trans = adjust.transform();
            for sym_trans in &symmetry_transforms {
                for joint_trans in &joint_transforms {
                    nodes[node_i].set_local_transformation(
                        (sym_trans * adjust_trans * joint_trans).to_superset(),
                    );
                    node_i += 1;
                }
            }
        }

        if !window.render_with_camera(&mut camera) {
            break;
        }
    }
}
