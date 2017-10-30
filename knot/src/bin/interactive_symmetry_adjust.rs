#![feature(conservative_impl_trait)]

extern crate kiss3d;
extern crate alga;
extern crate nalgebra;
extern crate glfw;

extern crate knot;
extern crate serde;
extern crate serde_json;

use std::env::args;
use std::process::exit;

use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::camera::ArcBall;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Point3};
use alga::general::SubsetOf;

use knot::joint::{discrete_angles, at_angles};
use knot::symmetry::symmetries;
use knot::symmetry_adjust;
use knot::visualize::joint_render::add_joints;
use knot::defaults::{COST_PARAMS, OPTIMIZATION_PARAMS, NUM_ANGLES, INITIAL_SYMMETRY_ADJUST,
                     SYMMETRY_COUNT, joint_spec};

fn main() {
    let spec = joint_spec();
    let rel_joint_angles: Vec<i32> = match args().nth(1) {
        Some(angles_json) => {
            serde_json::from_str(&angles_json).unwrap_or_else(|_| {
                eprintln!("Could not parse angles");
                exit(1);
            })
        }
        None => vec![2, 0, 15, 0, 0],
    };

    let joint_transforms = at_angles(
        discrete_angles(spec, NUM_ANGLES, rel_joint_angles.iter().cloned()),
        Isometry3::identity(),
    ).collect::<Vec<_>>();

    let symmetry_transforms = symmetries(SYMMETRY_COUNT).collect::<Vec<_>>();

    let last_joint_out = joint_transforms.last().unwrap() * spec.origin_to_out();

    let problem =
        symmetry_adjust::Problem::new(COST_PARAMS, last_joint_out, NUM_ANGLES, SYMMETRY_COUNT);

    let mut window = Window::new("Trefoil");
    window.set_light(Light::StickToCamera);
    let mut camera = ArcBall::new(Point3::new(0.0, 0.0, 20.0), Point3::origin());

    let mut nodes = add_joints(
        window.scene_mut(),
        &spec,
        0.5,
        rel_joint_angles.len() * (SYMMETRY_COUNT as usize) * 2,
    );

    let cost_joint_0_idx = rel_joint_angles.len() - 1;
    let cost_joint_1_idx = rel_joint_angles.len() * 4 - 1;

    nodes[cost_joint_0_idx].set_color(0.0, 1.0, 0.0);
    nodes[cost_joint_1_idx].set_color(1.0, 0.0, 1.0);

    let mut adjust = INITIAL_SYMMETRY_ADJUST;

    let opt_params = symmetry_adjust::OptimizationParams {
        radius_step: OPTIMIZATION_PARAMS.radius_step,
        radial_angle_step: OPTIMIZATION_PARAMS.radial_angle_step,
        // Intentionally slow descent rate for easy visualization
        descent_rate: OPTIMIZATION_PARAMS.descent_rate / 30.0,
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