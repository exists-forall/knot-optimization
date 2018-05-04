extern crate kiss3d;
extern crate alga;
extern crate nalgebra;
extern crate glfw;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::fs::File;
use std::process::exit;
use std::env::args;

use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::camera::ArcBall;

use nalgebra::Point3;
use alga::general::SubsetOf;

use knot::visualize::joint_render::{add_joints, Style};
use knot::report::{JointsParity, Transform, KnotGeometry};

fn from_hex_color(r: u8, g: u8, b: u8) -> (f32, f32, f32) {
    ((r as f32) / 255.0, (g as f32) / 255.0, (b as f32) / 255.0)
}

fn main() {
    let filename = args().nth(1).unwrap_or_else(|| {
        eprintln!("Expected a single input file");
        exit(1);
    });
    let file = File::open(&filename).unwrap_or_else(|_| {
        eprintln!("Could not open file {}", filename);
        exit(1);
    });
    let geometry: KnotGeometry = serde_json::from_reader(file).unwrap_or_else(|_| {
        eprintln!("Could not parse input file");
        exit(1);
    });

    let mut window = Window::new("Trefoil");
    window.set_light(Light::StickToCamera);
    {
        let (r, g, b) = from_hex_color(0x11, 0x11, 0x11);
        window.set_background_color(r, g, b);
    }

    let mut group = window.scene_mut().add_group();

    let total_nodes = match geometry.parity {
        JointsParity::Even => geometry.symmetries.len() * geometry.transforms.len(),
        JointsParity::Odd => {
            geometry.symmetries.len() * geometry.transforms.len() - geometry.symmetries.len() / 2
        }
    };

    let mut nodes = add_joints(
        &mut group,
        &geometry.joint_spec,
        geometry.num_angles,
        total_nodes,
        Style::Flat,
    );

    {
        let mut node_i = 0;
        for (i, symm) in geometry
            .symmetries
            .iter()
            .map(Transform::to_isometry)
            .enumerate()
        {
            for (j, trans) in geometry
                .transforms
                .iter()
                .map(Transform::to_isometry)
                .enumerate()
            {
                match geometry.parity {
                    JointsParity::Even => {}
                    JointsParity::Odd => {
                        if i % 2 == 0 && j == 0 {
                            continue;
                        }
                    }
                }

                nodes[node_i].set_local_transformation((symm * trans).to_superset());
                node_i += 1;
            }
        }
    }

    let mut camera = ArcBall::new(Point3::new(0.0, 0.0, 25.0), Point3::origin());

    while window.render_with_camera(&mut camera) {}
}
