#![feature(slice_patterns)]

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
use kiss3d::scene::SceneNode;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Point3, Translation3, Vector3, Matrix3, Rotation3, UnitQuaternion};
use alga::general::SubsetOf;

use knot::joint::{discrete_symmetric_angles, at_angles};
use knot::symmetry::symmetries;
use knot::visualize::joint_render::add_joints;
use knot::report::{KnotReports, JointsParity, RotationMatrix, Transform, KnotGeometry};

fn from_hex_color(r: u8, g: u8, b: u8) -> (f32, f32, f32) {
    ((r as f32) / 255.0, (g as f32) / 255.0, (b as f32) / 255.0)
}

fn array_to_vec3(arr: [f64; 3]) -> Vector3<f64> {
    Vector3::new(arr[0], arr[1], arr[2])
}

fn trans_to_isometry(trans: Transform) -> Isometry3<f64> {
    let [tx, ty, tz] = trans.translation;
    let translation = Translation3::new(tx, ty, tz);
    let rotation_mat = Rotation3::from_matrix_unchecked(Matrix3::from_columns(
        &[
            array_to_vec3(trans.rotation.col_x),
            array_to_vec3(trans.rotation.col_y),
            array_to_vec3(trans.rotation.col_z),
        ],
    ));
    let quaternion = UnitQuaternion::from_rotation_matrix(&rotation_mat);
    Isometry3::from_parts(translation, quaternion)
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

    let mut nodes = add_joints(
        &mut group,
        &geometry.joint_spec,
        (geometry.symmetries.len()) * geometry.transforms.len(),
    );

    {
        let mut node_i = 0;
        for symm in geometry.symmetries.iter().cloned().map(trans_to_isometry) {
            for trans in geometry.transforms.iter().cloned().map(trans_to_isometry) {
                nodes[node_i].set_local_transformation((symm * trans).to_superset());
                node_i += 1;
            }
        }
    }

    let mut camera = ArcBall::new(Point3::new(0.0, 0.0, 25.0), Point3::origin());

    while window.render_with_camera(&mut camera) {}
}
