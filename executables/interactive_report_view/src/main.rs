#![feature(conservative_impl_trait)]

extern crate kiss3d;
extern crate alga;
extern crate nalgebra;
extern crate glfw;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate serde_json;

extern crate knot;
extern crate knot_visualize;

use std::f64::consts::PI;
use std::fs::File;
use std::process::exit;
use std::env::args;

use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::camera::ArcBall;
use kiss3d::scene::SceneNode;
use glfw::{Action, WindowEvent, Key};

use nalgebra::{Isometry3, Point3};
use alga::general::SubsetOf;

use knot::joint::{JointSpec, discrete_angles, at_angles};
use knot::symmetry::symmetries;
use knot::symmetry_adjust;
use knot_visualize::joint_render::add_joints;

#[derive(Clone, Debug, Deserialize)]
struct KnotReport {
    angles: Vec<i32>,
    adjust_radius: f64,
    adjust_radial_angle: f64,
    cost: f64,
}

fn view_report(
    root: &mut SceneNode,
    num_angles: u32,
    spec: &JointSpec,
    symmetry_count: u32,
    report: &KnotReport,
) -> SceneNode {
    let mut group = root.add_group();

    let mut nodes = add_joints(&mut group, spec, 0.5, report.angles.len() * 6);
    let joint_transforms = at_angles(
        discrete_angles(*spec, num_angles, report.angles.iter().cloned()),
        Isometry3::identity(),
    ).collect::<Vec<_>>();

    let adjust_vars = symmetry_adjust::Vars {
        radius: report.adjust_radius,
        radial_angle: report.adjust_radial_angle,
    };
    let adjust_trans = adjust_vars.transform();

    let mut node_i = 0;
    for sym_trans in symmetries(symmetry_count) {
        for joint_trans in &joint_transforms {
            nodes[node_i].set_local_transformation(
                (sym_trans * adjust_trans * joint_trans)
                    .to_superset(),
            );
            node_i += 1;
        }
    }

    group
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
    let reports: Vec<KnotReport> = serde_json::from_reader(file).unwrap_or_else(|_| {
        eprintln!("Could not parse input file");
        exit(1);
    });
    if reports.len() == 0 {
        eprintln!("No reports to view");
        exit(1);
    }
    println!("Loaded {} reports", reports.len());

    let spec = JointSpec::new(1.0, 1.0, PI / 6.0);
    let symmetry_count = 3;
    let num_angles = 16;

    let mut window = Window::new("Trefoil");
    window.set_light(Light::StickToCamera);
    let mut camera = ArcBall::new(Point3::new(0.0, 0.0, 25.0), Point3::origin());

    let mut report_i = 0;

    let mut knot_node = view_report(
        window.scene_mut(),
        num_angles,
        &spec,
        symmetry_count,
        &reports[report_i],
    );

    while window.render_with_camera(&mut camera) {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(_, _, Action::Release, _) => {}
                WindowEvent::Key(code, _, _, _) => {
                    match code {
                        Key::Left => {
                            if report_i > 0 {
                                report_i -= 1;
                                window.remove(&mut knot_node);
                                knot_node = view_report(
                                    window.scene_mut(),
                                    num_angles,
                                    &spec,
                                    symmetry_count,
                                    &reports[report_i],
                                );
                                println!("Viewing report {}", report_i);
                                println!("{:#?}", &reports[report_i]);
                            }
                        }
                        Key::Right => {
                            if report_i < reports.len() - 1 {
                                report_i += 1;
                                window.remove(&mut knot_node);
                                knot_node = view_report(
                                    window.scene_mut(),
                                    num_angles,
                                    &spec,
                                    symmetry_count,
                                    &reports[report_i],
                                );
                                println!("Viewing report {}", report_i);
                                println!("{:#?}", &reports[report_i]);
                            }
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }
}
