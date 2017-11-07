#![feature(conservative_impl_trait)]

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

use nalgebra::{Isometry3, Point3};
use alga::general::SubsetOf;

use knot::joint::{JointSpec, discrete_angles, at_angles};
use knot::symmetry::symmetries;
use knot::visualize::joint_render::add_joints;
use knot::report::{KnotReport, KnotReports, JointsParity};

fn view_report(
    root: &mut SceneNode,
    num_angles: u32,
    spec: &JointSpec,
    symmetry_count: u32,
    parity: JointsParity,
    report: &KnotReport,
) -> SceneNode {
    let mut group = root.add_group();

    let mut nodes = add_joints(
        &mut group,
        spec,
        (symmetry_count as usize) *
            (2 * report.angles.len() +
                 match parity {
                     JointsParity::Even => 0,
                     JointsParity::Odd => 1,
                 }),
    );
    let joint_transforms = at_angles(
        discrete_angles(*spec, num_angles, report.angles.iter().cloned()),
        match parity {
            JointsParity::Even => Isometry3::identity(),
            JointsParity::Odd => spec.origin_to_symmetric() * spec.origin_to_out(),
        },
    ).collect::<Vec<_>>();

    let adjust_trans = report.symmetry_adjust.transform();

    let mut node_i = 0;
    let mut first_in_horseshoe = true;
    for sym_trans in symmetries(symmetry_count) {
        match parity {
            JointsParity::Even => {}
            JointsParity::Odd => {
                if first_in_horseshoe {
                    nodes[node_i].set_local_transformation(
                        (sym_trans * adjust_trans * spec.origin_to_symmetric())
                            .to_superset(),
                    );
                    nodes[node_i].set_color(0.2, 0.2, 0.2);
                    node_i += 1;
                }
            }
        }
        first_in_horseshoe = !first_in_horseshoe;
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
    let reports: KnotReports = serde_json::from_reader(file).unwrap_or_else(|_| {
        eprintln!("Could not parse input file");
        exit(1);
    });
    if reports.knots.len() == 0 {
        eprintln!("No reports to view");
        exit(1);
    }
    println!("Loaded {} reports", reports.knots.len());

    let mut window = Window::new("Trefoil");
    window.set_light(Light::StickToCamera);
    let mut camera = ArcBall::new(Point3::new(0.0, 0.0, 25.0), Point3::origin());

    let mut report_i = 0;

    let mut knot_node = view_report(
        window.scene_mut(),
        reports.num_angles,
        &reports.joint_spec,
        reports.symmetry_count,
        reports.parity,
        &reports.knots[report_i],
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
                                    reports.num_angles,
                                    &reports.joint_spec,
                                    reports.symmetry_count,
                                    reports.parity,
                                    &reports.knots[report_i],
                                );
                                println!("Viewing report {}", report_i);
                                println!("{:#?}", &reports.knots[report_i]);
                            }
                        }
                        Key::Right => {
                            if report_i < reports.knots.len() - 1 {
                                report_i += 1;
                                window.remove(&mut knot_node);
                                knot_node = view_report(
                                    window.scene_mut(),
                                    reports.num_angles,
                                    &reports.joint_spec,
                                    reports.symmetry_count,
                                    reports.parity,
                                    &reports.knots[report_i],
                                );
                                println!("Viewing report {}", report_i);
                                println!("{:#?}", &reports.knots[report_i]);
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
