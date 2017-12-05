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

use knot::joint::{discrete_angles, at_angles};
use knot::symmetry::symmetries;
use knot::visualize::joint_render::add_joints;
use knot::report::{KnotReports, JointsParity};

struct ReportsView {
    window: Window,
    camera: ArcBall,
    reports: KnotReports,
    viewed: Option<SceneNode>,
}

fn from_hex_color(r: u8, g: u8, b: u8) -> (f32, f32, f32) {
    ((r as f32) / 255.0, (g as f32) / 255.0, (b as f32) / 255.0)
}

impl ReportsView {
    fn new(window: Window, reports: KnotReports) -> Self {
        ReportsView {
            window,
            camera: ArcBall::new(Point3::new(0.0, 0.0, 25.0), Point3::origin()),
            reports,
            viewed: None,
        }
    }

    fn color(&self, i: usize) -> (f32, f32, f32) {
        let colors = [
            // From http://clrs.cc/
            from_hex_color(0xFF, 0x41, 0x36), // red
            from_hex_color(0xFF, 0x85, 0x1B), // orange
            from_hex_color(0xFF, 0xDC, 0x00), // yellow
            from_hex_color(0x2E, 0xCC, 0x40), // green
            from_hex_color(0x00, 0x74, 0xD9), // blue
            from_hex_color(0xB1, 0x0D, 0xC9), // purple
        ];

        colors[i % colors.len()]
    }

    fn view_report(&mut self, i: usize) {
        if i >= self.reports.knots.len() {
            return;
        }

        let report = &self.reports.knots[i];

        println!("Viewing report {}", i);
        println!("{:#?}", report);

        match self.viewed {
            Some(ref mut prev) => self.window.remove(prev),
            None => {}
        }

        let mut group = self.window.scene_mut().add_group();

        let mut nodes = add_joints(
            &mut group,
            &self.reports.joint_spec,
            (self.reports.symmetry_count as usize) *
                (2 * report.angles.len() +
                     match self.reports.parity {
                         JointsParity::Even => 0,
                         JointsParity::Odd => 1,
                     }),
        );
        let joint_transforms = at_angles(
            discrete_angles(
                self.reports.joint_spec,
                self.reports.num_angles,
                report.angles.iter().cloned(),
            ),
            match self.reports.parity {
                JointsParity::Even => Isometry3::identity(),
                JointsParity::Odd => {
                    self.reports.joint_spec.origin_to_symmetric() *
                        self.reports.joint_spec.origin_to_out()
                }
            },
        ).collect::<Vec<_>>();

        let adjust_trans = report.symmetry_adjust.transform();

        let mut node_i = 0;
        let mut first_in_horseshoe = true;
        for sym_trans in symmetries(self.reports.symmetry_count) {
            match self.reports.parity {
                JointsParity::Even => {}
                JointsParity::Odd => {
                    if first_in_horseshoe {
                        nodes[node_i].set_local_transformation(
                            (sym_trans * adjust_trans *
                                 self.reports.joint_spec.origin_to_symmetric())
                                .to_superset(),
                        );
                        nodes[node_i].set_color(0.2, 0.2, 0.2);
                        node_i += 1;
                    }
                }
            }
            first_in_horseshoe = !first_in_horseshoe;
            for (joint_i, joint_trans) in joint_transforms.iter().enumerate() {
                nodes[node_i].set_local_transformation(
                    (sym_trans * adjust_trans * joint_trans).to_superset(),
                );
                let (r, g, b) = self.color(joint_i);
                nodes[node_i].set_color(r, g, b);
                node_i += 1;
            }
        }

        self.viewed = Some(group);
    }

    fn render(&mut self) -> bool {
        self.window.render_with_camera(&mut self.camera)
    }
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
    {
        let (r, g, b) = from_hex_color(0x11, 0x11, 0x11);
        window.set_background_color(r, g, b);
    }

    let mut report_i = 0;
    let mut reports_view = ReportsView::new(window, reports);

    reports_view.view_report(0);

    while reports_view.render() {
        for event in reports_view.window.events().iter() {
            match event.value {
                WindowEvent::Key(_, _, Action::Release, _) => {}
                WindowEvent::Key(code, _, _, _) => {
                    match code {
                        Key::Left => {
                            if report_i > 0 {
                                report_i -= 1;
                                reports_view.view_report(report_i);
                            }
                        }
                        Key::Right => {
                            if report_i < reports_view.reports.knots.len() - 1 {
                                report_i += 1;
                                reports_view.view_report(report_i);
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
