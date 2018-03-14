#![feature(conservative_impl_trait)]

extern crate alga;
extern crate nalgebra;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::fs::File;
use std::process::exit;
use std::env::args;

use nalgebra::{Isometry3, Point3, Vector3, UnitQuaternion, Rotation3};
use alga::general::SubsetOf;

use knot::joint::{discrete_symmetric_angles, at_angles};
use knot::symmetry::symmetries;
use knot::visualize::joint_render::add_joints;
use knot::report::{KnotReports, JointsParity, RotationMatrix, Transform, KnotGeometry};

fn vec3_to_array(v: Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn isometry_to_serializable(iso: Isometry3<f64>) -> Transform {
    Transform {
        translation: vec3_to_array(iso.translation.vector),
        rotation: RotationMatrix {
            col_x: vec3_to_array(iso * Vector3::x_axis().to_superset()),
            col_y: vec3_to_array(iso * Vector3::y_axis().to_superset()),
            col_z: vec3_to_array(iso * Vector3::z_axis().to_superset()),
        },
    }
}

fn main() {
    let filename = args().nth(1).unwrap_or_else(|| {
        eprintln!("Expected a single input file");
        exit(1);
    });
    let index = args()
        .nth(2)
        .unwrap_or_else(|| {
            eprintln!("Expected a result index");
            exit(1);
        })
        .parse::<usize>()
        .unwrap_or_else(|_| {
            eprintln!("Index must be an integer");
            exit(1)
        });
    let file = File::open(&filename).unwrap_or_else(|_| {
        eprintln!("Could not open file {}", filename);
        exit(1);
    });
    let reports: KnotReports = serde_json::from_reader(file).unwrap_or_else(|_| {
        eprintln!("Could not parse input file");
        exit(1);
    });

    if !(index < reports.knots.len()) {
        eprintln!(
            "Index out of bounds -- only {} reports",
            reports.knots.len()
        );
        exit(1);
    }

    let knot = &reports.knots[index];

    let mut isometries = Vec::new();

    match reports.parity {
        JointsParity::Even => {}
        JointsParity::Odd => isometries.push(reports.joint_spec.origin_to_symmetric()),
    }

    isometries.extend(at_angles(
        discrete_symmetric_angles(
            reports.joint_spec,
            reports.num_angles,
            reports.parity,
            knot.angles.iter().cloned().map(|angle| angle as i32),
        ),
        match reports.parity {
            JointsParity::Even => Isometry3::identity(),
            JointsParity::Odd => {
                reports.joint_spec.origin_to_symmetric() * reports.joint_spec.origin_to_out()
            }
        },
    ));

    let transforms = isometries
        .iter()
        .cloned()
        .map(isometry_to_serializable)
        .collect::<Vec<_>>();

    let symms = symmetries(reports.symmetry_count)
        .map(|quat| isometry_to_serializable(quat.to_superset()))
        .collect::<Vec<_>>();

    let geometry = KnotGeometry {
        joint_spec: reports.joint_spec,
        num_angles: reports.num_angles,
        cost_params: reports.cost_params,
        parity: reports.parity,
        symmetries: symms,
        transforms,
    };

    println!("{}", serde_json::to_string_pretty(&geometry).unwrap());
}
