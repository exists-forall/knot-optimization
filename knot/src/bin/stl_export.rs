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
use std::io;

use nalgebra::{Isometry3, Point3, Translation3, Vector3, Matrix3, Rotation3, UnitQuaternion};
use alga::general::SubsetOf;

use knot::joint::{discrete_symmetric_angles, at_angles};
use knot::symmetry::symmetries;
use knot::visualize::joint_render::sliced_cylinder_faces;
use knot::report::{KnotReports, JointsParity, RotationMatrix, Transform, KnotGeometry};

type Face = [(Point3<f32>, Vector3<f32>); 3];

fn transform_face(trans: Isometry3<f32>, mut face: Face) -> Face {
    for &mut (ref mut coord, ref mut normal) in &mut face {
        *coord = trans * *coord;
        *normal = trans * *normal;
    }
    face
}

fn write_STL_face(writer: &mut io::Write, face: &Face) -> io::Result<()> {
    let avg_normal = nalgebra::normalize(&(face[0].1 + face[1].1 + face[2].1));
    writeln!(
        writer,
        "face normal {:e} {:e} {:e}",
        avg_normal.x,
        avg_normal.y,
        avg_normal.z
    )?;
    writeln!(writer, "  outer loop")?;
    for &(vertex, _) in face {
        writeln!(
            writer,
            "    vertex {:e} {:e} {:e}",
            vertex.x,
            vertex.y,
            vertex.z,
        )?;
    }
    writeln!(writer, "  endloop")?;
    writeln!(writer, "endfacet")?;
    Ok(())
}

fn write_STL(writer: &mut io::Write, faces: &[Face]) -> io::Result<()> {
    writeln!(writer, "solid knot")?;
    for face in faces {
        write_STL_face(writer, face)?;
    }
    writeln!(writer, "endsolid")?;
    Ok(())
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

    let faces_template = sliced_cylinder_faces(geometry.num_angles, &geometry.joint_spec);

    let mut faces: Vec<Face> = Vec::new();

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

            for face in &faces_template {
                faces.push(transform_face((symm * trans).to_superset(), *face));
            }
        }
    }

    write_STL(&mut io::stdout(), &faces).expect("IO Error");
}
