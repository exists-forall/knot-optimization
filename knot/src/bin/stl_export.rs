extern crate alga;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::env::args;
use std::fs::File;
use std::io;
use std::process::exit;

use alga::general::SubsetOf;
use nalgebra::{Isometry3, Point3, Vector3};

use knot::report::{JointsParity, KnotGeometry, Transform};
use knot::visualize::joint_render::sliced_cylinder_faces;

type Face = [(Point3<f32>, Vector3<f32>); 3];

fn transform_face(trans: Isometry3<f32>, mut face: Face) -> Face {
    for &mut (ref mut coord, ref mut normal) in &mut face {
        *coord = trans * *coord;
        *normal = trans * *normal;
    }
    face
}

fn write_stl_face(writer: &mut io::Write, face: &Face) -> io::Result<()> {
    let avg_normal = nalgebra::normalize(&(face[0].1 + face[1].1 + face[2].1));
    writeln!(
        writer,
        "face normal {:e} {:e} {:e}",
        avg_normal.x, avg_normal.y, avg_normal.z
    )?;
    writeln!(writer, "  outer loop")?;
    for &(vertex, _) in face {
        writeln!(
            writer,
            "    vertex {:e} {:e} {:e}",
            vertex.x, vertex.y, vertex.z,
        )?;
    }
    writeln!(writer, "  endloop")?;
    writeln!(writer, "endfacet")?;
    Ok(())
}

fn write_stl(writer: &mut io::Write, faces: &[Face]) -> io::Result<()> {
    writeln!(writer, "solid knot")?;
    for face in faces {
        write_stl_face(writer, face)?;
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

    let faces_template = sliced_cylinder_faces(geometry.num_angles as u16, &geometry.joint_spec);

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

    write_stl(&mut io::stdout(), &faces).expect("IO Error");
}
