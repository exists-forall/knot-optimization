use std::f32::consts::PI;
use std::rc::Rc;
use std::cell::RefCell;

use nalgebra::{Vector3, UnitQuaternion, Point3, Translation3};
use kiss3d::scene::SceneNode;
use kiss3d::resource::Mesh;

use knot::joint::JointSpec;

/// Count from `start` to `end` in `count` equally-spaced steps.
fn float_steps_inclusive(start: f32, end: f32, count: u32) -> impl Iterator<Item = f32> {
    let step = (end - start) / ((count - 1) as f32);
    (0..count).map(move |i| start + step * (i as f32))
}

/// Given a sequence of vertex indices [v0, v1, v2, v3, ...], tesselate them into triangles of the
/// form [(v0, v1, v2), (v1, v3, v2), ...].  This is called a [triangle
/// strip](https://en.wikipedia.org/wiki/Triangle_strip).
fn tesselate_strip<I: Iterator<Item = u32>>(mut vertices: I) -> impl Iterator<Item = Point3<u32>> {
    let mut v0_opt = vertices.next();
    let mut v1_opt = vertices.next();
    let mut reverse_winding = false;
    vertices.map(move |v2| {
        let v0 = v0_opt.expect(
            "Vertex iterator should not yield 'Some' after yielding 'None'",
        );
        let v1 = v1_opt.expect(
            "Vertex iterator should not yield 'Some' after yielding 'None'",
        );
        let tri = if reverse_winding {
            Point3::new(v1, v0, v2)
        } else {
            Point3::new(v0, v1, v2)
        };
        reverse_winding = !reverse_winding;
        v0_opt = v1_opt;
        v1_opt = Some(v2);
        tri
    })
}

/// Create a `kiss3d` `Mesh` for a sliced cylinder oriented along the y axis, with its base centered
/// at the origin, of average height `h`, radius `r`, and its bottom and top faces rotated at angles
/// of `bottom_angle` and `top_angle`, respectively, about the z axis.
fn sliced_cylinder_mesh(r: f32, h: f32, bottom_angle: f32, top_angle: f32, res: u32) -> Mesh {
    let mut coords = Vec::with_capacity(2 * res as usize);
    let mut normals = Vec::with_capacity(2 * res as usize);

    let bottom_slope = bottom_angle.tan();
    let top_slope = top_angle.tan();
    for theta in float_steps_inclusive(0.0, 2.0 * PI, res) {
        let cos = theta.cos();
        let sin = theta.sin();

        let x = cos * r;
        let z = sin * r;
        let bottom_coord = Point3::new(x, bottom_slope * x, z);
        let top_coord = Point3::new(x, h + top_slope * x, z);
        coords.push(bottom_coord);
        coords.push(top_coord);

        let normal = Vector3::new(cos, 0.0, sin);
        normals.push(normal);
        normals.push(normal);
    }

    let faces = tesselate_strip(0..2 * res).collect();

    Mesh::new(coords, faces, Some(normals), None, false)
}

/// Add `count` scenenodes to the given `root` `SceneNode` representing joints with geometry given
/// by `joint_spec` using cylinders of radius `radius`. Initially, the joint `SceneNodes`s are all
/// centered at the origin.
pub fn add_joints(
    root: &mut SceneNode,
    joint_spec: &JointSpec,
    radius: f32,
    count: usize,
) -> Vec<SceneNode> {

    let cyl0_mesh = Rc::new(RefCell::new(sliced_cylinder_mesh(
        radius,
        joint_spec.dist_in() as f32,
        0.0,
        joint_spec.bend_angle() as f32 / 2.0,
        20,
    )));

    let cyl1_mesh = Rc::new(RefCell::new(sliced_cylinder_mesh(
        radius,
        joint_spec.dist_out() as f32,
        -joint_spec.bend_angle() as f32 / 2.0,
        0.0,
        20,
    )));

    let mut color = false;

    (0..count)
        .map(|_| {
            let mut group = root.add_group();

            let mut cyl0 = group.add_mesh(cyl0_mesh.clone(), Vector3::new(1.0, 1.0, 1.0));
            let mut cyl1 = group.add_mesh(cyl1_mesh.clone(), Vector3::new(1.0, 1.0, 1.0));

            cyl0.set_local_translation(Translation3::new(0.0, -joint_spec.dist_in() as f32, 0.0));

            cyl1.set_local_rotation(UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                joint_spec.bend_angle() as f32,
            ));

            if color {
                group.set_color(1.0, 0.0, 0.0);
            } else {
                group.set_color(1.0, 1.0, 1.0);
            }

            color = !color;

            group
        })
        .collect()
}
