extern crate kiss3d;

use super::kiss3d::light::Light;
use super::kiss3d::window::Window;

use alga::general::SubsetOf;
use defaults;
use defaults::curve_visualize::VISUALIZE_PARAMS;
use super::input_curve::chain;
use joint::Point;
use optimize_tools::{RepulsionChain};
use symmetry::{symmetries, symmetries_with_skip};
use visualize::joint_render::{add_joints, Style};


pub fn visualize_bspline<F: Fn() -> bspline::BSpline<Point>>(
    bspline_generator: F, // Function returning the bspline. See the bspline package.
    sym_number: u32, // Number of identical sub-chains the knot should be split into.
    scale: f32, // Scalar multiple of size of bspline. Take a guess and adjust until it looks right.
) {
    let our_chain = RepulsionChain::new(
        chain(
            scale,
            VISUALIZE_PARAMS,
            0.0, // Return to initial weight
            0.0, // Descent rate
            bspline_generator,
            sym_number,
            defaults::cyl_spec(),
        ),
        symmetries(sym_number).map(|quat| quat.to_superset()).collect(),
        0, // Repulsion exponent
        0.0, // Repulsion strength
        0.0, // Max repulsion strength
    );

    let mut window = Window::new("Continuous Optimization");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(
        window.scene_mut(), // Scene
        &our_chain.spec, // JointSpec
        3, // Number of angles
        our_chain.joints.len() * 6, // Number of joints
        Style::Smooth,
    );

    while window.render() {
        {
            let mut i = 0;
            for sym in symmetries_with_skip(3, 2) {
                for &joint in &our_chain.joints {
                    nodes[i].set_color(0.5, 0.5, 0.5);
                    nodes[i].set_local_transformation((sym * joint).to_superset());
                    i += 1;
                }
            }
        }
    }
}
