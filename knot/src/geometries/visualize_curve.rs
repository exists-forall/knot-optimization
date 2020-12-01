extern crate kiss3d;

use super::kiss3d::light::Light;
use super::kiss3d::window::Window;
use super::kiss3d::event::{Action, Key, WindowEvent};

use alga::general::SubsetOf;
use defaults;
use defaults::continuous_optimization::{MAX_REPULSION_STRENGTH,
    REPULSION_EXPONENT, REPULSION_STRENGTH, RATE, REPULSION};
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
    let mut our_chain = RepulsionChain::new(
        chain(
            scale,
            VISUALIZE_PARAMS,
            0.0,
            RATE * 5.0,
            bspline_generator,
            sym_number,
            defaults::cyl_spec(),
        ),
        symmetries(sym_number).map(|quat| quat.to_superset()).collect(),
        REPULSION_EXPONENT,
        REPULSION_STRENGTH,
        MAX_REPULSION_STRENGTH,
    );

    let mut window = Window::new("Visualize Knot");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(
        window.scene_mut(), // Scene
        &our_chain.spec, // JointSpec
        12 as u16, // Number of angles
        our_chain.joints.len() * 6, // Number of joints
        Style::Smooth,
    );

    while window.render() {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(_, Action::Release, _) => {}
                WindowEvent::Key(code, _, _) => match code {
                    Key::Space => {
                        window.close()
                    }
                    _ => {}
                },
                _ => {}
            }
        }
        {
            let mut i = 0;
            let mut first = true;
            for sym in symmetries_with_skip(3, 2) {
                for &joint in &our_chain.joints {
                    if !first {
                        nodes[i].set_color(0.5, 0.5, 0.5);
                    }
                    nodes[i].set_local_transformation((sym * joint).to_superset());
                    i += 1;
                }
                first = false;
            }
        }
        for _ in 0..100 {
            our_chain.adaptive_optimize(&[2.0, 1.0, 0.5], 0.5);

            if REPULSION {
                our_chain.repulse();
            }

        }
    }
}
