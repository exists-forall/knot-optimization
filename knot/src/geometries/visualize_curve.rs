extern crate kiss3d;

use super::kiss3d::event::{Action, Key, WindowEvent};
use super::kiss3d::light::Light;
use super::kiss3d::window::Window;

use alga::general::SubsetOf;
use defaults;
use defaults::continuous_optimization::{
    COST_PARAMS, MAX_REPULSION_STRENGTH,
    REPULSION_EXPONENT, REPULSION_STRENGTH,
    RETURN_TO_INITIAL_WEIGHT
};
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
            COST_PARAMS,
            RETURN_TO_INITIAL_WEIGHT,
            0.0, // Descent rate
            bspline_generator,
            sym_number,
            defaults::cyl_spec(),
        ),
        symmetries(sym_number).map(|quat| quat.to_superset()).collect(),
        REPULSION_EXPONENT,
        REPULSION_STRENGTH,
        MAX_REPULSION_STRENGTH,
    );

    let mut window = Window::new("Continuous Optimization");
    window.set_light(Light::StickToCamera);

    let mut nodes = add_joints(
        window.scene_mut(), // Scene
        &our_chain.spec, // JointSpec
        50, // Number of angles
        our_chain.joints.len() * 6, // Number of joints
        Style::Flat,
    );

    while window.render() {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(_, Action::Release, _) => {}
                WindowEvent::Key(code, _, _) => match code {
                    Key::N => {
                        println!("Repulsion disabled");
                        our_chain.repulsion_strength = 0.0;
                    }
                    Key::M => {
                        println!("Repulsion enabled");
                        our_chain.repulsion_strength = REPULSION_STRENGTH;
                    }
                    Key::K => {
                        our_chain.repulsion_strength *= 1.5;
                        println!("Repulsion: {}", our_chain.repulsion_strength);
                    }
                    Key::J => {
                        our_chain.repulsion_strength /= 1.5;
                        println!("Repulsion: {}", our_chain.repulsion_strength);
                    }

                    Key::V => {
                        println!("Max repulsion disabled");
                        our_chain.max_repulsion_strength = 0.0001;
                    }
                    Key::B => {
                        our_chain.max_repulsion_strength = MAX_REPULSION_STRENGTH;
                        println!("Max repulsion enabled: {}", our_chain.max_repulsion_strength);
                    }
                    Key::H => {
                        our_chain.max_repulsion_strength *= 1.5;
                        println!("Max repulsion: {}", our_chain.max_repulsion_strength);
                    }
                    Key::G => {
                        our_chain.max_repulsion_strength /= 1.5;
                        println!("Max repulsion: {}", our_chain.max_repulsion_strength);
                    }
                    _ => {}
                },
                _ => {}
            }
        }

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
