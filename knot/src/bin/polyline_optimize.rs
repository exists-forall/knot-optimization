extern crate knot;
extern crate bspline;
extern crate alga;
extern crate kiss3d;
extern crate nalgebra;
extern crate serde;
extern crate serde_json;

use std::f64::consts::PI;

use kiss3d::light::Light;
use kiss3d::window::Window;

use knot::defaults;
use knot::optimize_tools::{RepulsionChain};
use knot::defaults::continuous_optimization::{
    COST_PARAMS, MAX_REPULSION_STRENGTH, RATE,
    REPULSION_EXPONENT, REPULSION_STRENGTH, RETURN_TO_INITIAL_WEIGHT,
    CONTINUOUS_PARAMS
};
use knot::geometries::custom_spline::generate_custom_spline;
use knot::joint::Point;
use knot::geometries::from_spline;
use knot::symmetry::{symmetries, symmetries_with_skip};
use knot::visualize::joint_render::{add_joints, Style};

use alga::general::SubsetOf;
use nalgebra::{Isometry3, UnitQuaternion, Vector3};

// Step 1: Create a bspline function! See trefoil_spline::generate_trefoil() for an example.
// Step 2: Visualize the knot with no locking costs to confirm that the knot matches your idea.

// Run gradient descent optimization on them to find "turn" angles

fn polyline_optimize<F: Fn() -> bspline::BSpline<Point> + Copy>(
    bspline_generator: F,
    knot_sym: u32, //sym
    scale: f32,
) {
    // Create a continuous knot using this curve
    let mut chain = RepulsionChain::new(
        from_spline::generic_chain(
            scale, // scale
            CONTINUOUS_PARAMS, // No angle locking weights.
            RETURN_TO_INITIAL_WEIGHT,
            RATE,
            defaults::joint_spec(),
            bspline_generator,
            knot_sym,
        ),
        symmetries(knot_sym).map(|quat| quat.to_superset()).collect(),
        REPULSION_EXPONENT,
        REPULSION_STRENGTH,
        MAX_REPULSION_STRENGTH,
    );

    // Run gradient descent on chain.
    for _ in 0..100000 {
        chain.adaptive_optimize(&[2.0, 1.0, 0.5], 0.5);
        chain.repulse();
    }

    // Impose locking weights, let it find its own best.
    chain.cost_params.locking_weight = 0.17;
    for _ in 0..50000 {
        chain.adaptive_optimize(&[2.0, 1.0, 0.5], 0.5);
        chain.repulse();
    }
    cost = chain.adaptive_optimize(&[2.0, 1.0, 0.5], 0.5);
    println!("{}", cost);
    prev_trans = Isometry3::identity();
    for &joint in &chain.joints {
        let trans_0 = prev_trans;
        let trans_1 = joint * chain.spec.origin_to_in();

        let axis_0 = trans_0 * Vector3::y_axis().to_superset();
        let axis_1 = trans_1 * Vector3::y_axis().to_superset();

        let align = UnitQuaternion::rotation_between(&axis_1, &axis_0)
            .unwrap_or(UnitQuaternion::identity());
        let aligned_rel_rotation =
            trans_0.rotation.inverse() * align * trans_1.rotation;
        let locking_angle = aligned_rel_rotation.angle();
        let locking_number =
            locking_angle / (2.0 * PI) * (chain.num_angles as f64);
        println!("{}", locking_number);
        prev_trans = joint * chain.spec.origin_to_out();
    }

    // let mut prev = 2.0;
    // let mut cost = 1.0;
    // let mut step = 0;
    // let mut count = 0;
    //
    // while count < 3 {
    //     step += 1;
    //     prev = cost;
    //     for _ in 0..10000 {
    //         chain.adaptive_optimize(&[2.0, 1.0, 0.5], 0.5);
    //         chain.repulse();
    //     }
    //     cost = chain.adaptive_optimize(&[2.0, 1.0, 0.5], 0.5);
    //     println!("{{{}, {}}},", step, cost);
    //     if (prev - cost > 0.00001) {
    //         count += 1;
    //     } else {
    //         count = 0;
    //     }
    // }

    let mut window = Window::new("Polyline Optimization");
    window.set_light(Light::StickToCamera);

    // Create chain in Window
    let mut nodes = add_joints(
        window.scene_mut(), // Scene
        &chain.spec, // JointSpec
        16 as u16, // Number of angles
        chain.joints.len() * 6, // Number of joints
        Style::Flat,
    );

    while window.render() {
        {
            let mut i = 0;
            for sym in symmetries_with_skip(3, 4) {
                for &joint in &chain.joints {
                    nodes[i].set_color(0.5, 0.5, 0.5);
                    nodes[i].set_local_transformation((sym * joint).to_superset());
                    i += 1;
                }
            }
        }
    }

    // Determine current angles of intersection, and how far they are from being a discrete angle
    // For each problematic angle, try both cases of knots and see what happens post-optimization
    // Normalize costs

}

// Each angle will be between two possibilities, leaving a total of 2^n possibilities for
// final chain links. Iterate through all of them find costs.


// Find best one.

fn main() {
    polyline_optimize(generate_custom_spline, 6, 4.0);
}
