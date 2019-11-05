use input_curve::chain;
use defaults::continuous_optimization::{CONTINUOUS_PARAMS, RETURN_TO_INITIAL_WEIGHT};


// Step 1: Create a bspline function! See trefoil_spline::generate_trefoil() for an example.
// Step 2: Visualize the knot with no locking costs to confirm that the knot matches your idea.



fn visualize_bspline<F: Fn() -> bspline::BSpline<Point>>(
    bspline_generator: F, // Function returning the bspline. See the bspline package.
    sym_number: u32, // Number of identical sub-chains the knot should be split into.
    scale: f32, // Scalar multiple of size of bspline. Take a guess and adjust until it looks right.
) {
    let our_chain = RepulsionChain::new(
        chain(
            scale,
            COST_PARAMS,
            RETURN_TO_INITIAL_WEIGHT,
            RATE, // Descent rate
            bspline_generator,
            sym_number,
        ),
        symmetries(sym_number).map(|quat| quat.to_superset()).collect(),
        REPULSION_EXPONENT,
        REPULSION_STRENGTH,
        MAX_REPULSION_STRENGTH,
    ),
}

// Run gradient descent optimization on them to find "turn" angles


// visualize to check?


// Each angle will be between two possibilities, leaving a total of 2^n possibilities for
// final chain links. Iterate through all of them find costs.


// Find best one.
