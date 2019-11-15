use input_curve::chain;
use knot::optimize_tools::{RepulsionChain};
use defaults::continuous_optimization::{CONTINUOUS_PARAMS, RETURN_TO_INITIAL_WEIGHT};



// Step 1: Create a bspline function! See trefoil_spline::generate_trefoil() for an example.
// Step 2: Visualize the knot with no locking costs to confirm that the knot matches your idea.


// Run gradient descent optimization on them to find "turn" angles


// visualize to check?


// Each angle will be between two possibilities, leaving a total of 2^n possibilities for
// final chain links. Iterate through all of them find costs.


// Find best one.
