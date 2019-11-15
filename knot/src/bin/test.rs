extern crate bspline;
extern crate knot;

use knot::geometries::visualize_curve;
use knot::geometries::trefoil_spline::generate_trefoil;

fn main() {
    visualize_curve::visualize_bspline(
        generate_trefoil,
        3, // sym
        1.4, // scale
    );
}
