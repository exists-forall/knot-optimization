extern crate bspline;
extern crate knot;

use knot::geometries::visualize_curve;
use knot::geometries::custom_spline::generate_custom_spline;

fn visualize() {
    visualize_curve::visualize_bspline(
        generate_custom_spline, // knot bspline generator
        6, // symmetry
        5.0, // scale
    );
}

fn main() {
    visualize();
}
