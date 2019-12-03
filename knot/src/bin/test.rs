extern crate bspline;
extern crate knot;

use knot::geometries::visualize_curve;
use knot::geometries::trefoil_spline::generate_trefoil;

fn cylindrical() {
    visualize_curve::visualize_bspline(
        generate_trefoil,
        3, // sym
        3.0, // scale
    );
}

fn main() {
    cylindrical();
}
