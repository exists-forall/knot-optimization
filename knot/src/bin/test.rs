extern crate bspline;
extern crate knot;

use knot::geometries::trefoil_spline;

fn main() {
    trefoil_spline::generate_trefoil();
}
