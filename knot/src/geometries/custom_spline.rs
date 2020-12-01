extern crate bspline;

use std::f64::consts::PI;
use joint::Point;

fn cosine(x : f64) -> f64 {
    x.cos()
}

fn sine(x : f64) -> f64 {
    x.sin()
}

//
pub fn generate_custom_spline() -> bspline::BSpline<Point> {

    // Degree of bspline.
    let degree = 3;

    // Declare parameters here, using notation similar to the examples here. Ensure that all
    // parameters are declared as f64 floats.
    // Sphere radius.
    let radius : f64 = 5.0;
    // latitude of outer upper crossing
    let lat_outer : f64 = 0.27;
    // latitude of inner upper crossing
    let lat_inner : f64 = 0.15;
    // amplitude of radial oscillation.
    let osc : f64 = 1.5;


    // Input points here. Points can reference parameters if defined, but can also simply be numbers.
    // Please make sure that ALL numbers, even constants, are input as floats.
    let points = vec![
        Point::new(radius+osc, 0.0, 0.0 ),
        Point::new((radius-osc)  *  cosine(PI/3.0) * cosine(lat_inner * PI),    (radius-osc) * sine(PI/3.0) * cosine(lat_inner * PI),    (radius-osc) * sine(lat_inner * PI)),
        Point::new((radius+osc) * cosine(PI) * cosine(lat_outer * PI), 0.0, (radius+osc) * sine(lat_outer * PI)),
        Point::new((radius-osc) * cosine(4.0 * PI/3.0),  (radius-osc) * sine(4.0 * PI/3.0), 0.0 ) ,
        Point::new((radius+osc) * cosine(5.0 * PI/3.0) * cosine(lat_outer * PI),   (radius+osc) * sine(5.0 * PI/3.0) * cosine(lat_outer * PI),   -(radius+osc) * sine(lat_outer * PI)),
        Point::new((radius-osc) * cosine(PI/3.0) * cosine(lat_inner * PI),     (radius-osc) * sine(PI/3.0) * cosine(lat_inner * PI),     -(radius-osc) * sine(lat_inner * PI)),
        Point::new((radius+osc) * cosine(2.0 * PI/3.0),  (radius+osc) * sine(2.0 * PI/3.0), 0.0 ) ,
        Point::new((radius-osc) * cosine(PI) * cosine(lat_inner * PI), 0.0,       (radius-osc) * sine(lat_inner * PI)),
        Point::new((radius+osc) * cosine(5.0 * PI/3.0) * cosine(lat_outer * PI),   (radius+osc) * sine(5.0 * PI/3.0) * cosine(lat_outer * PI),   (radius+osc) * sine(lat_outer * PI)),
        Point::new(radius-osc, 0.0, 0.0 ) ,
        Point::new((radius+osc) * cosine(PI/3.0) * cosine(lat_outer * PI),     (radius+osc) * sine(PI/3.0) * cosine(lat_outer * PI),     -(radius+osc) * sine(lat_outer * PI)),
        Point::new((radius-osc) * cosine(PI) * cosine(lat_inner * PI), 0.0,       -(radius-osc) * sine(lat_inner * PI)),
        Point::new((radius+osc) * cosine(4.0 * PI/3.0),  (radius+osc) * sine(4.0 * PI/3.0), 0.0 ) ,
        Point::new((radius-osc) * cosine(5.0 * PI/3.0) * cosine(lat_inner * PI),   (radius-osc) * sine(5.0 * PI/3.0) * cosine(lat_inner * PI),   (radius-osc) * sine(lat_inner * PI)) ,
        Point::new((radius+osc) * cosine(PI/3.0) * cosine(lat_outer * PI),     (radius+osc) * sine(PI/3.0) * cosine(lat_outer * PI),     (radius+osc) * sine(lat_outer * PI)) ,
        Point::new((radius-osc) * cosine(2.0 * PI/3.0),  (radius-osc) * sine(2.0 * PI/3.0), 0.0 ) ,
        Point::new((radius+osc) * cosine(PI) * cosine(lat_outer * PI), 0.0, -(radius+osc) * sine(lat_outer * PI)),
        Point::new((radius-osc) * cosine(5.0 * PI/3.0) * cosine(lat_inner * PI),   (radius-osc) * sine(5.0 * PI/3.0) * cosine(lat_inner * PI),   -(radius-osc) * sine(lat_inner * PI)),
    ];

    // Don't modify anything below here!
    let length = points.len() as i16;
    let knots: Vec<f32> = (1i16 - degree..length + degree - 1).map(f32::from).collect();
    let spline = bspline::BSpline::new(degree as usize, points, knots);

    spline
}
