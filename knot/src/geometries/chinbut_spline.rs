extern crate bspline;

use std::f64::consts::PI;
use joint::Point;

fn cosine(x : f64) -> f64 {
    x.cos()
}

fn sine(x : f64) -> f64 {
    x.sin()
}

pub fn generate_chinbutspline() -> bspline::BSpline<Point> {

    // Sphere radius.
    let radius : f64 = 5.0;

    // latitude of outer upper crossing
    let lat_outer : f64 = 0.27;

    // latitude of inner upper crossing
    let lat_inner : f64 = 0.15;

    // amplitude of radial oscillation.
    let osc : f64 = 1.5;

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

    let knots = vec![-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0,
                    11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0];

    let degree = 3;

    let spline = bspline::BSpline::new(degree, points, knots);
    // println!("\tt range = {:?,", spline.knot_domain());
    // println!("spline = {:?,", spline);

    // let step_size = 0.2;
    // let t_range = spline.knot_domain();
    // let steps = ((t_range.1 - t_range.0) / step_size) as usize;
    // for s in 0..steps + 1 {
    //     let t = step_size  *  s as f64 + t_range.0;
    //     let pt = spline.point(t);
    //     println!("Point {:?,: ({:?,, {:?,, {:?,)", s, pt.x, pt.y, pt.z);
    // ,
    spline
}
