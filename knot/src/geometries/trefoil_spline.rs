extern crate bspline;

use joint::Point;

pub fn generate_trefoil() -> bspline::BSpline<Point> {

    // Radius can be between 1 and 20.
    let radius : f64 = 5.0;

    // Separation angle can be between 1 and 30 degrees.
    let sep : f64 = 21.0;

    // Just a constant.
    let num : f64 = 0.0174;

    // Height can be between 1 and 9.
    let height : f64 = 3.0;

    let points = vec![Point::new(radius * (num * -(sep)).cos(), radius * (num * -(sep)).sin(), height),
                    Point::new(radius * (num * (sep)).cos(), radius * (num * (sep)).sin(), -height),
                    Point::new(radius * (num * -(sep + 120.0)).cos(), radius * (num * -(sep + 120.0)).sin(), height),
                    Point::new(radius * (num * (sep + 240.0)).cos(), radius * (num * (sep + 240.0)).sin(), -height),
                    Point::new(radius * (num * -(sep + 240.0)).cos(), radius * (num * -(sep + 240.0)).sin(), height),
                    Point::new(radius * (num * (sep + 120.0)).cos(), radius * (num * (sep + 120.0)).sin(), -height),
                    Point::new(radius * (num * -(sep)).cos(), radius * (num * -(sep)).sin(), height),
                    Point::new(radius * (num * (sep)).cos(), radius * (num * (sep)).sin(), -height),
                    Point::new(radius * (num * -(sep + 120.0)).cos(), radius * (num * -(sep + 120.0)).sin(), height)];

    let knots = vec![-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
    let degree = 3;

    let spline = bspline::BSpline::new(degree, points, knots);
    // println!("\tt range = {:?}", spline.knot_domain());
    // println!("spline = {:?}", spline);

    // let step_size = 0.2;
    // let t_range = spline.knot_domain();
    // let steps = ((t_range.1 - t_range.0) / step_size) as usize;
    // for s in 0..steps + 1 {
    //     let t = step_size * s as f64 + t_range.0;
    //     let pt = spline.point(t);
    //     println!("Point {:?}: ({:?}, {:?}, {:?})", s, pt.x, pt.y, pt.z);
    // }
    spline
}
