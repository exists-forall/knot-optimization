use nalgebra::Point3;

pub fn spherical(theta: f64, phi: f64, rho: f64) -> Point3<f64> {
    Point3::new(
        rho * theta.cos() * phi.cos(),
        rho * theta.sin() * phi.cos(),
        rho * phi.sin(),
    )
}
