use nalgebra::{Isometry3, UnitQuaternion, Translation3, Vector3};

#[derive(Clone, Copy, Debug)]
pub struct IsometryDifferential {
    // Reciprocal basis to infinitesimal translations
    pub d_x: f64,
    pub d_y: f64,
    pub d_z: f64,

    // Reciprocal basis to infinitesimal rotations
    pub d_i: f64,
    pub d_j: f64,
    pub d_k: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Steps {
    pub x: f64,
    pub y: f64,
    pub z: f64,

    pub i: f64,
    pub j: f64,
    pub k: f64,
}

impl Steps {
    pub fn new_uniform(step: f64) -> Self {
        Steps {
            x: step,
            y: step,
            z: step,

            i: step,
            j: step,
            k: step,
        }
    }
}

pub fn differentiate<F: Fn(&Isometry3<f64>) -> f64>(
    steps: &Steps,
    trans_0: Isometry3<f64>,
    f: F,
) -> (f64, IsometryDifferential) {
    let f_0 = f(&trans_0);

    let mut x_stepped = trans_0;
    x_stepped.translation.vector.x += steps.x;
    let x_rate = (f(&x_stepped) - f_0) / steps.x;

    let mut y_stepped = trans_0;
    y_stepped.translation.vector.y += steps.y;
    let y_rate = (f(&y_stepped) - f_0) / steps.y;

    let mut z_stepped = trans_0;
    z_stepped.translation.vector.z += steps.z;
    let z_rate = (f(&z_stepped) - f_0) / steps.z;

    let mut i_stepped = trans_0;
    i_stepped.rotation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), steps.i) *
        i_stepped.rotation;
    let i_rate = (f(&i_stepped) - f_0) / steps.i;

    let mut j_stepped = trans_0;
    j_stepped.rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), steps.j) *
        j_stepped.rotation;
    let j_rate = (f(&j_stepped) - f_0) / steps.j;

    let mut k_stepped = trans_0;
    k_stepped.rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), steps.k) *
        k_stepped.rotation;
    let k_rate = (f(&k_stepped) - f_0) / steps.k;

    let diff = IsometryDifferential {
        d_x: x_rate,
        d_y: y_rate,
        d_z: z_rate,

        d_i: i_rate,
        d_j: j_rate,
        d_k: k_rate,
    };

    (f_0, diff)
}

impl IsometryDifferential {
    pub fn scale(&self, factor: f64) -> Self {
        IsometryDifferential {
            d_x: self.d_x * factor,
            d_y: self.d_y * factor,
            d_z: self.d_z * factor,

            d_i: self.d_i * factor,
            d_j: self.d_j * factor,
            d_k: self.d_k * factor,
        }
    }

    pub fn magnitude_squ(&self, radius: f64) -> f64 {
        self.d_x * self.d_x + self.d_y * self.d_y + self.d_z * self.d_z +
            (self.d_i * self.d_i + self.d_j * self.d_j + self.d_k * self.d_k) / (radius * radius)
    }
}

pub fn apply_step(radius: f64, trans: &mut Isometry3<f64>, diff: &IsometryDifferential) {
    let delta_translation = Translation3::new(diff.d_x, diff.d_y, diff.d_z);
    let delta_rotation = UnitQuaternion::new(
        Vector3::new(diff.d_i, diff.d_j, diff.d_k) / (radius * radius),
    );

    trans.translation = delta_translation * trans.translation;
    trans.rotation = delta_rotation * trans.rotation;
}
