use rand::Rng;
use rand::distributions::{IndependentSample, Normal};

use cost::CostParams;
use symmetry_adjust::Problem;

use nalgebra::{UnitQuaternion, Quaternion, Unit, Isometry3, Translation3, Vector3};

fn rand_quaternion<R: Rng>(rng: &mut R) -> UnitQuaternion<f64> {
    let dist = Normal::new(0.0, 1.0);
    let w = dist.ind_sample(rng);
    let x = dist.ind_sample(rng);
    let y = dist.ind_sample(rng);
    let z = dist.ind_sample(rng);
    UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
}

fn rand_direction<R: Rng>(rng: &mut R) -> Unit<Vector3<f64>> {
    let dist = Normal::new(0.0, 1.0);
    let x = dist.ind_sample(rng);
    let y = dist.ind_sample(rng);
    let z = dist.ind_sample(rng);
    Unit::new_normalize(Vector3::new(x, y, z))
}

fn rand_translation<RadiusDist: IndependentSample<f64>, R: Rng>(
    radius_dist: &RadiusDist,
    rng: &mut R,
) -> Translation3<f64> {
    let dir = rand_direction(rng);
    let radius = radius_dist.ind_sample(rng);
    Translation3::from_vector(dir.unwrap() * radius)
}

fn rand_joint_trans<R: Rng>(rng: &mut R) -> Isometry3<f64> {
    let translation = rand_translation(&Normal::new(1.0, 1.0), rng);
    let rotation = rand_quaternion(rng);
    Isometry3::from_parts(translation, rotation)
}

pub fn rand_problem<R: Rng>(
    rng: &mut R,
    cost_params: CostParams,
    num_angles: u32,
    symmetry_count: u32,
) -> Problem {
    let last_joint_out = rand_joint_trans(rng);
    Problem::new(cost_params, last_joint_out, num_angles, symmetry_count)
}
