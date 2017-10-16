#![feature(conservative_impl_trait)]

extern crate rand;

extern crate alga;
extern crate nalgebra;
#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::fs::File;
use std::process::exit;
use std::env::args;
use rand::Rng;
use rand::distributions::{IndependentSample, Normal};

use nalgebra::{Translation3, Unit, UnitQuaternion, Quaternion, Isometry3, Vector3};

use knot::cost::{CostParams, Thresholds};
use knot::symmetry_adjust::{Problem, Vars, OptimizationParams};

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

#[derive(Clone, Debug, Serialize)]
struct ExperimentReport {
    descent_rate: f64,
    convergence: Vec<f64>,
}

fn run_experiments() -> Vec<Vec<ExperimentReport>> {
    let cost_params = CostParams {
        dist_weight: 5.0,
        axis_weight: 1.0,
        locking_weight: 1.0,
        thresholds: Thresholds {
            dist_for_axis: 4.0,
            axis_for_locking: 0.2,
        },
    };

    let radius_step = 0.01;
    let radial_angle_step = 0.01;

    let num_angles = 16;
    let symmetry_count = 3;

    let mut rng = rand::thread_rng();

    let problem_count = 100;
    let descent_rate_pows = -10..0;
    let descent_steps = 50;

    (0..problem_count)
        .map(|problem_i| {
            println!("Simulating problem {}", problem_i);
            let last_joint_out = rand_joint_trans(&mut rng);
            let problem = Problem::new(cost_params, last_joint_out, num_angles, symmetry_count);
            descent_rate_pows
                .clone()
                .map(|descent_rate_pow| {
                    println!("  Simulating descent rate = 2^{}", descent_rate_pow);
                    let descent_rate = (2.0f64).powi(descent_rate_pow);
                    let opt_params = OptimizationParams {
                        radius_step,
                        radial_angle_step,
                        descent_rate,
                    };
                    let mut vars = Vars {
                        radius: 10.0,
                        radial_angle: 0.0,
                    };
                    let convergence = (0..descent_steps)
                        .map(|_| problem.optimize(&opt_params, &mut vars))
                        .collect();
                    ExperimentReport {
                        descent_rate,
                        convergence,
                    }
                })
                .collect()
        })
        .collect()
}

fn main() {
    let filename = args().nth(1).unwrap_or_else(|| {
        eprintln!("Expected a single output file");
        exit(1);
    });
    let mut file = File::create(&filename).unwrap_or_else(|_| {
        eprintln!("Could not create file {}", filename);
        exit(1);
    });
    let results = run_experiments();
    serde_json::to_writer(&mut file, &results).expect("Could not write to file");
}
