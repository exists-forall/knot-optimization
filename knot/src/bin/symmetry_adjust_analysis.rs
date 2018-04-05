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

use knot::symmetry_adjust::OptimizationParams;
use knot::defaults::{COST_PARAMS, NUM_ANGLES, SYMMETRY_COUNT, INITIAL_SYMMETRY_ADJUST};
use knot::rand_problem::rand_problem;

#[derive(Clone, Debug, Serialize)]
struct ExperimentReport {
    descent_rate: f64,
    convergence: Vec<f64>,
}

fn run_experiments() -> Vec<Vec<ExperimentReport>> {
    let radius_step = 0.01;
    let radial_angle_step = 0.01;

    let mut rng = rand::thread_rng();

    let problem_count = 100;
    let descent_rate_pows = -10..0;
    let descent_steps = 50;

    (0..problem_count)
        .map(|problem_i| {
            println!("Simulating problem {}", problem_i);
            let problem = rand_problem(&mut rng, COST_PARAMS, NUM_ANGLES, SYMMETRY_COUNT);
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
                    let mut vars = INITIAL_SYMMETRY_ADJUST;
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
