extern crate rand;

extern crate alga;
extern crate nalgebra;
#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate serde_json;

extern crate knot;

use std::env::args;
use std::f64::consts::PI;
use std::fs::File;
use std::process::exit;

use knot::defaults::{COST_PARAMS, NUM_ANGLES, SYMMETRY_COUNT};
use knot::rand_problem::rand_problem;
use knot::symmetry_adjust::Vars;

#[derive(Clone, Copy, Debug, Serialize)]
struct Sample {
    radius: f64,
    radial_angle: f64,
    cost: f64,
}

type ExperimentReport = Vec<Sample>;

/// Count from `start` to `end` in `count` equally-spaced steps.
fn float_steps_inclusive(start: f64, end: f64, count: u32) -> impl Iterator<Item = f64> {
    let step = (end - start) / ((count - 1) as f64);
    (0..count).map(move |i| start + step * (i as f64))
}

const PROBLEM_COUNT: u32 = 100;

const RESOLUTION: u32 = 100;

fn run_experiments() -> Vec<ExperimentReport> {
    let mut rng = rand::thread_rng();

    (0..PROBLEM_COUNT)
        .map(|problem_i| {
            println!("Simulating problem {}", problem_i);
            let problem = rand_problem(&mut rng, COST_PARAMS, NUM_ANGLES, SYMMETRY_COUNT);
            let mut samples = Vec::with_capacity((RESOLUTION * RESOLUTION) as usize);
            for radius in float_steps_inclusive(-20.0, 20.0, RESOLUTION) {
                for radial_angle in float_steps_inclusive(0.0, 2.0 * PI, RESOLUTION) {
                    let vars = Vars {
                        radius,
                        radial_angle,
                    };
                    let sample = Sample {
                        radius,
                        radial_angle,
                        cost: problem.cost(&vars),
                    };
                    samples.push(sample);
                }
            }
            samples
        }).collect()
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
