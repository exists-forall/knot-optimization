#![feature(conservative_impl_trait)]

extern crate alga;
extern crate nalgebra;
extern crate serde;
extern crate serde_json;
extern crate rayon;

extern crate knot;

use std::cmp::Ordering;
use std::fs::File;
use std::process::exit;
use std::env::args;

use nalgebra::Isometry3;

use knot::joint::{JointSpec, at_angles, discrete_angles};
use knot::symmetry_adjust;
use knot::symmetry_adjust::Problem;
use knot::defaults::{COST_PARAMS, OPTIMIZATION_PARAMS, NUM_ANGLES, INITIAL_SYMMETRY_ADJUST,
                     SYMMETRY_COUNT, joint_spec};
use knot::report::{KnotReport, KnotReports};

use rayon::prelude::*;
use rayon::prelude::IntoParallelIterator;
use rayon::slice::ParallelSliceMut;

macro_rules! exhaustive {
    ($symbols: expr; $count: expr) => { {
        const SYMBOLS: u32 = $symbols;
        const COUNT: u32 = $count;
        (0u64..SYMBOLS.pow(COUNT) as u64).into_par_iter().map(|i| {
            let mut remaining = i;
            let mut result = [0u32; COUNT as usize];
            for place in 0..COUNT {
                result[place as usize] = (remaining % (SYMBOLS as u64)) as u32;
                remaining /= SYMBOLS as u64;
            }
            result
        })
    } }
}

const NUM_JOINTS: u32 = 5;

const OPTIMIZATION_STEPS: u32 = 4;

const KEEP_COUNT: usize = 2048;

#[derive(Clone, Copy)]
struct Knot {
    angles: [u32; NUM_JOINTS as usize],
    symmetry_adjust: symmetry_adjust::Vars,
    cost: f64,
}

fn generate_knot(spec: JointSpec, angles: [u32; NUM_JOINTS as usize]) -> Knot {
    let last_joint_trans = at_angles(
        discrete_angles(
            spec,
            NUM_ANGLES,
            angles.iter().cloned().map(|angle| angle as i32),
        ),
        Isometry3::identity(),
    ).last()
        .unwrap();
    let last_joint_out = last_joint_trans * spec.origin_to_out();
    let problem = Problem::new(COST_PARAMS, last_joint_out, NUM_ANGLES, SYMMETRY_COUNT);

    let mut vars = INITIAL_SYMMETRY_ADJUST;
    for _ in 0..OPTIMIZATION_STEPS {
        problem.optimize(&OPTIMIZATION_PARAMS, &mut vars);
    }

    let cost = problem.cost(&vars);

    Knot {
        angles,
        symmetry_adjust: vars,
        cost,
    }
}

/// Compare two floats, treating NaN as greater than all other values and equal to itself
fn nan_greatest(a: f64, b: f64) -> Ordering {
    a.partial_cmp(&b).unwrap_or_else(
        || match (a.is_nan(), b.is_nan()) {
            (true, false) => Ordering::Greater,
            (false, true) => Ordering::Less,
            (_, _) => Ordering::Equal,
        },
    )
}

fn generate_knots() -> Vec<Knot> {
    let spec = joint_spec();

    println!("Generating {} knots", NUM_ANGLES.pow(NUM_JOINTS));
    let mut knots = exhaustive!(NUM_ANGLES; NUM_JOINTS)
        .map(|angles| generate_knot(spec, angles))
        .collect::<Vec<_>>();

    println!("Generated {} knots", knots.len());

    println!("Sorting knots");
    knots.par_sort_unstable_by(|report_0, report_1| {
        nan_greatest(report_0.cost, report_1.cost)
    });
    println!("Sorted knots");

    knots
}

fn generate_reports() -> KnotReports {
    let knots = generate_knots();

    let reports = knots[0..KEEP_COUNT.min(knots.len())]
        .iter()
        .map(|knot| {
            KnotReport {
                angles: knot.angles.iter().map(|&angle| angle as i32).collect(),
                symmetry_adjust: knot.symmetry_adjust,
            }
        })
        .collect();

    KnotReports {
        joint_spec: joint_spec(),
        num_angles: NUM_ANGLES,
        symmetry_count: SYMMETRY_COUNT,
        knots: reports,
        cost_params: COST_PARAMS,
    }
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
    let reports = generate_reports();
    println!(
        "Serializing best {} knots to {}",
        reports.knots.len(),
        filename
    );
    serde_json::to_writer(&mut file, &reports).expect("Could not write to file");
}
