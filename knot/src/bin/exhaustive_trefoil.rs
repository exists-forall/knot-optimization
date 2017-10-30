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
use std::f64::consts::PI;

use nalgebra::Isometry3;

use knot::joint::{JointSpec, at_angles, discrete_angles};
use knot::symmetry_adjust;
use knot::symmetry_adjust::Problem;
use knot::defaults::{COST_PARAMS, OPTIMIZATION_PARAMS, NUM_ANGLES, initial_symmetry_adjusts,
                     SYMMETRY_COUNT, joint_spec};
use knot::report::{KnotReport, KnotReports};
use knot::filter::winding_angle;

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

const WINDING_ANGLE_TOLERANCE: f64 = 0.1;

const KEEP_COUNT: usize = 2048;

#[derive(Clone, Copy)]
struct Knot {
    angles: [u32; NUM_JOINTS as usize],
    symmetry_adjust: symmetry_adjust::Vars,
    cost: f64,
    good_candidate: bool,
}

/// Float variant which supports total ordering by considering NaN to be the greatest value and
/// equal to itself. Useful for sorting by cost, where NaN cost indicates that something has gone
/// very wrong.
#[derive(Clone, Copy, Debug, PartialOrd)]
struct NanGreatest(f64);

impl PartialEq for NanGreatest {
    fn eq(&self, other: &Self) -> bool {
        (self.0.is_nan() && other.0.is_nan()) || self.0 == other.0
    }

    fn ne(&self, other: &Self) -> bool {
        (self.0.is_nan() != other.0.is_nan()) || self.0 != other.0
    }
}

impl Eq for NanGreatest {}

impl Ord for NanGreatest {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.partial_cmp(&other.0).unwrap_or_else(|| match (
            self.0.is_nan(),
            other.0.is_nan(),
        ) {
            (true, false) => Ordering::Greater,
            (false, true) => Ordering::Less,
            (_, _) => Ordering::Equal,
        })
    }
}

/// Write the elements of an iterator into a slice.
fn fill_slice<T, I: Iterator<Item = T>>(slice: &mut [T], iter: I) {
    for (item, slot) in iter.zip(slice.iter_mut()) {
        *slot = item;
    }
}

fn generate_knot(spec: JointSpec, angles: [u32; NUM_JOINTS as usize]) -> Knot {
    let mut joint_transformations = [Isometry3::identity(); NUM_JOINTS as usize];

    fill_slice(
        &mut joint_transformations,
        at_angles(
            discrete_angles(
                spec,
                NUM_ANGLES,
                angles.iter().cloned().map(|angle| angle as i32),
            ),
            Isometry3::identity(),
        ),
    );

    let last_joint_trans = joint_transformations.last().unwrap();

    let last_joint_out = last_joint_trans * spec.origin_to_out();
    let problem = Problem::new(COST_PARAMS, last_joint_out, NUM_ANGLES, SYMMETRY_COUNT);

    let (vars, cost) = initial_symmetry_adjusts()
        .map(|mut vars| {
            for _ in 0..OPTIMIZATION_STEPS {
                problem.optimize(&OPTIMIZATION_PARAMS, &mut vars);
            }
            let cost = problem.cost(&vars);
            (vars, cost)
        })
        .min_by_key(|&(_, cost)| NanGreatest(cost))
        .unwrap();

    let symmetry_adjust_trans = vars.transform();
    let mut total_winding_angle = 0.0;
    for joint_trans in joint_transformations.iter() {
        total_winding_angle += winding_angle(&spec, &(symmetry_adjust_trans * joint_trans));
    }

    Knot {
        angles,
        symmetry_adjust: vars,
        cost,
        good_candidate: (total_winding_angle.abs() - 2.0 * PI / (SYMMETRY_COUNT as f64)).abs() <=
            WINDING_ANGLE_TOLERANCE,
    }
}

fn generate_knots() -> Vec<Knot> {
    let spec = joint_spec();

    println!("Generating {} candidate knots", NUM_ANGLES.pow(NUM_JOINTS));
    let mut knots = exhaustive!(NUM_ANGLES; NUM_JOINTS)
        .map(|angles| generate_knot(spec, angles))
        .filter(|knot| knot.good_candidate)
        .collect::<Vec<_>>();

    println!("Generated {} good knots", knots.len());

    println!("Sorting knots");
    // knots.par_sort_unstable_by_key(|report_0, report_1| {
    //     nan_greatest(report_0.cost, report_1.cost)
    // });
    knots.par_sort_unstable_by_key(|knot| NanGreatest(knot.cost));
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
