#![feature(conservative_impl_trait)]

extern crate alga;
extern crate nalgebra;
extern crate serde;
extern crate serde_json;
extern crate rayon;
extern crate clap;

extern crate knot;

use std::cmp::Ordering;
use std::fs::File;
use std::process::exit;
use std::f64::consts::PI;

use nalgebra::Isometry3;

use knot::joint::{JointSpec, at_angles, discrete_angles};
use knot::symmetry_adjust;
use knot::symmetry_adjust::Problem;
use knot::defaults;
use knot::defaults::{COST_PARAMS, OPTIMIZATION_PARAMS, NUM_ANGLES, initial_symmetry_adjusts};
use knot::report::{KnotReport, KnotReports};
use knot::filter::winding_angle;

use rayon::prelude::*;
use rayon::prelude::IntoParallelIterator;
use rayon::slice::ParallelSliceMut;

use clap::{Arg, App};

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

fn generate_knot(spec: JointSpec, symmetry: u32, angles: [u32; NUM_JOINTS as usize]) -> Knot {
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
    let problem = Problem::new(COST_PARAMS, last_joint_out, NUM_ANGLES, symmetry);

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
    let winding_goal = ((symmetry - 1) as f64) * PI / (symmetry as f64);

    Knot {
        angles,
        symmetry_adjust: vars,
        cost,
        good_candidate: (total_winding_angle.abs() - winding_goal).abs() <= WINDING_ANGLE_TOLERANCE,
    }
}

fn generate_knots(spec: JointSpec, symmetry: u32) -> Vec<Knot> {
    println!("Generating {} candidate knots", NUM_ANGLES.pow(NUM_JOINTS));
    let mut knots = exhaustive!(NUM_ANGLES; NUM_JOINTS)
        .map(|angles| generate_knot(spec, symmetry, angles))
        .filter(|knot| knot.good_candidate)
        .collect::<Vec<_>>();

    println!("Generated {} good knots", knots.len());

    println!("Sorting knots");
    knots.par_sort_unstable_by_key(|knot| NanGreatest(knot.cost));
    println!("Sorted knots");

    knots
}

fn generate_reports(spec: JointSpec, symmetry: u32) -> KnotReports {
    let knots = generate_knots(spec, symmetry);

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
        joint_spec: spec,
        num_angles: NUM_ANGLES,
        symmetry_count: symmetry,
        knots: reports,
        cost_params: COST_PARAMS,
    }
}

fn main() {
    let default_symmetry_str = defaults::SYMMETRY_COUNT.to_string();
    let default_bend_angle_str = defaults::joint_spec().bend_angle().to_degrees().to_string();

    let matches = App::new("Exhaustive Symmetric Knot Model Generator")
        .author("William Brandon <hypercube97@gmail.com>")
        .version("0.1.0")
        .arg(
            Arg::with_name("output")
                .long("output")
                .value_name("FILE.json")
                .help("Sets the output file path")
                .takes_value(true)
                .required(true),
        )
        .arg(
            Arg::with_name("symmetry")
                .long("symmetry")
                .value_name("INT")
                .default_value(&default_symmetry_str)
                .help("Sets dihedral-N symmetry"),
        )
        .arg(
            Arg::with_name("bend-angle")
                .long("bend-angle")
                .value_name("DEGREES")
                .default_value(&default_bend_angle_str)
                .help("Sets bend angle of all joints"),
        )
        .get_matches();

    let output = matches.value_of("output").unwrap();
    let symmetry = matches
        .value_of("symmetry")
        .unwrap()
        .parse::<u32>()
        .unwrap_or_else(|err| {
            eprintln!("Invalid symmetry: {}", err);
            exit(1);
        });
    let bend_angle = matches
        .value_of("bend-angle")
        .unwrap()
        .parse::<f64>()
        .unwrap_or_else(|err| {
            eprintln!("Invalid bend angle: {}", err);
            exit(1);
        })
        .to_radians();

    let mut file = File::create(&output).unwrap_or_else(|_| {
        eprintln!("Could not create file {}", output);
        exit(1);
    });
    let reports = generate_reports(JointSpec::new(1.0, 1.0, bend_angle), symmetry);
    println!(
        "Serializing best {} knots to {}",
        reports.knots.len(),
        output
    );
    serde_json::to_writer(&mut file, &reports).expect("Could not write to file");
}
