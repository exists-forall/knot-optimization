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
use std::f64::INFINITY;

use nalgebra::Isometry3;

use knot::joint::{JointSpec, at_angles, discrete_angles};
use knot::symmetry_adjust;
use knot::symmetry_adjust::Problem;
use knot::defaults;
use knot::defaults::{COST_PARAMS, NUM_ANGLES};
use knot::report::{KnotReport, KnotReports, JointsParity};
use knot::filter::{points, WindingAngles};

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

fn update_min<T: PartialOrd>(accum: &mut T, new: T) {
    if &new < accum {
        *accum = new;
    }
}

fn update_max<T: PartialOrd>(accum: &mut T, new: T) {
    if &new > accum {
        *accum = new;
    }
}

fn generate_knot(
    spec: JointSpec,
    symmetry: u32,
    skip: u32,
    angles: [u32; NUM_JOINTS as usize],
    parity: JointsParity,
) -> Knot {
    let mut joint_transformations = [Isometry3::identity(); NUM_JOINTS as usize];

    fill_slice(
        &mut joint_transformations,
        at_angles(
            discrete_angles(
                spec,
                NUM_ANGLES,
                angles.iter().cloned().map(|angle| angle as i32),
            ),
            match parity {
                JointsParity::Even => Isometry3::identity(),
                JointsParity::Odd => spec.origin_to_symmetric() * spec.origin_to_out(),
            },
        ),
    );

    let last_joint_trans = joint_transformations.last().unwrap();

    let last_joint_out = last_joint_trans * spec.origin_to_out();
    let problem = Problem::new(COST_PARAMS, last_joint_out, NUM_ANGLES, symmetry, skip);

    let (vars, cost) = problem.solve_direct();

    let symmetry_adjust_trans = vars.transform();
    let adjusted_points = points(spec, joint_transformations.iter().cloned()).map(
        |point| symmetry_adjust_trans * point,
    );

    let mut winding_angles = WindingAngles::new();
    let mut min_z = INFINITY;
    let mut max_z = -INFINITY;
    let mut min_r = INFINITY;
    let mut max_r = -INFINITY;
    for point in adjusted_points {
        winding_angles.next_point(point);

        update_min(&mut min_z, point.z);
        update_max(&mut max_z, point.z);

        let r = point.x.hypot(point.y);
        update_min(&mut min_r, r);
        update_max(&mut max_r, r);
    }

    let winding_goal = (skip as f64) * PI / (symmetry as f64);
    let good_winding = (winding_angles.total().abs() - winding_goal).abs() <=
        WINDING_ANGLE_TOLERANCE;
    let good_z = (max_z - min_z) >= spec.radius();
    let good_r = (max_r - min_r) >= 2.0 * spec.radius() && min_r >= spec.radius();

    Knot {
        angles,
        symmetry_adjust: vars,
        cost,
        good_candidate: good_winding && good_z && good_r,
    }
}

fn generate_knots(spec: JointSpec, symmetry: u32, skip: u32, parity: JointsParity) -> Vec<Knot> {
    println!("Generating {} candidate knots", NUM_ANGLES.pow(NUM_JOINTS));
    let mut knots = exhaustive!(NUM_ANGLES; NUM_JOINTS)
        .map(|angles| generate_knot(spec, symmetry, skip, angles, parity))
        .filter(|knot| knot.good_candidate)
        .collect::<Vec<_>>();

    println!("Generated {} good knots", knots.len());

    println!("Sorting knots");
    knots.par_sort_unstable_by_key(|knot| NanGreatest(knot.cost));
    println!("Sorted knots");

    knots
}

fn generate_reports(
    spec: JointSpec,
    symmetry: u32,
    skip: u32,
    parity: JointsParity,
) -> KnotReports {
    let knots = generate_knots(spec, symmetry, skip, parity);

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
        parity,
    }
}

fn main() {
    let default_symmetry_str = defaults::SYMMETRY_COUNT.to_string();
    let default_skip_str = (defaults::SYMMETRY_COUNT - 1).to_string();
    let default_bend_angle_str = defaults::joint_spec().bend_angle().to_degrees().to_string();
    let default_radius_str = defaults::joint_spec().radius().to_string();

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
            Arg::with_name("skip")
                .long("skip")
                .value_name("INT")
                .default_value(&default_skip_str)
                .help("Sets how many times the knot winds around the z axis"),
        )
        .arg(
            Arg::with_name("bend-angle")
                .long("bend-angle")
                .value_name("DEGREES")
                .default_value(&default_bend_angle_str)
                .help("Sets bend angle of all joints"),
        )
        .arg(
            Arg::with_name("radius")
                .long("radius")
                .value_name("FLOAT")
                .default_value(&default_radius_str)
                .help("Sets cylinder radius of all joints"),
        )
        .arg(Arg::with_name("odd").short("o").long("odd").help(
            "Use an odd number of segments in each horseshoe",
        ))
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
    let skip = matches
        .value_of("skip")
        .unwrap()
        .parse::<u32>()
        .unwrap_or_else(|err| {
            eprintln!("Invalid skip: {}", err);
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
    let radius = matches
        .value_of("radius")
        .unwrap()
        .parse::<f64>()
        .unwrap_or_else(|err| {
            eprintln!("Invalid joint radius: {}", err);
            exit(1);
        });
    let parity = if matches.is_present("odd") {
        JointsParity::Odd
    } else {
        JointsParity::Even
    };

    let mut file = File::create(&output).unwrap_or_else(|_| {
        eprintln!("Could not create file {}", output);
        exit(1);
    });
    let reports = generate_reports(
        JointSpec::new(1.0, 1.0, bend_angle, radius),
        symmetry,
        skip,
        parity,
    );
    println!(
        "Serializing best {} knots to {}",
        reports.knots.len(),
        output
    );
    serde_json::to_writer(&mut file, &reports).expect("Could not write to file");
}
