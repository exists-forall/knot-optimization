#![feature(inclusive_range_fields)]
#![feature(slice_patterns)]

extern crate alga;
extern crate nalgebra;
extern crate rand;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate serde_json;

#[allow(unused_imports)]
#[macro_use]
extern crate approx;

pub mod joint;
pub mod symmetry;
pub mod cost;
pub mod symmetry_adjust;
pub mod defaults;
pub mod rand_problem;
pub mod report;
pub mod visualize;
pub mod filter;
pub mod isometry_adjust;
pub mod continuous_optimize;
pub mod approx_locking_angle;
pub mod collision_grid;
pub mod collision_grid_trivial;
pub mod geometries;
