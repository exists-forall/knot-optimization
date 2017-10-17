#![feature(conservative_impl_trait)]

extern crate nalgebra;
extern crate alga;
extern crate rand;

#[allow(unused_imports)]
#[macro_use]
extern crate approx;

pub mod joint;
pub mod symmetry;
pub mod cost;
pub mod symmetry_adjust;
pub mod defaults;
pub mod rand_problem;
