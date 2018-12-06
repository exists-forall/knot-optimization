extern crate serde;
extern crate serde_json;

extern crate knot;

use std::fs::File;
use std::path::Path;
use knot::report::{
    CompleteKnotReports, KnotReport
};

fn parity_counter(all_knots: Vec<KnotReport>) {
    let mut parities: [u32; 16] = [0; 16];
    for knot_report in all_knots {
        let parity : usize = knot_report.angle_parity as usize;
        let count: u32 = &parities[parity] + 1;
        parities[parity] = count;
    }

    for x in 0 ..= 15 {
        println!("Parity: {}, Count: {}", x, parities[x])
    }
}

fn main() {
    let json_file_path = Path::new("trefoil_statistics/top_100.json");
    let json_file = File::open(json_file_path).expect("file not found");

    let complete_report: CompleteKnotReports =
        serde_json::from_reader(json_file).expect("error while reading json");

    parity_counter(complete_report.knots)
}
