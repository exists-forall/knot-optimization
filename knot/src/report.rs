use joint::JointSpec;
use symmetry_adjust;
use cost::{CostParams, Costs};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct KnotReport {
    pub angles: Vec<i32>,
    pub final_angle: f64,
    pub symmetry_adjust: symmetry_adjust::Vars,
    pub costs: Costs,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum JointsParity {
    /// Each "horseshoe" has angles.len() * 2 joints
    Even,

    /// Each "horseshoe" has angles.len() * 2 + 1 joints
    Odd,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct KnotReports {
    pub joint_spec: JointSpec,
    pub num_angles: u32,
    pub symmetry_count: u32,
    pub cost_params: CostParams,
    pub knots: Vec<KnotReport>,
    pub parity: JointsParity,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RotationMatrix {
    pub col_x: [f64; 3],
    pub col_y: [f64; 3],
    pub col_z: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Transform {
    pub translation: [f64; 3],
    pub rotation: RotationMatrix,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct KnotGeometry {
    pub joint_spec: JointSpec,
    pub num_angles: u32,
    pub cost_params: CostParams,
    pub parity: JointsParity,
    pub symmetries: Vec<Transform>,
    pub transforms: Vec<Transform>,
}
