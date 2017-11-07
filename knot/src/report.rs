use joint::JointSpec;
use symmetry_adjust;
use cost::CostParams;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct KnotReport {
    pub angles: Vec<i32>,
    pub symmetry_adjust: symmetry_adjust::Vars,
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
