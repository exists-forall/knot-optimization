use alga::general::SubsetOf;
use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};

use approx_locking_angle::locking_angle_opposing;
use cost::{CostParams, Costs};
use defaults;
use joint::{at_angles, discrete_symmetric_angles, JointSpec};
use symmetry::adjacent_symmetry;
use symmetry_adjust::{self, Problem};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct KnotReport {
    pub angles: Vec<i32>,
    pub final_angle: Option<f64>,
    pub angle_parity: i32,
    pub symmetry_adjust: Option<symmetry_adjust::Vars>,
    pub costs: Option<Costs>,
    pub total_cost: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompleteKnotReport {
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
    pub joint_spec: Option<JointSpec>,
    pub num_angles: Option<u32>,
    pub symmetry_count: u32,
    pub symmetry_skip: u32,
    pub cost_params: Option<CostParams>,
    pub knots: Vec<KnotReport>,
    pub parity: JointsParity,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompleteKnotReports {
    pub joint_spec: JointSpec,
    pub num_angles: u16,
    pub symmetry_count: u32,
    pub symmetry_skip: u32,
    pub cost_params: CostParams,
    pub knots: Vec<KnotReport>,
    pub parity: JointsParity,
}

pub fn complete_reports(reports: KnotReports) -> CompleteKnotReports {
    CompleteKnotReports {
        joint_spec: reports.joint_spec.unwrap_or(defaults::joint_spec()),
        num_angles: reports.num_angles.unwrap_or(defaults::NUM_ANGLES as u32) as u16,
        symmetry_count: reports.symmetry_count,
        symmetry_skip: reports.symmetry_skip,
        cost_params: reports.cost_params.unwrap_or(defaults::COST_PARAMS),
        knots: reports.knots,
        parity: reports.parity,
    }
}

pub fn complete_report(reports: &CompleteKnotReports, index: usize) -> CompleteKnotReport {
    let report = reports.knots[index].clone();

    let (vars, costs, final_angle) = if let (Some(vars), Some(costs), Some(final_angle)) =
        (report.symmetry_adjust, report.costs, report.final_angle)
    {
        (vars, costs, final_angle)
    } else {
        let mut last_joint_trans = Isometry3::identity();
        for trans in at_angles(
            discrete_symmetric_angles(
                reports.joint_spec,
                reports.num_angles,
                reports.parity,
                report.angles.iter().cloned().map(|angle| angle as i32),
            ),
            match reports.parity {
                JointsParity::Even => Isometry3::identity(),
                JointsParity::Odd => {
                    reports.joint_spec.origin_to_symmetric() * reports.joint_spec.origin_to_out()
                }
            },
        ) {
            last_joint_trans = trans;
        }

        let last_joint_out = last_joint_trans * reports.joint_spec.origin_to_out();
        let problem = Problem::new(
            reports.cost_params,
            last_joint_out,
            reports.num_angles,
            reports.symmetry_count,
            reports.symmetry_skip,
        );

        let (vars, _) = problem.solve_direct();
        let costs = problem.costs(&vars);

        let symmetry_adjust_trans = vars.transform();
        let final_angle = locking_angle_opposing(
            reports.num_angles,
            &(symmetry_adjust_trans * last_joint_out),
            &(adjacent_symmetry(reports.symmetry_count, reports.symmetry_skip)
                * symmetry_adjust_trans
                * last_joint_out),
        );

        (vars, costs, final_angle)
    };

    CompleteKnotReport {
        angles: report.angles,
        final_angle: final_angle,
        symmetry_adjust: vars,
        costs: costs,
    }
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

fn vec3_to_array(v: Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn array_to_vec3(arr: [f64; 3]) -> Vector3<f64> {
    Vector3::new(arr[0], arr[1], arr[2])
}

impl Transform {
    pub fn from_isometry(iso: Isometry3<f64>) -> Self {
        Transform {
            translation: vec3_to_array(iso.translation.vector),
            rotation: RotationMatrix {
                col_x: vec3_to_array(iso * Vector3::x_axis().to_superset()),
                col_y: vec3_to_array(iso * Vector3::y_axis().to_superset()),
                col_z: vec3_to_array(iso * Vector3::z_axis().to_superset()),
            },
        }
    }

    pub fn to_isometry(&self) -> Isometry3<f64> {
        let [tx, ty, tz] = self.translation;
        let translation = Translation3::new(tx, ty, tz);
        let rotation_mat = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
            array_to_vec3(self.rotation.col_x),
            array_to_vec3(self.rotation.col_y),
            array_to_vec3(self.rotation.col_z),
        ]));
        let quaternion = UnitQuaternion::from_rotation_matrix(&rotation_mat);
        Isometry3::from_parts(translation, quaternion)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct KnotGeometry {
    pub joint_spec: JointSpec,
    pub num_angles: u16,
    pub cost_params: CostParams,
    pub parity: JointsParity,
    pub symmetries: Vec<Transform>,
    pub transforms: Vec<Transform>,
}
