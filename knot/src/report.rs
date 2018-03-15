use nalgebra::{Isometry3, Vector3, Translation3, Matrix3, Rotation3, UnitQuaternion};
use alga::general::SubsetOf;

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
    pub symmetry_skip: u32,
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
        let rotation_mat = Rotation3::from_matrix_unchecked(Matrix3::from_columns(
            &[
                array_to_vec3(self.rotation.col_x),
                array_to_vec3(self.rotation.col_y),
                array_to_vec3(self.rotation.col_z),
            ],
        ));
        let quaternion = UnitQuaternion::from_rotation_matrix(&rotation_mat);
        Isometry3::from_parts(translation, quaternion)
    }
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
