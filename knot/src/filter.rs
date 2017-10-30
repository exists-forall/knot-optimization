use nalgebra::{Vector2, Isometry3};
use joint::JointSpec;

/// Returns the angle by which a given joint "winds" around the origin if it is situated at
/// the given position and orientation, between `-PI` and `PI`.
pub fn winding_angle(spec: &JointSpec, trans: &Isometry3<f64>) -> f64 {
    let in_vec_3 = (trans * spec.origin_to_in()).translation.vector;
    let out_vec_3 = (trans * spec.origin_to_out()).translation.vector;

    let in_vec_2 = Vector2::new(in_vec_3.x, in_vec_3.y);
    let out_vec_2 = Vector2::new(out_vec_3.x, out_vec_3.y);

    // These "cosine" and "sine" values are in fact scaled by the product of the lengths of the
    // two vectors, but atan2 doesn't care about scaling.
    let rel_cos = in_vec_2.dot(&out_vec_2);
    let rel_sin = in_vec_2.perp(&out_vec_2);

    rel_sin.atan2(rel_cos)
}
