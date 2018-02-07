use std::f64::consts::PI;
use std::iter::once;
use std::fmt::Debug;

use nalgebra::{Vector2, Vector3, Isometry3, Point3};
use joint::JointSpec;
use symmetry::symmetries;

// Swap to collision_grid to enable experimental spatial partitioning optimization
use collision_grid_trivial::{CollisionGrid, Bounds, BoundingBox, BoundedCollider, CheckCollision};

#[derive(Clone, Copy, Debug)]
enum PointsState {
    FirstPoint,
    MainPoints(Option<Isometry3<f64>>),
    Done,
}

#[derive(Clone, Copy, Debug)]
pub struct Points<I> {
    state: PointsState,
    spec: JointSpec,
    joints: I,
}

impl<I: Iterator<Item = Isometry3<f64>>> Iterator for Points<I> {
    type Item = Point3<f64>;

    fn next(&mut self) -> Option<Point3<f64>> {
        match self.state {
            PointsState::FirstPoint => {
                self.state = PointsState::MainPoints(None);
                Some(Point3::origin())
            }
            PointsState::MainPoints(prev_trans) => {
                match self.joints.next() {
                    Some(curr_trans) => {
                        self.state = PointsState::MainPoints(Some(curr_trans));
                        Some(Point3 { coords: curr_trans.translation.vector })
                    }
                    None => {
                        self.state = PointsState::Done;
                        Some(Point3 {
                            coords: (prev_trans.expect("Should have at least one joint") *
                                         self.spec.origin_to_out())
                                .translation
                                .vector,
                        })
                    }
                }
            }
            PointsState::Done => None,
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let (lower, upper) = self.joints.size_hint();
        (
            if lower > 0 { lower + 2 } else { 0 },
            upper.map(|u| if u > 0 { u + 2 } else { 0 }),
        )
    }
}

pub fn points<I: Iterator<Item = Isometry3<f64>>>(spec: JointSpec, joints: I) -> Points<I> {
    Points {
        state: PointsState::FirstPoint,
        spec,
        joints,
    }
}

struct WithSizeHint<I> {
    iter: I,
    size_hint: (usize, Option<usize>),
}

impl<I: Iterator> Iterator for WithSizeHint<I> {
    type Item = I::Item;

    fn next(&mut self) -> Option<I::Item> {
        self.iter.next()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.size_hint
    }

    // TODO: Assess performance implications of only implementing these two methods and not
    // forwarding other methods on the wrapped iterator.

    // FIXME: must adapt to underlying size!  Currently only valid when queried at start!

    // This is a useful prototyping tool, but it would be better to move to a more robust
    // size-passing system eventually.
}

fn with_size_hint<I: Iterator>(iter: I, size_hint: (usize, Option<usize>)) -> WithSizeHint<I> {
    WithSizeHint { iter, size_hint }
}

pub fn include_midpoints<I: Iterator<Item = Point3<f64>>>(
    it: I,
) -> impl Iterator<Item = Point3<f64>> {
    let mut prev_point_opt = None;
    let (lower, upper) = it.size_hint();
    with_size_hint(
        it.flat_map(move |point| {
            let midpoint = prev_point_opt.map(|prev_point| prev_point + (point - prev_point) * 0.5);
            prev_point_opt = Some(point);
            midpoint.into_iter().chain(once(point))
        }),
        (
            if lower >= 2 { lower * 2 - 1 } else { lower },
            upper.map(|u| if u >= 2 { u * 2 - 1 } else { u }),
        ),
    )
}

#[derive(Clone, Copy, Debug)]
pub struct WindingAngles {
    prev_vec: Option<Vector2<f64>>,
    total: f64,
}

impl WindingAngles {
    pub fn new() -> Self {
        WindingAngles {
            prev_vec: None,
            total: 0.0,
        }
    }

    pub fn next_point(&mut self, point: Point3<f64>) {
        let vec = Vector2::new(point.x, point.y);
        if let Some(prev_vec) = self.prev_vec {
            // These "cosine" and "sine" values are in fact scaled by the product of the lengths of
            // the two vectors, but atan2 doesn't care about scaling.
            let rel_cos = prev_vec.dot(&vec);
            let rel_sin = prev_vec.perp(&vec);
            self.total += rel_sin.atan2(rel_cos);
        }
        self.prev_vec = Some(vec);
    }

    pub fn total(&self) -> f64 {
        self.total
    }
}

#[derive(Clone, Copy, Debug)]
struct CollisionSphere {
    center: Point3<f64>,
    radius: f64,
    index: u32,
    point_count: u32,
    symmetry_index: u32,
    symmetry_count: u32,
    skip: u32,
}

impl BoundedCollider for CollisionSphere {
    fn bounding_box(&self) -> BoundingBox {
        BoundingBox {
            x: Bounds {
                min: self.center.x - self.radius,
                max: self.center.x + self.radius,
            },
            y: Bounds {
                min: self.center.y - self.radius,
                max: self.center.y + self.radius,
            },
            z: Bounds {
                min: self.center.z - self.radius,
                max: self.center.z + self.radius,
            },
        }
    }
}

fn one_apart(a: u32, b: u32) -> bool {
    a + 1 == b || a == b + 1
    // TODO: Adaptable neighbor test depth
    || a + 2 == b || a == b + 2
}

fn same_horseshoe_partner(symmetry_count: u32, symmetry_index: u32) -> u32 {
    if symmetry_index % 2 == 0 {
        (symmetry_index + 1) % (symmetry_count * 2)
    } else {
        (symmetry_index - 1) % (symmetry_count * 2)
    }
}

fn opposing_horseshoe_partner(symmetry_count: u32, skip: u32, symmetry_index: u32) -> u32 {
    if symmetry_index % 2 == 0 {
        (symmetry_index + skip * 2 + 1) % (symmetry_count * 2)
    } else {
        (symmetry_index + symmetry_count * 2 - skip * 2 - 1) % (symmetry_count * 2)
    }
}

fn connected_branches(
    symmetry_count: u32,
    skip: u32,
    symmetry_index1: u32,
    symmetry_index2: u32,
) -> bool {
    same_horseshoe_partner(symmetry_count, symmetry_index1) == symmetry_index2 ||
        opposing_horseshoe_partner(symmetry_count, skip, symmetry_index1) == symmetry_index2
}

fn is_extreme(x: u32, count: u32) -> bool {
    x == 0 || x + 1 == count
    // TOTDO: adaptable neighbor test depth
    || x + 2 == count || x == 1
}

// Small sylistic convenience, to avoid the confusion of having to arbitrarily refer to one of two
// values after establishing that they are in fact identical.
fn assert_same<T: Debug + PartialEq>(a: T, b: T) -> T {
    debug_assert_eq!(a, b);
    a
}

impl CheckCollision<CollisionSphere> for CollisionSphere {
    type CollisionData = CollisionSphere;

    fn check_collision(&self, other: &CollisionSphere) -> Option<Self::CollisionData> {
        let point_count = assert_same(self.point_count, other.point_count);
        let symmetry_count = assert_same(self.symmetry_count, other.symmetry_count);
        let skip = assert_same(self.skip, other.skip);

        let protected = if self.symmetry_index == other.symmetry_index {
            // Points on the same symmetry branch are collision-protected iff they are the same
            // point or immediate neighbors.
            self.index == other.index || one_apart(self.index, other.index)
        } else if connected_branches(
            symmetry_count,
            skip,
            self.symmetry_index,
            other.symmetry_index,
        )
        {
            // Points on adjacent symmetry branches are collision-protected iff they are the same
            // initial or final point (because they are then meant to exactly overlap in space), or
            // one is an initial or final point and the other is its immediate neighbor (because one
            // is then meant to exactly overlap the other's immediate neighbor).
            let self_extreme = is_extreme(self.index, point_count);
            let other_extreme = is_extreme(other.index, point_count);
            let overlapping = self_extreme && other_extreme && self.index == other.index;
            let neighbors = (self_extreme || other_extreme) && one_apart(self.index, other.index);
            overlapping || neighbors
        } else {
            false
        };

        let colliding = if protected {
            false
        } else {
            let total_rad = self.radius + other.radius;
            let dist_squ = (other.center - self.center).norm_squared();
            dist_squ < total_rad * total_rad
        };

        if colliding { Some(*other) } else { None }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionOutcome {
    Collision,
    NoCollisions,
}

fn exact_size<I: Iterator>(it: &I) -> usize {
    // Adapted from https://github.com/rust-lang/rust/blob/70f7d5842f29d4900f24420b030f144d21f3c5fc/src/libcore/iter/traits.rs#L687-L695
    let (lower, upper) = it.size_hint();
    if upper == Some(lower) {
        lower
    } else {
        panic!(
            "Iterator must be exact-size; got size bounds {} {:?}",
            lower,
            upper
        )
    }
}

// Set to true to enable symmetry-based spatial partitioning optimization
const USE_WEDGE: bool = false;

pub fn collisions_with_symmetry<I: Iterator<Item = Point3<f64>>>(
    symmetry_count: u32,
    skip: u32,
    centers: I,
    radius: f64,
) -> CollisionOutcome {
    // TODO: Calculate all global parameters outside function.  Possibly factor out "symmetry"
    // object wich does caching similar to JointSpec of cos, sin, vector isometries, etc?

    let wedge_plane1_normal = Vector3::new(
        -(2.0 * PI / (symmetry_count as f64)).sin(),
        (2.0 * PI / (symmetry_count as f64)).cos(),
        0.0,
    );

    let wedge_plane2_normal = Vector3::new(0.0, -1.0, 0.0);

    let symms = symmetries(symmetry_count).collect::<Vec<_>>();

    let mut grid = CollisionGrid::new(radius);

    let point_count = exact_size(&centers);

    for (index, center) in centers.enumerate() {
        for (symm_index, symm) in symms.iter().enumerate() {
            let symm_center = symm * center;
            if !USE_WEDGE ||
                (symm_center.coords.dot(&wedge_plane1_normal) <= radius &&
                     symm_center.coords.dot(&wedge_plane2_normal) <= radius)
            {
                let sphere = CollisionSphere {
                    center: symm_center,
                    radius,
                    index: index as u32,
                    point_count: point_count as u32,
                    symmetry_index: symm_index as u32,
                    symmetry_count,
                    skip,
                };

                let collisions = grid.collisions(&sphere);

                if collisions.len() > 0 {
                    for collision in collisions {
                        println!("Collision on: {} {} [syms {} {}]",
                            sphere.index,
                            collision.index,
                            sphere.symmetry_index,
                            collision.symmetry_index,
                        );
                    }
                    return CollisionOutcome::Collision;
                }

                grid.add(&sphere)
            }
        }
    }

    CollisionOutcome::NoCollisions
}

#[cfg(test)]
mod test {
    use filter::*;
    use std::f64::consts::PI;

    #[test]
    fn single_joint_points() {
        let spec = JointSpec::new(1.0, 1.0, PI / 4.0, 0.5);
        let joints = [spec.in_to_origin()];
        let points_vec: Vec<_> = points(spec, joints.iter().cloned()).collect();
        assert_relative_eq!(points_vec[0], Point3::origin());
        assert_relative_eq!(points_vec[1], Point3::new(0.0, 1.0, 0.0));
        assert_relative_eq!(
            points_vec[2],
            Point3::new(-(0.5f64).sqrt(), 1.0 + (0.5f64).sqrt(), 0.0)
        );
    }

    #[test]
    fn triplet_winding_angle() {
        let mut winding_angles = WindingAngles::new();

        winding_angles.next_point(Point3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(winding_angles.total(), 0.0);

        winding_angles.next_point(Point3::new(1.0, 0.5, 100.0));
        assert_relative_eq!(winding_angles.total(), (0.5f64).atan());

        winding_angles.next_point(Point3::new(2.0, 2.0, -70.0));
        assert_relative_eq!(winding_angles.total(), PI / 4.0);
    }

    #[test]
    fn trefoil_collision_protections() {
        // Convenience function to generate spheres representing points on a 5-element trefoil, for
        // the purpose of testing neighbor protections.
        fn sphere(index: u32, symmetry_index: u32) -> CollisionSphere {
            CollisionSphere {
                center: Point3::new(0.0, 0.0, 0.0),
                radius: 1.0,
                point_count: 7,
                symmetry_count: 3,
                skip: 2,

                index,
                symmetry_index,
            }
        }

        // Convenience function for testing collisions
        fn collide(sphere1: CollisionSphere, sphere2: CollisionSphere) -> bool {
            let collide_left_to_right = sphere1.check_collision(&sphere2).is_some();
            let collide_right_to_left = sphere2.check_collision(&sphere1).is_some();
            assert_same(collide_left_to_right, collide_right_to_left)
        }


        assert_eq!(same_horseshoe_partner(3, 0), 1);
        assert_eq!(same_horseshoe_partner(3, 1), 0);
        assert_eq!(same_horseshoe_partner(3, 2), 3);
        assert_eq!(same_horseshoe_partner(3, 3), 2);
        assert_eq!(same_horseshoe_partner(3, 4), 5);
        assert_eq!(same_horseshoe_partner(3, 5), 4);

        assert_eq!(opposing_horseshoe_partner(3, 2, 0), 5);
        assert_eq!(opposing_horseshoe_partner(3, 2, 1), 2);
        assert_eq!(opposing_horseshoe_partner(3, 2, 2), 1);
        assert_eq!(opposing_horseshoe_partner(3, 2, 3), 4);
        assert_eq!(opposing_horseshoe_partner(3, 2, 4), 3);
        assert_eq!(opposing_horseshoe_partner(3, 2, 5), 0);

        // Self-protection
        assert!(!collide(sphere(0, 0), sphere(0, 0)));

        // Same-branch neighbor protection
        assert!(!collide(sphere(1, 0), sphere(0, 0)));
        assert!(!collide(sphere(1, 0), sphere(2, 0)));

        // Same-horseshoe symmetrized counterpart protection
        assert!(!collide(sphere(0, 0), sphere(0, 1)));

        // Opposing-horseshoe symmetrized counterpart protection
        assert!(!collide(sphere(6, 0), sphere(6, 5)));
        assert!(!collide(sphere(6, 1), sphere(6, 2)));

        // Same-horseshoe symmetrized neighbor protection
        assert!(!collide(sphere(0, 0), sphere(1, 1)));
        assert!(!collide(sphere(0, 1), sphere(1, 0)));

        // Opposing-horseshoe symmetrized neighbor protection
        assert!(!collide(sphere(6, 0), sphere(5, 5)));
        assert!(!collide(sphere(5, 0), sphere(6, 5)));
    }
}
