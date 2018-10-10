use std::collections::HashMap;
use std::ops::RangeInclusive;

use nalgebra::Point3;

#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct BoundingBox {
    pub x: Bounds,
    pub y: Bounds,
    pub z: Bounds,
}

pub trait BoundedCollider {
    fn bounding_box(&self) -> BoundingBox;
}

pub trait CheckCollision<T>: BoundedCollider {
    type CollisionData;

    fn check_collision(&self, other: &T) -> Option<Self::CollisionData>;
}

#[derive(Clone, Debug)]
pub struct CollisionGrid<T> {
    cell_size: f64,
    cells: HashMap<Point3<i32>, Vec<T>>,
}

impl<T> CollisionGrid<T> {
    pub fn new(cell_size: f64) -> Self {
        CollisionGrid {
            cell_size,
            cells: HashMap::new(),
        }
    }

    fn coord_index(&self, coord: f64) -> i32 {
        (coord / self.cell_size).floor() as i32
    }

    fn bounds_indices(&self, bounds: Bounds) -> RangeInclusive<i32> {
        (self.coord_index(bounds.min)..=self.coord_index(bounds.max))
    }

    pub fn add(&mut self, collider: &T)
    where
        T: Clone + BoundedCollider,
    {
        let bounding_box = collider.bounding_box();

        for cell_x in self.bounds_indices(bounding_box.x) {
            for cell_y in self.bounds_indices(bounding_box.y) {
                for cell_z in self.bounds_indices(bounding_box.z) {
                    self.cells
                        .entry(Point3::new(cell_x, cell_y, cell_z))
                        .or_insert_with(|| Vec::new())
                        .push(collider.clone());
                }
            }
        }
    }

    pub fn collisions<U: CheckCollision<T>>(&self, test_collider: &U) -> Vec<U::CollisionData> {
        let mut results = Vec::new();

        let bounding_box = test_collider.bounding_box();

        for cell_x in self.bounds_indices(bounding_box.x) {
            for cell_y in self.bounds_indices(bounding_box.y) {
                for cell_z in self.bounds_indices(bounding_box.z) {
                    if let Some(cell) = self.cells.get(&Point3::new(cell_x, cell_y, cell_z)) {
                        for collider in cell {
                            if let Some(collision) = test_collider.check_collision(collider) {
                                results.push(collision);
                            }
                        }
                    }
                }
            }
        }

        results
    }
}
