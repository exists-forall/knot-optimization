// Designed for API compatibility with collision_grid, so that imports can be easily swapped out to
// experiment with the effects of spatial partitioning optimization.  May be worth using true
// conditional compilation for this at some point.

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
    colliders: Vec<T>,
}

impl<T> CollisionGrid<T> {
    pub fn new(_: f64) -> Self {
        CollisionGrid { colliders: Vec::new() }
    }

    pub fn add(&mut self, collider: &T)
    where
        T: Clone + BoundedCollider,
    {
        self.colliders.push(collider.clone());
    }

    pub fn collisions<U: CheckCollision<T>>(&self, test_collider: &U) -> Vec<U::CollisionData> {
        let mut results = Vec::new();

        for collider in &self.colliders {
            if let Some(collision) = test_collider.check_collision(collider) {
                results.push(collision);
            }
        }

        results
    }
}
