#![feature(conservative_impl_trait)]

extern crate nalgebra;
extern crate alga;
extern crate kiss3d;
extern crate knot;

pub mod joint_render;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
