use nalgebra::{Point3, Vector3};

#[repr(C)]
pub struct Sphere {
    pub center: Point3<f32>,
    pub radius: f32,
}

#[repr(C)]
pub struct InfinitePlane {
    pub normal: Vector3<f32>,
    pub origin_to_plane: f32,
}

pub struct ShapeSet<T> {
    shapes: Vec<T>,
}

impl<T> ShapeSet<T> {
    pub fn add(&mut self, shape: T) -> usize {
        let index = self.shapes.len();
        self.shapes.push(shape);
        return index;
    }
    pub fn remove(&mut self, index: usize) -> Option<T> {
        if index < self.shapes.len() {
            Some(self.shapes.swap_remove(index))
        } else {
            None
        }
    }

    pub fn shapes(&self) -> &[T] {
        return &self.shapes;
    }

    pub fn clear(&mut self) {
        self.shapes.clear();
    }
}

impl<T> Default for ShapeSet<T> {
    fn default() -> Self {
        Self { shapes: vec![] }
    }
}

#[derive(Default)]
pub struct ShapesManager {
    pub spheres: ShapeSet<Sphere>,
    pub infinite_planes: ShapeSet<InfinitePlane>,
}

impl ShapesManager {
    pub fn clear(&mut self) {
        self.spheres.clear();
        self.infinite_planes.clear();
    }
}
