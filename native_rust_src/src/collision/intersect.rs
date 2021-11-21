use nalgebra::{Point3, SimdPartialOrd, SimdValue, Vector3};

use super::shapes::{InfinitePlane, Sphere};

type Point = Point3<f32>;

const NORMALIZE_EPS: f32 = 0.001;

pub struct ContactInfo {
    pub contacted: bool,
    pub contact_position: Point,
    pub contact_normal: Vector3<f32>,
}

const CONTACTED_FALSE: ContactInfo = ContactInfo {
    contacted: false,
    contact_position: point![0., 0., 0.],
    contact_normal: vector![0., 0., 0.],
};

pub trait GetClosestSurfacePoint {
    fn get_closest_surface_point(&self, p: &Point) -> ContactInfo;
}

impl GetClosestSurfacePoint for Sphere {
    fn get_closest_surface_point(&self, p: &Point) -> ContactInfo {
        let c2p = p - self.center;
        let d2 = c2p.dot(&c2p);
        let r2 = self.radius * self.radius;
        if d2 < r2 {
            let normal = c2p
                .try_normalize(NORMALIZE_EPS)
                .unwrap_or(vector![0., 1., 0.]);
            return ContactInfo {
                contacted: true,
                contact_normal: normal,
                contact_position: self.center + normal * self.radius,
            };
        } else {
            return CONTACTED_FALSE;
        }
    }
}

impl GetClosestSurfacePoint for InfinitePlane {
    fn get_closest_surface_point(&self, p: &Point) -> ContactInfo {
        let pv: Vector3<f32> = vector![p.x, p.y, p.z];
        let d = self.normal.dot(&pv) + self.origin_to_plane;
        if d > 0.0 {
            return CONTACTED_FALSE;
        } else {
            return ContactInfo {
                contacted: true,
                contact_normal: self.normal,
                contact_position: p - self.normal * d,
            };
        }
    }
}
