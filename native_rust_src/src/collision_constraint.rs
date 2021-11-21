use nalgebra::{distance, Point3, Vector3};

use crate::constraints::{constraint::Constraint, particle::ParticleData};

pub struct CollisionConstraint {
    pub concat_position: Point3<f32>,
    pub concat_normal: Vector3<f32>,
    p_index: [usize; 1],
}

impl CollisionConstraint {
    pub fn new(
        p_index: usize,
        concat_position: Point3<f32>,
        normal: Vector3<f32>,
    ) -> CollisionConstraint {
        return CollisionConstraint {
            p_index: [p_index],
            concat_position: concat_position,
            concat_normal: normal,
        };
    }
}

const EPS: f32 = 0.001;

impl Constraint for CollisionConstraint {
    fn on_initialize(&mut self, _: &[ParticleData]) {}
    fn get_stiffness_inv(&self) -> f32 {
        return 0.;
    }
    fn get_value(&self, particles_data: &[ParticleData]) -> f32 {
        let pos = &particles_data[self.p_index[0]].predict_position;
        return (self.concat_position - pos).dot(&self.concat_normal).max(0.0)
    }

    fn get_grad(&self, particles_data: &[ParticleData], out_grads: &mut [Vector3<f32>]) {
        let pos = &particles_data[self.p_index[0]].predict_position;
        let target_pos = &self.concat_position;
        let norm_result = (pos - target_pos).try_normalize(EPS);
        match norm_result {
            Some(grad) => {
                out_grads[0] = grad;
            }
            None => out_grads[0] = Vector3::identity(),
        }
    }
    fn get_pindexes(&self) -> &[usize] {
        return &self.p_index;
    }

    fn get_constraint_particle_count() -> usize {
        return 1;
    }
}
