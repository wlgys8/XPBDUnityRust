use nalgebra::{distance, Point3, Vector3};

use crate::constraints::{
    constraint::{Constraint, ConstraintsData},
    particle::ParticleData,
    solver::ConstraintsSolver,
};

#[repr(C)]
pub struct DistanceConstraint {
    p_indexes: [usize; 2],
    stiffness_inv: f32,
    rest: f32,
}

impl DistanceConstraint {
    pub fn new(p1: usize, p2: usize, stiffness_inv: f32) -> DistanceConstraint {
        return DistanceConstraint {
            p_indexes: [p1, p2],
            stiffness_inv: stiffness_inv,
            rest: 0.0,
        };
    }

    fn distance(&self, particles: &[ParticleData]) -> f32 {
        let p1 = particles[self.p_indexes[0]].predict_position;
        let p2 = particles[self.p_indexes[1]].predict_position;
        return distance(&p1, &p2);
    }
}

impl Constraint for DistanceConstraint {
    fn on_initialize(&mut self, particles_data: &[ParticleData]) {
        self.rest = self.distance(particles_data);
    }
    fn get_stiffness_inv(&self) -> f32 {
        return self.stiffness_inv;
    }
    fn get_value(&self, particles: &[ParticleData]) -> f32 {
        return self.distance(particles) - self.rest;
    }
    fn get_grad(&self, particles: &[ParticleData], out_grads: &mut [Vector3<f32>]) {
        let p0 = &particles[self.p_indexes[0]].predict_position;
        let p1 = &particles[self.p_indexes[1]].predict_position;
        let n = (p0 - p1).normalize();
        out_grads[0] = n;
        out_grads[1] = -n;
    }
    fn get_pindexes(&self) -> &[usize] {
        return &self.p_indexes;
    }
    fn get_constraint_particle_count() -> usize {
        return 2;
    }
}

impl ConstraintsSolver for ConstraintsData<DistanceConstraint> {
    fn calculate_d_lambdas(
        &mut self,
        particles_data: &mut [crate::constraints::particle::ParticleData],
    ) {
        for (c_index, cs) in self.defines.iter().enumerate() {
            let c_data = &mut self.datas[c_index];
            let c_value = cs.get_value(particles_data);
            let a = c_data.alpha;
            let lambda = c_data.lambda;
            let mut sum_gwg = 0.;
            for (g_index, p_index) in cs.get_pindexes().iter().enumerate() {
                let grad = &c_data.grads[g_index];
                let p_data = &particles_data[*p_index];
                sum_gwg += p_data.w;
            }
            let d_lambda = -(a * lambda + c_value) / (sum_gwg + a);
            self.datas[c_index].d_lambda = d_lambda;
        }
    }
}
