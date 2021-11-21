use std::f32::NAN;

use crate::{
    constraints::particle::ParticlesCleaner,
    debug::{ConstraintsDebug, ParticlesDebug},
};

use super::{
    constraint::{Constraint, ConstraintData, ConstraintsData},
    particle::ParticleData,
};

pub trait ConstraintsSolver {
    fn initialize(&mut self, particles_data: &[ParticleData]);
    fn calculate_constraints_gradients(&mut self, particles_data: &mut [ParticleData]);
    fn clear_lambdas(&mut self);
    fn update_lambdas(&mut self);
    fn calculate_cache(&mut self, particles_data: &mut [ParticleData], dt: f32);
    fn calculate_d_lambdas(&mut self, particles_data: &mut [ParticleData]);
    fn calculate_d_positions(&mut self, particles_data: &mut [ParticleData]);

    fn solve(&mut self, particles_data: &mut [ParticleData], dt: f32, iterate_count: u32) {
        //clear lambdas to 0.0
        self.clear_lambdas();
        //calculate caches if not
        self.calculate_cache(particles_data, dt);
        //begin solve iterations
        for _ in 0..iterate_count {
            self.calculate_constraints_gradients(particles_data);
            //calculate delta lambdas
            self.calculate_d_lambdas(particles_data);
            //calculate delta positions
            particles_data.clear_d_positions();
            self.calculate_d_positions(particles_data);
            //update predict positions
            for p in particles_data.iter_mut() {
                p.predict_position += p.d_position;
            }
            //update lambdas
            self.update_lambdas();
        }
    }
}

impl<T> ConstraintsSolver for ConstraintsData<T>
where
    T: Constraint,
{
    fn initialize(&mut self, particles_data: &[ParticleData]) {
        for cs in &mut self.defines {
            cs.on_initialize(particles_data)
        }
    }

    fn calculate_constraints_gradients(&mut self, particles_data: &mut [ParticleData]) {
        for (c_index, cs) in self.defines.iter().enumerate() {
            let c_data = &mut self.datas[c_index];
            cs.get_grad(particles_data, &mut c_data.grads);
        }
    }

    fn clear_lambdas(&mut self) {
        for e in self.datas.iter_mut() {
            e.lambda = 0.
        }
    }
    default fn calculate_d_lambdas(&mut self, particles_data: &mut [ParticleData]) {
        for (c_index, cs) in self.defines.iter().enumerate() {
            let c_data = &mut self.datas[c_index];
            let c_value = cs.get_value(particles_data);
            let a = c_data.alpha;
            let lambda = c_data.lambda;
            let mut sum_gwg = 0.;
            for (g_index, p_index) in cs.get_pindexes().iter().enumerate() {
                let grad = &c_data.grads[g_index];
                let p_data = &particles_data[*p_index];
                sum_gwg += p_data.w * grad.dot(grad).min(f32::MAX).max(f32::MIN);
            }
            let d_lambda = -(a * lambda + c_value) / (sum_gwg + a).max(0.0001).min(f32::MAX);
            self.datas[c_index].d_lambda = d_lambda;
        }
    }
    fn update_lambdas(&mut self) {
        for c in self.datas.iter_mut() {
            c.lambda += c.d_lambda;
        }
    }

    fn calculate_cache(&mut self, particles_data: &mut [ParticleData], dt: f32) {
        ConstraintsData::calculate_cache(self, particles_data, dt);
    }

    fn calculate_d_positions(&mut self, particles_data: &mut [ParticleData]) {
        // self.datas.print_d_lambdas();
        for (c_index, c) in self.defines.iter().enumerate() {
            let c_data = &mut self.datas[c_index];
            for (idx, p_index) in c.get_pindexes().iter().enumerate() {
                let p_data = &mut particles_data[*p_index];
                let grad = &c_data.grads[idx];
                let d = p_data.w * c_data.d_lambda * grad;
                p_data.d_position += d;
            }
        }
    }
}

impl<T> ConstraintsSolver for &mut T
where
    T: ConstraintsSolver,
{
    fn initialize(&mut self, particles_data: &[ParticleData]) {
        (self as &mut T).initialize(particles_data);
    }
    fn calculate_constraints_gradients(&mut self, particles_data: &mut [ParticleData]) {
        (self as &mut T).calculate_constraints_gradients(particles_data);
    }

    fn clear_lambdas(&mut self) {
        (self as &mut T).clear_lambdas();
    }

    fn update_lambdas(&mut self) {
        (self as &mut T).update_lambdas();
    }

    fn calculate_cache(&mut self, particles_data: &mut [ParticleData], dt: f32) {
        (self as &mut T).calculate_cache(particles_data, dt);
    }

    fn calculate_d_lambdas(&mut self, particles_data: &mut [ParticleData]) {
        (self as &mut T).calculate_d_lambdas(particles_data);
    }

    fn calculate_d_positions(&mut self, particles_data: &mut [ParticleData]) {
        (self as &mut T).calculate_d_positions(particles_data);
    }
}

macro_rules! define_constraints_solver {
    ( $($head:ident,$index:tt),*)=>{
        impl<$($head),*> ConstraintsSolver for ($($head),*)
        where
            $($head: ConstraintsSolver),*
        {
            fn initialize(&mut self,particles_data:&[ParticleData]){
                $(
                    self.$index.initialize(particles_data);
                )*
            }
            fn calculate_constraints_gradients(&mut self, particles_data: &mut [ParticleData]) {
                $(
                    self.$index.calculate_constraints_gradients(particles_data);
                )*
            }

            fn clear_lambdas(&mut self) {
                $(
                    self.$index.clear_lambdas();
                )*
            }

            fn update_lambdas(&mut self) {
                $(
                    self.$index.update_lambdas();
                )*
            }

            fn calculate_cache(&mut self, particles_data: &mut [ParticleData], dt: f32) {
                $(
                    self.$index.calculate_cache(particles_data, dt);
                )*
            }

            fn calculate_d_lambdas(&mut self, particles_data: &mut [ParticleData]) {
                $(
                    self.$index.calculate_d_lambdas(particles_data);
                )*
            }

            fn calculate_d_positions(&mut self, particles_data: &mut [ParticleData]) {
                $(
                    self.$index.calculate_d_positions(particles_data);
                )*
            }
        }
    };
}

define_constraints_solver!(A, 0, B, 1);
define_constraints_solver!(A, 0, B, 1, C, 2);
define_constraints_solver!(A, 0, B, 1, C, 2, D, 3);
