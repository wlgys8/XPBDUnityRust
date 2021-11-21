use std::f32::NAN;

use nalgebra::{inf, Vector3};

use crate::constraints::{
    constraint::{Constraint, ConstraintData, ConstraintsData},
    particle::ParticlesDataBuilder,
    solver::ConstraintsSolver,
};

#[repr(C)]
#[derive(Debug)]
pub struct TriangleBendConstraint {
    pub p_indexes: [usize; 4],
    pub stiffness_inv: f32,
    rest: f32,
}

impl TriangleBendConstraint {
    pub fn new(p_indexes: [usize; 4], stiffness_inv: f32) -> TriangleBendConstraint {
        return TriangleBendConstraint {
            p_indexes: p_indexes,
            stiffness_inv: stiffness_inv,
            rest: 0.0,
        };
    }
    fn get_rest(&self, particles_data: &[crate::constraints::particle::ParticleData]) -> f32 {
        let (n1, n2) = self.get_n1_n2(particles_data);
        let n1_dot_n2 = n1.dot(&n2).max(-1.0).min(1.0);
        let rest = n1_dot_n2.acos();
        assert_ne!(rest, f32::NAN);
        return rest;
    }
    fn get_n1_n2(
        &self,
        particles_data: &[crate::constraints::particle::ParticleData],
    ) -> (Vector3<f32>, Vector3<f32>) {
        let p1 = &particles_data[self.p_indexes[0]].predict_position;
        let p2 = &particles_data[self.p_indexes[1]].predict_position;
        let p3 = &particles_data[self.p_indexes[2]].predict_position;
        let p4 = &particles_data[self.p_indexes[3]].predict_position;
        let p2 = p2 - p1;
        let p3 = p3 - p1;
        let p4 = p4 - p1;
        let n1 = p2.cross(&p3).normalize();
        let n2 = p2.cross(&p4).normalize();
        return (n1, n2);
    }

    fn get_q_and_d(
        &self,
        particles_data: &[crate::constraints::particle::ParticleData],
    ) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>, Vector3<f32>, f32) {
        let p1 = &particles_data[self.p_indexes[0]].predict_position;
        let p2 = &particles_data[self.p_indexes[1]].predict_position;
        let p3 = &particles_data[self.p_indexes[2]].predict_position;
        let p4 = &particles_data[self.p_indexes[3]].predict_position;

        let p2 = p2 - p1;
        let p3 = p3 - p1;
        let p4 = p4 - p1;
        let n1 = p2.cross(&p3).normalize();
        let n2 = p2.cross(&p4).normalize();
        let d = n1.dot(&n2).max(-1.0).min(1.0);

        let p2xp3_norm: f32 = p2.cross(&p3).norm();
        let p2xp4_norm: f32 = p2.cross(&p4).norm();
        debug_assert!(
            p2xp3_norm.is_finite() && p2xp4_norm.is_finite(),
            "p2xp3_norm = {},p2xp4_norm = {}",
            p2xp3_norm,
            p2xp4_norm
        );
        let q3 = (p2.cross(&n2) + n1.cross(&p2) * d) / p2xp3_norm;
        let q4 = (p2.cross(&n1) + n2.cross(&p2) * d) / p2xp4_norm;
        let q2 = -(p3.cross(&n2) + n1.cross(&p3) * d) / p2xp3_norm
            - (p4.cross(&n1) + n2.cross(&p4) * d) / p2xp4_norm;
        let q1 = -q2 - q3 - q4;

        return (q1, q2, q3, q4, d);
    }

    fn calculate_d_lambdas(
        &self,
        c_data: &mut ConstraintData,
        particles_data: &mut [crate::constraints::particle::ParticleData],
    ) -> f32 {
        let c_value = self.get_value(particles_data);
        let a = c_data.alpha;
        let lambda = c_data.lambda;
        let (q1, q2, q3, q4, d) = self.get_q_and_d(particles_data);
        let qs = [q1, q2, q3, q4];
        let one_minus_d2 = 1.0 - d * d;
        let mut sum_gwg = 0.;
        for (g_index, p_index) in self.get_pindexes().iter().enumerate() {
            let q = &qs[g_index];
            let p_data = &particles_data[*p_index];
            sum_gwg += p_data.w * q.dot(q);
        }

        let d_lambda_1 = -(a * lambda + c_value) * one_minus_d2;
        let d_lambda_2 = sum_gwg + a * one_minus_d2;

        if d_lambda_2 == 0.0 {
            return 0.0;
        }
        let d_lambda = d_lambda_1 / d_lambda_2;
        debug_assert!(!d_lambda.is_nan(), "d_lambda={}", d_lambda);
        return d_lambda;
    }
}

impl Constraint for TriangleBendConstraint {
    fn on_initialize(&mut self, particles_data: &[crate::constraints::particle::ParticleData]) {
        self.rest = self.get_rest(particles_data);
    }
    fn get_stiffness_inv(&self) -> f32 {
        return self.stiffness_inv;
    }
    fn get_value(&self, particles_data: &[crate::constraints::particle::ParticleData]) -> f32 {
        return self.get_rest(particles_data) - self.rest;
    }
    fn get_grad(
        &self,
        particles_data: &[crate::constraints::particle::ParticleData],
        out_grads: &mut [nalgebra::Vector3<f32>],
    ) {
        let (q1, q2, q3, q4, d) = self.get_q_and_d(particles_data);
        let e = (1.0 / (1.0 - d * d).sqrt()).min(f32::MAX).max(f32::MIN);
        out_grads[0] = e * q1;
        out_grads[1] = e * q2;
        out_grads[2] = e * q3;
        out_grads[3] = e * q4;
    }

    fn get_pindexes(&self) -> &[usize] {
        return &self.p_indexes;
    }

    fn get_constraint_particle_count() -> usize {
        return 4;
    }
}

impl ConstraintsSolver for ConstraintsData<TriangleBendConstraint> {
    fn calculate_d_lambdas(
        &mut self,
        particles_data: &mut [crate::constraints::particle::ParticleData],
    ) {
        for (c_index, cs) in self.defines.iter().enumerate() {
            let c_data = &mut self.datas[c_index];
            let d_lambda = cs.calculate_d_lambdas(c_data, particles_data);
            self.datas[c_index].d_lambda = d_lambda;
        }
    }
}

#[test]
fn test() {
    let positions = vec![
        point![0.0, 0.0, 0.0],
        point![1.0, -0.5, 0.0],
        point![1.0, 0.0, 1.0],
        point![0.0, -0.5, 1.0],
    ];
    let masses = vec![1.0, 1.0, 1.0, 1.0];

    let particles_data = ParticlesDataBuilder {
        positions: positions,
        masses: masses,
    }
    .build()
    .unwrap();

    let cs = TriangleBendConstraint {
        p_indexes: [0, 2, 1, 3],
        stiffness_inv: 0.,
        rest: 3.1415926,
    };

    println!("c_value = {}", cs.get_value(&particles_data));

    // assert_eq!(cs.get_rest(&particles_data), (-1.0 as f32).acos());

    let mut grads = vec![Vector3::zeros(); 4];
    cs.get_grad(&particles_data, &mut grads);
    println!("{:?}", grads);
}
