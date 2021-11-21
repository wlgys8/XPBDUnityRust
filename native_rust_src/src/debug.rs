use crate::constraints::{constraint::ConstraintData, particle::ParticleData};

pub trait ParticlesDebug {
    fn print_predict_positions(&self);
    fn print_d_positions(&self);
    fn print_velocities(&self);
}

pub trait ConstraintsDebug {
    fn print_d_lambdas(&self);
}

impl ParticlesDebug for [ParticleData] {
    fn print_predict_positions(&self) {
        if cfg!(debug_assertions) {
            println!("predict_positions:");
            for (idx, p) in self.iter().enumerate() {
                println!("{}-{}", idx, p.predict_position);
            }
        }
    }

    fn print_d_positions(&self) {
        if cfg!(debug_assertions) {
            println!("d_positions:");
            for (idx, p) in self.iter().enumerate() {
                println!("{}-{}", idx, p.d_position);
            }
        }
    }

    fn print_velocities(&self) {
        if cfg!(debug_assertions) {
            println!("velocities:");
            for (idx, p) in self.iter().enumerate() {
                println!("{}-{}", idx, p.velocity);
            }
        }
    }
}

impl ConstraintsDebug for [ConstraintData] {
    fn print_d_lambdas(&self) {
        println!("d_lambdas:");
        for (idx, c) in self.iter().enumerate() {
            println!("{} - {}", idx, c.d_lambda);
        }
    }
}
