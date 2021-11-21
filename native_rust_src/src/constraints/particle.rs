use nalgebra::{Point3, Vector3};

pub mod particle_flags_index {
    pub const ATTACHED: u8 = 0;
}
pub mod particle_flags {
    use super::particle_flags_index;
    pub const ATTACHED: u8 = 1 << particle_flags_index::ATTACHED;
}

#[derive(Clone, Debug)]
pub struct ParticleData {
    pub velocity: Vector3<f32>,
    pub position: Point3<f32>,
    pub predict_position: Point3<f32>,
    pub d_position: Vector3<f32>,
    pub w: f32, // 1 / mass
    pub flag: u8,
}

pub struct ParticlesDataBuilder {
    pub positions: Vec<Point3<f32>>,
    pub masses: Vec<f32>,
}

impl ParticlesDataBuilder {
    pub fn build(&self) -> Result<Vec<ParticleData>, &'static str> {
        if self.positions.len() != self.masses.len() {
            return Err("positions.len() != masses.len()");
        }
        let mut particles_data = Vec::<ParticleData>::with_capacity(self.positions.len());
        for (index, p) in self.positions.iter().enumerate() {
            particles_data.push(ParticleData {
                position: *p,
                predict_position: *p,
                w: 1.0 / self.masses[index],
                velocity: Default::default(),
                d_position: Default::default(),
                flag: 0,
            });
        }
        return Ok(particles_data);
    }
}

pub trait ParticlesCleaner {
    fn clear_d_positions(&mut self);
}

impl ParticlesCleaner for [ParticleData] {
    fn clear_d_positions(&mut self) {
        for e in self.iter_mut() {
            e.d_position = Vector3::zeros();
        }
    }
}
