use nalgebra::Vector3;

use super::particle::ParticleData;

pub trait Constraint {
    fn on_initialize(&mut self, particles_data: &[ParticleData]);
    fn get_stiffness_inv(&self) -> f32;
    fn get_constraint_particle_count() -> usize;
    fn get_pindexes(&self) -> &[usize];
    fn get_value(&self, particles_data: &[ParticleData]) -> f32;
    fn get_grad(&self, particles_data: &[ParticleData], out_grads: &mut [Vector3<f32>]);
}

#[derive(Clone)]
pub struct ConstraintData {
    //过程量，每次迭代都要重新计算
    pub lambda: f32,
    pub d_lambda: f32,
    pub grads: Vec<Vector3<f32>>,

    //缓存量，一开始就能为每个约束计算好
    pub alpha: f32, //stifiness_inv / (dt * dt)
    pub is_cache_dirty: bool,
}
impl Default for ConstraintData {
    fn default() -> Self {
        Self {
            lambda: Default::default(),
            d_lambda: Default::default(),
            grads: Default::default(),
            alpha: Default::default(),
            is_cache_dirty: true,
        }
    }
}

pub struct ConstraintsData<T: Constraint> {
    pub defines: Vec<T>,
    pub datas: Vec<ConstraintData>,
}

impl<T: Constraint> std::default::Default for ConstraintsData<T> {
    fn default() -> Self {
        Self {
            defines: vec![],
            datas: vec![],
        }
    }
}

impl<T: Constraint> From<Vec<T>> for ConstraintsData<T> {
    fn from(vec: Vec<T>) -> Self {
        let count = vec.len();
        return ConstraintsData {
            defines: vec,
            datas: vec![ConstraintData::default(); count],
        };
    }
}

impl<T: Constraint> ConstraintsData<T> {
    pub fn clear(&mut self) {
        self.defines.clear();
        self.datas.clear();
    }

    pub fn push(&mut self, define: T) {
        self.defines.push(define);
        self.datas.push(ConstraintData::default());
    }

    pub fn swap_remove(&mut self, index: usize) -> (T, ConstraintData) {
        let define = self.defines.swap_remove(index);
        let data = self.datas.swap_remove(index);
        return (define, data);
    }

    pub fn calculate_cache(&mut self, particles_data: &[ParticleData], dt: f32) {
        let grad_dim = T::get_constraint_particle_count();
        for (index, data) in self.datas.iter_mut().enumerate() {
            if !data.is_cache_dirty {
                continue;
            }
            let cs = &mut self.defines[index];
            data.is_cache_dirty = false;
            data.alpha = cs.get_stiffness_inv() / (dt * dt);
            data.grads = vec![Vector3::<f32>::zeros(); grad_dim];
        }
    }
}
