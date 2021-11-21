use std::collections::HashMap;

use nalgebra::{Point3, Vector3};

use crate::{
    collision::{
        intersect::{ContactInfo, GetClosestSurfacePoint},
        shapes::ShapesManager,
    },
    collision_constraint::CollisionConstraint,
    constraints::{
        constraint::{Constraint, ConstraintsData},
        particle::{particle_flags, particle_flags_index, ParticleData, ParticlesDataBuilder},
        solver::ConstraintsSolver,
    },
};

pub struct XPBD<T>
where
    T: ConstraintsSolver,
{
    dt: f32,
    iterate_count: u32,
    field_force: Vector3<f32>,
    field_acceleration: Vector3<f32>,
    constraints: T,
    collision_constraints: ConstraintsData<CollisionConstraint>,
    particles_data: Vec<ParticleData>,
    attached_particles: HashMap<usize, f32>,
    colliders: ShapesManager,
    bounciness: f32,
    dynamic_friction_factor: f32,
}

impl<T: ConstraintsSolver> XPBD<T> {
    fn calculate_predict_positions(
        particles_data: &mut [ParticleData],
        dt: f32,
        field_force: &Vector3<f32>,
        acc: &Vector3<f32>,
    ) {
        for p_data in particles_data {
            let flag = p_data.flag;
            let acc_scale = 1 - (flag & particle_flags::ATTACHED) >> particle_flags_index::ATTACHED;
            p_data.predict_position = p_data.position
                + dt * p_data.velocity
                + dt * dt * (field_force * p_data.w + acc * acc_scale as f32);
        }
    }

    fn try_add_collision_constraint(
        collisions: &mut ConstraintsData<CollisionConstraint>,
        p_index: usize,
        result: &ContactInfo,
    ) {
        if result.contacted {
            collisions.push(CollisionConstraint::new(
                p_index,
                result.contact_position.clone(),
                result.contact_normal.clone(),
            ));
        }
    }
    fn generate_collision_constraints(
        particles_data: &mut [ParticleData],
        shapes: &ShapesManager,
        collisions: &mut ConstraintsData<CollisionConstraint>,
    ) {
        collisions.clear();
        for (p_index, p) in particles_data.iter().enumerate() {
            let pos = &p.predict_position;
            for b in shapes.spheres.shapes() {
                let contact_info = b.get_closest_surface_point(&pos);
                Self::try_add_collision_constraint(collisions, p_index, &contact_info);
            }
            for s in shapes.infinite_planes.shapes() {
                let contact_info = s.get_closest_surface_point(&pos);
                Self::try_add_collision_constraint(collisions, p_index, &contact_info);
            }
        }
    }

    pub fn initialize(&mut self) {
        let particles_data = &mut self.particles_data;
        self.constraints.initialize(particles_data);
        self.constraints.calculate_cache(particles_data, self.dt);
    }

    pub fn update(&mut self) {
        let dt = self.dt;
        let particles_data = &mut self.particles_data;
        let user_constraints = &mut self.constraints;
        let collision_constraints = &mut self.collision_constraints;
        let colliders = &self.colliders;
        //预测位置计算
        Self::calculate_predict_positions(
            particles_data,
            self.dt,
            &self.field_force,
            &self.field_acceleration,
        );
        //碰撞检测，生成碰撞约束
        Self::generate_collision_constraints(particles_data, colliders, collision_constraints);
        //约束求解
        (user_constraints, collision_constraints).solve(particles_data, dt, self.iterate_count);
        //更新速度与位置
        for p_data in particles_data {
            p_data.velocity = (&p_data.predict_position - &p_data.position) / dt;
            p_data.position = p_data.predict_position;
        }
        //更新碰撞反馈
        for collision in self.collision_constraints.defines.iter() {
            let particle = &mut self.particles_data[collision.get_pindexes()[0]];
            particle.position = collision.concat_position + collision.concat_normal * 0.05;
            particle.predict_position = collision.concat_position + collision.concat_normal * 0.05;
            if particle.w == 0.0 {
                particle.velocity = Vector3::zeros();
            } else {
                let velocity_in = particle.velocity;
                //法线方向速度
                let mut velocity_on_normal =
                    velocity_in.dot(&collision.concat_normal) * collision.concat_normal;
                //切线方向速度
                let mut velocity_on_tangent = velocity_in - velocity_on_normal;
                //弹性碰撞
                //偷懒直接系数百分比衰减
                velocity_on_normal = -velocity_on_normal * self.bounciness;
                velocity_on_tangent = velocity_on_tangent * (1.0 - self.dynamic_friction_factor);
                particle.velocity = velocity_on_normal + velocity_on_tangent;
            }
        }
    }

    pub fn particles_count(&self) -> usize {
        return self.particles_data.len();
    }

    pub fn attach(&mut self, particle_index: usize, position: Point3<f32>) {
        let particle = &mut self.particles_data[particle_index];
        if !self.attached_particles.contains_key(&particle_index) {
            self.attached_particles.insert(particle_index, particle.w);
            particle.w = 0.;
        }
        particle.position = position;
        particle.flag |= particle_flags::ATTACHED;
    }

    pub fn detach(&mut self, particle_index: usize) -> bool {
        match self.attached_particles.remove(&particle_index) {
            Some(original_w) => {
                let particle = &mut self.particles_data[particle_index];
                particle.w = original_w;
                particle.flag &= !particle_flags::ATTACHED;
                return true;
            }
            _ => false,
        }
    }

    pub fn add_field_force(&mut self, force: Vector3<f32>) {
        self.field_force += force;
    }

    pub fn add_acceleration_field(&mut self, acc: Vector3<f32>) {
        self.field_acceleration += acc;
    }

    pub fn field_force(&self) -> &Vector3<f32> {
        return &self.field_force;
    }

    pub fn field_acceleration(&self) -> &Vector3<f32> {
        return &self.field_acceleration;
    }

    pub fn get_position(&self, index: usize) -> &Point3<f32> {
        return &self.particles_data[index].position;
    }

    pub fn particles_data(&self) -> &[ParticleData] {
        return &self.particles_data;
    }

    pub fn constraints(&self) -> &T {
        return &self.constraints;
    }

    pub fn constraints_mut(&mut self) -> &mut T {
        return &mut self.constraints;
    }

    pub fn collider_manager_mut(&mut self) -> &mut ShapesManager {
        return &mut self.colliders;
    }
}

pub trait ConstraintsSolverBuilder {
    type Type: ConstraintsSolver;
    fn build(self) -> Self::Type;
}

impl<T> ConstraintsSolverBuilder for Vec<T>
where
    T: Constraint,
{
    type Type = ConstraintsData<T>;
    fn build(self) -> Self::Type {
        return Self::Type::from(self);
    }
}

impl<T1, T2> ConstraintsSolverBuilder for (T1, T2)
where
    T1: ConstraintsSolverBuilder,
    T2: ConstraintsSolverBuilder,
{
    type Type = (T1::Type, T2::Type);
    fn build(self) -> Self::Type {
        (self.0.build(), self.1.build())
    }
}

pub struct XPBDBuilder<T>
where
    T: ConstraintsSolverBuilder + Default,
{
    pub dt: f32,
    pub iterate_count: u8,
    pub constraints: T,
    pub positions: Vec<Point3<f32>>,
    pub masses: Vec<f32>,
    pub bounciness: f32,
    pub dynamic_friction_factor: f32,
}

impl<T: ConstraintsSolverBuilder + Default> XPBDBuilder<T> {
    pub fn build(&mut self) -> XPBD<T::Type> {
        let particles_data = ParticlesDataBuilder {
            positions: std::mem::take(&mut self.positions),
            masses: std::mem::take(&mut self.masses),
        }
        .build()
        .unwrap();
        let constraints = std::mem::take(&mut self.constraints);
        let constraints_data = constraints.build();
        let mut xpbd = XPBD {
            dt: self.dt,
            iterate_count: (self.iterate_count as u32).max(1),
            constraints: constraints_data,
            particles_data: particles_data,
            collision_constraints: Default::default(),
            field_force: Default::default(),
            attached_particles: Default::default(),
            colliders: Default::default(),
            field_acceleration: Default::default(),
            bounciness: self.bounciness,
            dynamic_friction_factor: self.dynamic_friction_factor,
        };
        xpbd.initialize();
        return xpbd;
    }
}
