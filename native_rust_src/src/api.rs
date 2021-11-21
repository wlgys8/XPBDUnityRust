use std::ffi::c_void;

use nalgebra::{Point3, Vector3};

use crate::{
    collision::shapes::{InfinitePlane, ShapesManager},
    constraints::{constraint::ConstraintsData, particle::ParticleData},
    constraints_impl::{
        distance_constraint::DistanceConstraint, triangle_bend_constraint::TriangleBendConstraint,
    },
    xpbd::{XPBDBuilder, XPBD},
};
#[repr(C)]
struct VecRawParts {
    data: *const c_void,
    length: u32,
    capacity: u32,
}

fn from_raw_parts<T>(parts: &VecRawParts) -> Vec<T> {
    unsafe {
        if parts.capacity == 0 {
            return Vec::new();
        } else {
            let element_size = std::mem::size_of::<T>();
            Vec::<T>::from_raw_parts(
                parts.data as *mut T,
                parts.length as usize / element_size,
                parts.capacity as usize / element_size,
            )
        }
    }
}

type XPBDSolverExportType = XPBD<(
    ConstraintsData<DistanceConstraint>,
    ConstraintsData<TriangleBendConstraint>,
)>;

#[repr(C)]
struct XPBDCreateOptions {
    dt: f32,
    constraints_solve_iter_count: u8,
    distance_constraints: VecRawParts,
    triangle_bend_constraints: VecRawParts,
    positions: VecRawParts,
    masses: VecRawParts,
    bounciness: f32,
    dynamic_friction_factor: f32,
}

#[no_mangle]
extern "C" fn create_array(byte_length: u32, out_raw_parts_ptr: *mut VecRawParts) {
    let mut vec = Vec::<u8>::new();
    vec.resize(byte_length as usize, 0);
    let raw = Vec::into_raw_parts(vec);
    let mut raw_parts = unsafe { out_raw_parts_ptr.as_mut().unwrap() };
    raw_parts.data = raw.0 as *const c_void;
    raw_parts.length = raw.1 as u32;
    raw_parts.capacity = raw.2 as u32;
}

#[no_mangle]
extern "C" fn destroy_array(raw_parts: VecRawParts) {
    unsafe {
        Vec::from_raw_parts(
            raw_parts.data as *mut u8,
            raw_parts.length as usize,
            raw_parts.capacity as usize,
        );
    }
}

#[no_mangle]
extern "C" fn create_xpbd_solver(options: XPBDCreateOptions) -> *mut XPBDSolverExportType {
    let distance_constraints: Vec<DistanceConstraint> =
        from_raw_parts(&options.distance_constraints);
    let bend_constraints: Vec<TriangleBendConstraint> =
        from_raw_parts(&options.triangle_bend_constraints);

    // println!("bending count = {}", bend_constraints.len());
    // println!("bending = {:?}", bend_constraints[0]);

    let xphd = XPBDBuilder {
        dt: options.dt,
        constraints: (distance_constraints, bend_constraints),
        positions: from_raw_parts(&options.positions),
        masses: from_raw_parts(&options.masses),
        iterate_count: options.constraints_solve_iter_count,
        bounciness: options.bounciness,
        dynamic_friction_factor: options.dynamic_friction_factor,
    }
    .build();
    return Box::into_raw(Box::new(xphd));
}

#[no_mangle]
extern "C" fn update_xpbd_solver(ptr: *mut XPBDSolverExportType) {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    xpbd.update();
}

#[no_mangle]
extern "C" fn destroy_xpbd_solver(ptr: *mut XPBDSolverExportType) {
    unsafe {
        Box::from_raw(ptr);
    }
}

#[no_mangle]
extern "C" fn add_field_force(ptr: *mut XPBDSolverExportType, force: Vector3<f32>) {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    xpbd.add_field_force(force);
}

#[no_mangle]
extern "C" fn add_acceleration_field(ptr: *mut XPBDSolverExportType, acc: Vector3<f32>) {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    xpbd.add_acceleration_field(acc);
}

#[no_mangle]
extern "C" fn get_particles_count(ptr: *mut XPBDSolverExportType) -> i32 {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    return xpbd.particles_count() as i32;
}

#[no_mangle]
extern "C" fn get_particles(ptr: *mut XPBDSolverExportType) -> *const ParticleData {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    let particles = xpbd.particles_data();
    let ptr = particles.as_ptr();
    return ptr;
}

#[no_mangle]
extern "C" fn attach_particle(
    ptr: *mut XPBDSolverExportType,
    particle_index: u32,
    position: Point3<f32>,
) {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    xpbd.attach(particle_index as usize, position);
}

#[no_mangle]
extern "C" fn detach_particle(ptr: *mut XPBDSolverExportType, particle_index: u32) -> bool {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    return xpbd.detach(particle_index as usize);
}

#[no_mangle]
extern "C" fn get_position(ptr: *mut XPBDSolverExportType, index: i32, pos: *mut Point3<f32>) {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    unsafe {
        *pos = xpbd.get_position(index as usize).clone();
    }
}

#[no_mangle]
extern "C" fn get_collider_manager(ptr: *mut XPBDSolverExportType) -> *mut ShapesManager {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    return xpbd.collider_manager_mut();
}

#[no_mangle]
extern "C" fn add_infinite_plane_collider(ptr: *mut ShapesManager, shape: InfinitePlane) -> u32 {
    let shapes_manager = unsafe { ptr.as_mut().unwrap() };
    return shapes_manager.infinite_planes.add(shape) as u32;
}

#[no_mangle]
extern "C" fn remove_infinite_plane_collider(ptr: *mut ShapesManager, id: u32) -> bool {
    let shapes_manager = unsafe { ptr.as_mut().unwrap() };
    return shapes_manager.infinite_planes.remove(id as usize).is_some();
}

#[no_mangle]
extern "C" fn clear_colliders(ptr: *mut ShapesManager) {
    let shapes_manager = unsafe { ptr.as_mut().unwrap() };
    shapes_manager.clear();
}

#[no_mangle]
extern "C" fn copy_positions(
    ptr: *mut XPBDSolverExportType,
    target_position_array: *mut Point3<f32>,
) {
    let xpbd = unsafe { ptr.as_mut().unwrap() };
    for (index, particle) in xpbd.particles_data().iter().enumerate() {
        unsafe {
            let item = target_position_array.offset(index as isize);
            *item = particle.position;
        }
    }
}
