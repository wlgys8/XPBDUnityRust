use nalgebra::Point3;

use crate::{
    collision::shapes::InfinitePlane,
    constraints::{constraint::ConstraintsData, solver::ConstraintsSolver},
    constraints_impl::{
        distance_constraint::DistanceConstraint, triangle_bend_constraint::TriangleBendConstraint,
    },
    debug::ParticlesDebug,
    xpbd::XPBDBuilder,
};

#[test]
fn test_cloth() {
    let size_x = 3 as usize;
    let size_y = 3 as usize;

    let get_particle_index = move |i: usize, j: usize| i as usize * size_y + j as usize;

    let particle_count = size_x * size_y;
    let constraint_count = size_x * (size_y - 1) + (size_x - 1) * size_y;

    let mut positions: Vec<Point3<f32>> = Vec::with_capacity(particle_count);
    let mut masses = Vec::with_capacity(particle_count);
    for i in 0..size_x {
        for j in 0..size_y {
            positions.push(point![i as f32 * 1., j as f32 * -1., 0.]);
            masses.push(1.);
        }
    }

    let mut distance_constraints: Vec<DistanceConstraint> = Vec::with_capacity(constraint_count);
    for i in 0..size_x {
        for j in 0..size_y - 1 {
            distance_constraints.push(DistanceConstraint::new(
                get_particle_index(i, j),
                get_particle_index(i, j + 1),
                0.,
            ))
        }
    }
    for j in 0..size_y {
        for i in 0..size_x - 1 {
            distance_constraints.push(DistanceConstraint::new(
                get_particle_index(i, j),
                get_particle_index(i + 1, j),
                0.,
            ))
        }
    }
    let mut xphd = XPBDBuilder {
        dt: 0.005,
        constraints: distance_constraints,
        positions: positions,
        masses: masses,
        iterate_count: 4,
        bounciness: 1.0,
        dynamic_friction_factor: 1.0,
    }
    .build();

    xphd.add_field_force(vector![0., -9.8, 0.]);
    xphd.attach(0, point![0., 1., 0.]);

    for i in 0..2 {
        xphd.update();
        // if i > 20 {
        //     xphd.particles_data().print_predict_positions();
        // }
    }
}

#[test]
fn test_traigle_bend() {
    let positions = vec![
        point![0.0, 0.0, 0.0],
        point![1.0, 0.0, 0.0],
        point![1.0, 0.0, 1.0],
        point![0.0, 0.0, 1.0],
    ];
    let masses = vec![0.25, 0.25, 0.25, 0.25];
    let distance_constraints = vec![
        DistanceConstraint::new(0, 1, 0.),
        DistanceConstraint::new(1, 2, 0.),
        DistanceConstraint::new(2, 3, 0.),
        DistanceConstraint::new(3, 0, 0.),
        DistanceConstraint::new(0, 2, 0.),
    ];

    let bend_constraints = vec![TriangleBendConstraint::new([0, 2, 1, 3], 1.)];
    let mut xphd = XPBDBuilder {
        dt: 0.005,
        constraints: (distance_constraints, bend_constraints),
        positions: positions,
        masses: masses,
        iterate_count: 4,
        bounciness: 1.0,
        dynamic_friction_factor: 1.0,
    }
    .build();

    xphd.add_acceleration_field(vector![0., -9.8, 0.]);
    xphd.attach(0, point![0., 0., 0.]);
    // xphd.attach(2, point![1., 0., 1.]);

    for i in 0..1000 {
        xphd.update();
        xphd.particles_data().print_predict_positions();
    }
}

#[test]
fn test_infinite_plane_collision() {
    let positions = vec![point![0.0, 0.1, 0.0], point![1.0, 0.1, 0.0]];
    let masses = vec![1.0, 1.0];
    let distance_constraints = vec![DistanceConstraint::new(0, 1, 0.)];
    let mut xphd = XPBDBuilder {
        dt: 0.005,
        constraints: distance_constraints,
        positions: positions,
        masses: masses,
        iterate_count: 4,
        bounciness: 1.0,
        dynamic_friction_factor: 1.0,
    }
    .build();

    xphd.collider_manager_mut().infinite_planes.add({
        InfinitePlane {
            normal: vector![0.0, 1.0, 0.0],
            origin_to_plane: 0.0,
        }
    });

    xphd.add_acceleration_field(vector![0., -9.8, 0.]);
    for i in 0..100 {
        xphd.update();
        xphd.particles_data().print_predict_positions();
    }
}

#[test]
fn test_cube() {
    let positions = vec![
        point![0.5, -0.5, 0.5],
        point![0.5, 0.5, 0.5],
        point![-0.5, 0.5, 0.5],
        point![-0.5, -0.5, 0.5],
        point![0.5, 0.5, -0.5],
        point![-0.5, 0.5, -0.5],
        point![0.5, -0.5, -0.5],
        point![-0.5, -0.5, -0.5],
    ];
    let masses = vec![0.1 / 8.0; 8];

    let dis_stiffness_inv = 0.01;
    let distance_constraints = vec![
        DistanceConstraint::new(0, 1, dis_stiffness_inv),
        DistanceConstraint::new(1, 2, dis_stiffness_inv),
        DistanceConstraint::new(2, 0, dis_stiffness_inv),
        DistanceConstraint::new(2, 3, dis_stiffness_inv),
        DistanceConstraint::new(3, 0, dis_stiffness_inv),
        DistanceConstraint::new(1, 4, dis_stiffness_inv),
        DistanceConstraint::new(4, 5, dis_stiffness_inv),
        DistanceConstraint::new(5, 1, dis_stiffness_inv),
        DistanceConstraint::new(5, 2, dis_stiffness_inv),
        DistanceConstraint::new(4, 6, dis_stiffness_inv),
        DistanceConstraint::new(6, 7, dis_stiffness_inv),
        DistanceConstraint::new(7, 4, dis_stiffness_inv),
        DistanceConstraint::new(7, 5, dis_stiffness_inv),
        DistanceConstraint::new(6, 0, dis_stiffness_inv),
        DistanceConstraint::new(3, 6, dis_stiffness_inv),
        DistanceConstraint::new(3, 7, dis_stiffness_inv),
        DistanceConstraint::new(5, 3, dis_stiffness_inv),
        DistanceConstraint::new(1, 6, dis_stiffness_inv),
    ];

    let bending_constraints = vec![
        TriangleBendConstraint::new([0, 1, 2, 6], 0.01),
        TriangleBendConstraint::new([1, 2, 0, 5], 0.01),
        TriangleBendConstraint::new([2, 0, 1, 3], 0.01),
        TriangleBendConstraint::new([2, 3, 0, 5], 0.01),
        TriangleBendConstraint::new([3, 0, 2, 6], 0.01),
        TriangleBendConstraint::new([1, 4, 5, 6], 0.01),
        TriangleBendConstraint::new([4, 5, 1, 7], 0.01),
        TriangleBendConstraint::new([5, 1, 4, 2], 0.01),
        TriangleBendConstraint::new([5, 2, 1, 3], 0.01),
        TriangleBendConstraint::new([4, 6, 7, 1], 0.01),
        TriangleBendConstraint::new([6, 7, 4, 3], 0.01),
        TriangleBendConstraint::new([7, 4, 6, 5], 0.01),
        TriangleBendConstraint::new([7, 5, 4, 3], 0.01),
        TriangleBendConstraint::new([6, 0, 3, 1], 0.01),
        TriangleBendConstraint::new([3, 6, 0, 7], 0.01),
        TriangleBendConstraint::new([3, 7, 6, 5], 0.01),
        TriangleBendConstraint::new([5, 3, 2, 7], 0.01),
        TriangleBendConstraint::new([1, 6, 4, 0], 0.01),
    ];
    let mut xphd = XPBDBuilder {
        dt: 0.005,
        constraints: (distance_constraints, bending_constraints),
        positions: positions,
        masses: masses,
        iterate_count: 4,
        bounciness: 1.0,
        dynamic_friction_factor: 1.0,
    }
    .build();

    xphd.add_acceleration_field(vector![0., -9.8, 0.]);
    xphd.attach(0, point![0.5, -0.5, 0.5]);
    for _ in 0..10 {
        xphd.update();
        xphd.particles_data().print_predict_positions();
    }
}

#[test]
fn test_two_point_swing() {
    let positions = vec![point![0., 0., 0.], point![1., 0., 0.]];
    let masses = vec![1., 1.];
    let stiffness_inv = 0.0;
    let constraints = vec![DistanceConstraint::new(0, 1, stiffness_inv)];

    let mut xphd = XPBDBuilder {
        dt: 0.005,
        constraints: constraints,
        positions: positions,
        masses: masses,
        iterate_count: 4,
        bounciness: 1.0,
        dynamic_friction_factor: 1.0,
    }
    .build();

    xphd.add_acceleration_field(vector![0., -9.8, 0.]);
    xphd.attach(0, point![0., 0., 0.]);

    for _ in 0..10 {
        xphd.update();
        xphd.particles_data().print_predict_positions();
        // xphd.particles_data().print_velocities();
    }
}
