#![feature(specialization, const_generics, vec_into_raw_parts)]

use crate::{
    constraints::solver::ConstraintsSolver,
    constraints_impl::distance_constraint::DistanceConstraint, xpbd::XPBDBuilder,
};
#[macro_use]
extern crate nalgebra as na;
mod api;
mod collision;
mod collision_constraint;
mod constraints;
mod constraints_impl;
mod debug;
mod test;
mod xpbd;

#[test]
fn test2() {
    let positions = vec![point![-1., 1., 0.], point![1., 1., 0.]];
    let masses = vec![1., 1.];
    let mut xphd = XPBDBuilder {
        dt: 0.05,
        constraints: (vec![DistanceConstraint::new(0, 1, 0.)]),
        positions: positions,
        masses: masses,
        iterate_count: 4,
        bounciness: 1.0,
        dynamic_friction_factor: 1.0,
    }
    .build();

    xphd.add_acceleration_field(vector![0., -9.8, 0.]);
    xphd.attach(1, point![1., 1., 0.]);

    for i in 0..20 {
        xphd.update();
        println!("{},{}", xphd.get_position(0), xphd.get_position(1));
    }

    //remove distance constraint
    println!("remove distance constraint!!");
    xphd.constraints_mut().swap_remove(0);

    for i in 0..20 {
        xphd.update();
        println!("{},{}", xphd.get_position(0), xphd.get_position(1));
    }
}

#[test]
fn test() {
    let inv_stiffness = 0.;
    let positions = vec![point![-1., 0., 0.], point![1., 0., 0.], point![0., 1., 0.]];
    let masses = vec![1.0; 3];

    let distance_constraints = vec![
        DistanceConstraint::new(0, 1, inv_stiffness),
        DistanceConstraint::new(1, 2, inv_stiffness),
        DistanceConstraint::new(2, 0, inv_stiffness),
    ];

    let mut builder = XPBDBuilder {
        dt: 0.005,
        positions: positions,
        masses: masses,
        constraints: distance_constraints,
        iterate_count: 4,
        bounciness: 1.0,
        dynamic_friction_factor: 1.0,
    };

    let mut xpbd = builder.build();
    xpbd.add_field_force(vector![0., -9.8, 0.]);
    for i in 0..100 {
        // println!("step begin {}",i);
        xpbd.update();
        // for (idx, p) in xpbd.particles_data().iter().enumerate() {
        //     println!("{}-{}", idx, p.position);
        // }
    }
    // for (idx, p) in xpbd.particles_data().iter().enumerate() {
    //     println!("{}, pos = {}", idx, p.predict_position);
    // }
}
