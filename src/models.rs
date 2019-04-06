/**
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/


use crate::RigidBodyState;
use crate::physical_types::*;
use crate::planet::{ExternalForceEnvironment, GeoConstraintBox, PlanetEarth};

use nalgebra::{Vector3};

pub type DynamicModelFn = fn(
    actuators: &ActuatorControls,
    interval: TimeIntervalUnits,
    enviro: &ExternalForceEnvironment,
    motion: &mut RigidBodyState);

pub struct ForcesAndTorques {
    pub forces: Vector3<ForceUnits>,
    pub torques: Vector3<TorqueUnits>,
}

impl ForcesAndTorques {

    pub fn sum(first: &ForcesAndTorques, second: &ForcesAndTorques) -> ForcesAndTorques {
        ForcesAndTorques {
            forces: first.forces + second.forces,
            torques: first.torques + second.torques,
        }
    }

}


///mass of the simulated vehicle
pub const VEHICLE_MASS_VTOL_HYBRID: MassUnits = 0.8;
//const VEHICLE_MOMENT_INERTIA:Matrix3<MomentInertiaUnits> = Matrix3::new(
//    0.005,  0.0,    0.0,
//    0.0,    0.005,  0.0,
//    0.0,    0.0,    0.009
//);

/// rotors are commonly rated in weight-equivalent thrust, eg "grams"
const MAX_ROTOR_THRUST_WEIGHT_EQUIV_VTOL_HYBRID: MassUnits = 0.82;
/// convert weight-equivalent to thrust force: F = m * g
const MAX_ROTOR_THRUST_VTOL_HYBRID: ForceUnits = MAX_ROTOR_THRUST_WEIGHT_EQUIV_VTOL_HYBRID * PlanetEarth::STD_GRAVITY_ACCEL ;
/// the number of lift rotors this model has
pub const NUM_LIFT_ROTORS_VTOL_HYBRID: usize = 4;


pub const MIN_TAKEOFF_ROTOR_FRAC_VTOL_HYBRID:f32 =
    (VEHICLE_MASS_VTOL_HYBRID/ MAX_ROTOR_THRUST_WEIGHT_EQUIV_VTOL_HYBRID) /
        (NUM_LIFT_ROTORS_VTOL_HYBRID as f32);

/// The maximum acceleration achievable by rotors (excludes drag and gravity)
pub const MAX_ROTOR_ACCEL_VTOL_HYBRID: f32 =
    (NUM_LIFT_ROTORS_VTOL_HYBRID as f32 * MAX_ROTOR_THRUST_VTOL_HYBRID)/
        VEHICLE_MASS_VTOL_HYBRID ;

/// Actuator outputs -1 .. 1 : Mapping depends on vehicle
/// Rotor channels will scale from 0..1
pub type ActuatorControls = [f32;16];

pub fn vtol_hybrid_model_fn (
    actuators: &ActuatorControls,
    interval: TimeIntervalUnits,
    enviro: &ExternalForceEnvironment,
    motion: &mut RigidBodyState)
{
    enforce_constraints(&enviro.constraint, motion);

    // collect internal (actuator) forces
    let int_fortorks = internal_forces_and_torques_vtol_hybrid(&actuators);
    // collect external forces (such as gravity and wind)
    let ext_fortorks = external_forces_and_torques_vtol_hybrid(&motion, &enviro);
    // sum all forces
    let sum_fortorks = ForcesAndTorques::sum(&int_fortorks, &ext_fortorks);

    let prev_accel = motion.inertial_accel;

    //update current position and velocity based on prior step state
    for i in 0..3 {
        motion.inertial_accel[i] = sum_fortorks.forces[i] / VEHICLE_MASS_VTOL_HYBRID;

        if !motion.translation_constrained[i] {
            //use trapezoid rule to find new velocity and position from latest accel
            let dvel = interval * ((prev_accel[i] + motion.inertial_accel[i])/ 2.0);
            let prev_vel = motion.inertial_velocity[i];
            motion.inertial_velocity[i] = prev_vel + dvel;

            //use trapezoid rule to find new position from latest vel and prev_vel
            let dpos = interval * ((prev_vel + motion.inertial_velocity[i])/2.0);
            motion.inertial_position[i] += dpos;

//            if i == 2 {
//                println!("pos: {:0.7} prev_accel: {:0.6} cur_accel: {:0.6}",
//                         motion.inertial_position[2], prev_accel[2], motion.inertial_accel[2]);
//            }

//            if  (2 == i) && (crate::planet::PlanetEarth::STD_GRAVITY_ACCEL != motion.inertial_accel[2]) {
//                println!("dvel {:0.3} dpos: {:0.4}", dvel, dpos);
//                println!("int: {:0.4} pos: {:0.2} a_z: {:0.2} vel: {:0.3}",
//                        interval,
//                         motion.inertial_position[2],
//                         motion.inertial_accel[2],
//                         motion.inertial_velocity[2]);
//            }
        }
        else {
            motion.inertial_velocity[i] = 0.0;
        }
        //TODO handle torque

    }


//    println!("interval: {} accel: {} vel: {} pos: {}", interval,
//             motion.inertial_accel, motion.inertial_velocity, motion.inertial_position);
}



///
/// sum all forces and torques of actuators
///
fn internal_forces_and_torques_vtol_hybrid(actuators: &ActuatorControls) -> ForcesAndTorques {
    //TODO sum torques of actuators

    // only the first N actuators are rotors
    let rotor_acts = &actuators[..NUM_LIFT_ROTORS_VTOL_HYBRID];
    let body_z_force:ForceUnits = rotor_acts.iter().fold(0.0,
        |acc, act| acc + act * MAX_ROTOR_THRUST_VTOL_HYBRID
    );

    //TODO properly transform axial_force
    let axial_force = Vector3::new(0.0, 0.0, -body_z_force);

    ForcesAndTorques {
        forces: axial_force,
        torques: Vector3::zeros()
    }
}

///
///
///
fn external_forces_and_torques_vtol_hybrid(_kinematic: &RigidBodyState,
                                           enviro: &ExternalForceEnvironment) -> ForcesAndTorques {
    //TODO add drag force
    let grav_force = enviro.gravity * VEHICLE_MASS_VTOL_HYBRID;
    let total_force = grav_force;

    ForcesAndTorques {
        forces: total_force,
        torques: Vector3::zeros()
    }
}



///
/// Check whether the constraints prevent the body from moving in a particular dimension
///
fn enforce_constraints(constraint: &GeoConstraintBox, motion: &mut RigidBodyState ) {
    //TODO enforce x-y constraints?
    let z_pos = motion.inertial_position[2];
    let z_vel = motion.inertial_velocity[2];
    let z_accel = motion.inertial_accel[2];

    // if vehicle is on the floor and has downward Z momentum, need to be grounded
    if (z_pos >= constraint.minimum[2]) &&
        ((z_vel > 0.0) || (z_accel > 0.0)) {
        motion.translation_constrained[2] = true;
        motion.inertial_velocity[2] = 0.0; //constrained: cannot move
        motion.inertial_position[2] = constraint.minimum[2];
    }
    else {
        //unconstrain Z
        motion.translation_constrained[2] = false;
    }
}



/*
To consider:
- Mass
- Angular Inertia
- Location and torque of actuators
- Translation force of actuators
- Wind?
*/



#[cfg(test)]
mod tests {
use super::*;
use crate::planet::{Planetary, PlanetEarth};
use assert_approx_eq::assert_approx_eq;

    const HIGH_ALTITUDE_SAMPLE: DistanceUnits = -500.0; //start at 500m up

    fn expected_accel_from_actuator_frac(frac: f32, enviro: &ExternalForceEnvironment) -> AccelUnits {
        let expected_accel = -(frac * MAX_ROTOR_ACCEL_VTOL_HYBRID )
            + enviro.gravity[2];
        expected_accel
    }

    fn descending_alt(last_alt: DistanceUnits, cur_alt: DistanceUnits) -> bool {
        //assume with NED that more negative altitudes are higher
        cur_alt > last_alt
    }

    fn ascending_alt(last_alt: DistanceUnits, cur_alt: DistanceUnits) -> bool {
        //assume with NED that more negative altitudes are higher
        cur_alt < last_alt
    }

    #[test]
    fn test_vtol_landing_trajectory() {
        let mut motion = RigidBodyState::new();
        motion.inertial_position[2] = HIGH_ALTITUDE_SAMPLE;
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(100000 as TimeBaseUnits);

        const WEIGHT_BALANCE_POINT:f32 = MIN_TAKEOFF_ROTOR_FRAC_VTOL_HYBRID ;
        const SLOW_DESCENT_ACTUATOR:f32 = (WEIGHT_BALANCE_POINT - 0.001);
        let expected_accel = expected_accel_from_actuator_frac(SLOW_DESCENT_ACTUATOR, &enviro);

        let mut last_alt: DistanceUnits = motion.inertial_position[2];
        // check 15 seconds worth of travel
        let num_steps = (15.0 / step_interval) as u32;
        for i in 0..num_steps {
            //actuators are set to less than that needed to stay aloft
            let actuators: ActuatorControls = [SLOW_DESCENT_ACTUATOR; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            let cur_alt = motion.inertial_position[2];

            println!("{:03} cur_alt: {} last_alt: {}", i, cur_alt, last_alt);

            if cur_alt != enviro.constraint.minimum[2] {
                assert_approx_eq!(motion.inertial_accel[2], expected_accel);
                //check that we are always descending
                let alt_check = descending_alt(last_alt, cur_alt);
                assert_eq!(true, alt_check);
            } else {
                //verify grounded
                assert_approx_eq!(motion.inertial_accel[2], 0.0);
            }
            last_alt = cur_alt;
        }
    }

    #[test]
    fn test_vtol_takeoff_trajectory() {
        //rotation is implicitly zero after init
        let mut motion = RigidBodyState::new();
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(100000 as TimeBaseUnits);

        let actuator_frac = 0.4;
        let expected_accel = expected_accel_from_actuator_frac(actuator_frac, &enviro);

        let mut last_alt: DistanceUnits = motion.inertial_position[2];

        // check 15 seconds worth of travel
        let num_steps = (15.0 / step_interval) as u32;
        for i in 0..num_steps {
            //actuator output is more than sufficient to takeoff
            let actuators: ActuatorControls = [actuator_frac; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            assert_approx_eq!( motion.inertial_accel[2], expected_accel); //TODO derive from vehicle mass and rotor thrust

            let cur_alt = motion.inertial_position[2];
            //if i > 1 {
                let cur_accel = motion.inertial_accel[2];
                let cur_vel = motion.inertial_velocity[2];
                //check that we are always rising
                let alt_check =  ascending_alt(last_alt, cur_alt);
                //if !alt_check {
                    println!("{:03} cur_accel: {:0.4} cur_vel: {:.4} cur_alt: {:.4} last_alt: {:.4}",
                             i, cur_accel, cur_vel, cur_alt, last_alt);
                //}
                assert_eq!(true, alt_check);
            //}
            last_alt = cur_alt;
        }

    }

    #[test]
    fn test_vtol_freefall_trajectory() {
        //shut off actuators and look for an uncontrolled descent
        let mut motion = RigidBodyState::new();
        motion.inertial_position[2] = HIGH_ALTITUDE_SAMPLE;
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(1000000 as TimeBaseUnits);
        let expected_accel = enviro.gravity[2];

        let mut last_alt: DistanceUnits = motion.inertial_position[2];
        // check 5 seconds worth of travel
        let num_steps = (5.0 / step_interval) as u32;
        for i in 0..num_steps {
            let actuators: ActuatorControls = [0.0; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            assert_approx_eq!(motion.inertial_accel[2], expected_accel);

            let cur_alt = motion.inertial_position[2];
            println!("{:03} cur_alt: {} last_alt: {}", i, cur_alt, last_alt);

            //check that we are always descending
            let alt_check = descending_alt(last_alt, cur_alt);
            assert_eq!(true, alt_check);
            last_alt = cur_alt;
        }
    }

    #[test]
    fn test_find_min_takeoff_thrust() {
        let mut motion = RigidBodyState::new();
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(1000 as TimeBaseUnits);
        let actuator_step:f32 = 1E-4;
        let mut cur_actuator:f32 = 0.0;
        const WEIGHT_BALANCE_POINT:f32 = MIN_TAKEOFF_ROTOR_FRAC_VTOL_HYBRID;

        // increase rotor actuator output until the vehicle lifts off the ground
        loop {
            let actuators: ActuatorControls = [cur_actuator; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            let z_accel = motion.inertial_accel[2];
            if z_accel < 0.0 { //rising off the ground
                break;
            }
            cur_actuator += actuator_step;
        }

        println!("tookoff at: {} expected: {}",cur_actuator, WEIGHT_BALANCE_POINT);
        assert_approx_eq!(cur_actuator, WEIGHT_BALANCE_POINT, 1.1E-4);
    }

}


