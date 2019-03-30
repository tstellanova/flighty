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



const VEHICLE_MASS_VTOL_HYBRID: MassUnits = 1.0;
//const VEHICLE_MOMENT_INERTIA:Matrix3<MomentInertiaUnits> = Matrix3::new(
//    0.005,  0.0,    0.0,
//    0.0,    0.005,  0.0,
//    0.0,    0.0,    0.009
//);

/// rotors are commonly rated in weight-equivalent thrust, eg "grams"
const MAX_ROTOR_THRUST_WEIGHT_EQUIV_VTOL_HYBRID: MassUnits = 1.1;
/// convert weight-equivalent to thrust force: F = m * g
const MAX_ROTOR_THRUST_VTOL_HYBRID: ForceUnits = MAX_ROTOR_THRUST_WEIGHT_EQUIV_VTOL_HYBRID * PlanetEarth::STD_GRAVITY_ACCEL ;
/// the number of lift rotors this model has
const NUM_LIFT_ROTORS_VTOL_HYBRID: usize = 4;

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

    for i in 0..3 {
        motion.inertial_accel[i] = sum_fortorks.forces[i] / VEHICLE_MASS_VTOL_HYBRID;

        if !motion.translation_constrained[i] {
            motion.inertial_velocity[i] += motion.inertial_accel[i] * interval;
            motion.inertial_position[i] += motion.inertial_velocity[i] * interval;
        }
    }

    //recheck constraints after applying acceleration
    enforce_constraints(&enviro.constraint, motion);

    //TODO handle torque

//    println!("interval: {} accel: {} vel: {} pos: {}", interval,
//             motion.inertial_accel, motion.inertial_velocity, motion.inertial_position);
}



///
/// sum all forces and torques of actuators
///
fn internal_forces_and_torques_vtol_hybrid(actuators: &ActuatorControls) -> ForcesAndTorques {
    //TODO sum torques of actuators

    // only the first N actuators are vertical lift rotors
    let rotor_acts = &actuators[..NUM_LIFT_ROTORS_VTOL_HYBRID];
    let z_force:ForceUnits = rotor_acts.iter().fold(0.0,
        |acc, act| acc + act * MAX_ROTOR_THRUST_VTOL_HYBRID
    );

    //TODO properly transform axial_force
    let axial_force = Vector3::new(0.0, 0.0, -z_force);
    //println!("axial_force: {}", axial_force);

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
    //const VEHICLE_CROSS_SECTION_DRAG:ForceUnits = 0.01;

    let grav_force = enviro.gravity * VEHICLE_MASS_VTOL_HYBRID;
    //TODO calculate drag forces
    //let air_drag_force = (kinematic.inertial_velocity - enviro.wind) *  VEHICLE_CROSS_SECTION_DRAG;

    let total_force = grav_force; // + air_drag_force;

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

    // if vehicle is on the floor and has nonzero Z momentum, need to be grounded
    if (z_pos >= constraint.minimum[2]) && (z_vel > 0.0) {
        motion.translation_constrained[2] = true;
        motion.inertial_velocity[2] = 0.0; //constrained: cannot move
        motion.inertial_position[2] = constraint.minimum[2];
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

    #[test]
    fn test_vtol_landing_trajectory() {
        let mut motion = RigidBodyState::new();
        motion.inertial_position[2] = -500.0; //start at 500m up
        let mut last_alt: DistanceUnits = motion.inertial_position[2];
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(100000 as TimeBaseUnits);

        const WEIGHT_BALANCE_POINT:f32 =
            (VEHICLE_MASS_VTOL_HYBRID/ MAX_ROTOR_THRUST_WEIGHT_EQUIV_VTOL_HYBRID)
                / (NUM_LIFT_ROTORS_VTOL_HYBRID as f32);
        const SLOW_DESCENT_ACTUATOR:f32 = (WEIGHT_BALANCE_POINT - 0.001);

        // check 15 seconds worth of travel
        let num_steps = (15.0 / step_interval) as u32;
        for _i in 0..num_steps {
            //actuators are set to less than that needed to stay aloft
            let actuators: ActuatorControls = [SLOW_DESCENT_ACTUATOR; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            let cur_alt = motion.inertial_position[2];

            if cur_alt != enviro.constraint.minimum[2] {
                assert_approx_eq!(motion.inertial_accel[2], 0.043149948);
                //check that we are always descending
                let alt_check = cur_alt > last_alt;//NED descending altitude
                if !alt_check {
                    println!("cur_alt: {} last_alt: {}", cur_alt, last_alt);
                }
                last_alt = cur_alt;
                assert_eq!( true, alt_check );
            }
            else {
                assert_approx_eq!(motion.inertial_accel[2], 0.0);
            }

        }
    }

    #[test]
    fn test_vtol_takeoff_trajectory() {
        //rotation is implicitly zero after init
        let mut motion = RigidBodyState::new();
        let mut last_alt: DistanceUnits = motion.inertial_position[2];
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(100000 as TimeBaseUnits);

        // check 15 seconds worth of travel
        let num_steps = (15.0 / step_interval) as u32;
        for _i in 0..num_steps {
            //actuator output is more than sufficient to takeoff
            let actuators: ActuatorControls = [0.40; 16];
            println!("last_alt : {}", last_alt);
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            assert_approx_eq!( motion.inertial_accel[2], -7.4530544);

            let cur_alt = motion.inertial_position[2];
            //check that we are always rising
            let alt_check = cur_alt < last_alt; //NED ascending altitude
            if !alt_check {
                println!("cur_alt: {} last_alt: {}", cur_alt, last_alt);
            }

            last_alt = cur_alt;
            assert_eq!( true, alt_check );
        }
    }

    #[test]
    fn test_vtol_freefall_trajectory() {
        //shut off actuators and look for an uncontrolled descent
        let mut motion = RigidBodyState::new();
        motion.inertial_position[2] = -500.0; //start at 100m up
        let mut last_alt: DistanceUnits = motion.inertial_position[2];
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(1000000 as TimeBaseUnits);

        // check 5 seconds worth of travel
        let num_steps = (5.0 / step_interval) as u32;
        for _i in 0..num_steps {
            let actuators: ActuatorControls = [0.0; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            assert_approx_eq!(motion.inertial_accel[2], enviro.gravity[2]);

            let cur_alt = motion.inertial_position[2];
            //check that we are always descending
            let alt_check = cur_alt > last_alt;//NED descending altitude
            if !alt_check {
                println!("cur_alt: {} last_alt: {}", cur_alt, last_alt);
            }
            last_alt = cur_alt;
            assert_eq!( true, alt_check );
        }
    }

    #[test]
    fn test_find_min_takeoff_thrust() {
        let mut motion = RigidBodyState::new();
        let enviro = PlanetEarth::default_local_environment();
        let step_interval:TimeIntervalUnits =  time_base_delta_to_interval(100 as TimeBaseUnits);
        let actuator_step:f32 = 1E-4;
        let mut cur_actuator:f32 = 0.0;
        const WEIGHT_BALANCE_POINT:f32 =
            (VEHICLE_MASS_VTOL_HYBRID/ MAX_ROTOR_THRUST_WEIGHT_EQUIV_VTOL_HYBRID)
                / (NUM_LIFT_ROTORS_VTOL_HYBRID as f32);
        //

        // increase rotor actuator output until the vehicle accelerates off the ground
        loop {
            let actuators: ActuatorControls = [cur_actuator; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &enviro, &mut motion);
            let cur_accel = motion.inertial_accel[2];
            if cur_accel < 0.0 { //rising off the ground
                break;
            }

            cur_actuator += actuator_step;
        }

        println!("tookoff at: {} expected: {}",cur_actuator, WEIGHT_BALANCE_POINT);
        assert_approx_eq!(cur_actuator, WEIGHT_BALANCE_POINT, 1E-4);
    }

}


