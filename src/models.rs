/**
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/


use crate::RigidBodyState;
use crate::physical_types::*;
use crate::planet::{ExternalForceEnvironment, GeoConstraintBox, PlanetEarth};

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use std::f32::consts::PI;
use std::ops::Mul;

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
    // collect external forces (such as gravity, wind, drag)
    let ext_fortorks = external_forces_and_torques_vtol_hybrid(&motion, &enviro);
    // sum all forces
    let sum_fortorks = ForcesAndTorques::sum(&int_fortorks, &ext_fortorks);

    apply_forces_vtol_hybrid(&sum_fortorks.forces, interval, motion);
    apply_torques_vtol_hybrid(&sum_fortorks.torques, interval, motion);

//    println!("interval: {} accel: {} vel: {} pos: {}", interval,
//             motion.inertial_accel, motion.inertial_velocity, motion.inertial_position);
}

/// Apply translation forces to rigid body, changing linear acceleration
fn apply_forces_vtol_hybrid(forces: &Vector3<ForceUnits>,
                            interval: TimeIntervalUnits, motion: &mut RigidBodyState) {
    // update linear acceleration
    let prev_accel = motion.inertial_accel;
    motion.inertial_accel =   forces / VEHICLE_MASS_VTOL_HYBRID;

    //update current velocity and position based on prior step state
    for i in 0..3 {
        if !motion.translation_constrained[i] {
            //use trapezoid rule to find new velocity from latest and previous accel
            let dvel = interval * ((prev_accel[i] + motion.inertial_accel[i]) / 2.0);
            let prev_vel = motion.inertial_velocity[i];
            motion.inertial_velocity[i] = prev_vel + dvel;

            //use trapezoid rule to find new position from latest and previous velocity
            let dpos = interval * ((prev_vel + motion.inertial_velocity[i]) / 2.0);
            motion.inertial_position[i] += dpos;
        }
        else {
            motion.inertial_velocity[i] = 0.0;
        }
    }
}


/// Apply torque to rigid body, changing angular acceleration
fn apply_torques_vtol_hybrid(torques: &Vector3<TorqueUnits>,  interval: TimeIntervalUnits,
                             motion: &mut RigidBodyState) {
    let prev_accel = motion.body_angular_accel;

    //Handle torques if they don't sum to zero
    if torques.magnitude() > 0.0 {
        motion.body_angular_accel =  INV_MOMENTS_HYBRID_VTOL.mul(torques);

        if !motion.rotation_locked() {
            //use trapezoid rule to find new angular velocity from latest accel
            let dvel = interval*((prev_accel + motion.body_angular_accel)/2.0);
            let prev_vel = motion.body_angular_velocity;
            motion.body_angular_velocity = prev_vel + dvel;

            //use trapezoid rule to find new angular position from latest vel and prev_vel
            let dpos = interval * ((prev_vel + motion.body_angular_velocity)/2.0);
            motion.body_angular_position += dpos;
        }
        else {
            motion.body_angular_velocity = Vector3::zeros();
        }
    }

}



///
/// sum all forces and torques of actuators
///
fn internal_forces_and_torques_vtol_hybrid(actuators: &ActuatorControls) -> ForcesAndTorques {

    // only the first N actuators are rotors
    let rotor_acts = &actuators[..NUM_LIFT_ROTORS_VTOL_HYBRID];
    let mut sum_torks = Vector3::zeros();
    for i in 0..NUM_LIFT_ROTORS_VTOL_HYBRID {
        let act = rotor_acts[i];
        let thrust_force = act * MAX_ROTOR_THRUST_VTOL_HYBRID;
        let thrust_vec:Vector3<ForceUnits> = Vector3::new(0.0, 0.0, -thrust_force );
        let act_pos =   ROTOR_POSITIONS_VTOL_HYBRID[i];
        //the torque contribution is the thrust crossed with the rotor position in body frame
        let tork =  act_pos.cross(&thrust_vec);
        //println!("tf[{}]: {:0.2} tork: {:?} ",i,thrust_force, tork);
        sum_torks += tork;
        //TODO add in propeller drag torque, affects rotation about Z axis (yaw)
    }

    //sum total rotor thrust force for translation
    let total_thrust:ForceUnits = rotor_acts.iter().fold(0.0,
        |acc, act| acc + act * MAX_ROTOR_THRUST_VTOL_HYBRID
    );

    //TODO properly transform axial_force
    let axial_force = Vector3::new(0.0, 0.0, -total_thrust);

    ForcesAndTorques {
        forces: axial_force,
        torques: sum_torks
    }
}

///
///
///
fn external_forces_and_torques_vtol_hybrid(_kinematic: &RigidBodyState,
                                           enviro: &ExternalForceEnvironment) -> ForcesAndTorques {
    //TODO add drag force, wind force
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
        //unconstrain Z translation
        motion.translation_constrained[2] = false;
    }

    //TODO what about travel on x-y plane, with unconstrained Yaw?
    if motion.translation_constrained[2] {
        //constrain rotation
        motion.rotation_constrained[0] = true;
        motion.rotation_constrained[1] = true;
        motion.rotation_constrained[2] = true;
    }
    else {
        //unconstrain rotation
        motion.rotation_constrained[0] = false;
        motion.rotation_constrained[1] = false;
        motion.rotation_constrained[2] = false;
    }

}


const NUM_ROTORS_VTOL_HYBRID: usize = 4;
const ROTOR_ARM_LEN_VTOL_HYBRID: DistanceUnits = 100E-3;

const M00_MOMENT_VTOL_HYBRID:MomentInertiaUnits = 0.005;
const M11_MOMENT_VTOL_HYBRID:MomentInertiaUnits = 0.005;
const M22_MOMENT_VTOL_HYBRID:MomentInertiaUnits = 0.009;

lazy_static! {

    /// The body-frame positions of the lift rotors for this vehicle
    static ref ROTOR_POSITIONS_VTOL_HYBRID: [Vector3<DistanceUnits>;NUM_ROTORS_VTOL_HYBRID]  = {
        //initial rotor positions are simple plus-shape
        let mut all_rotors:[Vector3<DistanceUnits>;NUM_ROTORS_VTOL_HYBRID]  = [
            Vector3::new(0.0, ROTOR_ARM_LEN_VTOL_HYBRID, 0.0),
            Vector3::new(0.0, -ROTOR_ARM_LEN_VTOL_HYBRID, 0.0),
            Vector3::new(ROTOR_ARM_LEN_VTOL_HYBRID, 0.0, 0.0),
            Vector3::new(-ROTOR_ARM_LEN_VTOL_HYBRID, 0.0, 0.0),
            ];

        //rotate the rotor positions around the Z axis
        let quarter_rot:UnitQuaternion<DistanceUnits> =
            UnitQuaternion::from_euler_angles(0.0,0.0,-PI/4.0);
        for i in 0..NUM_ROTORS_VTOL_HYBRID {
            let pos = all_rotors[i];
            all_rotors[i] = quarter_rot * pos;
        }

        all_rotors
    };

    /// Moments of inertia for the vehicle
    static ref MOMENTS_HYBRID_VTOL:Matrix3<MomentInertiaUnits> = Matrix3::new(
        M00_MOMENT_VTOL_HYBRID,  0.0,    0.0,
        0.0,    M11_MOMENT_VTOL_HYBRID,  0.0,
        0.0,    0.0,    M22_MOMENT_VTOL_HYBRID
    );

    /// Used for finding angular acceleration from torques
    static ref INV_MOMENTS_HYBRID_VTOL:Matrix3<MomentInertiaUnits> =
        MOMENTS_HYBRID_VTOL.pseudo_inverse(1E-6).unwrap();

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

//    #[test]
//    fn examine_rotor_positions() {
//        for i in 0..4 {
//            let pos = ROTOR_POSITIONS_VTOL_HYBRID[i];
//            println!("pos: {:?}", pos);
//        }
//        assert_eq!(false, true);
//    }

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

        //verify that since the lift rotors were all matched, there should be zero torque
        assert_approx_eq!(motion.body_angular_accel.magnitude(), 0.0);
    }

    #[test]
    fn test_apply_forces_vtol_hybrid() {
        //verify that when we apply forces to various axes, we get the expected linear accel

        let interval: TimeIntervalUnits = 1E-3;
        const KNOWN_FORCE:ForceUnits = 10.0;
        let expected_accel = KNOWN_FORCE / VEHICLE_MASS_VTOL_HYBRID;

        let mut state: RigidBodyState = RigidBodyState::new();
        let forces: Vector3<ForceUnits> = Vector3::new(KNOWN_FORCE, 0.0, 0.0);
        apply_forces_vtol_hybrid(&forces, interval, &mut state);
        println!("la00: {:?} expected: {:0.4}", state.inertial_accel,expected_accel);
        assert_approx_eq!(state.inertial_accel.magnitude(), expected_accel);
        assert_approx_eq!(state.inertial_accel[0], expected_accel);

        let mut state: RigidBodyState = RigidBodyState::new();
        let forces: Vector3<ForceUnits> = Vector3::new(-KNOWN_FORCE, 0.0, 0.0);
        apply_forces_vtol_hybrid(&forces, interval, &mut state);
        println!("la01: {:?} expected: {:0.4}", state.inertial_accel,-expected_accel);
        assert_approx_eq!(state.inertial_accel.magnitude(), expected_accel);
        assert_approx_eq!(state.inertial_accel[0], -expected_accel);

        let mut state: RigidBodyState = RigidBodyState::new();
        let forces: Vector3<ForceUnits> = Vector3::new(0.0, KNOWN_FORCE, 0.0);
        apply_forces_vtol_hybrid(&forces, interval, &mut state);
        println!("la10: {:?} expected: {:0.4}", state.inertial_accel,expected_accel);
        assert_approx_eq!(state.inertial_accel.magnitude(), expected_accel);
        assert_approx_eq!(state.inertial_accel[1], expected_accel);

        let mut state: RigidBodyState = RigidBodyState::new();
        let forces: Vector3<ForceUnits> = Vector3::new(0.0, -KNOWN_FORCE, 0.0);
        apply_forces_vtol_hybrid(&forces, interval, &mut state);
        println!("la11: {:?} expected: {:0.4}", state.inertial_accel,-expected_accel);
        assert_approx_eq!(state.inertial_accel.magnitude(), expected_accel);
        assert_approx_eq!(state.inertial_accel[1], -expected_accel);

        let mut state: RigidBodyState = RigidBodyState::new();
        let forces: Vector3<ForceUnits> = Vector3::new(0.0, 0.0, KNOWN_FORCE);
        apply_forces_vtol_hybrid(&forces, interval, &mut state);
        println!("la20: {:?} expected: {:0.4}", state.inertial_accel,expected_accel);
        assert_approx_eq!(state.inertial_accel.magnitude(), expected_accel);
        assert_approx_eq!(state.inertial_accel[2], expected_accel);

        let mut state: RigidBodyState = RigidBodyState::new();
        let forces: Vector3<ForceUnits> = Vector3::new(0.0, 0.0, -KNOWN_FORCE);
        apply_forces_vtol_hybrid(&forces, interval, &mut state);
        println!("la21: {:?} expected: {:0.4}", state.inertial_accel,-expected_accel);
        assert_approx_eq!(state.inertial_accel.magnitude(), expected_accel);
        assert_approx_eq!(state.inertial_accel[2], -expected_accel);

    }

    #[test]
    fn test_apply_torques_vtol_hybrid() {
        // Verify that when we apply torques to various axes, we get the expected angular accel
        let interval: TimeIntervalUnits = 1E-3;
        const KNOWN_TORQUE:TorqueUnits = 10.0;

        let expected_accel = KNOWN_TORQUE / M00_MOMENT_VTOL_HYBRID;
        let mut state: RigidBodyState = RigidBodyState::new();
        let torques: Vector3<TorqueUnits> = Vector3::new(KNOWN_TORQUE, 0.0, 0.0);
        apply_torques_vtol_hybrid(&torques,  interval,&mut state);
        println!("aa00: {:?} expected: {:0.4}", state.body_angular_accel,expected_accel);
        assert_approx_eq!(state.body_angular_accel.magnitude(), expected_accel);
        assert_approx_eq!(state.body_angular_accel[0],expected_accel);
        assert_approx_eq!(state.body_angular_accel[1],0.0);
        assert_approx_eq!(state.body_angular_accel[2],0.0);

        let mut state: RigidBodyState = RigidBodyState::new();
        let torques: Vector3<TorqueUnits> = Vector3::new(-KNOWN_TORQUE, 0.0, 0.0);
        apply_torques_vtol_hybrid(&torques,  interval,&mut state);
        println!("aa01: {:?} expected: {:0.4}", state.body_angular_accel,-expected_accel);
        assert_approx_eq!(state.body_angular_accel.magnitude(), expected_accel, 1E-3);
        assert_approx_eq!(state.body_angular_accel[0],-expected_accel);

        let expected_accel = KNOWN_TORQUE / M11_MOMENT_VTOL_HYBRID;
        let mut state: RigidBodyState = RigidBodyState::new();
        let torques: Vector3<TorqueUnits> = Vector3::new(0.0, KNOWN_TORQUE, 0.0);
        apply_torques_vtol_hybrid(&torques,  interval,&mut state);
        println!("aa10: {:?} expected: {:0.4}", state.body_angular_accel,expected_accel);
        assert_approx_eq!(state.body_angular_accel.magnitude(), expected_accel, 1E-3);
        assert_approx_eq!(state.body_angular_accel[1],expected_accel);

        let mut state: RigidBodyState = RigidBodyState::new();
        let torques: Vector3<TorqueUnits> = Vector3::new(0.0, -KNOWN_TORQUE, 0.0);
        apply_torques_vtol_hybrid(&torques,  interval,&mut state);
        println!("aa11: {:?} expected: {:0.4}", state.body_angular_accel,-expected_accel);
        assert_approx_eq!(state.body_angular_accel.magnitude(), expected_accel, 1E-3);
        assert_approx_eq!(state.body_angular_accel[1],-expected_accel);

        let expected_accel = KNOWN_TORQUE / M22_MOMENT_VTOL_HYBRID;
        let mut state: RigidBodyState = RigidBodyState::new();
        let torques: Vector3<TorqueUnits> = Vector3::new(0.0, 0.0, KNOWN_TORQUE);
        apply_torques_vtol_hybrid(&torques,  interval,&mut state);
        println!("aa20: {:?} expected: {:0.4}", state.body_angular_accel,expected_accel);
        assert_approx_eq!(state.body_angular_accel.magnitude(), expected_accel, 1E-3);
        assert_approx_eq!(state.body_angular_accel[2],expected_accel, 1E-3);

        let mut state: RigidBodyState = RigidBodyState::new();
        let torques: Vector3<TorqueUnits> = Vector3::new(0.0, 0.0, -KNOWN_TORQUE);
        apply_torques_vtol_hybrid(&torques,  interval,&mut state);
        println!("aa21: {:?} expected: {:0.4}", state.body_angular_accel,-expected_accel);
        assert_approx_eq!(state.body_angular_accel.magnitude(), expected_accel, 1E-3);
        assert_approx_eq!(state.body_angular_accel[2],-expected_accel, 1E-3);

        //test combined multi-axis torque
        let expected_accel0 = KNOWN_TORQUE / M00_MOMENT_VTOL_HYBRID;
        let expected_accel1 = KNOWN_TORQUE / M11_MOMENT_VTOL_HYBRID;
        let expected_accel2 = KNOWN_TORQUE / M22_MOMENT_VTOL_HYBRID;
        let mut state: RigidBodyState = RigidBodyState::new();
        let torques: Vector3<TorqueUnits> = Vector3::new(KNOWN_TORQUE, KNOWN_TORQUE, KNOWN_TORQUE);
        apply_torques_vtol_hybrid(&torques,  interval,&mut state);
        println!("aaZZ: {:?} expected: [{:0.4},{:0.4},{:0.4}] ", state.body_angular_accel,
                 expected_accel0, expected_accel1 ,expected_accel2);

        assert_approx_eq!(state.body_angular_accel[0],expected_accel0, 1E-3);
        assert_approx_eq!(state.body_angular_accel[1],expected_accel1, 1E-3);
        assert_approx_eq!(state.body_angular_accel[2],expected_accel2, 1E-3);

    }

}


