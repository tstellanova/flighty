


use crate::RigidBodyState;
use crate::physical_types::TimeBaseUnits;


pub type KinematicModelFn = fn(&ActuatorOutputs, interval: TimeBaseUnits, &mut RigidBodyState);


pub fn empty_model_fn(_actuators: &ActuatorOutputs, _interval: TimeBaseUnits, _kin: &mut RigidBodyState) {
    //does nothing
}

/// Actuator outputs -1 .. 1 : Mapping depends on vehicle
pub type ActuatorOutputs = [f32;16];


pub fn vtol_hybrid_model_fn(actuators: &ActuatorOutputs,
                            interval: TimeBaseUnits,
                            motion: &mut RigidBodyState) {

    let interval_frac: f32 = (interval as f32) / 1e6; //convert to frac seconds
    println!("act0: {} interval: {} ", actuators[0], interval);

    if actuators[0] != 0.0 && interval != 0 {
        //TODO map actuators to physical moment arms for torques etc

        motion.inertial_accel[0] = 0.001;
        motion.inertial_accel[1] = 0.001;
        motion.inertial_accel[2] = -0.001;

        motion.inertial_velocity[0] += motion.inertial_accel[0] * interval_frac;
        motion.inertial_velocity[1] += motion.inertial_accel[1] * interval_frac;
        motion.inertial_velocity[2] += motion.inertial_accel[2] * interval_frac;

        motion.inertial_position[0] += motion.inertial_velocity[0] * interval_frac;
        motion.inertial_position[1] += motion.inertial_velocity[1] * interval_frac;
        motion.inertial_position[2] += motion.inertial_velocity[2] * interval_frac;
    }
    else {
        //TODO a model with proper inertia
        motion.inertial_accel.fill(0.0);

        motion.inertial_position[0] += motion.inertial_velocity[0] * interval_frac;
        motion.inertial_position[1] += motion.inertial_velocity[1] * interval_frac;
        motion.inertial_position[2] += motion.inertial_velocity[2] * interval_frac;

    }


}


/*
To consider:

- Mass
- Angular Inertia
- Location and torque of actuators
- Translation force of actuators

*/




