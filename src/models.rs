


use crate::RigidBodyState;
use crate::physical_types::TimeIntervalUnits;


pub type KinematicModelFn = fn(&ActuatorOutputs, interval: TimeIntervalUnits, &mut RigidBodyState);


pub fn empty_model_fn(_actuators: &ActuatorOutputs, _interval: TimeIntervalUnits, _kin: &mut RigidBodyState) {
    //does nothing
}

/// Actuator outputs -1 .. 1 : Mapping depends on vehicle
pub type ActuatorOutputs = [f32;16];


pub fn vtol_hybrid_model_fn(actuators: &ActuatorOutputs,
                            interval: TimeIntervalUnits,
                            motion: &mut RigidBodyState) {


    if actuators[0] != 0.0 && interval != 0.0 {
        //TODO map actuators to physical moment arms for torques etc

        //TODO tmp : zero rotation -> rise or fall
        if (motion.body_angular_position[0] == 0.0) &&
            (motion.body_angular_position[1] == 0.0) &&
            (motion.body_angular_position[2] == 0.0) {

            motion.inertial_accel[0] = 0.0;
            motion.inertial_accel[1] = 0.0;
            if actuators[0] > 0.5 {
                motion.inertial_accel[2] = -1.0; //rise
            }
            else {
                motion.inertial_accel[2] = 1.0; //drop
            }
        }
        else {
            motion.inertial_accel[0] = 0.001;
            motion.inertial_accel[1] = 0.001;
            motion.inertial_accel[2] = -0.001;
        }

        motion.inertial_velocity[0] += motion.inertial_accel[0] * interval;
        motion.inertial_velocity[1] += motion.inertial_accel[1] * interval;
        motion.inertial_velocity[2] += motion.inertial_accel[2] * interval;

        motion.inertial_position[0] += motion.inertial_velocity[0] * interval;
        motion.inertial_position[1] += motion.inertial_velocity[1] * interval;
        motion.inertial_position[2] += motion.inertial_velocity[2] * interval;
    }
    else {
        //TODO a model with proper inertia
        motion.inertial_accel.fill(0.0);

        motion.inertial_position[0] += motion.inertial_velocity[0] * interval;
        motion.inertial_position[1] += motion.inertial_velocity[1] * interval;
        motion.inertial_position[2] += motion.inertial_velocity[2] * interval;
    }

    println!("interval: {} accel: {} vel: {} pos: {}", interval,
             motion.inertial_accel, motion.inertial_velocity, motion.inertial_position);

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
    use crate::physical_types::*;

    #[test]
    fn test_vtol_landing_trajectory() {
        let mut motion = RigidBodyState::new();
        motion.inertial_position[2] = 100.0; //start at 100m up
        let mut last_alt: DistanceUnits = motion.inertial_position[2];

        let step_time_delta:TimeBaseUnits = 100000; //microseconds
        let step_interval:TimeIntervalUnits =
            (step_time_delta as TimeIntervalUnits) * TIME_BASE_DELTA_TO_INTERVAL;

        // check 15 seconds worth of travel
        let num_steps = (15.0 / step_interval) as u32;

        for _i in 0..num_steps {
            //actuators are set to less than that needed to stay aloft
            let actuators: ActuatorOutputs = [0.25; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &mut motion);
            let cur_alt = motion.inertial_position[2];
            let alt_check = cur_alt > last_alt;//NED
            if !alt_check {
                println!("cur_alt: {} last_alt: {}", cur_alt, last_alt);
            }
            last_alt = cur_alt;
            assert_eq!( true, alt_check );
        }
    }

    #[test]
    fn test_vtol_takeoff_trajectory() {
        //rotation is implicitly zero after init
        let mut motion = RigidBodyState::new();
        let mut last_alt: DistanceUnits = motion.inertial_position[2];

        let step_time_delta:TimeBaseUnits = 100000; //microseconds
        let step_interval:TimeIntervalUnits =
            (step_time_delta as TimeIntervalUnits) * TIME_BASE_DELTA_TO_INTERVAL;

        // check 15 seconds worth of travel
        let num_steps = (15.0 / step_interval) as u32;

        for _i in 0..num_steps {
            //actuator output is more than sufficient to stay aloft
            let actuators: ActuatorOutputs = [0.51; 16];
            vtol_hybrid_model_fn(&actuators, step_interval, &mut motion);
            let cur_alt = motion.inertial_position[2];
            let alt_check = cur_alt < last_alt; //NED
            if !alt_check {
                println!("cur_alt: {} last_alt: {}", cur_alt, last_alt);
            }
            last_alt = cur_alt;
            assert_eq!( true, alt_check );
        }
    }




}


