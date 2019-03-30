/**
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/

use std::time::{Duration, SystemTime};

use crate::{VirtualVehicleState, PhysicalSensors};
use crate::models::*;
use crate::physical_types::*;
use crate::planet::Planetary;

pub struct Simulato {
    boot_time: SystemTime,
    simulated_time: TimeBaseUnits,
    pub abstime_offset: TimeBaseUnits,
    pub vehicle_state: VirtualVehicleState,
    vehicle_model: DynamicModelFn,
    pub sensed: PhysicalSensors,
}


impl Simulato {
    pub fn new(home: &GlobalPosition) -> Self {
        Simulato {
            boot_time: SystemTime::now(),
            simulated_time: 0,
            abstime_offset: 500,

            vehicle_state: VirtualVehicleState::new(home),
            //TODO model transfer function hardcoded for now. Make configurable by env vars
            vehicle_model: vtol_hybrid_model_fn,
            sensed: PhysicalSensors::new()
        }
    }

    pub fn set_simulated_time(&mut self, time:TimeBaseUnits) {
        self.simulated_time = time;
    }

    pub fn get_simulated_time(&self) -> u64 {
        self.simulated_time
    }

    pub fn elapsed(&self) -> Duration {
        self.boot_time.elapsed().unwrap()
    }

    pub fn elapsed_since(&self, old: TimeBaseUnits) -> TimeBaseUnits {
        let diff:TimeBaseUnits;
        if self.simulated_time < old {
            diff = 0; //time should be monotonically increasing?
        }
        else {
            diff = self.simulated_time - old;
        }
        diff
    }


    pub fn micros_from_duration(duration: &Duration) -> TimeBaseUnits {
        (duration.as_secs() * 1000000) + (duration.subsec_micros() as TimeBaseUnits)
    }

    pub fn increment_simulated_time(&mut self) {
        //TODO allow simulated time to decouple from real time
        let new_real_time = self.elapsed();
        self.set_simulated_time(Self::micros_from_duration(&new_real_time));
    }

    /// Heart of the update loop:
    ///  - Given a set of current actuator outputs,
    ///  - Use the model to update the idealized virtual state of the vehicle.
    ///  - Then update the vehicle's sensed state.
    ///
    pub fn update(&mut self, time: TimeBaseUnits, actuators: &ActuatorControls) {
        let dt: TimeIntervalUnits = time_base_delta_to_interval(time - self.vehicle_state.base_time);
        self.vehicle_state.base_time = time;
        //println!("time {} dt {:.*} act: {:?}", time, 5, dt, actuators);

        (self.vehicle_model)(actuators, dt, &self.vehicle_state.local_env,  &mut self.vehicle_state.kinematic);

        let new_global_pos =
            self.vehicle_state.planet.position_at_distance(
                &self.vehicle_state.kinematic.inertial_position);

        self.vehicle_state.set_global_position(&new_global_pos, false);
        self.sensed.update_from_virtual(&self.vehicle_state);

    }

    pub fn get_ref_position(&self) -> GlobalPosition {
        self.vehicle_state.planet.get_reference_position()
    }
}





#[cfg(test)]
mod tests {
    use super::*;
    use rand::prelude::*;

    fn get_test_reference_position() -> GlobalPosition {
        GlobalPosition {
            lat: 37.8716, //degrees
            lon: -122.2727, //degrees
            alt_wgs84: 10.0 //meters
        }
    }

    fn check_basic_motion( state: &VirtualVehicleState) -> bool {
        let accel_z = state.kinematic.inertial_accel[2];
        let vel_z = state.kinematic.inertial_velocity[2];
        let motion_check;
        if accel_z != 0.0  && !state.kinematic.translation_constrained[2] {
            motion_check = vel_z != 0.0;
        }
        else {
            motion_check = true;
        }

        if !motion_check {
            println!("accel_z {} vel_z {} ", accel_z, vel_z);
        }

        motion_check
    }

    //TODO fix this random actuator test-- fails for small actuator values
    #[test]
    fn test_rand_actuator() {
        const TEST_INTERVAL: TimeBaseUnits = 1000; //millisecond
        let mut simulato = Simulato::new(&get_test_reference_position());
        let mut rng = rand::thread_rng();
        simulato.increment_simulated_time();
        let start_time = simulato.get_simulated_time();

        for i in 0..100 {
            let next_time = i*TEST_INTERVAL + start_time;
            let rand_act_level = rng.gen::<f32>().abs();
            let actuators: ActuatorControls = [rand_act_level; 16];

            simulato.update(next_time, &actuators);
            assert_eq!(true, check_basic_motion(&simulato.vehicle_state));
        }
    }


    quickcheck! {
        fn quickcheck_serial_update(act_val: f32 ) -> bool {
            let actuators: ActuatorControls = [act_val; 16];
            let mut simulato = Simulato::new(&get_test_reference_position());
            simulato.increment_simulated_time();

            const TEST_INTERVAL: TimeBaseUnits = 1000; //millisecond
            let time = simulato.get_simulated_time() + TEST_INTERVAL;

            simulato.set_simulated_time(time);
            simulato.update(time, &actuators);
            if !check_basic_motion( &simulato.vehicle_state) {
                return false;
            }

            let time = time + TEST_INTERVAL;
            simulato.set_simulated_time(time);
            simulato.update(time, &actuators);

            check_basic_motion(&simulato.vehicle_state)
        }
    }

}