

use std::time::{Duration, SystemTime};

use crate::{VirtualVehicleState, SensedPhysicalState};
use crate::models::*;
use crate::physical_types::*;
use crate::planet::Planetary;

pub struct Simulato {
    boot_time: SystemTime,
    simulated_time: TimeBaseUnits,
    pub abstime_offset: TimeBaseUnits,
    pub vehicle_state: VirtualVehicleState,
    vehicle_model: KinematicModelFn,
    pub sensed: SensedPhysicalState,
}


impl Simulato {
    pub fn new() -> Self {
        let home = GlobalPosition {
            lat: 37.8001024,
            lon: -122.1997184,
            alt: 10.0
        };
        Simulato {
            boot_time: SystemTime::now(),
            simulated_time: 0,
            abstime_offset: 500,

            vehicle_state: VirtualVehicleState::new(&home),
            //TODO hardcoded for now. Should be configurable by env vars
            vehicle_model: vtol_hybrid_model_fn,
            sensed: SensedPhysicalState::new()
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
        //TODO decide who actually generates the time base updates
        let dt: TimeIntervalUnits =
            ((time - self.vehicle_state.base_time) as TimeIntervalUnits) * TIME_BASE_DELTA_TO_INTERVAL;
        self.vehicle_state.base_time = time;
        println!("time {} dt {:.*} act: {:?}", time, 5, dt, actuators);

        (self.vehicle_model)(actuators, dt, &mut self.vehicle_state.kinematic);

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
    //use rand::Rng;

    fn check_basic_motion(time:TimeBaseUnits, state: &VirtualVehicleState) -> bool {
        let time_check = state.base_time == time;
        if !time_check {
            println!("times: {}   {} ", time, state.base_time);
        }

        let accel_z = state.kinematic.inertial_accel[2];
        let vel_z = state.kinematic.inertial_velocity[2];
        let motion_check;
        if accel_z != 0.0 {
            motion_check = vel_z != 0.0;
        }
        else {
            motion_check = true;
        }

        if !motion_check {
            println!("accel_z {} vel_z {} ", accel_z, vel_z);
        }

        (time_check && motion_check)
    }

    //TODO fix this random actuator test-- fails for small actuator values
//    #[test]
//    fn test_simulato() {
//        let mut simulato = Simulato::new();
//        let mut actuators:ActuatorOutputs = [0.51; 16];
//
//        assert_eq!(simulato.vehicle_state.base_time, 0);
//        let mut rng = rand::thread_rng();
//
//        for i in 0..100 {
//            let time:TimeBaseUnits = i * 100000;
//            let rand_act_level = rng.gen::<f32>();
//            actuators[0] = rand_act_level;
//
//            simulato.update(time, &actuators);
//            assert_eq!(true, check_basic_motion(time, &simulato.vehicle_state));
//        }
//    }

    quickcheck! {
        fn check_simulato_initial_update(
            time:TimeBaseUnits,
            act_val: f32 ) -> bool {

            let actuators: ActuatorControls = [act_val; 16];
            let mut simulato = Simulato::new();

            simulato.update(time, &actuators);
            if !check_basic_motion(time, &simulato.vehicle_state) {
             return false;
             }

            let time = 2*time;
            simulato.update(time, &actuators);
            check_basic_motion(time, &simulato.vehicle_state)
        }
    }

}