


use crate::{VirtualVehicleState, SensedPhysicalState};
use crate::models::*;
use crate::physical_types::*;
use crate::planet::Planetary;

pub struct Simulato {
    vehicle_state: VirtualVehicleState,
    vehicle_model: KinematicModelFn,
    sensed_state: SensedPhysicalState,
}


impl Simulato {
    pub fn new() -> Self {
        let home = GlobalPosition {
            lat: 37.8001024,
            lon: -122.1997184,
            alt: 10.0
        };
        Simulato {
            vehicle_state: VirtualVehicleState::new(&home),
            //TODO hardcoded for now. Should be configurable by env vars
            vehicle_model: vtol_hybrid_model_fn,
            sensed_state: SensedPhysicalState::new()
        }
    }

    /// Heart of the update loop:
    ///  - Given a set of current actuator outputs,
    ///  - Use the model to update the idealized virtual state of the vehicle.
    ///  - Then update the vehicle's sensed state.
    ///
    pub fn update(&mut self, time: TimeBaseUnits, actuators: &ActuatorOutputs) {
        let dt = time - self.vehicle_state.base_time;
        self.vehicle_state.base_time = time;
        println!("time {} dt {} ", time, dt);

        (self.vehicle_model)(actuators, dt, &mut self.vehicle_state.kinematic);
        self.sensed_state.update_from_virtual(&self.vehicle_state);
    }

    pub fn get_ref_position(&self) -> GlobalPosition {
        self.vehicle_state.planet.get_reference_position()
    }
}





#[cfg(test)]
mod tests {
    use crate::simulato::Simulato;
    use crate::physical_types::*;
    use crate::models::ActuatorOutputs;
    use rand::Rng;


    #[test]
    fn test_simulato() {
        let mut simulato = Simulato::new();
        let mut actuators:ActuatorOutputs = [0.5; 16];

        assert_eq!(simulato.vehicle_state.base_time, 0);
        let mut rng = rand::thread_rng();

        for i in 0..100 {
            let time:TimeBaseUnits = i * 10;
            let rand_act_level = rng.gen::<f32>();
            actuators[0] = rand_act_level;

            simulato.update(time, &actuators);
            assert_eq!(simulato.vehicle_state.base_time, time);

            //println!("act0: {} vel0: {} ", actuators[0], vel0);

            //TODO
            if time != 0 {
                let vel0 = simulato.vehicle_state.kinematic.inertial_velocity[0];
                assert_ne!(vel0, 0.0);
            }
        }
    }

    quickcheck! {
        fn check_simulato_update(
            time:TimeBaseUnits,
            act_val: f32 ) -> bool {

            let actuators: ActuatorOutputs = [act_val; 16];
            let mut simulato = Simulato::new();
            simulato.update(time, &actuators);
            let time_check = simulato.vehicle_state.base_time == time;
            if !time_check {
                println!("times: {}   {} ", time, simulato.vehicle_state.base_time);
            }

            //TODO check stimulus of simulato
            time_check

        }
    }

}