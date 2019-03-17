


use crate::{VirtualVehicleState, SensedPhysicalState};
use crate::models::*;
use crate::physical_types::*;

pub struct Simulato {
    vehicle_state: VirtualVehicleState,
    vehicle_model: ApplyUpdatesFn,
    sensed_state: SensedPhysicalState,
}


impl Simulato {
    pub fn new() -> Self {
        let home = GlobalPosition {
            lat: 37.8001024,
            lon: -122.1997184,
            alt: 500.0
        };
        Simulato {
            vehicle_state: VirtualVehicleState::new(&home),
            vehicle_model: empty_model_fn,
            sensed_state: SensedPhysicalState::new()
        }
    }

    pub fn load_vehicle_model(&mut self) {
        //TODO hardcoded for now. Should be configurable by env vars
        self.vehicle_model = vtol_hybrid_model_fn;
    }

    /**
    Heart of the update loop:
    - Given a set of current actuator outputs,
    - Use the model to update the idealized virtual state of the vehicle.
    - Then update the vehicle's sensed state.
    */
    pub fn update(&mut self, actuators: &ActuatorOutputs) {
        (self.vehicle_model)(actuators, &mut self.vehicle_state);
        self.sensed_state.update_from_virtual(&self.vehicle_state);
    }
}

//TODO define
pub struct ActuatorOutputs {
    pub outputs:Vec<u32>,
}