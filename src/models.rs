


use crate::{VirtualVehicleState};
use crate::simulato::ActuatorOutputs;



pub type ApplyUpdatesFn = fn(&ActuatorOutputs, &mut VirtualVehicleState);


pub fn empty_model_fn(_actuators: &ActuatorOutputs, _virt: &mut VirtualVehicleState) {
    //does nothing
}

pub fn vtol_hybrid_model_fn(actuators: &ActuatorOutputs, virt: &mut VirtualVehicleState) {
    if actuators.outputs[0] > 0 {
        //TODO map actuators to physical moment arms
        virt.inertial_position[0] += 0.1;
        virt.inertial_position[1] += 0.1;
        virt.inertial_position[2] += 0.1;

        virt.inertial_velocity[0] = 1.0;
        virt.inertial_velocity[1] = 1.0;
        virt.inertial_velocity[2] = 0.1;
    }
    else {
        virt.inertial_velocity[0] = 0.0;
        virt.inertial_velocity[1] = 0.0;
        virt.inertial_velocity[2] = 0.0;
    }


}





