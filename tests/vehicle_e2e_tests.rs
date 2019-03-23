extern crate flighty;

use flighty::simulato::*;
//use flighty::*;
use flighty::physical_types::*;
use flighty::models::ActuatorOutputs;

#[test]
pub fn test_vehicle_stuff() {

    let mut sim = Simulato::new();

    let mut actuators: ActuatorOutputs = [0.0; 16];

    for i in 0..1000 {
        let time:TimeBaseUnits = i * 10;
        actuators[0] = 0.1;
        sim.update(time, &actuators);
        //TODO actually test something
    }

}