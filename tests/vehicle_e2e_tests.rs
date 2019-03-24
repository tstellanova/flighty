extern crate flighty;

use flighty::simulato::*;
//use flighty::*;
use flighty::physical_types::*;
use flighty::models::ActuatorControls;

#[test]
pub fn test_vehicle_e2e() {

    let mut sim = Simulato::new();

    for i in 0..1000 {
        let time:TimeBaseUnits = i * 10;
        let actuators: ActuatorControls = [0.55; 16];
        sim.update(time, &actuators);
        //TODO test something useful
    }

}
