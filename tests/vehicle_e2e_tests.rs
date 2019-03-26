extern crate flighty;

use flighty::simulato::*;
//use flighty::*;
use flighty::physical_types::*;
use flighty::models::ActuatorControls;

#[test]
pub fn test_vehicle_e2e() {

    let mut sim = Simulato::new();

    let mut last_time:TimeBaseUnits = 0;
    for _i in 0..100 {
        let actuators: ActuatorControls = [0.55; 16];
        //TODO replace this hack perf test with benchmark?
        sim.increment_simulated_time();
        let time = sim.get_simulated_time();
        sim.update(time, &actuators);
        if 0 != last_time {
            let dt = time - last_time;
            assert_eq!( (dt < 100), true);
        }
        last_time = time;

    }

}
