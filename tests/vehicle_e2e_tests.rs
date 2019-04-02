extern crate flighty;

use flighty::simulato::*;

use flighty::physical_types::*;
use flighty::models::ActuatorControls;
use assert_approx_eq::assert_approx_eq;

fn get_test_reference_position() -> GlobalPosition {
    GlobalPosition {
        lat: 37.8716, //degrees
        lon: -122.2727, //degrees
        alt_wgs84: 10.0 //meters
    }
}

#[test]
pub fn test_vehicle_takeoff() {

    let mut sim = Simulato::new(&get_test_reference_position());

    const EXPECTED_Z_ACCEL: AccelUnits = -30.400618; //TODO derive from vehicle mass and rotor thrust

    for _i in 0..100 {
        //max takeoff thrust
        let actuators: ActuatorControls = [1.0; 16];

        sim.increment_simulated_time();
        sim.update(&actuators);

        let z_accel = sim.vehicle_state.kinematic.inertial_accel[2];
        println!("z_accel exp {} act {}",EXPECTED_Z_ACCEL, z_accel);
        assert_approx_eq!(z_accel, EXPECTED_Z_ACCEL);
    }

}
