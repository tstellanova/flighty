extern crate flighty;

use flighty::simulato::*;

use flighty::physical_types::*;
use flighty::models::*;
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
    let expected_accel =  flighty::models::MAX_ROTOR_ACCEL_VTOL_HYBRID -
        flighty::planet::PlanetEarth::STD_GRAVITY_ACCEL;

    println!("thrust: {:0.3} grav: {:0.3} expected_accel: {:0.3}",
             flighty::models::MAX_ROTOR_ACCEL_VTOL_HYBRID,
             flighty::planet::PlanetEarth::STD_GRAVITY_ACCEL,
             expected_accel);

    let actuators: ActuatorControls = [1.0; 16];

    let mut last_gps_alt = sim.vehicle_state.get_global_position().alt_wgs84;
    let mut last_z_vel:SpeedUnits = 0.0;

    for i in 0..100 {
        //max takeoff thrust
        sim.set_simulated_time( (i * 2000) + 500);
        sim.update(&actuators);

        //ensure that with constant rotor thrust, accel is constant
        let z_accel = sim.vehicle_state.kinematic.inertial_accel[2];
        println!("z_accel exp: {} act: {}", expected_accel, z_accel);
        assert_approx_eq!(z_accel, expected_accel);

        //ensure that Z velocity is ever increasing
        let z_vel = sim.vehicle_state.kinematic.inertial_velocity[2];
        println!("z_vel: {:.4} last_z_vel: {:0.4}", z_vel, last_z_vel);
        assert_ne!(last_z_vel, z_vel);
        assert_eq!(true, z_vel > last_z_vel);
        last_z_vel = z_vel;

        //ensure that gps altitude is ever increasing
        let gps_alt = sim.vehicle_state.get_global_position().alt_wgs84;
        println!("gps_alt: {:.4} last_gps_alt: {:0.4}", gps_alt, last_gps_alt);
        assert_ne!(gps_alt, last_gps_alt);
        assert_eq!(true, gps_alt > last_gps_alt);
        last_gps_alt = gps_alt;

    }

}
