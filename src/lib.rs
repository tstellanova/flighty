/**
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/


use nalgebra::{Vector3};


mod physical_types;

use physical_types::*;

mod sensors;
use sensors::SensorLike;

mod models;

pub mod simulato;

pub mod planet;
use planet::Planet;

/**
Tracks the "ideal" state of the vehicle, without sensor uncertainty.
We work with two reference frames:
- Inertial frame: the North-East-Down (NED) external frame that references gravity ("down")
- Body frame: references the vehicle's axes
*/
pub struct VirtualVehicleState {

    /// Current time according the vehicle internal clock
    pub base_time: TimeBaseUnits,

    /// Generalized vehicle temperature
    pub base_temperature: TemperatureUnits,

    /// Global position
    global_position: GlobalPosition,

    /// Home position (a reference position upon which inertial frame is centered)
    home_position: GlobalPosition,

    /// Local position: inertial frame distance from "home"
    pub inertial_position: Vector3<DistanceUnits>,

    /// Translational velocity in inertial frame
    pub inertial_velocity: Vector3<SpeedUnits>,

    /// Translational acceleration in inertial frame
    pub inertial_accel:Vector3<AccelUnits>,

    /// Angular position in body frame
    pub body_angular_position: Vector3<AngularPosUnits>,

    /// Angular velocity in body frame
    pub body_angular_velocity: Vector3<AngularSpeedUnits>,

    /// Angular acceleration in body frame
    pub body_angular_accel:Vector3<AngularAccelUnits>,

    /// Idealized magnetic field at the current location
    pub base_mag_field: Vector3<MagUnits>,

    /// Idealized airspeed (relative speed of air past the nose of the vehicle)
    pub relative_airspeed: SpeedUnits,

    /// Idealized atmospheric air pressure at the current location
    pub local_air_pressure: PressureUnits,

}

impl VirtualVehicleState {

    pub fn new(home:&GlobalPosition) -> Self {
        let mut inst = VirtualVehicleState {
            base_time: 0,
            base_temperature: 0.0,
            global_position: GlobalPosition {
                lat: 0.0,
                lon: 0.0,
                alt: 0.0
            },
            home_position: *home,

            base_mag_field: Vector3::new(0.0, 0.0, 0.0),
            local_air_pressure: 0.0,

            inertial_position: Vector3::new(0.0, 0.0, 0.0),
            inertial_velocity: Vector3::new(0.0, 0.0, 0.0),
            inertial_accel: Vector3::new(0.0, 0.0, 0.0),

            body_angular_position: Vector3::new(0.0, 0.0, 0.0),
            body_angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            body_angular_accel: Vector3::new(0.0, 0.0, 0.0),
            relative_airspeed: 0.0,
        };

        inst.set_global_position(home, true);

        inst
    }

    pub fn set_global_position(&mut self, pos: &GlobalPosition,  update_local_pos: bool) {
        self.global_position = *pos;
        let baro_press = Planet::altitude_to_baro_pressure(self.global_position.alt);
        self.local_air_pressure = baro_press;
        self.base_mag_field = Planet::calculate_mag_field(pos);
        if update_local_pos {
            self.inertial_position =
                Planet::calculate_inertial_position(&self.home_position, pos);
        }
    }

    pub fn get_global_position(&self) -> GlobalPosition {
        self.global_position
    }



}




/**
Simulated sensed state
*/
pub struct SensedPhysicalState {

    pub temperature: TemperatureUnits,

    ///--- Data arriving directly from sensors:
    /// GPS
    pub gps: sensors::GlobalPositionSensor,

    /// Gyro
    pub gyro: sensors::GyroSensor,

    /// Accelerometer
    pub accel: sensors::AccelSensor,

    /// Magnetometer
    pub mag: sensors::MagSensor,

    /// Airspeed (differential pressure)
    pub airspeed: sensors::AirspeedSensor,
}

impl SensedPhysicalState {
    pub fn new() ->  Self {
        SensedPhysicalState {
            temperature: 0.0,
            gps: sensors::GlobalPositionSensor::new(),
            gyro: sensors::GyroSensor::new(),
            accel: sensors::AccelSensor::new(),
            mag: sensors::MagSensor::new(),
            airspeed: sensors::AirspeedSensor::new(),
        }
    }

    pub fn update_from_virtual(&mut self, virt: &VirtualVehicleState) {
        self.gps.update(virt);
        self.gyro.update(virt);
        self.accel.update(virt);
        self.mag.update(virt);
        self.airspeed.update(virt);

        // No need to simulate wandering temperature?
        self.temperature = virt.base_temperature;
    }
}


#[cfg(test)]
mod tests {
    use crate::{SensedPhysicalState, VirtualVehicleState};
    use crate::physical_types::*;
    use crate::sensors;
    use crate::simulato::Simulato;

    #[test]
    fn test_phys_init() {
        let pos = GlobalPosition {
            lat: 0.0,
            lon: 0.0,
            alt: 0.0
        };
        let mut virt = VirtualVehicleState::new(&pos);
        let mut sensed = SensedPhysicalState::new();
        virt.relative_airspeed = 55.0;
        virt.local_air_pressure = sensors::SEA_LEVEL_AIR_PRESSURE;

        sensed.update_from_virtual(&virt);
        assert_eq!(virt.global_position.alt, pos.alt);

        let accel = sensed.accel.senso.peek();
        let diff = (accel.0 - 0.0).abs();
        assert_eq!( diff < 1E-3, true);

        let diff_press = sensed.airspeed.peek();
        let diff = (diff_press - 0.0).abs();
        assert_eq!((diff < 1.0), false);

    }

    #[test]
    fn test_simulato() {
        let mut simulato = Simulato::new();
        simulato.load_vehicle_model();

    }

}
