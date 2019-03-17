/**
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/


//extern crate nalgebra as na;
use nalgebra::{Vector3}; //, Rotation3};


mod physical_types;

use physical_types::*;



/**
Tracks the "ideal" state of the vehicle, without sensor uncertainty
*/
pub struct VirtualVehicleState {

    /// Current time as measured at the vehicle
    pub base_time: TimeBaseUnits,

    /// Generalized vehicle temperature
    pub base_temperature: TemperatureUnits,

    ///--- Global position ----
    pub global_position: GlobalPosition,

    /// Local position in meters from "home"
    pub local_position: Vector3<DistanceUnits>,

    /// Linear velocity in body frame
    pub body_velocity: Vector3<SpeedUnits>,

    /// Linear acceleration in body frame
    pub body_accel:Vector3<AccelUnits>,

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
        VirtualVehicleState {
            base_time: 0,
            base_temperature: 0.0,
            global_position: *home,
            local_position: Vector3::new(0.0, 0.0, 0.0),
            body_velocity: Vector3::new(0.0, 0.0, 0.0),
            body_accel: Vector3::new(0.0, 0.0, 0.0),
            body_angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            body_angular_accel: Vector3::new(0.0, 0.0, 0.0),
            base_mag_field: Vector3::new(0.0, 0.0, 0.0),
            relative_airspeed: 0.0,
            local_air_pressure: 0.0
        }
    }

}


mod sensors;
use sensors::SensorLike;

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

}
