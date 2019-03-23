/**
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/


use nalgebra::{Vector3};


pub mod physical_types;

use physical_types::*;

mod sensors;
use sensors::SensorLike;

pub mod models;

pub mod simulato;

pub mod planet;
use planet::{Planetary, PlanetEarth};

#[cfg(test)]
#[macro_use]
extern crate quickcheck;


/// Stores the linear and angular
/// position, velocity, acceleration of a rigid body (6 degrees of freedom)
///
/// We use two reference frames:
/// - Inertial frame: the North-East-Down (NED) external frame that references gravity ("down")
/// - Body frame: references the vehicle's axes
///
pub struct RigidBodyState {
    /// Local position: inertial frame distance from "home"
    pub inertial_position: Vector3<DistanceUnits>,

    /// Translational velocity in inertial frame
    pub inertial_velocity: Vector3<SpeedUnits>,

    /// Translational acceleration in inertial frame
    pub inertial_accel:Vector3<AccelUnits>,

    /// Angular position in body frame (aka "attitude")
    pub body_angular_position: Vector3<AngularPosUnits>,

    /// Angular velocity in body frame (aka "rotation speed")
    pub body_angular_velocity: Vector3<AngularSpeedUnits>,

    /// Angular acceleration in body frame (aka "rotation rate")
    pub body_angular_accel:Vector3<AngularAccelUnits>,
}

impl RigidBodyState {
    pub fn new() -> RigidBodyState {
        RigidBodyState {
            inertial_position: Vector3::new(0.0, 0.0, 0.0),
            inertial_velocity: Vector3::new(0.0, 0.0, 0.0),
            inertial_accel: Vector3::new(0.0, 0.0, 0.0),

            body_angular_position: Vector3::new(0.0, 0.0, 0.0),
            body_angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            body_angular_accel: Vector3::new(0.0, 0.0, 0.0),
        }
    }
}


/// Tracks the "ideal" state of the vehicle, without sensor uncertainty.
pub struct VirtualVehicleState {
    /// Current time according the vehicle internal clock
    pub base_time: TimeBaseUnits,

    /// The kinematic rigid body state of the vehicle
    pub kinematic: RigidBodyState,

    /// Global position
    global_position: GlobalPosition,

    /// Generalized vehicle temperature
    pub base_temperature: TemperatureUnits,

    /// Idealized magnetic field at the current location
    pub base_mag_field: Vector3<MagUnits>,

    /// Idealized airspeed (relative speed of air past the nose of the vehicle)
    pub relative_airspeed: SpeedUnits,

    /// Idealized atmospheric air pressure at the current location
    pub local_air_pressure: PressureUnits,

    /// The planetary environment the vehicle is in
    planet: PlanetEarth,

}

impl VirtualVehicleState {

    /// - ref_position : A reference global position from which inertial frame is measured
    pub fn new(ref_position:&GlobalPosition) -> Self {
        let mut inst = VirtualVehicleState {
            base_time: 0,
            kinematic:  RigidBodyState::new(),
            global_position: *ref_position,
            base_temperature: 0.0,
            base_mag_field: Vector3::new(0.0, 0.0, 0.0),
            local_air_pressure: 0.0,
            relative_airspeed: 0.0,
            planet: PlanetEarth::new(&ref_position),
        };

        inst.set_global_position(ref_position, true);
        inst
    }

    pub fn set_global_position(&mut self, pos: &GlobalPosition,  update_local_pos: bool) {
        self.global_position = *pos;
        let baro_press = PlanetEarth::altitude_to_baro_pressure(self.global_position.alt);
        self.local_air_pressure = baro_press;
        self.base_mag_field = self.planet.calculate_mag_field(pos);
        if update_local_pos {
            self.kinematic.inertial_position =
                self.planet.calculate_relative_distance(pos);
        }
    }

    pub fn get_global_position(&self) -> GlobalPosition {
        self.global_position
    }


}



/// Simulated sensed state
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


    #[test]
    fn test_phys_init() {
        const SEA_LEVEL_AIR_PRESSURE: PressureUnits = 1013.25;

        let pos = GlobalPosition {
            lat: 0.0,
            lon: 0.0,
            alt: 0.0
        };
        let mut virt = VirtualVehicleState::new(&pos);
        let mut sensed = SensedPhysicalState::new();
        virt.relative_airspeed = 55.0;
        // 101325 Pascals in millibars
        virt.local_air_pressure = SEA_LEVEL_AIR_PRESSURE;

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
