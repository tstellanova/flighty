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
use planet::{ExternalForceEnvironment, Planetary, PlanetEarth};

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

    /// The dynamic rigid body state of the vehicle
    pub kinematic: RigidBodyState,

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

    /// Simple snapshot of the local force environment
    local_env: ExternalForceEnvironment,

    /// Global position
    global_position: GlobalPosition,

}



impl VirtualVehicleState {

    /// - ref_position : A reference global position from which inertial frame is measured
    pub fn new(ref_position:&GlobalPosition) -> Self {
        let mut inst = VirtualVehicleState {
            base_time: 0,
            kinematic:  RigidBodyState::new(),
            global_position: *ref_position,
            base_temperature: PlanetEarth::STD_TEMP as TemperatureUnits,
            base_mag_field: Vector3::new(0.0, 0.0, 0.0),
            local_air_pressure: 0.0,
            relative_airspeed: 0.0,
            planet: PlanetEarth::new(&ref_position),
            local_env: PlanetEarth::default_local_environment(),
        };

        inst.set_global_position(ref_position, true);
        inst
    }

    pub fn set_global_position(&mut self, pos: &GlobalPosition, update_local_pos: bool) {
        self.global_position = *pos;
        self.local_air_pressure =
            PlanetEarth::altitude_to_baro_pressure(self.global_position.alt);
        self.base_mag_field = self.planet.calculate_mag_field(pos);

        //TODO update ExternalForceEnvironment
        //self.local_env
//        local_env: PlanetEarth::default_local_environment();

        if update_local_pos {
            self.kinematic.inertial_position =
                self.planet.calculate_relative_distance(pos);
        }

    }

    pub fn get_global_position(&self) -> GlobalPosition {
        self.global_position
    }

    pub fn get_local_air_density(&self) -> f32 {
        const GAS_CONSTANT_R1: f32 = 287.05;
        self.local_air_pressure / (GAS_CONSTANT_R1 * self.base_temperature)
    }

}



/// Simulates data arriving from sensors.
///
pub struct PhysicalSensors {

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

    /// Atmospheric pressure
    pub baro: sensors::AirPressureSensor,
}

impl PhysicalSensors {
    pub fn new() ->  Self {
        PhysicalSensors {
            gps: sensors::GlobalPositionSensor::new(),
            gyro: sensors::GyroSensor::new(),
            accel: sensors::AccelSensor::new(),
            mag: sensors::MagSensor::new(),
            airspeed: sensors::AirspeedSensor::new(),
            baro: sensors::AirPressureSensor::new(),
        }
    }

    pub fn update_from_virtual(&mut self, virt: &VirtualVehicleState) {
        self.gps.update(virt);
        self.gyro.update(virt);
        self.accel.update(virt);
        self.mag.update(virt);
        self.airspeed.update(virt);
        self.baro.update(virt);

//        println!("sensed {} alt {:.3} gyro {:.6} accel {:.6} mag {:.6} asp {:.6} baro {:.6}",
//                 virt.base_time,
//                 self.gps.get_val().alt,
//                 self.gyro.get_val()[0],
//                 self.accel.get_val()[0],
//                 self.mag.get_val()[0],
//                 self.airspeed.get_val(),
//                 self.baro.get_val());
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_phys_init() {
        const SEA_LEVEL_AIR_PRESSURE: PressureUnits = 1013.25;

        let pos = GlobalPosition {
            lat: 0.0,
            lon: 0.0,
            alt: 0.0
        };
        let mut virt = VirtualVehicleState::new(&pos);
        let mut sensed = PhysicalSensors::new();
        virt.relative_airspeed = 55.0;
        // 101325 Pascals in millibars
        virt.local_air_pressure = SEA_LEVEL_AIR_PRESSURE;

        sensed.update_from_virtual(&virt);
        assert_eq!(virt.global_position.alt, pos.alt);

        let accel = sensed.accel.senso.get_val();
        assert_approx_eq!(accel[0], 0.0, 1E-3);

        let diff_press = sensed.airspeed.get_val();
        assert_approx_eq!(diff_press, 18.528276, 1E-2);//TODO verify speed from diff pressure
    }


    #[test]
    fn test_rigid_body_init() {
        let motion = RigidBodyState::new();
        let zero_accel:Vector3<AccelUnits> = Vector3::new(0.0, 0.0, 0.0);
        let zero_velocity:Vector3<SpeedUnits> = Vector3::new(0.0, 0.0, 0.0);
        let zero_position:Vector3<DistanceUnits> = Vector3::new(0.0, 0.0, 0.0);
        assert_eq!(zero_accel, motion.inertial_accel);
        assert_eq!(zero_velocity, motion.inertial_velocity);
        assert_eq!(zero_position, motion.inertial_position);

        let zero_accel:Vector3<AngularAccelUnits> = Vector3::new(0.0, 0.0, 0.0);
        let zero_velocity:Vector3<AngularSpeedUnits> = Vector3::new(0.0, 0.0, 0.0);
        let zero_position:Vector3<AngularPosUnits> = Vector3::new(0.0, 0.0, 0.0);
        assert_eq!(zero_accel, motion.body_angular_accel);
        assert_eq!(zero_velocity, motion.body_angular_velocity);
        assert_eq!(zero_position, motion.body_angular_position);
    }

//    #[test]
//    fn test_set_global_position() {
//        let home = GlobalPosition {
//            lat: 37.8,
//            lon: -122.2,
//            alt: 10.0
//        };
//
//        let new_pos = GlobalPosition {
//            lat: 37.8001024,
//            lon: -122.1997184,
//            alt: 100.0
//        };
//        let mut virt = VirtualVehicleState::new(&home);
//
//        virt.set_global_position(&new_pos, true);
//
//        let expected_air_pressure = PlanetEarth::altitude_to_baro_pressure(new_pos.alt);
//        assert_approx_eq!(expected_air_pressure, virt.local_air_pressure);
//    }



}
