


extern crate num;

use num::NumCast;


use sensulator::Sensulator;

use crate::VirtualVehicleState;

use crate::physical_types::*;
use std::marker::PhantomData;



const GYRO_REL_ERR : GyroUnits = 0.0000266;
const ACCEL_REL_ERR : AccelUnits = 0.0000267;
const MAG_REL_ERR : MagUnits = 0.0000265;
const GPS_DEGREES_REL_ERR: LatLonUnits =  1E-5;
const GPS_ALTITUDE_REL_ERR: DistanceUnits = 1.0;
//const ALT_REL_ERR: DistanceUnits = 1.5;
const DIFF_PRESS_REL_ERR: PressureUnits = 1E-3;

const AIR_PRESS_REL_ERR: PressureUnits = 1E-3;

const GAS_CONSTANT_R1: f32 = 287.05;




pub trait SensorLike {
    fn new() -> Self;
    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self ;
}


pub struct Sensor3d<T:NumCast> {
    inner: [Sensulator;3],
    phantom: PhantomData<T>
}

impl <T:NumCast> Sensor3d<T> {

    fn new(deviation: T) -> Self {
        let dev: f32 = num::cast(deviation).unwrap();
        Sensor3d {
            inner: [
                Sensulator::new(0.0, 0.0, dev),
                Sensulator::new(0.0, 0.0, dev),
                Sensulator::new(0.0, 0.0, dev),
            ],
            phantom: PhantomData
        }
    }

    pub fn peek(&self) -> (T, T, T) {
        (num::cast(self.inner[0].peek()).unwrap(),
         num::cast(self.inner[1].peek()).unwrap(),
         num::cast(self.inner[2].peek()).unwrap(), )
    }


}


pub struct GyroSensor {
    senso: Sensor3d<GyroUnits>
}

impl SensorLike for GyroSensor {
    fn new() -> Self {
        GyroSensor {
            senso: Sensor3d::new(GYRO_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState)  -> &mut Self {
        //TODO functionalize this?
        self.senso.inner[0].set_center_value(state.body_angular_velocity[0]);
        self.senso.inner[1].set_center_value(state.body_angular_velocity[1]);
        self.senso.inner[2].set_center_value(state.body_angular_velocity[2]);
        self
    }
}

pub struct AccelSensor {
    pub senso: Sensor3d<AccelUnits>
}

impl SensorLike for AccelSensor {
    fn new() -> Self {
        AccelSensor {
            senso: Sensor3d::new(ACCEL_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState)  -> &mut Self {
        //TODO functionalize this?
        self.senso.inner[0].set_center_value(state.inertial_accel[0]);
        self.senso.inner[1].set_center_value(state.inertial_accel[1]);
        self.senso.inner[2].set_center_value(state.inertial_accel[2]);
        self
    }
}


pub struct MagSensor {
    senso: Sensor3d<MagUnits>
}

impl SensorLike for MagSensor {
    fn new() -> Self {
        MagSensor {
            senso: Sensor3d::new(MAG_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState)  -> &mut Self {
        //TODO functionalize this?
        self.senso.inner[0].set_center_value(state.base_mag_field[0]);
        self.senso.inner[1].set_center_value(state.base_mag_field[1]);
        self.senso.inner[2].set_center_value(state.base_mag_field[2]);
        self
    }
}

pub struct GlobalPositionSensor {
    lat: Sensulator,
    lon: Sensulator,
    alt: Sensulator,
}

impl SensorLike for GlobalPositionSensor {

    fn new() -> Self {
        GlobalPositionSensor {
            lat: Sensulator::new(0.0, 0.0, GPS_DEGREES_REL_ERR as f32),
            lon: Sensulator::new(0.0, 0.0, GPS_DEGREES_REL_ERR as f32),
            alt: Sensulator::new(0.0, 0.0, GPS_ALTITUDE_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self {
        self.lat.set_center_value(state.global_position.lat as f32);
        self.lon.set_center_value(state.global_position.lon as f32);
        self.alt.set_center_value(state.global_position.alt);
        self
    }
}

pub struct AirspeedSensor {
    alt_pressure: PressureUnits,
    airspeed: SpeedUnits,
    diff_press: Sensulator,
}

impl SensorLike for AirspeedSensor {
    fn new() -> Self {
        AirspeedSensor {
            alt_pressure: 0.0,
            airspeed: 0.0,
            diff_press: Sensulator::new(0.0, 0.0, DIFF_PRESS_REL_ERR ),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self {
        //convert airspeed to differential pressure
        self.airspeed =  state.relative_airspeed;
        self.alt_pressure = state.local_air_pressure;
        //R = 287.05 J/(kg*degK)  for dry air
        let air_density_rho = self.alt_pressure / (GAS_CONSTANT_R1 * state.base_temperature);

        // veloctiy = sqrt ( (2/rho) * (Ptotal - Pstatic) )
        // vel^2 = (2/rho) * DP
        //  DP = (rho / 2) * vel^2
        let dp = (self.airspeed * self.airspeed) * (air_density_rho / 2.0);
        self.diff_press.set_center_value(dp);

        self
    }
}

impl  AirspeedSensor {
    pub fn peek(&self) -> PressureUnits {
        self.diff_press.peek()
    }
}



/**
Sensor for ambient air pressure.
Normally corresponds to altitude, temperature, and humidity.
*/
pub struct AirPressureSensor {
    pressure: Sensulator,
}


impl SensorLike for AirPressureSensor {
    fn new() -> Self {
        AirPressureSensor {
            pressure: Sensulator::new(0.0, 0.0, AIR_PRESS_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self {
        self.pressure.set_center_value(state.local_air_pressure);
        self
    }
}


impl AirPressureSensor {

}
