



use num::NumCast;

use sensulator::Sensulator;

use crate::VirtualVehicleState;

use crate::physical_types::*;
use std::marker::PhantomData;
use sensulator::MeasureVal;


const MIN_SENSOR_REL_ERR:f32  = 1E-5;
const MIN_SENSOR_ABS_ERR:f32 = 1E-6;

const GYRO_REL_ERR: GyroUnits = 0.0000266;
const GYRO_ABS_ERR: GyroUnits = MIN_SENSOR_ABS_ERR;

const ACCEL_REL_ERR: AccelUnits = 0.0000267;
const ACCEL_ABS_ERR : AccelUnits = MIN_SENSOR_ABS_ERR;

const MAG_REL_ERR: MagUnits = 0.0000265;
const MAG_ABS_ERR: MagUnits = MIN_SENSOR_ABS_ERR;


const GPS_DEGREES_REL_ERR: LatLonUnits =  MIN_SENSOR_REL_ERR as LatLonUnits;
const GPS_DEGREES_ABS_ERR: LatLonUnits = MIN_SENSOR_ABS_ERR as LatLonUnits;


// this range appears to allow EKF fusion to begin
const ALT_ABS_ERR: DistanceUnits = MIN_SENSOR_ABS_ERR;
const ALT_REL_ERR: DistanceUnits = 1.5;


//const ALT_REL_ERR: DistanceUnits = 1.5;

const DIFF_PRESS_REL_ERR: PressureUnits = 1E-3;
const DIFF_PRESS_ABS_ERR: PressureUnits = MIN_SENSOR_ABS_ERR;

const AIR_PRESS_REL_ERR: PressureUnits = 1E-3;





pub trait SensorLike {
    fn new() -> Self;
    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self ;
}


pub struct Sensor3d<T:NumCast> {
    inner: [Sensulator;3],
    raw_val: [MeasureVal; 3],
    phantom: PhantomData<T>
}

impl <T:NumCast> Sensor3d<T> {

    fn new(absolute_err: T, deviation: T) -> Self {
        let abs_err: f32 = num::cast(absolute_err).unwrap();
        let dev: f32 = num::cast(deviation).unwrap();
        Sensor3d {
            inner: [
                Sensulator::new(0.0, abs_err, dev),
                Sensulator::new(0.0, abs_err, dev),
                Sensulator::new(0.0, abs_err, dev),
            ],
            raw_val: [0.0 , 0.0 , 0.0 ],
            phantom: PhantomData
        }
    }

    fn set_val_from_inner(&mut self) {
        self.raw_val = [
            self.inner[0].measure(),
            self.inner[1].measure(),
            self.inner[2].measure(),
            ];
    }

    pub fn get_val(&self) -> [MeasureVal; 3] {
        self.raw_val
    }


}



pub struct GyroSensor {
    senso: Sensor3d<GyroUnits>
}

impl GyroSensor {
    pub fn get_val(&self) -> [GyroUnits; 3] {
        self.senso.get_val()
    }
}

impl SensorLike for GyroSensor {
    fn new() -> Self {
        GyroSensor {
            senso: Sensor3d::new(GYRO_ABS_ERR, GYRO_REL_ERR)
         }
    }

    fn update(&mut self, state: &VirtualVehicleState)  -> &mut Self {
        //TODO functionalize this?
        self.senso.inner[0].set_center_value(state.dynamics.body_angular_velocity[0]);
        self.senso.inner[1].set_center_value(state.dynamics.body_angular_velocity[1]);
        self.senso.inner[2].set_center_value(state.dynamics.body_angular_velocity[2]);
        self.senso.set_val_from_inner();
        self
    }
}


pub struct AccelSensor {
    pub senso: Sensor3d<AccelUnits>,
}

impl AccelSensor {
    pub fn get_val(&self) -> [AccelUnits; 3] {
        self.senso.get_val()
    }
}

impl SensorLike for AccelSensor {
    fn new() -> Self {
        AccelSensor {
            senso: Sensor3d::new(ACCEL_ABS_ERR,ACCEL_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState)  -> &mut Self {
        //TODO functionalize this?
        self.senso.inner[0].set_center_value(state.dynamics.inertial_accel[0]);
        self.senso.inner[1].set_center_value(state.dynamics.inertial_accel[1]);
        self.senso.inner[2].set_center_value(state.dynamics.inertial_accel[2]);
        self.senso.set_val_from_inner();
        self
    }
}

pub struct MagSensor {
    senso: Sensor3d<MagUnits>
}

impl MagSensor {
    pub fn get_val(&self) -> [MagUnits; 3] {
        self.senso.get_val()
    }
}

impl SensorLike for MagSensor {
    fn new() -> Self {
        MagSensor {
            senso: Sensor3d::new(MAG_ABS_ERR,MAG_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState)  -> &mut Self {
        //TODO functionalize this?
        self.senso.inner[0].set_center_value(state.base_mag_field[0]);
        self.senso.inner[1].set_center_value(state.base_mag_field[1]);
        self.senso.inner[2].set_center_value(state.base_mag_field[2]);
        self.senso.set_val_from_inner();
        self
    }
}

pub struct GlobalPositionSensor {
    lat: Sensulator,
    lon: Sensulator,
    alt: Sensulator,
    value:GlobalPosition,
}

impl GlobalPositionSensor {
    pub fn get_val(&self) -> GlobalPosition {
        self.value
    }
}
impl SensorLike for GlobalPositionSensor {

    fn new() -> Self {
        GlobalPositionSensor {
            lat: Sensulator::new(0.0, GPS_DEGREES_ABS_ERR as MeasureVal, GPS_DEGREES_REL_ERR as MeasureVal),
            lon: Sensulator::new(0.0, GPS_DEGREES_ABS_ERR as MeasureVal, GPS_DEGREES_REL_ERR as MeasureVal),
            alt: Sensulator::new(0.0, ALT_ABS_ERR, ALT_REL_ERR),
            value: GlobalPosition {
                lat: 0.0,
                lon: 0.0,
                alt: 0.0
            }
        }
    }

    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self {
        self.lat.set_center_value(state.global_position.lat as MeasureVal);
        self.lon.set_center_value(state.global_position.lon as MeasureVal);
        self.alt.set_center_value(state.global_position.alt);
        self.value = GlobalPosition {
            lat: self.lat.measure() as LatLonUnits,
            lon: self.lon.measure() as LatLonUnits,
            alt: self.alt.measure() as DistanceUnits
        };
        self
    }

}

pub struct AirspeedSensor {
    diff_press: Sensulator,
}

impl SensorLike for AirspeedSensor {
    fn new() -> Self {
        AirspeedSensor {
            diff_press: Sensulator::new(0.0, DIFF_PRESS_ABS_ERR, DIFF_PRESS_REL_ERR ),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self {
        //convert airspeed to differential pressure
        //R = 287.05 J/(kg*degK)  for dry air
        let air_density_rho = state.get_local_air_density();

        // veloctiy = sqrt ( (2/rho) * (Ptotal - Pstatic) )
        // vel^2 = (2/rho) * DP
        //  DP = (rho / 2) * vel^2
        let dp = (state.relative_airspeed * state.relative_airspeed) * (air_density_rho / 2.0);
        self.diff_press.set_center_value(dp);
        self.diff_press.measure();
        self
    }
}

impl  AirspeedSensor {
    pub fn get_val(&self) -> PressureUnits {
        self.diff_press.peek()
    }
}



///
/// Sensor for ambient air pressure.
/// Normally corresponds to altitude, temperature, and humidity.
///
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
        //println!("airpress: {}", state.local_air_pressure);
        self.pressure.set_center_value(state.local_air_pressure);
        self.pressure.measure();
        self
    }

}

impl AirPressureSensor {
    pub fn get_val(&self) -> PressureUnits {
        self.pressure.peek()
    }
}
