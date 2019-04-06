



use num::NumCast;

use sensulator::Sensulator;

use crate::VirtualVehicleState;

use crate::physical_types::*;
use std::marker::PhantomData;
use sensulator::MeasureVal;
use nalgebra::Vector3;

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
const ALT_REL_ERR: DistanceUnits = 1.5;
const ALT_ABS_ERR: DistanceUnits = MIN_SENSOR_ABS_ERR;


// GPS horizontal velocity errors
const GPS_HVEL_REL_ERR: SpeedUnits = 0.03; //TODO verify -- based on req_hdrift
const GPS_HVEL_ABS_ERR: SpeedUnits = MIN_SENSOR_ABS_ERR;



// GPS vertical velocity errors
const GPS_VVEL_REL_ERR: SpeedUnits = 0.05; //TODO verify -- based on req_vdrift
const GPS_VVEL_ABS_ERR: SpeedUnits = MIN_SENSOR_ABS_ERR;


// differential pressure errors (used for eg Pitot airspeed)
const DIFF_PRESS_REL_ERR: PressureUnits = 1E-3;
const DIFF_PRESS_ABS_ERR: PressureUnits = MIN_SENSOR_ABS_ERR;

// barometer air pressure errors

const AIR_PRESS_REL_ERR: PressureUnits = 1E-3;
const AIR_PRESS_ABS_ERR: PressureUnits = MIN_SENSOR_ABS_ERR;


pub trait SensorLike {
    fn new() -> Self;
    /// pull updated data from input state
    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self ;
    /// generate a new set of simulated measurements
    fn remeasure(&mut self);
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
        self.senso.inner[0].set_center_value(state.kinematic.body_angular_velocity[0]);
        self.senso.inner[1].set_center_value(state.kinematic.body_angular_velocity[1]);
        self.senso.inner[2].set_center_value(state.kinematic.body_angular_velocity[2]);
        self.remeasure();
        self
    }

    fn remeasure(&mut self) {
        self.senso.set_val_from_inner();
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
        self.senso.inner[0].set_center_value(state.kinematic.inertial_accel[0]);
        self.senso.inner[1].set_center_value(state.kinematic.inertial_accel[1]);
        self.senso.inner[2].set_center_value(state.kinematic.inertial_accel[2]);
        self.remeasure();
        self
    }

    fn remeasure(&mut self) {
        self.senso.set_val_from_inner();
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
        self.remeasure();
        self
    }

    fn remeasure(&mut self) {
        self.senso.set_val_from_inner();
    }
}


/// velocity: N, E, D, ground speed
pub type GlobalNEDVelocity = [SpeedUnits; 4];

pub struct GlobalPositionSensor {
    lat: Sensulator,
    lon: Sensulator,
    alt: Sensulator,
    global_pos_value:GlobalPosition,

    vel_north: Sensulator,
    vel_east: Sensulator,
    vel_down: Sensulator,
    velocity_value: GlobalNEDVelocity,

    delayed_velocity: Vector3<SpeedUnits>,
}




impl GlobalPositionSensor {
    pub fn get_global_pos(&self) -> GlobalPosition {
        self.global_pos_value
    }

    pub fn get_velocity(&self) -> GlobalNEDVelocity {
        self.velocity_value
    }

//    fn get_gps_vel_exp_weighted_moving_avg(new_val: SpeedUnits, ema: SpeedUnits) -> SpeedUnits {
//        const GPS_DELAY_WINDOW:SpeedUnits = 1.0;
//        const GPS_DELAY_ALPHA:SpeedUnits = 2.0 / (GPS_DELAY_WINDOW + 1.0); //TODO update rate dependent
//        const ONE_MINUS_GPS_DELAY_ALPHA:SpeedUnits = 1.0 - GPS_DELAY_ALPHA;
//        (GPS_DELAY_ALPHA * new_val) + (ONE_MINUS_GPS_DELAY_ALPHA*ema)
//    }
}

impl SensorLike for GlobalPositionSensor {

    fn new() -> Self {
        GlobalPositionSensor {
            lat: Sensulator::new(0.0, GPS_DEGREES_ABS_ERR as MeasureVal, GPS_DEGREES_REL_ERR as MeasureVal),
            lon: Sensulator::new(0.0, GPS_DEGREES_ABS_ERR as MeasureVal, GPS_DEGREES_REL_ERR as MeasureVal),
            alt: Sensulator::new(0.0, ALT_ABS_ERR, ALT_REL_ERR),

            vel_north: Sensulator::new(0.0, GPS_HVEL_ABS_ERR, GPS_HVEL_REL_ERR),
            vel_east: Sensulator::new(0.0, GPS_HVEL_ABS_ERR, GPS_HVEL_REL_ERR),
            vel_down: Sensulator::new(0.0, GPS_VVEL_ABS_ERR, GPS_VVEL_REL_ERR),

            global_pos_value: GlobalPosition { lat: 0.0, lon: 0.0, alt_wgs84: 0.0 },

            velocity_value: [0.0; 4],
            delayed_velocity: Vector3::zeros(),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self {
        self.lat.set_center_value(state.global_position.lat as MeasureVal);
        self.lon.set_center_value(state.global_position.lon as MeasureVal);
        self.alt.set_center_value(state.global_position.alt_wgs84);

        //TODO  GPS velocity should be delayed
        self.delayed_velocity = state.kinematic.inertial_velocity;

//        for i in 0..3 {
//            self.delayed_velocity[i] = Self::get_gps_vel_exp_weighted_moving_avg(
//                state.kinematic.inertial_velocity[i], self.delayed_velocity[i]);
//        }

        self.vel_north.set_center_value(self.delayed_velocity[0]);
        self.vel_east.set_center_value(self.delayed_velocity[1]);
        self.vel_down.set_center_value(self.delayed_velocity[2]);

        self.remeasure();
        self
    }

    fn remeasure(&mut self) {
        self.global_pos_value = GlobalPosition {
            lat: self.lat.measure() as LatLonUnits,
            lon: self.lon.measure() as LatLonUnits,
            alt_wgs84: self.alt.measure() as DistanceUnits
        };

        let v_north = self.vel_north.measure();
        let v_east = self.vel_east.measure();
        let v_down = self.vel_down.measure();
        let v_ground =  (v_north.exp2() + v_east.exp2()).sqrt();

        self.velocity_value = [
            v_north,
            v_east,
            v_down,
            v_ground
        ];
    }

}

pub struct AirspeedSensor {
    senso: Sensulator,
}

impl SensorLike for AirspeedSensor {
    fn new() -> Self {
        AirspeedSensor {
            senso: Sensulator::new(0.0, DIFF_PRESS_ABS_ERR, DIFF_PRESS_REL_ERR ),
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
        self.senso.set_center_value(dp);
        self.remeasure();
        self
    }

    fn remeasure(&mut self) {
        self.senso.measure();
    }
}

impl  AirspeedSensor {
    pub fn get_val(&self) -> PressureUnits {
        self.senso.peek()
    }
}


///
/// Sensor for ambient air pressure.
/// Normally corresponds to altitude, temperature, and humidity.
///
pub struct AirPressureSensor {
    senso: Sensulator,
}

impl SensorLike for AirPressureSensor {
    fn new() -> Self {
        AirPressureSensor {
            senso: Sensulator::new(0.0, AIR_PRESS_ABS_ERR, AIR_PRESS_REL_ERR),
        }
    }

    fn update(&mut self, state: &VirtualVehicleState) -> &mut Self {
        //println!("airpress: {}", state.local_air_pressure);
        self.senso.set_center_value(state.local_air_pressure);
        self.remeasure();
        self
    }

    fn remeasure(&mut self) {
        self.senso.measure();
    }

}

impl AirPressureSensor {
    pub fn get_val(&self) -> PressureUnits {
        self.senso.peek()
    }
}
