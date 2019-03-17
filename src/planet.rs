
use std::f64::consts::PI;
use nalgebra::{Vector3};

use crate::physical_types::*;


/**
Planetary environment calculations
*/
pub struct Planet {

}

impl Planet {

    const STD_PRESS: f64 = 101325.0;  // static pressure at sea level (Pa)
    const STD_TEMP: f64 = 288.15;    // standard temperature at sea level (K)
    const LAPSE_RATE: f64 = -0.0065;   // standard temp altitude lapse rate (K/m)
    const MOL_MASS : f64 = 0.0289644;  // molar mass of Earth's air (kg/mol)
    const ACCEL_G : f64 = 9.80665;    // gravity acceleration (m/s^2)
    const GAS_CONSTANT_R2 : f64 = 8.31432;    // universal gas constant, R (J/(K*mol))
    const ALT_CONVERSION_EXP:f64 =
        (Self::ACCEL_G * Self::MOL_MASS) / (Self::GAS_CONSTANT_R2 * Self::LAPSE_RATE);

    /**
    Convert altitude (meters) to standard barometric pressure (Millibars)
    Note: this formula is likely only useful under 10k feet altitude
    */
    pub fn altitude_to_baro_pressure(alt: DistanceUnits) -> PressureUnits {
        let big_alt: f64 = alt.into();
        let base = Self::STD_TEMP / (Self::STD_TEMP + (Self::LAPSE_RATE * big_alt));
        let val: f64 = Self::STD_PRESS * base.powf(Self::ALT_CONVERSION_EXP); //in Pascals
        let val = val / 100.0; // Pascals -> Millibars
        (val as PressureUnits)
    }

    /**
    Given a position on the planet,
    calculate the expected magentic field.
    */
    pub fn calculate_mag_field(_pos: &GlobalPosition) -> Vector3<MagUnits> {
        //TODO calculate mag field from planetary factors
        Vector3::new(
            0.0,
            0.0,
            0.0
        )
    }


    /// Radius of the planet
    const PLANETARY_RADIUS: f64 = 6371000.0;
    const DEGREES_PER_RADIAN: f64 = 180.0 / PI;
    const RADIANS_PER_DEGREE: f64 = PI / 180.0;
    const GLOBAL_DEGREES_PER_METER: f64 = Self::PLANETARY_RADIUS * Self::DEGREES_PER_RADIAN;

    /**
    Calculate the relative distance between two global positions.
    */
    pub fn calculate_inertial_position(home: &GlobalPosition, pos: &GlobalPosition)
        -> Vector3<DistanceUnits> {
        let lat0_rad = home.lat * Self::RADIANS_PER_DEGREE;
        let lon0_rad = home.lon * Self::RADIANS_PER_DEGREE;
        let cos_lat0 = lat0_rad.cos();
        let sin_lat0 = lat0_rad.sin();

        let lat1_rad = pos.lat * Self::RADIANS_PER_DEGREE;
        let lon1_rad = pos.lon * Self::RADIANS_PER_DEGREE;
        let cos_lat1 = lat1_rad.cos();
        let sin_lat1 = lat1_rad.sin();

        let diff_lon = lon1_rad - lon0_rad;
        let cos_diff_lon = diff_lon.cos();
        let sin_diff_lon = diff_lon.sin();

        let sin_prod = sin_lat0 * sin_lat1;
        let cos_prod = cos_lat0 * cos_lat1 * cos_diff_lon;
        let c_fact = (sin_prod + cos_prod).acos();
        let k_fact;
        if 0.0 == c_fact {
            k_fact = Self::PLANETARY_RADIUS;
        }
        else {
            k_fact = Self::PLANETARY_RADIUS * (c_fact / c_fact.sin());
        }


        Vector3::new(
            (k_fact * cos_lat1 * sin_diff_lon)  as DistanceUnits ,
            (k_fact * (cos_lat0 * (sin_lat1 - sin_lat0) * cos_lat1 * cos_diff_lon)) as DistanceUnits  ,
            home.alt - pos.alt
        )
    }

    pub fn add_distance_to_position(
        pos: &GlobalPosition, dist: &Vector3<DistanceUnits>) -> GlobalPosition {
        // for latitude, degrees per meter is held constant
        let lat_offset = (dist[0]  as f64)/ Self::GLOBAL_DEGREES_PER_METER;
        // for longitude, degrees per meter varies with latitude (gets weird at the poles)
        let lon_offset = (dist[1] as f64) /
            Self::GLOBAL_DEGREES_PER_METER * ((Self::RADIANS_PER_DEGREE * pos.lat).cos());

        GlobalPosition {
            lat: pos.lat + lat_offset,
            lon: pos.lon + lon_offset,
            alt: pos.alt - dist[2]
        }
    }

}