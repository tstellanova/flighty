

use std::f64::consts::PI;

use nalgebra::{Rotation3, Vector3};

use crate::physical_types::*;


/**
Planetary environment calculations
*/
pub struct Planet {
    ref_position: GlobalPosition,
    /// -- Precalculated global positioning values ---
    lat0_rad: f64,  //home.lat * Self::RADIANS_PER_DEGREE;
    lon0_rad: f64, // = home.lon * Self::RADIANS_PER_DEGREE;
    cos_lat0: f64, // = lat0_rad.cos();
    sin_lat0: f64, // = lat0_rad.sin();
}

impl Planet {

    pub fn new(ref_pos: &GlobalPosition) -> Planet {
        let mut inst = Planet {
            ref_position: *ref_pos,
            lat0_rad: 0.0,
            lon0_rad: 0.0,
            cos_lat0: 0.0,
            sin_lat0: 0.0
        };
        inst.set_reference_position(ref_pos);

        inst
    }

    pub fn set_reference_position(&mut self, pos: &GlobalPosition) {
        self.ref_position = *pos;
        self.precalc_geo_references();
        //TODO precalc mag declination etc
    }

    fn precalc_geo_references(&mut self) {
        self.lat0_rad = self.ref_position.lat * Self::RADIANS_PER_DEGREE;
        self.lon0_rad = self.ref_position.lon * Self::RADIANS_PER_DEGREE;
        self.cos_lat0 = self.lat0_rad.cos();
        self.sin_lat0 = self.lat0_rad.sin();
    }

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


    /// Radius of the planet
    const PLANETARY_RADIUS: f64 = 6371000.0;
    const DEGREES_PER_RADIAN: f64 = 180.0 / PI;
    const RADIANS_PER_DEGREE: f64 = PI / 180.0;
    const GLOBAL_DEGREES_PER_METER: f64 = Self::PLANETARY_RADIUS * Self::DEGREES_PER_RADIAN;

    /**
    Calculate the relative distance from our reference position.
    */
    pub fn calculate_relative_distance(&self, pos: &GlobalPosition)  -> Vector3<DistanceUnits> {
        let lat1_rad = pos.lat * Self::RADIANS_PER_DEGREE;
        let lon1_rad = pos.lon * Self::RADIANS_PER_DEGREE;
        let cos_lat1 = lat1_rad.cos();
        let sin_lat1 = lat1_rad.sin();

        let diff_lon = lon1_rad - self.lon0_rad;
        let cos_diff_lon = diff_lon.cos();
        let sin_diff_lon = diff_lon.sin();

        let sin_prod = self.sin_lat0 * sin_lat1;
        let cos_prod = self.cos_lat0 * cos_lat1 * cos_diff_lon;
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
            (k_fact * (self.cos_lat0 * (sin_lat1 - self.sin_lat0) * cos_lat1 * cos_diff_lon)) as DistanceUnits  ,
            self.ref_position.alt - pos.alt
        )
    }

    /**
    Calculate the relative distance between two global positions.
    */
    pub fn calculate_inertial_distance(home: &GlobalPosition, pos: &GlobalPosition)
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

    /**
    Calculate a new GlobalPosition as a distance offset from another position.
    */
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

    /**
    Given a position on the planet,
    calculate the expected magnetic field.
    @see gufm1: https://pdfs.semanticscholar.org/0175/7d8d373355c0a2ae5c189ea2c95ca7bc0a25.pdf
    @see NOAA calculators: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

    */
    pub fn calculate_mag_field(_pos: &GlobalPosition) -> Vector3<MagUnits> {
        //TODO calculate mag field from planetary factors
        Vector3::new(
            0.0,
            0.0,
            0.0
        )
    }


    /**
    Calculate magnetic field from the magnetic inclination and declination.
    */
    pub fn mag_field_from_declination_inclination(
        declination: AngularDegreesUnits, inclination: AngularDegreesUnits) -> Vector3<MagUnits> {
        let decl_rad = ((declination as f64) * Self::RADIANS_PER_DEGREE) as f32;
        let incl_rad = ((inclination as f64) * Self::RADIANS_PER_DEGREE) as f32;

        let incl_field = Vector3::new(
            incl_rad.cos(),
            0.0,
            incl_rad.sin());

        let rotator = Rotation3::from_axis_angle(&Vector3::z_axis(), decl_rad);
        let net_field = rotator * incl_field;
        net_field
    }




}

#[cfg(test)]
mod tests {

    #[macro_use]
    use approx::*;

    use crate::planet::*;
    use crate::physical_types::{GlobalPosition};
    use crate::physical_types::*;

    #[test]
    fn test_local_to_global_conversions() {
        let home_pos = GlobalPosition {
            lat: 37.8716, //degrees
            lon: -122.2727, //degrees
            alt: 10.0 //meters
        };
        let clone_home = home_pos.clone();

        let mut planet = Planet::new(&home_pos);
        //distance from home position should essentially be zero
        let dist =
            planet.calculate_relative_distance(&clone_home);
        println!("distance: {:?} ", dist);
        assert_eq!(dist[0], 0.0);
        assert_eq!(dist[1], 0.0);
        assert_eq!(dist[2], 0.0);

        //TODO:
//        let offset_pos =
//            Planet::add_distance_to_position(&clone_home, &Vector3::new(1.0,1.0,1.0));
//        println!("offset_pos: {:?}", offset_pos);
//
//        let dist =
//            planet.calculate_relative_distance(&offset_pos);
//        println!("distance: {:?} ", dist);
//
//        assert_eq!(dist[0], 1.0);
//        assert_eq!(dist[1], 1.0);
//        assert_eq!(dist[2], 1.0);

    }


    // Mag sample data obtained from NOAA site:

    // Berkeley: Latitude: 37.87160 degrees, Longitude: -122.27270 degrees
    const BERKELEY_CA_MAG_DATA: [f32; 8] =
        [2019.21096, 13.45808, 61.28492, 23151.3, 48186.3, 22515.6, 5388.1, 42260.3];

    // Zurich: Latitude: 47.37690 degrees, Longitude: 8.54170 degrees
    const ZURICH_CH_MAG_DATA: [f32; 8] =
        [2019.21096,2.62243,63.36259,21539.3,48042.1,21516.8,985.5,42943.0];


    //(0) Date in decimal years
    //(1) Declination in decimal degrees
    //(2) Inclination in decimal degrees
    //(3) Horintensity in nanoTesla (nT)
    //(4) Totalintensity in nanoTesla (nT)
    //(5) Xcomponent in nanoTesla (nT)
    //(6) Ycomponent in nanoTesla (nT)
    //(7) Zcomponent in nanoTesla (nT)
    fn check_mag_matches(sample_mag_data: &[f32; 8]) -> (Vector3<MagUnits>, Vector3<MagUnits>) {

        let declination_degs = sample_mag_data[1];
        let inclination_degs = sample_mag_data[2];

        let total_mag = sample_mag_data[4];

        let expected = Vector3::new(
            sample_mag_data[5] / total_mag,
            sample_mag_data[6] / total_mag,
            sample_mag_data[7] / total_mag,
            );

        let actual =
            Planet::mag_field_from_declination_inclination(
            declination_degs, inclination_degs);

        return (actual, expected);
    }

    #[test]
    fn test_mag_from_declination() {
        let berk_tuple = check_mag_matches(&BERKELEY_CA_MAG_DATA);
        relative_eq!(berk_tuple.0, berk_tuple.1, epsilon = 1.0e-6);

        let zurich_pair = check_mag_matches(&ZURICH_CH_MAG_DATA);
        relative_eq!(zurich_pair.0, zurich_pair.1, epsilon = 1.0e-6);

    }
}