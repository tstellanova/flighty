

use std::f64::consts::PI;

use nalgebra::{Rotation3, Vector3};

use crate::physical_types::*;



/// Planetary environment calculations
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

    /// Calculate the relative distance from our reference position.
    //TODO switch to Haversine?
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
            k_fact = (Self::PLANETARY_RADIUS * c_fact) / c_fact.sin();
        }

        let x = k_fact * (self.cos_lat0 * sin_lat1 - self.sin_lat0 * cos_lat1 * cos_diff_lon) ;
        let y = k_fact * cos_lat1 * sin_diff_lon ;
        let z = self.ref_position.alt - pos.alt;

        Vector3::new( x as DistanceUnits, y as DistanceUnits, z as DistanceUnits )
    }

    /// Calculate a position at a given distance from our reference position
    //TODO fix
    pub fn position_at_distance(&self, pos: &Vector3<DistanceUnits>) -> GlobalPosition {
        let x_rad = (pos[0] as f64) / Self::PLANETARY_RADIUS;
        let y_rad = (pos[1] as f64) / Self::PLANETARY_RADIUS;
        let c_fact = (x_rad*x_rad + y_rad*y_rad).sqrt();

        let lat_rad;
        let lon_rad;
        if 0.0 == c_fact {
            lat_rad = self.lat0_rad;
            lon_rad = self.lon0_rad;
        }
        else {
            let sin_c = c_fact.sin();
            let cos_c = c_fact.cos();
            lat_rad = (cos_c * self.sin_lat0 + (x_rad * sin_c * self.cos_lat0) / c_fact).asin();

            let first = y_rad * sin_c;
            let second = c_fact * self.cos_lat0 * cos_c - x_rad * self.sin_lat0 * sin_c;
            lon_rad = self.ref_position.lon + first.atan2(second);
        }

        GlobalPosition {
            lat: (lat_rad * Self::DEGREES_PER_RADIAN) as LatLonUnits,
            lon: (lon_rad * Self::DEGREES_PER_RADIAN) as LatLonUnits,
            alt: self.ref_position.alt - pos[2]
        }
    }


    /// Calculate the relative distance between two global positions.
    //TODO fix
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
            k_fact = (Self::PLANETARY_RADIUS * c_fact) / c_fact.sin();
        }

        let x = k_fact * (cos_lat0 * sin_lat1 - sin_lat0 * cos_lat1 * cos_diff_lon) ;
        let y = k_fact * cos_lat1 * sin_diff_lon ;
        let z = home.alt - pos.alt;

        Vector3::new( x as DistanceUnits, y as DistanceUnits, z as DistanceUnits )
    }


    /// Calculate a new GlobalPosition as a distance offset from another position.
    //TODO fix
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

    /// returns Horizontal, Total intensity
    pub fn mag_field_intensity(field: &Vector3<MagUnits>) -> (MagUnits, MagUnits) {
        let h_intensity_sq = field[0] * field[0] + field[1] * field[1];
        let t_intensity = (field[2] * field[2] + h_intensity_sq).sqrt();
        (h_intensity_sq.sqrt(), t_intensity)
    }


}

#[cfg(test)]
mod tests {

    #[macro_use]
    use assert_approx_eq::assert_approx_eq;

    use crate::planet::*;
    use crate::physical_types::{MagUnits, GlobalPosition};

    #[test]
    fn test_precalc() {
        let zero_dist = Vector3::new(0.0, 0.0, 0.0);
        let test_pos = GlobalPosition {
            lat: 45.0, //degrees
            lon: -45.0, //degrees
            alt: 10.0 //meters
        };

        let mut planet = Planet::new(&test_pos);
        println!("lat0_rad {} lon0_rad {} cos_lat0 {} sin_lat0 {} ",
            planet.lat0_rad,
            planet.lon0_rad,
            planet.cos_lat0,
            planet.sin_lat0
        );

        assert_approx_eq!(planet.cos_lat0, 0.707106, 1.0e-6);
        assert_approx_eq!(planet.sin_lat0, 0.707106, 1.0e-6);
        assert_approx_eq!(planet.lat0_rad, (PI/4.0), 1.0e-6);
        assert_approx_eq!(planet.lon0_rad, (-PI/4.0), 1.0e-6);
    }


    // Mag sample data obtained from NOAA site:

    // Berkeley: Latitude: 37.87160 degrees, Longitude: -122.27270 degrees
    const BERKELEY_CA_MAG_DATA: [f32; 8] =
        [2019.21096, 13.45808, 61.28492, 23151.3, 48186.3, 22515.6, 5388.1, 42260.3];

    // Zurich: Latitude: 47.37690 degrees, Longitude: 8.54170 degrees
    const ZURICH_CH_MAG_DATA: [f32; 8] =
        [2019.21096, 2.62243, 63.36259, 21539.3, 48042.1, 21516.8, 985.5, 42943.0];


    ///* sample_mag_data:
    /// 0. Date in decimal years
    /// 1. Declination in decimal degrees
    /// 2. Inclination in decimal degrees
    /// 3. Horintensity in nanoTesla (nT)
    /// 4. Totalintensity in nanoTesla (nT)
    /// 5. Xcomponent in nanoTesla (nT)
    /// 6. Ycomponent in nanoTesla (nT)
    /// 7. Zcomponent in nanoTesla (nT)
    ///* returns (actual, expected) normalized magnetic field values
    fn gen_mag_comparison_pair(sample_mag_data: &[f32; 8])
        -> (Vector3<MagUnits>, Vector3<MagUnits>) {

        let declination_degs = sample_mag_data[1];
        let inclination_degs = sample_mag_data[2];

        let total_mag = sample_mag_data[4];

        //normalize the components by the provided total magnitude
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
        // verify that the output of mag_field_from_declination_inclination matches expected from NOAA

        let (actual, expected) = gen_mag_comparison_pair(&BERKELEY_CA_MAG_DATA);
        assert_approx_eq!(actual[0], expected[0], 1.0e-6);
        assert_approx_eq!(actual[1], expected[1], 1.0e-6);
        assert_approx_eq!(actual[2], expected[2], 1.0e-6);
        //TODO intensity lacks proper scale
//        let (h_intensity, t_intensity) = Planet::mag_field_intensity(&actual);
//        assert_approx_eq!(h_intensity, BERKELEY_CA_MAG_DATA[3], 1.0e-6);
//        assert_approx_eq!(t_intensity, BERKELEY_CA_MAG_DATA[4], 1.0e-6);

        let (actual, expected) = gen_mag_comparison_pair(&ZURICH_CH_MAG_DATA);
        assert_approx_eq!(actual[0], expected[0], 1.0e-6);
        assert_approx_eq!(actual[1], expected[1], 1.0e-6);
        assert_approx_eq!(actual[2], expected[2], 1.0e-6);
        //TODO intensity lacks proper scale
//        let (h_intensity, t_intensity) = Planet::mag_field_intensity(&actual);
//        assert_approx_eq!(h_intensity, ZURICH_CH_MAG_DATA[3], 1.0e-6);
//        assert_approx_eq!(t_intensity, ZURICH_CH_MAG_DATA[4], 1.0e-6);
    }


    #[test]
    fn test_position_at_distance() {
        let zero_dist = Vector3::new(0.0, 0.0, 0.0 );
        let home_pos = GlobalPosition {
            lat: 37.8716, //degrees
            lon: -122.2727, //degrees
            alt: 10.0 //meters
        };
        // 37.88
        // -122.28
        println!("home_pos {:?} ", home_pos);


        let mut planet = Planet::new(&home_pos);
        let offset_pos1 = planet.position_at_distance(&zero_dist);
        println!("offset_pos1 {:?} ", offset_pos1);
        assert_eq!(offset_pos1.alt, home_pos.alt);
        assert_approx_eq!(offset_pos1.lat, home_pos.lat, 1.0e-6);
        assert_approx_eq!(offset_pos1.lon, home_pos.lon, 1.0e-6);

        let known_dist = Vector3::new(1000.0, 1000.0, -1000.0 );
        let offset_pos2 = planet.position_at_distance(&known_dist);
        println!("offset_pos2 {:?} ", offset_pos2);
        assert_eq!(offset_pos2.alt, 1010.0);
        assert_approx_eq!(offset_pos2.lat, 37.8805926, 1.0e-6);
        assert_approx_eq!(offset_pos2.lon, 37.8805, 1.0e-6);

    }

    #[test]
    fn test_local_to_global_conversions() {
        let zero_dist = Vector3::new(0.0, 0.0, 0.0 );
        let home_pos = GlobalPosition {
            lat: 37.8716, //degrees
            lon: -122.2727, //degrees
            alt: 10.0 //meters
        };
        let clone_home = home_pos.clone();

        let dist =
            Planet::calculate_inertial_distance(&home_pos, &clone_home);
        println!("inertial_dist: {:?} ", dist);
        assert_eq!(zero_dist, dist);

        let mut planet = Planet::new(&home_pos);
        //distance from home position should essentially be zero
        let dist =
            planet.calculate_relative_distance(&clone_home);
        println!("relative_dist: {:?} ", dist);
        assert_eq!(zero_dist, dist);


        //TODO:
        let known_dist = Vector3::new(100.0,100.0,100.0);
        let offset_pos =
            Planet::add_distance_to_position(&clone_home, &known_dist);
        println!("offset_pos: {:?}", offset_pos);

        let dist =
            Planet::calculate_inertial_distance(&home_pos, &offset_pos);
        println!("inertial_dist: {:?} ", dist);

        let dist =
            planet.calculate_relative_distance(&offset_pos);
        println!("relative_dist: {:?} ", dist);

        //TODO fix distance calculations
        //inertial_dist:  [0.018981751, 0.014983975, 100.0]
        assert_eq!(dist, known_dist);

    }

}