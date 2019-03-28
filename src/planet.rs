


use nalgebra::{Rotation3, Vector3};

use crate::physical_types::*;






pub trait Planetary {

    /// Set a reference or "home" position from which relative distances can be calculated
    /// Allows preloading / precalculation of eg magnetic fields.
    fn set_reference_position(&mut self, pos: &GlobalPosition);

    fn get_reference_position(&self) -> GlobalPosition;

    /// get barometric pressure at a given altitude
    /// assumes standard atmosphere and temperature for the planet
    fn altitude_to_baro_pressure(alt: DistanceUnits) -> PressureUnits;

    /// calculate relative distance from reference position
    fn calculate_relative_distance(&self, pos: &GlobalPosition) -> Vector3<DistanceUnits>;

    /// calculate magenetic field at a given location on the planet
    fn calculate_mag_field(&self, _pos: &GlobalPosition) -> Vector3<MagUnits>;

    /// calculate a new position some distance from reference position
    fn position_at_distance(&self, dist: &Vector3<DistanceUnits>) -> GlobalPosition;

    fn default_local_environment() -> ExternalForceEnvironment;

}

/// Planetary environment calculations
pub struct PlanetEarth {

    /// The radius of this planet
    radius: HighResDistanceUnits,

    /// reference or "home" position
    ref_position: GlobalPosition,

    /// Precalculated magnetic field at the reference position
    ref_mag_field:  Vector3<MagUnits>,
}


///
/// Refer to [Ed William's Aviation Formulary](http://www.edwilliams.org/avform.htm)
/// for examples and discussion of geolocation functions.
///
///
impl Planetary for PlanetEarth {

    fn get_reference_position(&self) -> GlobalPosition {
        self.ref_position
    }

    fn set_reference_position(&mut self, pos: &GlobalPosition) {
        self.ref_position = *pos;
        self.precalc_geo_references();
        self.precalc_mag_environment();
    }

    /// Calculate the relative distance from our reference position.
    fn calculate_relative_distance(&self, pos: &GlobalPosition)  -> Vector3<DistanceUnits> {
        let (vec, _total) =
            Self::haversine_distance(self.radius,&self.ref_position, pos);
        vec
    }

    /// Calculate a position at a given distance from our reference position
    fn position_at_distance(&self, dist: &Vector3<DistanceUnits>) -> GlobalPosition {
        Self::add_offset_to_position(
            self.radius,
            &self.ref_position,
            dist
        )
    }

    /**
    Convert altitude (meters) to standard barometric pressure (Millibars)
    Note: this formula is likely only useful under 10k feet altitude
    */
    fn altitude_to_baro_pressure(alt: DistanceUnits) -> PressureUnits {
        let big_alt: f64 = alt.into();
        let base = Self::STD_TEMP / (Self::STD_TEMP + (Self::LAPSE_RATE * big_alt));
        let val: f64 = Self::STD_PRESS * base.powf(Self::ALT_CONVERSION_EXP); //in Pascals
        let val = val / 100.0; // Pascals -> Millibars
        (val as PressureUnits)
    }

    /// Given a position on the planet, calculate the expected magnetic field.
    /// - See [gufm1]( https://pdfs.semanticscholar.org/0175/7d8d373355c0a2ae5c189ea2c95ca7bc0a25.pdf)
    /// - See [NOAA calculators](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml)
    ///
    fn calculate_mag_field(&self, _pos: &GlobalPosition) -> Vector3<MagUnits> {
        //TODO calculate mag field from planetary factors
        //for now we just return the same field as the reference position
        self.ref_mag_field
    }

    fn default_local_environment() -> ExternalForceEnvironment {
        ExternalForceEnvironment {
            gravity: Vector3::new(0.0, 0.0, 9.801405599),
            wind: Vector3::zeros(),
            constraint: GeoConstraintBox {
                minimum: Vector3::new(-1000.0, -1000.0, 0.0),
                maximum: Vector3::new(1000.0, 1000.0, 1000.0)
            }
        }
    }

}


impl PlanetEarth {
    /// Radius of the planet
    const WGS84_EARTH_RADIUS: HighResDistanceUnits = 6.378137e6;

    const STD_PRESS: f64 = 101325.0;  // static pressure at sea level (Pa)
    pub const STD_TEMP: f64 = 288.15;    // standard temperature at sea level (K)
    const LAPSE_RATE: f64 = -0.0065;   // standard temp altitude lapse rate (K/m)
    const MOL_MASS : f64 = 0.0289644;  // molar mass of Earth's air (kg/mol)
    const ACCEL_G : f64 = 9.80665;    // gravity acceleration (m/s^2)
    const GAS_CONSTANT_R2 : f64 = 8.31432;    // universal gas constant, R (J/(K*mol))
    const ALT_CONVERSION_EXP:f64 =
        (Self::ACCEL_G * Self::MOL_MASS) / (Self::GAS_CONSTANT_R2 * Self::LAPSE_RATE);

    pub fn new(ref_pos: &GlobalPosition) -> PlanetEarth {
        let mut inst = PlanetEarth {
            radius: Self::WGS84_EARTH_RADIUS,
            ref_position: *ref_pos,
            ref_mag_field: Vector3::zeros()
        };

        inst.set_reference_position(ref_pos);

        inst
    }


    fn precalc_geo_references(&mut self) {
        //TODO any precalculation geolocation factors
    }

    fn precalc_mag_environment(&mut self) {

        //TODO precalc mag field from location instead of hardcoding

        // Given by NOAA for Berkeley, CA:
        // in nanotesla: nT
        // total_intensity, x,y,z:
        // 48186.3, 22515.6, 5388.1, 42260.3
        // 1 nanotesla = 1.0E-5 gauss

        self.ref_mag_field = Vector3::new(
            22515.6e-5,
            5388.1e-5,
            42260.3e-5
        );

    }


    /// Calculate a new GlobalPosition as a distance offset from a starting position.
    /// - pos : starting position
    /// - dist : distance offsets in North, East, Down (NED) frame
    /// - planet_radius: radius of the planet
    ///
    pub fn add_offset_to_position(
        planet_radius: HighResDistanceUnits,
        pos: &GlobalPosition,
        dist: &Vector3<DistanceUnits>
    ) -> GlobalPosition {

        let lat_fact = pos.lat.to_radians().cos();
        // coordinate offsets (radians)
        let d_lat = (dist[0] as HighResDistanceUnits) / planet_radius;
        let d_lon = (dist[1]as HighResDistanceUnits) / (planet_radius * lat_fact);

        GlobalPosition {
            lat: pos.lat + d_lat.to_degrees(),
            lon: pos.lon + d_lon.to_degrees(),
            alt: pos.alt - dist[2]
        }
    }

    /// Calculate the great circle distance between two points on the planet.
    pub fn haversine_distance(planet_radius: HighResDistanceUnits,
                              pos1: &GlobalPosition,
                              pos2: &GlobalPosition)
        -> (Vector3<DistanceUnits>, HighResDistanceUnits) {

        let lat1_rad:f64 = pos1.lat.to_radians();
        let lat2_rad:f64 = pos2.lat.to_radians();

        let lat_diff = (pos2.lat - pos1.lat).to_radians();
        let lat_diff_half = lat_diff/2.0;
        let lat_diff_half_sin = lat_diff_half.sin();

        let lon_diff = (pos2.lon - pos1.lon).to_radians();
        let lon_diff_half = lon_diff/2.0;
        let lon_diff_half_sin = lon_diff_half.sin();

        let a = (lat_diff_half_sin * lat_diff_half_sin) +
                lat1_rad.cos() * lat2_rad.cos() * (lon_diff_half_sin * lon_diff_half_sin);

        let c = 2.0 * (a.sqrt()).atan2((1.0 - a).sqrt());

        let total_dist = c * planet_radius;

        //TODO may also be able to calculate bearing from Haversine components we already have
        let first = lon_diff.sin() * lat2_rad.cos();
        let second = lat1_rad.cos()*lat2_rad.sin() - lat1_rad.sin()*lat2_rad.cos()*lon_diff.cos();
        let bearing = first.atan2(second);

        let x_dist = total_dist * bearing.cos();
        let y_dist = total_dist * bearing.sin();

        (
            Vector3::new(
                x_dist as DistanceUnits,
                y_dist as DistanceUnits,
                pos2.alt - pos1.alt ),
            total_dist
        )
    }

    /**
    Calculate magnetic field from the magnetic inclination and declination.
    */
    pub fn mag_field_from_declination_inclination(
        declination: AngularDegreesUnits, inclination: AngularDegreesUnits) -> Vector3<MagUnits> {
        let decl_rad = declination.to_radians() as f32;
        let incl_rad = inclination.to_radians() as f32;

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

/// Simplistic hardwall box that limits dynamic motion
pub struct GeoConstraintBox {
    pub minimum: Vector3<DistanceUnits>,
    pub maximum: Vector3<DistanceUnits>,
}

pub struct ExternalForceEnvironment {
    /// get local gravity at current position
    pub gravity: Vector3<AccelUnits>,
    /// get local wind speed at current position
    pub wind: Vector3<SpeedUnits>,
    /// get rough box that limits motion at current position, including ground/floor
    pub constraint: GeoConstraintBox,
}



#[cfg(test)]
mod tests {
    //#[macro_use]
    use assert_approx_eq::assert_approx_eq;

    use crate::planet::*;
    use crate::physical_types::{MagUnits, GlobalPosition};

    fn get_reference_position() -> GlobalPosition {
        GlobalPosition {
            lat: 37.8716, //degrees
            lon: -122.2727, //degrees
            alt: 10.0 //meters
        }
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
            PlanetEarth::mag_field_from_declination_inclination(
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
        let zero_dist = Vector3::zeros();
        let home_pos = get_reference_position();

        let planet = PlanetEarth::new(&home_pos);
        let offset_pos1 = planet.position_at_distance(&zero_dist);
        println!("offset_pos1 {:?} ", offset_pos1);
        assert_eq!(offset_pos1.alt, home_pos.alt);
        assert_approx_eq!(offset_pos1.lat, home_pos.lat, 1.0e-6);
        assert_approx_eq!(offset_pos1.lon, home_pos.lon, 1.0e-6);

        let known_dist = Vector3::new(1000.0, 1000.0, -1000.0 );
        let offset_pos2 = planet.position_at_distance(&known_dist);
        println!("offset_pos2 {:?} ", offset_pos2);
        assert_approx_eq!(offset_pos2.alt, 1010.0);
        assert_approx_eq!(offset_pos2.lat, 37.8805831528412, 1.0e-6);
        assert_approx_eq!(offset_pos2.lon, -122.26132011145224, 1.0e-6);

    }



    #[test]
    fn test_calculate_relative_distance() {
        let zero_dist = Vector3::zeros();
        let home_pos = get_reference_position();
        let clone_home = home_pos.clone();

        let planet = PlanetEarth::new(&home_pos);
        //distance from home position should essentially be zero
        let dist =
            planet.calculate_relative_distance(&clone_home);
        println!("relative_dist: {:?} ", dist);
        assert_eq!(zero_dist, dist);

        //TODO test other relative distance
    }



    #[test]
    fn test_haversine_distance() {
        let home_pos = get_reference_position();
        let clone_home = home_pos.clone();
        let next_pos = GlobalPosition {
            lat: 38.0,
            lon: -122.0,
            alt: 0.0
        };

        let (_vec, dist) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &next_pos);
        println!("dist: {}", dist);
        assert_approx_eq!(27.889e3, dist, 5.0);//27.889183319007888 km from WGS84

        let (_vec, dist) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &clone_home);
        println!("clone home dist: {}", dist);
        assert_approx_eq!(0.0, dist, 1E-12);

        //verify that a zero offset position has zero distance
        let zero_dist = Vector3::zeros();
        let offset_pos = PlanetEarth::add_offset_to_position(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &zero_dist, );
        let (_vec, dist) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &offset_pos);
        println!("zero_dist dist: {}", dist);
        assert_approx_eq!(0.0, dist, 1E-12);

        //verify that adding a known distance works
        let known_dist = Vector3::new(100.0,100.0,-100.0);
        let offset_pos = PlanetEarth::add_offset_to_position(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &known_dist);
        let (_vec, dist) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &offset_pos);
        println!("known_dist dist: {}", dist);
        assert_approx_eq!(dist, 141.421356237309505, 0.25);

        //TODO look at components
    }

}
