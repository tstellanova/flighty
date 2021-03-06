/**
Copyright (c) 2019 Todd Stellanova
LICENSE: See LICENSE file
*/



use nalgebra::{Rotation3, Vector3};
use crate::physical_types::*;
//use serde::{Deserialize};
use rstar::{ RTree};


#[path = "../build/geomag_record.rs"]
mod geomag_record ;

use geomag_record::GeomagRecord;

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
    fn local_environment(&self) ->ExternalForceEnvironment;

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
    fn calculate_mag_field(&self, _pos: &GlobalPosition) -> Vector3<MagUnits> {
        //for now we just return the field precalculated from the reference position
        self.ref_mag_field
    }

    fn default_local_environment() -> ExternalForceEnvironment {
        ExternalForceEnvironment {
            gravity: Vector3::new(0.0, 0.0, Self::STD_GRAVITY_ACCEL),
            wind: Vector3::zeros(),
            constraint: GeoConstraintBox {
                minimum: Vector3::new(-1000.0, -1000.0, Self::MIN_SIM_ALTITUDE),
                maximum: Vector3::new(1000.0, 1000.0, Self::MAX_SIM_ALTITUDE)
            }
        }
    }

    fn local_environment(&self) -> ExternalForceEnvironment {
        let local_floor =  - self.ref_position.alt_wgs84;
        ExternalForceEnvironment {
            gravity: Vector3::new(0.0, 0.0, Self::STD_GRAVITY_ACCEL),
            wind: Vector3::zeros(),
            constraint: GeoConstraintBox {
                minimum: Vector3::new(-1000.0, -1000.0, local_floor),
                maximum: Vector3::new(1000.0, 1000.0, Self::MAX_SIM_ALTITUDE)
            }
        }
    }

}


/// The magnetic field grid
static EARTH_MAG_GRID: &str =  include_str!(concat!(env!("OUT_DIR"), "/earth_mag.json"));

lazy_static! {
    static ref EARTH_MAG_TABLE : RTree<GeomagRecord> = serde_json::from_str(EARTH_MAG_GRID).unwrap();
}

impl PlanetEarth {


    /// Radius of the planet
    const WGS84_EARTH_RADIUS: HighResDistanceUnits = 6.378137e6;
    pub const STD_GRAVITY_ACCEL: AccelUnits = 9.80665;

    /// max simulated altitude in NED
    const MAX_SIM_ALTITUDE: DistanceUnits = 10000.0;
    /// minimum simulated altitude in NED
    const MIN_SIM_ALTITUDE: DistanceUnits = 0.0;

    // See code provided by NOAA in the public domain:
    // https://www.ngdc.noaa.gov/geomag/WMM/wmm_ldownload.shtml
    // -180...180 degrees longitude
    // -90..90 degrees latitude
    // 360 * 180 = 64800 points of mag field
    // mag field grid generated from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfgrid
//    const MAG_FIELD_GRID: [[f32; 3]; 64800] = [
//        []
//    ;]

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


    ///
    /// Calculate the local magnetic field environment given the current reference position.
    /// This gives us a chance to precalculate and cache the estimated field.
    ///
    fn precalc_mag_environment(&mut self) {
        let search_point = [self.ref_position.lat, self.ref_position.lon];

        let parent_cell = EARTH_MAG_TABLE.nearest_neighbor(&search_point).unwrap();
        //println!("closest parent_cell:\n{:?}\n{:?}", parent_cell, search_point);

        // NOAA WMM2015 data is given in nanoteslas,
        // 1 nanotesla = 1.0E-5 gauss
        self.ref_mag_field[0] = parent_cell.mag_x * 1E-5;
        self.ref_mag_field[1] = parent_cell.mag_y * 1E-5;
        self.ref_mag_field[2] = parent_cell.mag_z * 1E-5;

        //println!("ref_mag_field: {:?}", self.ref_mag_field);

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

        // alt_wgs84 is in meters above WGS84 ellipsoid;
        // dist is in intertial frame (NED)
        let d_alt_wgs84 = -dist[2];

        GlobalPosition {
            lat: pos.lat + d_lat.to_degrees(),
            lon: pos.lon + d_lon.to_degrees(),
            alt_wgs84: pos.alt_wgs84 + d_alt_wgs84
        }
    }

    /// Calculate the great circle distance between two points on the planet.
    /// returns the inertial frame distance vector as well as the "ground distance"
    ///
    pub fn haversine_distance(planet_radius: HighResDistanceUnits,
                              pos1: &GlobalPosition,
                              pos2: &GlobalPosition)
        -> (/* inertial_distance */ Vector3<DistanceUnits>, /*ground_distance */ HighResDistanceUnits) {

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
        // alt_wgs84 is in meters above WGS84 ellipsoid; inertial frame is in NED
        let z_dist = pos2.alt_wgs84 - pos1.alt_wgs84;
        (
            Vector3::new(
                x_dist as DistanceUnits,
                y_dist as DistanceUnits,
                z_dist as DistanceUnits),
            total_dist
        )
    }

    ///
    /// Calculate an approximate magnetic field from the magnetic inclination and declination.
    /// Declination is the angle between measured magnetic north and true north.
    /// Inclination or dip is a measure of how much magnetic fields line rise above or
    /// dive below the horizontal ground plane of the compass.
    ///
    pub fn mag_field_from_declination_inclination(
        declination: AngularDegreesUnits, inclination: AngularDegreesUnits) -> Vector3<MagUnits> {
        let decl_rad = declination.to_radians() as f32;
        let incl_rad = inclination.to_radians() as f32;

        let incl_field = Vector3::new(  incl_rad.cos(), 0.0,  incl_rad.sin());
        let rotator = Rotation3::from_axis_angle(&Vector3::z_axis(), decl_rad);
        let net_field = rotator * incl_field;
        net_field
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
    use assert_approx_eq::assert_approx_eq;

    use crate::planet::*;
    use crate::physical_types::{MagUnits, GlobalPosition};

    // Mag sample data obtained from NOAA site:

    /// Berkeley: Latitude: 37.87160 degrees, Longitude: -122.27270 degrees
    const BERKELEY_LATITUDE: f64 = 37.87160;
    const BERKELEY_LONGITUDE: f64 = -122.27270;
    const BERKELEY_CA_MAG_DATA: [f32; 8] =
        [2019.21096, 13.45808, 61.28492, 23151.3, 48186.3, 22515.6, 5388.1, 42260.3];

    /// Zurich: Latitude: 47.37690 degrees, Longitude: 8.54170 degrees
    const ZURICH_CH_MAG_DATA: [f32; 8] =
        [2019.21096, 2.62243, 63.36259, 21539.3, 48042.1, 21516.8, 985.5, 42943.0];

    /// Hilo, Hawaii: Latitude: 19.729722 degrees, Longitude: -155.090000
    const HILO_HI_MAG_DATA: [f32; 8] =
        [2019.27123, 9.47178, 36.47267, 27770.7, 34534.7, 27392.1, 4570.0, 20528.8];

    const ALL_SAMPLE_CITIES: [[f32;8] ; 3] = [
        BERKELEY_CA_MAG_DATA,
        ZURICH_CH_MAG_DATA,
        HILO_HI_MAG_DATA
    ];

    fn get_test_reference_position() -> GlobalPosition {
        GlobalPosition {
            lat:BERKELEY_LATITUDE,
            lon:BERKELEY_LONGITUDE,
            alt_wgs84: 10.0
        }
    }

    ///* sample_mag_data:
    /// 0. Date in decimal years
    /// 1. Declination in decimal degrees
    /// 2. Inclination in decimal degrees
    /// 3. Horintensity in nanoTesla (nT)
    /// 4. Totalintensity in nanoTesla (nT)
    /// 5. Xcomponent in nanoTesla (nT)
    /// 6. Ycomponent in nanoTesla (nT)
    /// 7. Zcomponent in nanoTesla (nT)
    ///* returns (estimated, expected) normalized magnetic field values
    fn gen_mag_comparison_pair(sample_mag_data: &[f32; 8])
        -> (Vector3<MagUnits>, Vector3<MagUnits>) {

        let decl_degs = sample_mag_data[1];
        let incl_degs = sample_mag_data[2];
        let total_mag = sample_mag_data[4];

        //normalize the components by the provided total magnitude
        let expected = Vector3::new(
            sample_mag_data[5] / total_mag,
            sample_mag_data[6] / total_mag,
            sample_mag_data[7] / total_mag,
            );

        let estimated =
            PlanetEarth::mag_field_from_declination_inclination(decl_degs, incl_degs);

        return (estimated, expected);
    }

    #[test]
    fn test_mag_from_declination() {
        // verify that the output of mag_field_from_declination_inclination matches expected from NOAA

        for i in 0..ALL_SAMPLE_CITIES.len() {
            let (estimated, expected) = gen_mag_comparison_pair(&ALL_SAMPLE_CITIES[i]);
            assert_approx_eq!(estimated[0], expected[0], 2.0e-6);
            assert_approx_eq!(estimated[1], expected[1], 2.0e-6);
            assert_approx_eq!(estimated[2], expected[2], 2.0e-6);
        }
    }


    #[test]
    fn test_earth_mag_cell_lookup() {
        // for comparison:
        //40.0,     -122.5,         21739.344   5406.754,   43971.156
        //37.87160, -122.27270,     22515.6,    5388.1,     42260.3
        //35.00,    -125.00,	    23578.587,  5527.177,   39108.935
        let home_pos = get_test_reference_position();

        let needle = [home_pos.lat, home_pos.lon];
        let parent_cell = EARTH_MAG_TABLE.nearest_neighbor(&needle).unwrap();
        println!("found parent_cell: {:?}", parent_cell);
        assert_approx_eq!(parent_cell.lat_deg, home_pos.lat, geomag_record::GEOMAG_CELL_LAT_RADIUS);
        assert_approx_eq!(parent_cell.lon_deg, home_pos.lon, geomag_record::GEOMAG_CELL_LON_RADIUS);

        let planet = PlanetEarth::new(&home_pos);
        let mfield = planet.calculate_mag_field(&home_pos);

        //verify geomag values estimated from grid are within 5% of expected values
        assert_approx_eq!(mfield[0], BERKELEY_CA_MAG_DATA[5], (BERKELEY_CA_MAG_DATA[5]*0.05).abs());
        assert_approx_eq!(mfield[1], BERKELEY_CA_MAG_DATA[6], (BERKELEY_CA_MAG_DATA[6]*0.05).abs());
        assert_approx_eq!(mfield[2], BERKELEY_CA_MAG_DATA[7], (BERKELEY_CA_MAG_DATA[7]*0.05).abs());
    }

    #[test]
    fn test_position_at_distance() {
        let zero_dist = Vector3::zeros();
        let home_pos = get_test_reference_position();

        let planet = PlanetEarth::new(&home_pos);
        let offset_pos1 = planet.position_at_distance(&zero_dist);
        println!("offset_pos1 {:?} ", offset_pos1);
        assert_eq!(offset_pos1.alt_wgs84, home_pos.alt_wgs84);
        assert_approx_eq!(offset_pos1.lat, home_pos.lat, 1.0e-6);
        assert_approx_eq!(offset_pos1.lon, home_pos.lon, 1.0e-6);

        let known_dist = Vector3::new(1000.0, 1000.0, 1000.0 );
        let offset_pos2 = planet.position_at_distance(&known_dist);
        println!("offset_pos2 {:?} ", offset_pos2);

        //WGS84 distance is meters above ellipsoid: local distance is in NED
        assert_approx_eq!(offset_pos2.alt_wgs84, home_pos.alt_wgs84 - known_dist[2]);
        assert_approx_eq!(offset_pos2.lat, 37.8805831528412, 1.0e-6);
        assert_approx_eq!(offset_pos2.lon, -122.26132011145224, 1.0e-6);
    }



    #[test]
    fn test_calculate_relative_distance() {
        let zero_dist = Vector3::zeros();
        let home_pos = get_test_reference_position();
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
        let home_pos = get_test_reference_position();
        let clone_home = home_pos.clone();
        let next_pos = GlobalPosition {
            lat: 38.0,
            lon: -122.0,
            alt_wgs84: 20.0
        };

        let (inertial_dist1, surface_dist1) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &next_pos);
        println!("dist: {}", surface_dist1);
        assert_approx_eq!(27.889e3, surface_dist1,  5.0);//27.889183319007888 km from WGS84
        // remember that alt_wgs84 has opposite sense from inertial NED "down"
        assert_approx_eq!(10.0, inertial_dist1[2], 1E-9);//check altitude

        let (_vec, surface_dist2) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &clone_home);
        println!("clone home dist: {}", surface_dist2);
        assert_approx_eq!(0.0, surface_dist2, 1E-12);

        //verify that a zero offset position has zero distance
        let zero_dist = Vector3::zeros();
        let offset_pos = PlanetEarth::add_offset_to_position(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &zero_dist, );
        let (_vec, surface_dist3) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &offset_pos);
        println!("zero_dist dist: {}", surface_dist3);
        assert_approx_eq!(0.0, surface_dist3, 1E-12);

        //verify that adding a known distance works
        let known_dist = Vector3::new(1000.0,1000.0, 100.0);
        let offset_pos = PlanetEarth::add_offset_to_position(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &known_dist);
        let (inertial_dist4, surface_dist4) = PlanetEarth::haversine_distance(
            PlanetEarth::WGS84_EARTH_RADIUS, &home_pos, &offset_pos);

        assert_approx_eq!(inertial_dist4[0], known_dist[0], 0.125);
        assert_approx_eq!(inertial_dist4[1], known_dist[1], 0.125);
        assert_approx_eq!(inertial_dist4[2], -known_dist[2], 1E-9);

        println!(" surface_dist4: {}", surface_dist4);
        assert_approx_eq!(surface_dist4, 1414.1704497959352, 0.25);


        //TODO look at components
    }

}
