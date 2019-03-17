
/// physical type definitions

mod raw_types;


/// For distance measurements
pub type DistanceUnits = raw_types::Meters;

/// For speed measurements
pub type SpeedUnits = raw_types::MetersPerSecond;
pub type AngularSpeedUnits = raw_types::RadiansPerSecond;
pub type GyroUnits = AngularSpeedUnits;

/// For acceleration measurements
pub type AccelUnits = raw_types::MetersPerSecondPerSecond;
pub type AngularAccelUnits = raw_types::RadiansPerSecondPerSecond;

/// For temperature measurements
pub type TemperatureUnits = raw_types::DegreesCelsius;

/// For magnetometers
pub type MagUnits = raw_types::MagUnitsGauss;

/// For latitude and longitude
pub type LatLonUnits = raw_types::WGS84Degrees;

pub type TimeBaseUnits = raw_types::TimeMicroseconds;

// multiply millibars by 100 to get Pascals
/// For barometer, airspeed
pub type PressureUnits = raw_types::PressureUnitMillibar;


#[derive(Clone, Copy)]
pub struct GlobalPosition {
    pub lat: LatLonUnits,
    pub lon: LatLonUnits,
    pub alt: DistanceUnits,
}