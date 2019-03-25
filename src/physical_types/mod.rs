
/// physical type definitions

mod raw_types;


/// For distance measurements
pub type DistanceUnits = raw_types::Meters;
pub type HighResDistanceUnits = raw_types::HighResolutionMeters;

/// For angular position measurements
pub type AngularPosUnits = raw_types::Radians;
pub type AngularDegreesUnits = raw_types::AngleDegrees;

/// For speed measurements
pub type SpeedUnits = raw_types::MetersPerSecond;
pub type AngularSpeedUnits = raw_types::RadiansPerSecond;
pub type GyroUnits = AngularSpeedUnits;

/// For acceleration measurements
pub type AccelUnits = raw_types::MetersPerSecondPerSecond;
pub type AngularAccelUnits = raw_types::RadiansPerSecondPerSecond;

/// For temperature measurements
pub type TemperatureUnits = raw_types::DegreesKelvin;

/// For magnetometers
pub type MagUnits = raw_types::MagUnitsGauss;

/// For latitude and longitude
pub type LatLonUnits = raw_types::WGS84Degrees;

/// monotonicaly increasing time base units
pub type TimeBaseUnits = raw_types::TimeWholeMicroseconds;

/// time interval between events
pub type TimeIntervalUnits = raw_types::TimeSecondsF32;

/// multiplier for converting TimeBase delta to TimeInterval
pub const TIME_BASE_DELTA_TO_INTERVAL: TimeIntervalUnits = 1E-6;

/// For barometer, airspeed
pub type PressureUnits = raw_types::PressureUnitsMillibar;



#[derive(Clone, Copy, Debug)]
pub struct GlobalPosition {
    pub lat: LatLonUnits,
    pub lon: LatLonUnits,
    pub alt: DistanceUnits,
}