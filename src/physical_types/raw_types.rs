/// Raw physical type definitions


/// private types
pub type Meters = f32;
pub type MetersPerSecond = f32;
pub type MetersPerSecondPerSecond = f32;
pub type Radians = f32;
pub type RadiansPerSecond = f32;
pub type RadiansPerSecondPerSecond = f32;
pub type AngleDegrees = f32;

/// For temperature measurements
pub type DegreesCelsius = f32;

/// For magnetometers
pub type MagUnitsGauss = f32;

/// For latitude and longitude
pub type WGS84Degrees = f64;

pub type TimeMicroseconds = u64;

/// For airspeed, barometer etc
pub type PressureUnitsMillibar = f32;
// multiply millibars by 100 to get Pascals
//pub type PressureUnitsPascal = f32;
