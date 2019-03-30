/// Raw physical type definitions


pub type Meters = f32;
pub type MetersPerSecond = f32;
pub type MetersPerSecondPerSecond = f32;
pub type Radians = f32;
pub type RadiansPerSecond = f32;
pub type RadiansPerSecondPerSecond = f32;
pub type AngleDegrees = f32;

pub type HighResolutionMeters = f64;

/// For temperature measurements
pub type DegreesKelvin = f32;

/// For magnetometers
pub type MagUnitsGauss = f32;

/// For latitude and longitude
pub type WGS84Degrees = f64;

/// integer microseconds
pub type TimeWholeMicroseconds = u64;

/// fractional seconds
pub type TimeSecondsF32 = f32;

/// For airspeed, barometer etc
pub type PressureUnitsMillibar = f32;
// multiply millibars by 100 to get Pascals
//pub type PressureUnitsPascal = f32;


pub type Newton = f32;
pub type NewtonMeter = f32;

pub type Kilogram = f32;
pub type KilogramMeterSquared = f32;
