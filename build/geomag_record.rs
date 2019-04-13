

use serde::{Serialize, Deserialize};


pub const GEOMAG_CELL_LAT_RADIUS: f64 = 2.5;
pub const GEOMAG_CELL_LON_RADIUS: f64 = 1.25;
const GEOMAG_CELL_TOTAL_RADIUS_2:f64 =
    GEOMAG_CELL_LAT_RADIUS * GEOMAG_CELL_LAT_RADIUS + GEOMAG_CELL_LON_RADIUS * GEOMAG_CELL_LON_RADIUS;

/// Stores the magnetic field for a particular geo grid location
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct GeomagRecord {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub mag_x: f32,
    pub mag_y: f32,
    pub mag_z: f32,
}


impl rstar::RTreeObject for GeomagRecord {
    type Envelope = rstar::AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_point([self.lat_deg, self.lon_deg] )
    }
}

impl rstar::PointDistance for GeomagRecord {

    /// Returns the squared distance from this cell's center to another point
    fn distance_2(&self, point: &[f64; 2]) -> f64  {
        //this is a crude innacurate distance //TODO use Haversine instead?
        let d_lat = self.lat_deg - point[0];
        let d_lon = self.lon_deg - point[1];
        let d_lon_adj = d_lon * self.lat_deg.cos();

        //return squared distance
        d_lat.powi(2) + d_lon_adj.powi(2)
    }

    /// Use bounding box to determine whether another point falls inside this cell
    fn contains_point(&self, point:&[f64; 2]) -> bool  {
        let dist = self.distance_2(&point);
        dist <= GEOMAG_CELL_TOTAL_RADIUS_2
    }
}




