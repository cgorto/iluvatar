use glam::{DMat3, DVec3, Vec3};

use crate::GeoPosition;

const WGS84_A: f64 = 6378137.0;
const WGS84_B: f64 = 6356752.314245;
const WGS84_E_SQ: f64 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);

impl GeoPosition {
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    pub fn to_ecef(&self) -> DVec3 {
        let lat_rad = self.latitude.to_radians();
        let lon_rad = self.longitude.to_radians();

        let sin_lat = lat_rad.sin();
        let cos_lat = lat_rad.cos();
        let sin_lon = lon_rad.sin();
        let cos_lon = lon_rad.cos();

        let n = WGS84_A / (1.0 - WGS84_E_SQ * sin_lat * sin_lat).sqrt();

        DVec3::new(
            (n + self.altitude) * cos_lat * cos_lon,
            (n + self.altitude) * cos_lat * sin_lon,
            (n * (1.0 - WGS84_E_SQ) + self.altitude) * sin_lat,
        )
    }

    pub fn from_ecef(ecef: DVec3) -> Self {
        let x = ecef.x;
        let y = ecef.y;
        let z = ecef.z;

        let lon = y.atan2(x);
        let p = (x * x + y * y).sqrt();

        // Iterative solution for latitude
        let mut lat = (z / p).atan();
        for _ in 0..10 {
            let sin_lat = lat.sin();
            let n = WGS84_A / (1.0 - WGS84_E_SQ * sin_lat * sin_lat).sqrt();
            lat = (z + WGS84_E_SQ * n * sin_lat).atan2(p);
        }

        let sin_lat = lat.sin();
        let n = WGS84_A / (1.0 - WGS84_E_SQ * sin_lat * sin_lat).sqrt();
        let alt = p / lat.cos() - n;

        Self {
            latitude: lat.to_degrees(),
            longitude: lon.to_degrees(),
            altitude: alt,
        }
    }

    pub fn to_local_enu(&self, origin: &GeoPosition) -> Vec3 {
        let origin_ecef = origin.to_ecef();
        let point_ecef = self.to_ecef();
        let diff = point_ecef - origin_ecef;

        let rotation = enu_rotation_matrix(origin);
        let local = rotation * diff;

        Vec3::new(local.x as f32, local.y as f32, local.z as f32)
    }

    pub fn from_local_enu(local: Vec3, origin: &GeoPosition) -> Self {
        let local_dvec = DVec3::new(local.x as f64, local.y as f64, local.z as f64);
        let rotation = enu_rotation_matrix(origin).transpose();
        let diff = rotation * local_dvec;

        let origin_ecef = origin.to_ecef();
        let point_ecef = origin_ecef + diff;

        Self::from_ecef(point_ecef)
    }

    pub fn distance_to(&self, other: &GeoPosition) -> f64 {
        let ecef_self = self.to_ecef();
        let ecef_other = other.to_ecef();
        ecef_self.distance(ecef_other)
    }
}

fn enu_rotation_matrix(origin: &GeoPosition) -> DMat3 {
    let lat_rad = origin.latitude.to_radians();
    let lon_rad = origin.longitude.to_radians();

    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    // ENU rotation matrix (ECEF to ENU)
    DMat3::from_cols(
        DVec3::new(-sin_lon, cos_lon, 0.0),
        DVec3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat),
        DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat),
    )
}

#[derive(Debug, Clone)]
pub struct LocalCoordinateSystem {
    pub origin: GeoPosition,
    rotation: DMat3,
    rotation_inv: DMat3,
}

impl LocalCoordinateSystem {
    pub fn new(origin: GeoPosition) -> Self {
        let rotation = enu_rotation_matrix(&origin);
        let rotation_inv = rotation.transpose();
        Self {
            origin,
            rotation,
            rotation_inv,
        }
    }

    pub fn geo_to_local(&self, pos: &GeoPosition) -> Vec3 {
        let origin_ecef = self.origin.to_ecef();
        let point_ecef = pos.to_ecef();
        let diff = point_ecef - origin_ecef;
        let local = self.rotation * diff;
        Vec3::new(local.x as f32, local.y as f32, local.z as f32)
    }

    pub fn local_to_geo(&self, local: Vec3) -> GeoPosition {
        let local_dvec = DVec3::new(local.x as f64, local.y as f64, local.z as f64);
        let diff = self.rotation_inv * local_dvec;
        let origin_ecef = self.origin.to_ecef();
        let point_ecef = origin_ecef + diff;
        GeoPosition::from_ecef(point_ecef)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ecef_roundtrip() {
        let pos = GeoPosition::new(47.6062, -122.3321, 100.0);
        let ecef = pos.to_ecef();
        let back = GeoPosition::from_ecef(ecef);

        assert!((pos.latitude - back.latitude).abs() < 1e-9);
        assert!((pos.longitude - back.longitude).abs() < 1e-9);
        assert!((pos.altitude - back.altitude).abs() < 1e-6);
    }

    #[test]
    fn test_local_enu_roundtrip() {
        let origin = GeoPosition::new(47.6062, -122.3321, 0.0);
        let point = GeoPosition::new(47.6072, -122.3311, 50.0);

        let local = point.to_local_enu(&origin);
        let back = GeoPosition::from_local_enu(local, &origin);

        assert!((point.latitude - back.latitude).abs() < 1e-9);
        assert!((point.longitude - back.longitude).abs() < 1e-9);
        assert!((point.altitude - back.altitude).abs() < 1e-6);
    }
}
