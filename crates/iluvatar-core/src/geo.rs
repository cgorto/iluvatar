use glam::{DMat3, DVec3, Vec3};

use crate::GeoPosition;
use crate::types::{GeoValidationError, MAX_ALTITUDE_M, MIN_ALTITUDE_M};

const WGS84_A: f64 = 6378137.0;
const WGS84_B: f64 = 6356752.314245;
const WGS84_E_SQ: f64 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);

impl GeoPosition {
    /// Creates a new `GeoPosition` without validation.
    ///
    /// # Warning
    /// This constructor does NOT validate the input coordinates. Invalid values
    /// (latitude outside [-90, 90], longitude outside [-180, 180], NaN, or infinity)
    /// will cause garbage output from coordinate conversion methods like `to_ecef()`.
    ///
    /// For untrusted input (e.g., GPS data from external sources), use
    /// [`new_checked`](Self::new_checked) instead.
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    /// Creates a new `GeoPosition` with full validation.
    ///
    /// Validates that:
    /// - Latitude is in [-90, 90] degrees
    /// - Longitude is in [-180, 180] degrees
    /// - Altitude is in [-500m, 100km] (Dead Sea to Kármán line)
    /// - All values are finite (not NaN or infinity)
    ///
    /// Use this constructor for untrusted input such as GPS data from external
    /// sources in a distributed system.
    ///
    /// # Errors
    /// Returns [`GeoValidationError`] if any coordinate is invalid.
    ///
    /// # Example
    /// ```
    /// use iluvatar_core::GeoPosition;
    ///
    /// // Valid position
    /// let pos = GeoPosition::new_checked(47.6062, -122.3321, 100.0).unwrap();
    ///
    /// // Invalid latitude
    /// let err = GeoPosition::new_checked(200.0, 0.0, 0.0);
    /// assert!(err.is_err());
    /// ```
    pub fn new_checked(
        latitude: f64,
        longitude: f64,
        altitude: f64,
    ) -> Result<Self, GeoValidationError> {
        // Check for NaN/infinity first (these would pass range checks)
        if !latitude.is_finite() {
            return Err(GeoValidationError::LatitudeNotFinite(latitude));
        }
        if !longitude.is_finite() {
            return Err(GeoValidationError::LongitudeNotFinite(longitude));
        }
        if !altitude.is_finite() {
            return Err(GeoValidationError::AltitudeNotFinite(altitude));
        }

        // Range checks
        if !(-90.0..=90.0).contains(&latitude) {
            return Err(GeoValidationError::LatitudeOutOfRange(latitude));
        }
        if !(-180.0..=180.0).contains(&longitude) {
            return Err(GeoValidationError::LongitudeOutOfRange(longitude));
        }
        if !(MIN_ALTITUDE_M..=MAX_ALTITUDE_M).contains(&altitude) {
            return Err(GeoValidationError::AltitudeOutOfRange(altitude));
        }

        Ok(Self {
            latitude,
            longitude,
            altitude,
        })
    }

    /// Creates a new `GeoPosition` with validation, but without altitude bounds checking.
    ///
    /// This is useful when you need to validate lat/lon but the altitude may be
    /// outside Earth's typical range (e.g., satellite positions, simulation data).
    ///
    /// Validates that:
    /// - Latitude is in [-90, 90] degrees
    /// - Longitude is in [-180, 180] degrees
    /// - All values are finite (not NaN or infinity)
    pub fn new_checked_unbounded_altitude(
        latitude: f64,
        longitude: f64,
        altitude: f64,
    ) -> Result<Self, GeoValidationError> {
        if !latitude.is_finite() {
            return Err(GeoValidationError::LatitudeNotFinite(latitude));
        }
        if !longitude.is_finite() {
            return Err(GeoValidationError::LongitudeNotFinite(longitude));
        }
        if !altitude.is_finite() {
            return Err(GeoValidationError::AltitudeNotFinite(altitude));
        }

        if !(-90.0..=90.0).contains(&latitude) {
            return Err(GeoValidationError::LatitudeOutOfRange(latitude));
        }
        if !(-180.0..=180.0).contains(&longitude) {
            return Err(GeoValidationError::LongitudeOutOfRange(longitude));
        }

        Ok(Self {
            latitude,
            longitude,
            altitude,
        })
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

        // Threshold for "near pole" - when p is this small relative to z,
        // we're close enough to the pole that standard formulas become unstable.
        // Using 1e-10 * |z| as threshold since we need sub-millimeter precision
        // at Earth scale (~6e6 m), giving us ~0.6mm threshold.
        let near_pole_threshold = 1e-10 * z.abs().max(1.0);

        let (lat, alt) = if p < near_pole_threshold {
            // At or very near a pole: x ≈ 0, y ≈ 0
            // Latitude is ±90° depending on sign of z
            let lat = std::f64::consts::FRAC_PI_2.copysign(z);

            // At the poles, the ECEF to geodetic relationship simplifies:
            // z = (N * (1 - e²) + h) * sin(lat)
            // where sin(lat) = ±1 at poles
            // So: h = |z| - N * (1 - e²)
            // where N at the pole (sin²(lat) = 1) is: a / sqrt(1 - e²)
            let n_pole = WGS84_A / (1.0 - WGS84_E_SQ).sqrt();
            let alt = z.abs() - n_pole * (1.0 - WGS84_E_SQ);

            (lat, alt)
        } else {
            // Standard iterative Bowring method for non-polar regions
            let mut lat = (z / p).atan();
            for _ in 0..10 {
                let sin_lat = lat.sin();
                let n = WGS84_A / (1.0 - WGS84_E_SQ * sin_lat * sin_lat).sqrt();
                lat = (z + WGS84_E_SQ * n * sin_lat).atan2(p);
            }

            let sin_lat = lat.sin();
            let cos_lat = lat.cos();
            let n = WGS84_A / (1.0 - WGS84_E_SQ * sin_lat * sin_lat).sqrt();

            // Choose altitude formula based on latitude to avoid division by near-zero
            // Near equator (|lat| < 45°): use alt = p / cos(lat) - N
            // Near poles (|lat| >= 45°): use alt = z / sin(lat) - N * (1 - e²)
            let alt = if cos_lat.abs() > sin_lat.abs() {
                p / cos_lat - n
            } else {
                z / sin_lat - n * (1.0 - WGS84_E_SQ)
            };

            (lat, alt)
        };

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

    /// Create a GeoPosition from local-mode (x, y, z) meter values.
    ///
    /// In local coordinate mode, the GeoPosition fields carry meters:
    ///   - `longitude` ← x (right axis)
    ///   - `latitude`  ← y (forward axis)
    ///   - `altitude`  ← z (up axis)
    pub fn from_local_xyz(x: f64, y: f64, z: f64) -> Self {
        Self {
            latitude: y,
            longitude: x,
            altitude: z,
        }
    }

    /// Convert to local meters relative to an origin (local coordinate mode).
    ///
    /// In local mode, the GeoPosition fields are interpreted as:
    ///   - `longitude` → x, `latitude` → y, `altitude` → z
    ///
    /// Returns `Vec3(x - origin_x, y - origin_y, z - origin_z)`.
    pub fn to_local_meters(&self, origin: &GeoPosition) -> Vec3 {
        Vec3::new(
            (self.longitude - origin.longitude) as f32,
            (self.latitude - origin.latitude) as f32,
            (self.altitude - origin.altitude) as f32,
        )
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
    origin_ecef: DVec3,
    rotation: DMat3,
    rotation_inv: DMat3,
}

impl LocalCoordinateSystem {
    pub fn new(origin: GeoPosition) -> Self {
        let origin_ecef = origin.to_ecef();
        let rotation = enu_rotation_matrix(&origin);
        let rotation_inv = rotation.transpose();
        Self {
            origin,
            origin_ecef,
            rotation,
            rotation_inv,
        }
    }

    pub fn geo_to_local(&self, pos: &GeoPosition) -> Vec3 {
        let point_ecef = pos.to_ecef();
        let diff = point_ecef - self.origin_ecef;
        let local = self.rotation * diff;
        Vec3::new(local.x as f32, local.y as f32, local.z as f32)
    }

    pub fn local_to_geo(&self, local: Vec3) -> GeoPosition {
        let local_dvec = DVec3::new(local.x as f64, local.y as f64, local.z as f64);
        let diff = self.rotation_inv * local_dvec;
        let point_ecef = self.origin_ecef + diff;
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
        // Precision loss expected due to f32 conversion in local ENU
        assert!((point.altitude - back.altitude).abs() < 1e-3);
    }

    #[test]
    fn test_north_pole_roundtrip() {
        // Test the exact north pole - this was a critical bug where
        // the altitude formula failed due to division by zero/near-zero
        let north_pole = GeoPosition::new(90.0, 0.0, 100.0);
        let ecef = north_pole.to_ecef();
        let back = GeoPosition::from_ecef(ecef);

        assert!(
            (back.latitude - 90.0).abs() < 1e-9,
            "latitude should be 90°, got {}",
            back.latitude
        );
        assert!(
            (back.altitude - 100.0).abs() < 1e-3,
            "altitude should be 100m, got {}",
            back.altitude
        );
    }

    #[test]
    fn test_south_pole_roundtrip() {
        // Test the south pole
        let south_pole = GeoPosition::new(-90.0, 45.0, 500.0);
        let ecef = south_pole.to_ecef();
        let back = GeoPosition::from_ecef(ecef);

        assert!(
            (back.latitude - (-90.0)).abs() < 1e-9,
            "latitude should be -90°, got {}",
            back.latitude
        );
        assert!(
            (back.altitude - 500.0).abs() < 1e-3,
            "altitude should be 500m, got {}",
            back.altitude
        );
    }

    #[test]
    fn test_exact_pole_ecef() {
        // Test with ECEF coordinates that are exactly x=0, y=0
        // This is the case that originally triggered NaN/wrong values
        let ecef = DVec3::new(0.0, 0.0, WGS84_B + 100.0);
        let pos = GeoPosition::from_ecef(ecef);

        assert!(
            (pos.latitude - 90.0).abs() < 1e-9,
            "latitude should be 90°, got {}",
            pos.latitude
        );
        assert!(
            (pos.altitude - 100.0).abs() < 1e-3,
            "altitude should be 100m, got {}",
            pos.altitude
        );
    }

    #[test]
    fn test_exact_south_pole_ecef() {
        // Exact south pole with negative z
        let ecef = DVec3::new(0.0, 0.0, -(WGS84_B + 200.0));
        let pos = GeoPosition::from_ecef(ecef);

        assert!(
            (pos.latitude - (-90.0)).abs() < 1e-9,
            "latitude should be -90°, got {}",
            pos.latitude
        );
        assert!(
            (pos.altitude - 200.0).abs() < 1e-3,
            "altitude should be 200m, got {}",
            pos.altitude
        );
    }

    #[test]
    fn test_near_pole_latitude() {
        // Test high latitudes that are near but not exactly at the pole
        // These should also work correctly with the improved altitude formula
        for lat in [89.0, 89.9, 89.99, 89.999, 89.9999] {
            let pos = GeoPosition::new(lat, 45.0, 1000.0);
            let ecef = pos.to_ecef();
            let back = GeoPosition::from_ecef(ecef);

            assert!(
                (pos.latitude - back.latitude).abs() < 1e-9,
                "lat {} roundtrip failed: got {}",
                lat,
                back.latitude
            );
            assert!(
                (pos.altitude - back.altitude).abs() < 1e-3,
                "lat {} altitude roundtrip failed: expected {}, got {}",
                lat,
                pos.altitude,
                back.altitude
            );
        }
    }

    #[test]
    fn test_equator_and_mid_latitudes() {
        // Verify we didn't break normal cases
        let test_cases = [
            (0.0, 0.0, 0.0),       // Equator, prime meridian, sea level
            (0.0, 180.0, 1000.0),  // Equator, date line
            (45.0, -90.0, 500.0),  // Mid latitude
            (-45.0, 90.0, 8848.0), // Southern mid latitude, Everest altitude
        ];

        for (lat, lon, alt) in test_cases {
            let pos = GeoPosition::new(lat, lon, alt);
            let ecef = pos.to_ecef();
            let back = GeoPosition::from_ecef(ecef);

            assert!(
                (pos.latitude - back.latitude).abs() < 1e-9,
                "roundtrip failed for ({}, {}, {}): lat {} vs {}",
                lat,
                lon,
                alt,
                pos.latitude,
                back.latitude
            );
            assert!(
                (pos.longitude - back.longitude).abs() < 1e-9,
                "roundtrip failed for ({}, {}, {}): lon {} vs {}",
                lat,
                lon,
                alt,
                pos.longitude,
                back.longitude
            );
            assert!(
                (pos.altitude - back.altitude).abs() < 1e-3,
                "roundtrip failed for ({}, {}, {}): alt {} vs {}",
                lat,
                lon,
                alt,
                pos.altitude,
                back.altitude
            );
        }
    }

    #[test]
    fn test_new_checked_valid_positions() {
        // Boundary values that should be valid
        let valid_cases = [
            (0.0, 0.0, 0.0),             // Origin
            (90.0, 0.0, 0.0),            // North pole
            (-90.0, 0.0, 0.0),           // South pole
            (0.0, 180.0, 0.0),           // Date line (positive)
            (0.0, -180.0, 0.0),          // Date line (negative)
            (47.6062, -122.3321, 100.0), // Seattle
            (-33.8688, 151.2093, 58.0),  // Sydney
            (0.0, 0.0, -500.0),          // Minimum altitude (Dead Sea depth)
            (0.0, 0.0, 100_000.0),       // Maximum altitude (Kármán line)
        ];

        for (lat, lon, alt) in valid_cases {
            let result = GeoPosition::new_checked(lat, lon, alt);
            assert!(
                result.is_ok(),
                "Expected ({}, {}, {}) to be valid, got {:?}",
                lat,
                lon,
                alt,
                result.err()
            );
            let pos = result.unwrap();
            assert_eq!(pos.latitude, lat);
            assert_eq!(pos.longitude, lon);
            assert_eq!(pos.altitude, alt);
        }
    }

    #[test]
    fn test_new_checked_latitude_out_of_range() {
        let invalid_latitudes = [90.1, -90.1, 91.0, -91.0, 180.0, 200.0, -200.0];
        for lat in invalid_latitudes {
            let result = GeoPosition::new_checked(lat, 0.0, 0.0);
            assert!(
                matches!(result, Err(GeoValidationError::LatitudeOutOfRange(v)) if v == lat),
                "Expected LatitudeOutOfRange for lat={}, got {:?}",
                lat,
                result
            );
        }
    }

    #[test]
    fn test_new_checked_longitude_out_of_range() {
        let invalid_longitudes = [180.1, -180.1, 181.0, -181.0, 360.0, -360.0];
        for lon in invalid_longitudes {
            let result = GeoPosition::new_checked(0.0, lon, 0.0);
            assert!(
                matches!(result, Err(GeoValidationError::LongitudeOutOfRange(v)) if v == lon),
                "Expected LongitudeOutOfRange for lon={}, got {:?}",
                lon,
                result
            );
        }
    }

    #[test]
    fn test_new_checked_altitude_out_of_range() {
        let invalid_altitudes = [-500.1, -1000.0, 100_000.1, 200_000.0];
        for alt in invalid_altitudes {
            let result = GeoPosition::new_checked(0.0, 0.0, alt);
            assert!(
                matches!(result, Err(GeoValidationError::AltitudeOutOfRange(v)) if v == alt),
                "Expected AltitudeOutOfRange for alt={}, got {:?}",
                alt,
                result
            );
        }
    }

    #[test]
    fn test_new_checked_nan_values() {
        // NaN latitude
        let result = GeoPosition::new_checked(f64::NAN, 0.0, 0.0);
        assert!(matches!(
            result,
            Err(GeoValidationError::LatitudeNotFinite(_))
        ));

        // NaN longitude
        let result = GeoPosition::new_checked(0.0, f64::NAN, 0.0);
        assert!(matches!(
            result,
            Err(GeoValidationError::LongitudeNotFinite(_))
        ));

        // NaN altitude
        let result = GeoPosition::new_checked(0.0, 0.0, f64::NAN);
        assert!(matches!(
            result,
            Err(GeoValidationError::AltitudeNotFinite(_))
        ));
    }

    #[test]
    fn test_new_checked_infinity_values() {
        // Positive infinity
        let result = GeoPosition::new_checked(f64::INFINITY, 0.0, 0.0);
        assert!(matches!(
            result,
            Err(GeoValidationError::LatitudeNotFinite(_))
        ));

        let result = GeoPosition::new_checked(0.0, f64::INFINITY, 0.0);
        assert!(matches!(
            result,
            Err(GeoValidationError::LongitudeNotFinite(_))
        ));

        let result = GeoPosition::new_checked(0.0, 0.0, f64::INFINITY);
        assert!(matches!(
            result,
            Err(GeoValidationError::AltitudeNotFinite(_))
        ));

        // Negative infinity
        let result = GeoPosition::new_checked(f64::NEG_INFINITY, 0.0, 0.0);
        assert!(matches!(
            result,
            Err(GeoValidationError::LatitudeNotFinite(_))
        ));

        let result = GeoPosition::new_checked(0.0, f64::NEG_INFINITY, 0.0);
        assert!(matches!(
            result,
            Err(GeoValidationError::LongitudeNotFinite(_))
        ));

        let result = GeoPosition::new_checked(0.0, 0.0, f64::NEG_INFINITY);
        assert!(matches!(
            result,
            Err(GeoValidationError::AltitudeNotFinite(_))
        ));
    }

    #[test]
    fn test_new_checked_unbounded_altitude() {
        // Should accept altitudes outside normal bounds
        let result = GeoPosition::new_checked_unbounded_altitude(0.0, 0.0, 500_000.0);
        assert!(result.is_ok());

        let result = GeoPosition::new_checked_unbounded_altitude(0.0, 0.0, -10_000.0);
        assert!(result.is_ok());

        // Should still reject NaN/infinity
        let result = GeoPosition::new_checked_unbounded_altitude(0.0, 0.0, f64::NAN);
        assert!(matches!(
            result,
            Err(GeoValidationError::AltitudeNotFinite(_))
        ));

        // Should still reject invalid lat/lon
        let result = GeoPosition::new_checked_unbounded_altitude(200.0, 0.0, 0.0);
        assert!(matches!(
            result,
            Err(GeoValidationError::LatitudeOutOfRange(_))
        ));
    }

    #[test]
    fn test_validation_error_display() {
        let err = GeoValidationError::LatitudeOutOfRange(200.0);
        assert_eq!(err.to_string(), "latitude 200 out of range [-90, 90]");

        let err = GeoValidationError::LongitudeOutOfRange(-300.0);
        assert_eq!(err.to_string(), "longitude -300 out of range [-180, 180]");

        let err = GeoValidationError::AltitudeOutOfRange(200_000.0);
        assert!(err.to_string().contains("altitude 200000 out of range"));

        let err = GeoValidationError::LatitudeNotFinite(f64::NAN);
        assert!(err.to_string().contains("not finite"));
    }
}
