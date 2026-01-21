//! GPS + IMU sensor fusion for camera localization.
//!
//! This module provides the [`Localizer`] trait and implementations for determining
//! camera pose (position + orientation) in real-world coordinates.
//!
//! # Architecture
//!
//! ```text
//! GPS Reader ──┬──> GpsImuLocalizer ──> CameraPose
//! IMU Reader ──┘         │
//!                        ├── Position from GPS (WGS84)
//!                        ├── Orientation from IMU (quaternion)
//!                        └── Dead reckoning when GPS unavailable
//! ```
//!
//! # Coordinate Systems
//!
//! - **GPS**: WGS84 latitude/longitude/altitude
//! - **IMU**: Orientation as quaternion, typically in NED or sensor-local frame
//! - **Output**: Position in WGS84, orientation in ENU-aligned frame for raymarching
//!
//! The raymarcher expects orientation in a Bevy-compatible frame where:
//! - Camera looks down -Z (Bevy convention)
//! - Y is up (Bevy convention, converted to ENU Z-up internally)

use glam::Quat;
use iluvatar_core::{CameraPose, GeoPosition, LocalizationStatus, PoseUncertainty};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use thiserror::Error;
use tracing::{debug, info, warn};

fn now_micros() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}

#[derive(Debug, Error)]
pub enum LocalizationError {
    #[error("GPS device not available")]
    GpsUnavailable,
    #[error("IMU device not available")]
    ImuUnavailable,
    #[error("No valid fix")]
    NoFix,
    #[error("Dead reckoning timeout exceeded")]
    DeadReckoningTimeout,
    #[error("GPS connection error: {0}")]
    GpsConnection(String),
    #[error("GPS protocol error: {0}")]
    GpsProtocol(String),
}

/// GPS fallback behavior when signal is degraded
#[derive(Debug, Clone, Copy, Default)]
pub enum GpsFallbackMode {
    /// Continue with last known position, tracking uncertainty growth
    #[default]
    DeadReckoning,
    /// Return error immediately when GPS is unavailable
    Strict,
}

// ============================================================================
// GPS Reader Abstraction
// ============================================================================

/// A GPS position fix with quality information
#[derive(Debug, Clone)]
pub struct GpsFix {
    pub position: GeoPosition,
    /// Horizontal accuracy estimate in meters (95% confidence)
    pub horizontal_accuracy: f32,
    /// Vertical accuracy estimate in meters (95% confidence)
    pub vertical_accuracy: f32,
    /// Fix quality: 2D (lat/lon only) or 3D (with altitude)
    pub fix_3d: bool,
    /// Course over ground in degrees from true north (if moving)
    pub course: Option<f32>,
    /// Speed over ground in m/s (if moving)
    pub speed: Option<f32>,
}

/// Abstract GPS data source
pub trait GpsReader: Send {
    /// Attempt to read the current GPS fix.
    /// Returns None if no fix is available (e.g., acquiring satellites).
    fn read_fix(&mut self) -> Result<Option<GpsFix>, LocalizationError>;
}

/// Mock GPS reader for testing - returns a fixed position
pub struct MockGpsReader {
    position: GeoPosition,
    fail_after: Option<Instant>,
}

impl MockGpsReader {
    pub fn new(position: GeoPosition) -> Self {
        Self {
            position,
            fail_after: None,
        }
    }

    /// Create a mock that returns fixes until the given instant, then fails
    pub fn with_failure_at(position: GeoPosition, fail_at: Instant) -> Self {
        Self {
            position,
            fail_after: Some(fail_at),
        }
    }
}

impl GpsReader for MockGpsReader {
    fn read_fix(&mut self) -> Result<Option<GpsFix>, LocalizationError> {
        if let Some(fail_at) = self.fail_after {
            if Instant::now() >= fail_at {
                return Ok(None);
            }
        }
        Ok(Some(GpsFix {
            position: self.position,
            horizontal_accuracy: 2.5, // Typical GPS accuracy
            vertical_accuracy: 5.0,
            fix_3d: true,
            course: None,
            speed: None,
        }))
    }
}

// ============================================================================
// IMU Reader Abstraction
// ============================================================================

/// Orientation reading from an IMU
#[derive(Debug, Clone, Copy)]
pub struct ImuReading {
    /// Orientation as a quaternion
    pub orientation: Quat,
    /// Angular velocity in rad/s (if available, for dead reckoning)
    pub angular_velocity: Option<glam::Vec3>,
}

/// Abstract IMU data source
pub trait ImuReader: Send {
    /// Read the current orientation from the IMU.
    fn read_orientation(&mut self) -> Result<ImuReading, LocalizationError>;
}

/// Mock IMU that returns a fixed orientation
pub struct MockImuReader {
    orientation: Quat,
}

impl MockImuReader {
    pub fn new(orientation: Quat) -> Self {
        Self { orientation }
    }

    /// Create a mock IMU with identity orientation (camera level, pointing north)
    pub fn level() -> Self {
        Self::new(Quat::IDENTITY)
    }

    /// Create from yaw angle in degrees (rotation around vertical axis)
    /// 0 = north, 90 = east, 180 = south, 270 = west
    pub fn with_yaw(yaw_degrees: f32) -> Self {
        // Yaw rotation around Y axis (up in Bevy space)
        Self::new(Quat::from_rotation_y(yaw_degrees.to_radians()))
    }

    /// Create from Euler angles in degrees: [yaw, pitch, roll]
    pub fn from_euler_degrees(euler: [f32; 3]) -> Self {
        let [yaw, pitch, roll] = euler;
        // Apply rotations in yaw-pitch-roll order
        // Yaw around Y, Pitch around X, Roll around Z (Bevy convention)
        let quat = Quat::from_euler(
            glam::EulerRot::YXZ,
            yaw.to_radians(),
            pitch.to_radians(),
            roll.to_radians(),
        );
        Self::new(quat)
    }
}

impl ImuReader for MockImuReader {
    fn read_orientation(&mut self) -> Result<ImuReading, LocalizationError> {
        Ok(ImuReading {
            orientation: self.orientation,
            angular_velocity: None,
        })
    }
}

// ============================================================================
// gpsd Implementation (behind feature flag)
// ============================================================================

#[cfg(feature = "real")]
mod gpsd_impl {
    use super::*;
    use gpsd_proto::{Mode, ResponseData, Tpv, get_data, handshake};
    use std::io::{BufReader, BufWriter};
    use std::net::TcpStream;

    /// GPS reader that connects to gpsd
    pub struct GpsdReader {
        reader: BufReader<TcpStream>,
        #[allow(dead_code)]
        writer: BufWriter<TcpStream>,
    }

    impl GpsdReader {
        /// Connect to gpsd at the given address (e.g., "localhost:2947")
        pub fn connect(address: &str) -> Result<Self, LocalizationError> {
            info!(address, "Connecting to gpsd");

            let stream = TcpStream::connect(address)
                .map_err(|e| LocalizationError::GpsConnection(e.to_string()))?;

            stream
                .set_read_timeout(Some(Duration::from_millis(100)))
                .map_err(|e| LocalizationError::GpsConnection(e.to_string()))?;

            let reader = BufReader::new(
                stream
                    .try_clone()
                    .map_err(|e| LocalizationError::GpsConnection(e.to_string()))?,
            );
            let mut writer = BufWriter::new(stream);

            // Perform gpsd handshake
            let mut reader_for_handshake = reader;
            let handshake_result = handshake(&mut reader_for_handshake, &mut writer);

            match handshake_result {
                Ok(()) => {
                    info!("gpsd handshake successful");
                }
                Err(e) => {
                    return Err(LocalizationError::GpsProtocol(format!(
                        "Handshake failed: {:?}",
                        e
                    )));
                }
            }

            Ok(Self {
                reader: reader_for_handshake,
                writer,
            })
        }

        fn tpv_to_fix(tpv: &Tpv) -> Option<GpsFix> {
            // Need at least a 2D fix with lat/lon
            match tpv.mode {
                Mode::NoFix => return None,
                Mode::Fix2d | Mode::Fix3d => {}
            }

            let lat = tpv.lat?;
            let lon = tpv.lon?;

            // For altitude, prefer HAE (height above ellipsoid) for WGS84 consistency,
            // fall back to MSL, then to 0
            let alt = tpv.alt_hae.or(tpv.alt_msl).or(tpv.alt).unwrap_or(0.0) as f64;

            // Validate the position
            let position = match GeoPosition::new_checked(lat, lon, alt) {
                Ok(p) => p,
                Err(e) => {
                    warn!(?e, lat, lon, alt, "Invalid GPS position received");
                    return None;
                }
            };

            Some(GpsFix {
                position,
                horizontal_accuracy: tpv.eph.unwrap_or(5.0), // Default 5m if not reported
                vertical_accuracy: tpv.epv.unwrap_or(10.0),  // Default 10m if not reported
                fix_3d: matches!(tpv.mode, Mode::Fix3d),
                course: tpv.track,
                speed: tpv.speed,
            })
        }
    }

    impl GpsReader for GpsdReader {
        fn read_fix(&mut self) -> Result<Option<GpsFix>, LocalizationError> {
            // Try to read a TPV message from gpsd
            match get_data(&mut self.reader) {
                Ok(response) => match response {
                    ResponseData::Tpv(tpv) => Ok(Self::tpv_to_fix(&tpv)),
                    ResponseData::Sky(_) => Ok(None), // Satellite info, not position
                    _ => Ok(None),
                },
                Err(gpsd_proto::GpsdError::IoError(ref e))
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    // No data available yet, not an error
                    Ok(None)
                }
                Err(e) => Err(LocalizationError::GpsProtocol(format!("{:?}", e))),
            }
        }
    }
}

#[cfg(feature = "real")]
pub use gpsd_impl::GpsdReader;

// ============================================================================
// Localizer Trait and Implementations
// ============================================================================

/// Abstract localization interface
pub trait Localizer: Send {
    fn get_pose(&mut self) -> Result<CameraPose, LocalizationError>;
    fn status(&self) -> LocalizationStatus;
}

/// Dummy localizer for testing - returns a fixed pose
pub struct DummyLocalizer {
    pose: CameraPose,
}

impl DummyLocalizer {
    pub fn new(position: GeoPosition, orientation: Quat) -> Self {
        Self {
            pose: CameraPose {
                position,
                orientation,
                timestamp: now_micros(),
                uncertainty: PoseUncertainty::default(),
                status: LocalizationStatus::Nominal,
            },
        }
    }

    pub fn with_position(lat: f64, lon: f64, alt: f64) -> Self {
        Self::new(GeoPosition::new(lat, lon, alt), Quat::IDENTITY)
    }
}

impl Localizer for DummyLocalizer {
    fn get_pose(&mut self) -> Result<CameraPose, LocalizationError> {
        self.pose.timestamp = now_micros();
        Ok(self.pose)
    }

    fn status(&self) -> LocalizationStatus {
        self.pose.status
    }
}

// ============================================================================
// GPS + IMU Fusion Localizer
// ============================================================================

/// GPS + IMU fusion localizer.
///
/// This localizer combines GPS position data with IMU orientation data to produce
/// a complete camera pose. When GPS is temporarily unavailable, it uses dead
/// reckoning based on the last known position, with uncertainty that grows over time.
///
/// # Dead Reckoning Model
///
/// Position uncertainty grows as √t during dead reckoning, modeling random walk drift.
/// The growth rate depends on assumed IMU noise characteristics:
/// - Typical MEMS IMU: ~0.1 m/s² noise → ~0.5m drift after 10s
/// - High-quality IMU: ~0.01 m/s² noise → ~0.05m drift after 10s
///
/// # Coordinate Frames
///
/// - GPS provides position in WGS84 (lat/lon/alt)
/// - IMU provides orientation, typically in NED or sensor-local frame
/// - Output orientation is in Bevy-compatible frame (Y-up, looking down -Z)
pub struct GpsImuLocalizer<G: GpsReader, I: ImuReader> {
    gps: G,
    imu: I,
    fallback_mode: GpsFallbackMode,
    dead_reckoning_timeout: Duration,

    // State
    last_gps_fix: Option<GpsFix>,
    last_gps_time: Option<Instant>,
    dead_reckoning_start: Option<Instant>,
    current_status: LocalizationStatus,

    // Dead reckoning parameters
    /// Position drift rate during dead reckoning (m/s^0.5)
    /// This models random walk: position error ∝ √(time)
    drift_rate: f32,
}

impl<G: GpsReader, I: ImuReader> GpsImuLocalizer<G, I> {
    /// Default drift rate for consumer-grade MEMS IMU (m per sqrt(second))
    /// After 10 seconds: 0.15 * √10 ≈ 0.47m drift
    /// After 30 seconds: 0.15 * √30 ≈ 0.82m drift
    const DEFAULT_DRIFT_RATE: f32 = 0.15;

    pub fn new(
        gps: G,
        imu: I,
        fallback_mode: GpsFallbackMode,
        dead_reckoning_timeout: Duration,
    ) -> Self {
        Self {
            gps,
            imu,
            fallback_mode,
            dead_reckoning_timeout,
            last_gps_fix: None,
            last_gps_time: None,
            dead_reckoning_start: None,
            current_status: LocalizationStatus::Unavailable,
            drift_rate: Self::DEFAULT_DRIFT_RATE,
        }
    }

    /// Set the position drift rate for dead reckoning (meters per sqrt(second))
    pub fn with_drift_rate(mut self, rate: f32) -> Self {
        self.drift_rate = rate;
        self
    }

    /// Calculate position uncertainty based on GPS accuracy and dead reckoning time
    fn calculate_uncertainty(&self, gps_fix: &GpsFix) -> PoseUncertainty {
        let base_horizontal = gps_fix.horizontal_accuracy;
        let base_vertical = gps_fix.vertical_accuracy;

        // Add drift uncertainty if dead reckoning
        let (horizontal, vertical) = if let Some(dr_start) = self.dead_reckoning_start {
            let elapsed_secs = dr_start.elapsed().as_secs_f32();
            let drift = self.drift_rate * elapsed_secs.sqrt();
            (base_horizontal + drift, base_vertical + drift)
        } else {
            (base_horizontal, base_vertical)
        };

        PoseUncertainty {
            // In ENU: X=east, Y=north, Z=up
            position_stddev: glam::Vec3::new(horizontal, horizontal, vertical),
            // Orientation uncertainty - typically small for IMUs
            // ~1 degree = ~0.017 radians
            orientation_stddev: glam::Vec3::splat(0.017),
        }
    }

    /// Try to get a fresh GPS fix, updating internal state
    fn update_gps(&mut self) -> Result<Option<&GpsFix>, LocalizationError> {
        match self.gps.read_fix() {
            Ok(Some(fix)) => {
                debug!(
                    lat = fix.position.latitude,
                    lon = fix.position.longitude,
                    alt = fix.position.altitude,
                    accuracy_h = fix.horizontal_accuracy,
                    "GPS fix acquired"
                );
                self.last_gps_fix = Some(fix);
                self.last_gps_time = Some(Instant::now());
                self.dead_reckoning_start = None;
                self.current_status = LocalizationStatus::Nominal;
                Ok(self.last_gps_fix.as_ref())
            }
            Ok(None) => {
                // No fix available - start or continue dead reckoning
                if self.last_gps_fix.is_some() && self.dead_reckoning_start.is_none() {
                    info!("GPS fix lost, entering dead reckoning mode");
                    self.dead_reckoning_start = Some(Instant::now());
                }

                // Update status
                if let Some(dr_start) = self.dead_reckoning_start {
                    let elapsed = dr_start.elapsed();
                    if elapsed > self.dead_reckoning_timeout {
                        self.current_status = LocalizationStatus::Unavailable;
                    } else {
                        self.current_status = LocalizationStatus::DeadReckoning {
                            duration_ms: elapsed.as_millis() as u64,
                        };
                    }
                } else {
                    self.current_status = LocalizationStatus::Unavailable;
                }

                Ok(self.last_gps_fix.as_ref())
            }
            Err(e) => {
                warn!(?e, "GPS read error");
                // Treat as no fix
                if self.last_gps_fix.is_some() && self.dead_reckoning_start.is_none() {
                    self.dead_reckoning_start = Some(Instant::now());
                }
                Ok(self.last_gps_fix.as_ref())
            }
        }
    }
}

impl<G: GpsReader, I: ImuReader> Localizer for GpsImuLocalizer<G, I> {
    fn get_pose(&mut self) -> Result<CameraPose, LocalizationError> {
        // 1. Try to get GPS position
        self.update_gps()?;

        let gps_fix = match (&self.last_gps_fix, self.fallback_mode) {
            (Some(fix), _) => fix,
            (None, GpsFallbackMode::Strict) => return Err(LocalizationError::NoFix),
            (None, GpsFallbackMode::DeadReckoning) => {
                return Err(LocalizationError::GpsUnavailable);
            }
        };

        // Check dead reckoning timeout
        if let Some(dr_start) = self.dead_reckoning_start {
            if dr_start.elapsed() > self.dead_reckoning_timeout {
                return Err(LocalizationError::DeadReckoningTimeout);
            }
        }

        // 2. Get IMU orientation
        let imu_reading = self.imu.read_orientation()?;

        // 3. Compute uncertainty (grows during dead reckoning)
        let uncertainty = self.calculate_uncertainty(gps_fix);

        // 4. Build the pose
        Ok(CameraPose {
            position: gps_fix.position,
            orientation: imu_reading.orientation,
            timestamp: now_micros(),
            uncertainty,
            status: self.current_status,
        })
    }

    fn status(&self) -> LocalizationStatus {
        self.current_status
    }
}

// ============================================================================
// Builder for easy construction
// ============================================================================

/// Builder for creating a GpsImuLocalizer with various configurations
pub struct LocalizerBuilder {
    #[cfg(feature = "real")]
    gps_address: Option<String>,
    fixed_position: Option<GeoPosition>,
    fixed_orientation: Option<Quat>,
    fallback_mode: GpsFallbackMode,
    dead_reckoning_timeout: Duration,
}

impl Default for LocalizerBuilder {
    fn default() -> Self {
        Self {
            #[cfg(feature = "real")]
            gps_address: None,
            fixed_position: None,
            fixed_orientation: None,
            fallback_mode: GpsFallbackMode::DeadReckoning,
            dead_reckoning_timeout: Duration::from_secs(30),
        }
    }
}

impl LocalizerBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Use gpsd at the given address (e.g., "localhost:2947")
    #[cfg(feature = "real")]
    pub fn with_gpsd(mut self, address: impl Into<String>) -> Self {
        self.gps_address = Some(address.into());
        self
    }

    /// Use a fixed position (for testing or when GPS is pre-surveyed)
    pub fn with_fixed_position(mut self, position: GeoPosition) -> Self {
        self.fixed_position = Some(position);
        self
    }

    /// Use a fixed orientation (for cameras with known mounting)
    pub fn with_fixed_orientation(mut self, orientation: Quat) -> Self {
        self.fixed_orientation = Some(orientation);
        self
    }

    /// Set the fallback mode for GPS outages
    pub fn with_fallback_mode(mut self, mode: GpsFallbackMode) -> Self {
        self.fallback_mode = mode;
        self
    }

    /// Set the dead reckoning timeout
    pub fn with_dead_reckoning_timeout(mut self, timeout: Duration) -> Self {
        self.dead_reckoning_timeout = timeout;
        self
    }

    /// Build a localizer with mock GPS/IMU (for testing)
    pub fn build_mock(self) -> GpsImuLocalizer<MockGpsReader, MockImuReader> {
        let position = self
            .fixed_position
            .unwrap_or_else(|| GeoPosition::new(0.0, 0.0, 0.0));
        let orientation = self.fixed_orientation.unwrap_or(Quat::IDENTITY);

        GpsImuLocalizer::new(
            MockGpsReader::new(position),
            MockImuReader::new(orientation),
            self.fallback_mode,
            self.dead_reckoning_timeout,
        )
    }

    /// Build a localizer with real gpsd and mock IMU
    #[cfg(feature = "real")]
    pub fn build_gpsd_mock_imu(
        self,
    ) -> Result<GpsImuLocalizer<GpsdReader, MockImuReader>, LocalizationError> {
        let address = self.gps_address.as_deref().unwrap_or("localhost:2947");
        let gps = GpsdReader::connect(address)?;
        let orientation = self.fixed_orientation.unwrap_or(Quat::IDENTITY);

        Ok(GpsImuLocalizer::new(
            gps,
            MockImuReader::new(orientation),
            self.fallback_mode,
            self.dead_reckoning_timeout,
        ))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread::sleep;

    #[test]
    fn test_mock_gps_reader() {
        let mut reader = MockGpsReader::new(GeoPosition::new(47.6062, -122.3321, 100.0));

        let fix = reader.read_fix().unwrap().unwrap();
        assert!((fix.position.latitude - 47.6062).abs() < 1e-6);
        assert!((fix.position.longitude - (-122.3321)).abs() < 1e-6);
        assert!(fix.fix_3d);
    }

    #[test]
    fn test_mock_imu_reader() {
        let mut reader = MockImuReader::level();
        let reading = reader.read_orientation().unwrap();
        assert!((reading.orientation.x).abs() < 1e-6);
        assert!((reading.orientation.y).abs() < 1e-6);
        assert!((reading.orientation.z).abs() < 1e-6);
        assert!((reading.orientation.w - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_mock_imu_with_yaw() {
        let mut reader = MockImuReader::with_yaw(90.0);
        let reading = reader.read_orientation().unwrap();
        // 90 degree yaw around Y axis
        let expected = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        assert!((reading.orientation.x - expected.x).abs() < 1e-5);
        assert!((reading.orientation.y - expected.y).abs() < 1e-5);
        assert!((reading.orientation.z - expected.z).abs() < 1e-5);
        assert!((reading.orientation.w - expected.w).abs() < 1e-5);
    }

    #[test]
    fn test_dummy_localizer() {
        let mut localizer = DummyLocalizer::with_position(47.6062, -122.3321, 100.0);

        let pose = localizer.get_pose().unwrap();
        assert!((pose.position.latitude - 47.6062).abs() < 1e-6);
        assert_eq!(localizer.status(), LocalizationStatus::Nominal);
    }

    #[test]
    fn test_gps_imu_localizer_nominal() {
        let mut localizer = LocalizerBuilder::new()
            .with_fixed_position(GeoPosition::new(47.6062, -122.3321, 100.0))
            .build_mock();

        let pose = localizer.get_pose().unwrap();
        assert!((pose.position.latitude - 47.6062).abs() < 1e-6);
        assert_eq!(pose.status, LocalizationStatus::Nominal);

        // Uncertainty should be GPS accuracy only (no dead reckoning drift)
        assert!(pose.uncertainty.position_stddev.x < 5.0);
    }

    #[test]
    fn test_gps_imu_localizer_dead_reckoning() {
        let fail_at = Instant::now() + Duration::from_millis(10);
        let gps =
            MockGpsReader::with_failure_at(GeoPosition::new(47.6062, -122.3321, 100.0), fail_at);
        let imu = MockImuReader::level();

        let mut localizer = GpsImuLocalizer::new(
            gps,
            imu,
            GpsFallbackMode::DeadReckoning,
            Duration::from_secs(30),
        );

        // First call should succeed and establish position
        let pose1 = localizer.get_pose().unwrap();
        assert_eq!(pose1.status, LocalizationStatus::Nominal);

        // Wait for GPS to "fail"
        sleep(Duration::from_millis(20));

        // Second call should enter dead reckoning
        let pose2 = localizer.get_pose().unwrap();
        assert!(matches!(
            pose2.status,
            LocalizationStatus::DeadReckoning { .. }
        ));

        // Position should still be available (from dead reckoning)
        assert!((pose2.position.latitude - 47.6062).abs() < 1e-6);

        // Uncertainty should have grown
        assert!(pose2.uncertainty.position_stddev.x > pose1.uncertainty.position_stddev.x);
    }

    #[test]
    fn test_dead_reckoning_timeout() {
        let fail_at = Instant::now() + Duration::from_millis(5);
        let gps =
            MockGpsReader::with_failure_at(GeoPosition::new(47.6062, -122.3321, 100.0), fail_at);
        let imu = MockImuReader::level();

        let mut localizer = GpsImuLocalizer::new(
            gps,
            imu,
            GpsFallbackMode::DeadReckoning,
            Duration::from_millis(50), // Very short timeout for testing
        );

        // First call establishes position
        let _ = localizer.get_pose().unwrap();

        // Wait for GPS to fail
        sleep(Duration::from_millis(10));

        // Should be in dead reckoning
        let pose = localizer.get_pose().unwrap();
        assert!(matches!(
            pose.status,
            LocalizationStatus::DeadReckoning { .. }
        ));

        // Wait for timeout
        sleep(Duration::from_millis(60));

        // Should now fail with timeout error
        let result = localizer.get_pose();
        assert!(matches!(
            result,
            Err(LocalizationError::DeadReckoningTimeout)
        ));
    }

    #[test]
    fn test_strict_mode_no_initial_fix() {
        // GPS reader that never returns a fix
        struct NoFixGpsReader;
        impl GpsReader for NoFixGpsReader {
            fn read_fix(&mut self) -> Result<Option<GpsFix>, LocalizationError> {
                Ok(None)
            }
        }

        let mut localizer = GpsImuLocalizer::new(
            NoFixGpsReader,
            MockImuReader::level(),
            GpsFallbackMode::Strict,
            Duration::from_secs(30),
        );

        let result = localizer.get_pose();
        assert!(matches!(result, Err(LocalizationError::NoFix)));
    }

    #[test]
    fn test_uncertainty_growth_formula() {
        // Test that uncertainty grows as sqrt(time)
        let fix = GpsFix {
            position: GeoPosition::new(0.0, 0.0, 0.0),
            horizontal_accuracy: 2.5,
            vertical_accuracy: 5.0,
            fix_3d: true,
            course: None,
            speed: None,
        };

        let gps = MockGpsReader::new(GeoPosition::new(0.0, 0.0, 0.0));
        let imu = MockImuReader::level();
        let mut localizer = GpsImuLocalizer::new(
            gps,
            imu,
            GpsFallbackMode::DeadReckoning,
            Duration::from_secs(100),
        );

        // Simulate dead reckoning for 100 seconds
        localizer.last_gps_fix = Some(fix.clone());
        localizer.dead_reckoning_start = Some(Instant::now() - Duration::from_secs(100));

        let uncertainty = localizer.calculate_uncertainty(&fix);

        // After 100 seconds: drift = 0.15 * sqrt(100) = 1.5m
        // Total horizontal = 2.5 + 1.5 = 4.0m
        let expected_horizontal = 2.5 + 0.15 * 10.0;
        assert!((uncertainty.position_stddev.x - expected_horizontal).abs() < 0.1);
    }

    #[test]
    fn test_builder_pattern() {
        let localizer = LocalizerBuilder::new()
            .with_fixed_position(GeoPosition::new(47.6062, -122.3321, 100.0))
            .with_fixed_orientation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2))
            .with_fallback_mode(GpsFallbackMode::Strict)
            .with_dead_reckoning_timeout(Duration::from_secs(60))
            .build_mock();

        assert!(matches!(localizer.fallback_mode, GpsFallbackMode::Strict));
        assert_eq!(localizer.dead_reckoning_timeout, Duration::from_secs(60));
    }

    #[test]
    fn test_status_transitions() {
        let fail_at = Instant::now() + Duration::from_millis(10);
        let gps =
            MockGpsReader::with_failure_at(GeoPosition::new(47.6062, -122.3321, 100.0), fail_at);
        let imu = MockImuReader::level();

        let mut localizer = GpsImuLocalizer::new(
            gps,
            imu,
            GpsFallbackMode::DeadReckoning,
            Duration::from_millis(100),
        );

        // Initial status is unavailable
        assert_eq!(localizer.status(), LocalizationStatus::Unavailable);

        // After getting a fix, status is nominal
        let _ = localizer.get_pose().unwrap();
        assert_eq!(localizer.status(), LocalizationStatus::Nominal);

        // After GPS loss, status transitions to dead reckoning
        sleep(Duration::from_millis(15));
        let _ = localizer.get_pose().unwrap();
        assert!(matches!(
            localizer.status(),
            LocalizationStatus::DeadReckoning { .. }
        ));

        // After timeout, status becomes unavailable
        sleep(Duration::from_millis(120));
        let _ = localizer.get_pose(); // Will return error
        assert_eq!(localizer.status(), LocalizationStatus::Unavailable);
    }
}
