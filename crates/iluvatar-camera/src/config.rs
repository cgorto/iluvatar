use iluvatar_core::{
    CalibrationData, CalibrationError, CameraId, CameraIntrinsics, DistortionModel, GeoOrigin,
    RaymarchConfig,
};
use serde::Deserialize;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("Failed to read config file: {0}")]
    Read(#[from] std::io::Error),
    #[error("Failed to parse config: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("Failed to load calibration: {0}")]
    Calibration(#[from] CalibrationError),
}

#[derive(Debug, Clone, Deserialize)]
pub struct CameraConfig {
    pub identity: IdentityConfig,
    pub hardware: HardwareConfig,
    pub network: NetworkConfig,
    pub processing: ProcessingConfig,
    pub localization: LocalizationConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct IdentityConfig {
    pub camera_id: CameraId,
}

#[derive(Debug, Clone, Deserialize)]
pub struct HardwareConfig {
    pub device: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    /// Path to a calibration JSON file (output from camera-intrinsic-calibration or OpenCV).
    /// If specified, the camera will use calibrated intrinsics including distortion correction.
    #[serde(default)]
    pub calibration_file: Option<String>,
    /// Inline calibration parameters (alternative to calibration_file).
    /// If both are specified, calibration_file takes precedence.
    #[serde(default)]
    pub calibration: Option<InlineCalibration>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NetworkConfig {
    pub server_address: String,
    #[serde(default = "default_connection_timeout_secs")]
    pub connection_timeout_secs: u64,
    #[serde(default = "default_frame_buffer_size")]
    pub frame_buffer_size: usize,
    /// Maximum number of reconnection attempts before giving up.
    /// With exponential backoff (100ms to 30s), 100 attempts ≈ 30 minutes.
    #[serde(default = "default_max_reconnect_attempts")]
    pub max_reconnect_attempts: u32,
    /// Maximum total time to spend attempting reconnection (seconds).
    /// Whichever limit (attempts or timeout) is reached first will stop reconnection.
    #[serde(default = "default_reconnect_timeout_secs")]
    pub reconnect_timeout_secs: u64,
    /// Interval between heartbeat messages (seconds).
    /// Heartbeats keep the connection alive and let the server detect stale cameras.
    #[serde(default = "default_heartbeat_interval_secs")]
    pub heartbeat_interval_secs: u64,
    /// TLS configuration for server certificate verification.
    #[serde(default)]
    pub tls: TlsConfig,
}

/// TLS configuration for secure server connections.
///
/// By default, the system uses certificate pinning via fingerprint verification.
/// For development/testing, you can disable verification (NOT RECOMMENDED for production).
#[derive(Debug, Clone, Deserialize)]
pub struct TlsConfig {
    /// SHA-256 fingerprint of the expected server certificate.
    /// Format: hex string, e.g., "a1b2c3d4..." (64 characters, no colons).
    ///
    /// Generate with: openssl x509 -in server.crt -noout -sha256 -fingerprint | tr -d ':'
    /// Or: openssl x509 -in server.crt -outform DER | sha256sum
    #[serde(default)]
    pub certificate_fingerprint: Option<String>,

    /// Path to a PEM-encoded CA certificate file for chain verification.
    /// If specified, the server certificate chain will be verified against this CA.
    /// Can be used together with or instead of certificate_fingerprint.
    #[serde(default)]
    pub ca_cert_path: Option<String>,

    /// DANGEROUS: Skip all certificate verification.
    /// Only use this for development/testing with self-signed certificates.
    /// This option logs a warning at startup and should NEVER be used in production.
    #[serde(default)]
    pub dangerous_skip_verification: bool,
}

impl Default for TlsConfig {
    fn default() -> Self {
        Self {
            certificate_fingerprint: None,
            ca_cert_path: None,
            dangerous_skip_verification: false,
        }
    }
}

fn default_connection_timeout_secs() -> u64 {
    30
}

fn default_frame_buffer_size() -> usize {
    4
}

fn default_max_reconnect_attempts() -> u32 {
    100
}

fn default_reconnect_timeout_secs() -> u64 {
    1800 // 30 minutes
}

fn default_heartbeat_interval_secs() -> u64 {
    15
}

#[derive(Debug, Clone, Deserialize)]
pub struct ProcessingConfig {
    #[serde(default = "default_difference_threshold")]
    pub difference_threshold: u8,
    #[serde(default = "default_motion_threshold_fraction")]
    pub motion_threshold_fraction: f32,
    #[serde(default)]
    pub raymarch: RaymarchSettings,
    /// Geographic origin for the voxel grid coordinate system.
    ///
    /// This field is required. All camera positions and voxel coordinates are computed
    /// relative to this origin. Typically set to the center of the monitored area.
    pub grid_origin: GeoOrigin,
}

fn default_difference_threshold() -> u8 {
    20
}

fn default_motion_threshold_fraction() -> f32 {
    0.001 // 0.1% of pixels
}

#[derive(Debug, Clone, Deserialize)]
pub struct RaymarchSettings {
    #[serde(default = "default_max_distance")]
    pub max_distance: f32,
    #[serde(default = "default_step_size")]
    pub step_size: f32,
}

impl Default for RaymarchSettings {
    fn default() -> Self {
        Self {
            max_distance: default_max_distance(),
            step_size: default_step_size(),
        }
    }
}

fn default_max_distance() -> f32 {
    500.0
}

fn default_step_size() -> f32 {
    0.5
}

/// Inline camera calibration parameters.
///
/// These can be specified directly in the config file as an alternative to
/// loading from a separate calibration JSON file.
///
/// # Example TOML
///
/// ```toml
/// [hardware.calibration]
/// model = "opencv5"  # or "kb4" for fisheye
/// fx = 800.0
/// fy = 800.0
/// cx = 960.0
/// cy = 540.0
/// distortion = [-0.2, 0.1, 0.001, -0.001, 0.0]
/// ```
#[derive(Debug, Clone, Deserialize)]
pub struct InlineCalibration {
    /// Camera model type: "opencv5", "kb4", "pinhole", or "none"
    pub model: String,
    /// Horizontal focal length in pixels
    pub fx: f32,
    /// Vertical focal length in pixels
    pub fy: f32,
    /// Principal point x-coordinate in pixels
    pub cx: f32,
    /// Principal point y-coordinate in pixels
    pub cy: f32,
    /// Distortion coefficients (5 for opencv5, 4 for kb4, empty for pinhole/none)
    #[serde(default)]
    pub distortion: Vec<f32>,
}

impl InlineCalibration {
    /// Convert to a DistortionModel.
    pub fn to_distortion_model(&self) -> Result<DistortionModel, CalibrationError> {
        match self.model.to_lowercase().as_str() {
            "opencv5" | "plumb_bob" | "brown_conrady" => {
                if self.distortion.len() != 5 {
                    return Err(CalibrationError::WrongDistortionCount {
                        expected: 5,
                        got: self.distortion.len(),
                    });
                }
                Ok(DistortionModel::OpenCV5 {
                    k1: self.distortion[0],
                    k2: self.distortion[1],
                    p1: self.distortion[2],
                    p2: self.distortion[3],
                    k3: self.distortion[4],
                })
            }
            "kb4" | "kannala_brandt4" | "fisheye" => {
                if self.distortion.len() != 4 {
                    return Err(CalibrationError::WrongDistortionCount {
                        expected: 4,
                        got: self.distortion.len(),
                    });
                }
                Ok(DistortionModel::KannalaBrandt4 {
                    k1: self.distortion[0],
                    k2: self.distortion[1],
                    k3: self.distortion[2],
                    k4: self.distortion[3],
                })
            }
            "none" | "pinhole" => Ok(DistortionModel::None),
            other => Err(CalibrationError::UnsupportedModel(other.to_string())),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct LocalizationConfig {
    /// GPS device path (for serial NMEA) or gpsd address (e.g., "localhost:2947")
    pub gps_device: String,

    /// Timeout waiting for initial GPS fix (seconds)
    #[serde(default = "default_gps_timeout_secs")]
    pub gps_timeout_secs: u64,

    /// Maximum time to continue dead reckoning before marking localization unavailable (seconds)
    #[serde(default = "default_dead_reckoning_timeout_secs")]
    pub dead_reckoning_timeout_secs: u64,

    /// IMU device path (optional - if not provided, orientation comes from config or defaults to level)
    #[serde(default)]
    pub imu_device: Option<String>,

    /// Fixed orientation to use when no IMU is available.
    /// Specified as Euler angles in degrees: [yaw, pitch, roll]
    /// Yaw is rotation around vertical axis (0 = north, 90 = east)
    #[serde(default)]
    pub fixed_orientation: Option<[f32; 3]>,
}

fn default_gps_timeout_secs() -> u64 {
    60
}

fn default_dead_reckoning_timeout_secs() -> u64 {
    30
}

impl CameraConfig {
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let content = std::fs::read_to_string(path)?;
        let config: CameraConfig = toml::from_str(&content)?;
        Ok(config)
    }

    pub fn connection_timeout(&self) -> Duration {
        Duration::from_secs(self.network.connection_timeout_secs)
    }

    pub fn gps_timeout(&self) -> Duration {
        Duration::from_secs(self.localization.gps_timeout_secs)
    }

    pub fn dead_reckoning_timeout(&self) -> Duration {
        Duration::from_secs(self.localization.dead_reckoning_timeout_secs)
    }

    pub fn reconnect_timeout(&self) -> Duration {
        Duration::from_secs(self.network.reconnect_timeout_secs)
    }

    pub fn heartbeat_interval(&self) -> Duration {
        Duration::from_secs(self.network.heartbeat_interval_secs)
    }

    pub fn frame_interval(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.hardware.fps as f64)
    }

    pub fn resolution(&self) -> (u32, u32) {
        (self.hardware.width, self.hardware.height)
    }

    pub fn pixel_count(&self) -> u32 {
        self.hardware.width * self.hardware.height
    }

    pub fn motion_pixel_threshold(&self) -> u32 {
        ((self.pixel_count() as f32) * self.processing.motion_threshold_fraction) as u32
    }

    /// Load camera intrinsics, using calibration data if available.
    ///
    /// Priority:
    /// 1. calibration_file (if specified and valid)
    /// 2. inline calibration (if specified)
    /// 3. Default 90° FOV assumption (fallback)
    ///
    /// Returns an error if calibration is specified but invalid.
    pub fn to_intrinsics(&self) -> Result<CameraIntrinsics, ConfigError> {
        let (width, height) = self.resolution();

        // Try calibration file first
        if let Some(ref calib_path) = self.hardware.calibration_file {
            let data = CalibrationData::load(Path::new(calib_path))?;

            // Validate resolution matches
            if data.width != width || data.height != height {
                tracing::warn!(
                    "Calibration resolution {}x{} differs from config {}x{}. \
                     Using calibration parameters but config resolution.",
                    data.width,
                    data.height,
                    width,
                    height
                );
            }

            // Use the calibration data but override resolution from config
            let distortion = match data.model_type.to_uppercase().as_str() {
                "OPENCV5" | "PLUMB_BOB" | "BROWN_CONRADY" => {
                    if data.distortion.len() != 5 {
                        return Err(ConfigError::Calibration(
                            CalibrationError::WrongDistortionCount {
                                expected: 5,
                                got: data.distortion.len(),
                            },
                        ));
                    }
                    DistortionModel::OpenCV5 {
                        k1: data.distortion[0] as f32,
                        k2: data.distortion[1] as f32,
                        p1: data.distortion[2] as f32,
                        p2: data.distortion[3] as f32,
                        k3: data.distortion[4] as f32,
                    }
                }
                "KB4" | "KANNALA_BRANDT4" | "FISHEYE" => {
                    if data.distortion.len() != 4 {
                        return Err(ConfigError::Calibration(
                            CalibrationError::WrongDistortionCount {
                                expected: 4,
                                got: data.distortion.len(),
                            },
                        ));
                    }
                    DistortionModel::KannalaBrandt4 {
                        k1: data.distortion[0] as f32,
                        k2: data.distortion[1] as f32,
                        k3: data.distortion[2] as f32,
                        k4: data.distortion[3] as f32,
                    }
                }
                "NONE" | "PINHOLE" => DistortionModel::None,
                other => {
                    return Err(ConfigError::Calibration(
                        CalibrationError::UnsupportedModel(other.to_string()),
                    ));
                }
            };

            return Ok(CameraIntrinsics::from_calibration(
                data.fx as f32,
                data.fy as f32,
                data.cx as f32,
                data.cy as f32,
                width,
                height,
                distortion,
            ));
        }

        // Try inline calibration
        if let Some(ref calib) = self.hardware.calibration {
            let distortion = calib.to_distortion_model()?;
            return Ok(CameraIntrinsics::from_calibration(
                calib.fx, calib.fy, calib.cx, calib.cy, width, height, distortion,
            ));
        }

        // Fallback: default 90° FOV assumption
        tracing::warn!(
            "No calibration data available for camera. Using default 90° FOV assumption. \
             For accurate tracking, provide calibration via calibration_file or inline calibration."
        );

        Ok(CameraIntrinsics::from_fov(
            width,
            height,
            std::f32::consts::FRAC_PI_2,
        ))
    }

    /// Load camera intrinsics with fallback on error.
    ///
    /// Like `to_intrinsics()` but logs errors and returns default intrinsics
    /// instead of propagating errors. Useful for non-critical paths.
    pub fn to_intrinsics_or_default(&self) -> CameraIntrinsics {
        match self.to_intrinsics() {
            Ok(intrinsics) => intrinsics,
            Err(e) => {
                tracing::error!("Failed to load calibration: {e}. Using default intrinsics.");
                let (width, height) = self.resolution();
                CameraIntrinsics::from_fov(width, height, std::f32::consts::FRAC_PI_2)
            }
        }
    }

    pub fn to_raymarch_config(&self) -> RaymarchConfig {
        RaymarchConfig {
            max_distance: self.processing.raymarch.max_distance,
            step_size: self.processing.raymarch.step_size,
            ..Default::default()
        }
    }
}
