use glam::{Quat, UVec2, UVec3, Vec2, Vec3};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::path::Path;

/// Check if all components of a Vec3 are finite (not NaN or infinity).
///
/// This is a helper function for validating coordinate data at protocol boundaries.
/// Returns `true` if all components are finite, `false` if any component is NaN or infinite.
///
/// # Examples
/// ```
/// use glam::Vec3;
/// use iluvatar_core::is_finite_vec3;
///
/// assert!(is_finite_vec3(Vec3::new(1.0, 2.0, 3.0)));
/// assert!(!is_finite_vec3(Vec3::new(f32::NAN, 0.0, 0.0)));
/// assert!(!is_finite_vec3(Vec3::new(0.0, f32::INFINITY, 0.0)));
/// ```
#[inline]
pub fn is_finite_vec3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// Check if an f32 value is finite (not NaN or infinity).
#[inline]
pub fn is_finite_f32(v: f32) -> bool {
    v.is_finite()
}

pub type CameraId = u64;
pub type ObjectId = u64;
pub type Timestamp = u64;

/// Determines how positions are interpreted throughout the system.
///
/// - `Gps`: Positions are WGS84 (latitude, longitude, altitude). The raymarcher
///   converts to local meters via the ENU (East-North-Up) transform. Suitable
///   for outdoor deployments with GPS-equipped cameras.
///
/// - `Local`: Positions are direct (x, y, z) in meters. No geodetic math is
///   performed. Suitable for indoor or tabletop setups where cameras are placed
///   at known positions measured with a tape measure.
///
/// In local mode, the `GeoPosition` fields carry meter values:
///   - `latitude`  → y (forward axis)
///   - `longitude` → x (right axis)
///   - `altitude`  → z (up axis)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
pub enum CoordinateMode {
    #[default]
    Gps,
    Local,
}

/// Altitude bounds for validation (meters).
/// -500m covers the Dead Sea and mining operations.
/// 100km is the Kármán line (edge of space).
pub const MIN_ALTITUDE_M: f64 = -500.0;
pub const MAX_ALTITUDE_M: f64 = 100_000.0;

/// Error type for geographic coordinate validation failures.
#[derive(Debug, Clone, PartialEq)]
pub enum GeoValidationError {
    /// Latitude must be in range [-90, 90]
    LatitudeOutOfRange(f64),
    /// Longitude must be in range [-180, 180]
    LongitudeOutOfRange(f64),
    /// Altitude must be in range [MIN_ALTITUDE_M, MAX_ALTITUDE_M]
    AltitudeOutOfRange(f64),
    /// Latitude contains NaN or infinity
    LatitudeNotFinite(f64),
    /// Longitude contains NaN or infinity
    LongitudeNotFinite(f64),
    /// Altitude contains NaN or infinity
    AltitudeNotFinite(f64),
}

impl fmt::Display for GeoValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeoValidationError::LatitudeOutOfRange(v) => {
                write!(f, "latitude {v} out of range [-90, 90]")
            }
            GeoValidationError::LongitudeOutOfRange(v) => {
                write!(f, "longitude {v} out of range [-180, 180]")
            }
            GeoValidationError::AltitudeOutOfRange(v) => {
                write!(
                    f,
                    "altitude {v} out of range [{}, {}]",
                    MIN_ALTITUDE_M, MAX_ALTITUDE_M
                )
            }
            GeoValidationError::LatitudeNotFinite(v) => {
                write!(f, "latitude {v} is not finite (NaN or infinity)")
            }
            GeoValidationError::LongitudeNotFinite(v) => {
                write!(f, "longitude {v} is not finite (NaN or infinity)")
            }
            GeoValidationError::AltitudeNotFinite(v) => {
                write!(f, "altitude {v} is not finite (NaN or infinity)")
            }
        }
    }
}

impl std::error::Error for GeoValidationError {}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CameraPose {
    pub position: GeoPosition,
    pub orientation: Quat,
    pub timestamp: Timestamp,
    pub uncertainty: PoseUncertainty,
    pub status: LocalizationStatus,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GeoPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PoseUncertainty {
    pub position_stddev: Vec3,
    pub orientation_stddev: Vec3,
}

impl Default for PoseUncertainty {
    fn default() -> Self {
        Self {
            position_stddev: Vec3::splat(1.0),
            orientation_stddev: Vec3::splat(0.01),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, PartialEq)]
pub enum LocalizationStatus {
    #[default]
    Nominal,
    DeadReckoning {
        duration_ms: u64,
    },
    Unavailable,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    /// Focal length in pixels (fx, fy). May differ due to non-square pixels.
    pub focal_length: Vec2,
    /// Principal point in pixels (cx, cy). Optical center of the image.
    pub principal_point: Vec2,
    /// Image resolution (width, height).
    pub resolution: UVec2,
    /// Field of view (for backward compatibility and quick approximations).
    pub fov: Fov,
    /// Lens distortion model. Defaults to None (no distortion correction).
    #[serde(default)]
    pub distortion: DistortionModel,
}

impl CameraIntrinsics {
    /// Create intrinsics from calibration parameters.
    ///
    /// This is the preferred way to create intrinsics when you have calibration data.
    /// FOV is computed from focal length and resolution.
    pub fn from_calibration(
        fx: f32,
        fy: f32,
        cx: f32,
        cy: f32,
        width: u32,
        height: u32,
        distortion: DistortionModel,
    ) -> Self {
        // Compute FOV from focal length
        // FOV = 2 * atan(sensor_size / (2 * focal_length))
        // For pixels: FOV = 2 * atan(resolution / (2 * focal_length))
        let fov_h = 2.0 * (width as f32 / (2.0 * fx)).atan();
        let fov_v = 2.0 * (height as f32 / (2.0 * fy)).atan();

        Self {
            focal_length: Vec2::new(fx, fy),
            principal_point: Vec2::new(cx, cy),
            resolution: UVec2::new(width, height),
            fov: Fov {
                horizontal: fov_h,
                vertical: fov_v,
            },
            distortion,
        }
    }

    /// Create intrinsics from a simple field-of-view specification.
    ///
    /// This is a fallback for when no calibration data is available.
    /// Assumes principal point at image center and no distortion.
    pub fn from_fov(width: u32, height: u32, horizontal_fov: f32) -> Self {
        let aspect = width as f32 / height as f32;
        let vertical_fov = horizontal_fov / aspect;

        // Compute focal length from FOV
        // focal_length = resolution / (2 * tan(FOV/2))
        let fx = width as f32 / (2.0 * (horizontal_fov / 2.0).tan());
        let fy = height as f32 / (2.0 * (vertical_fov / 2.0).tan());

        Self {
            focal_length: Vec2::new(fx, fy),
            principal_point: Vec2::new(width as f32 / 2.0, height as f32 / 2.0),
            resolution: UVec2::new(width, height),
            fov: Fov {
                horizontal: horizontal_fov,
                vertical: vertical_fov,
            },
            distortion: DistortionModel::None,
        }
    }

    /// Convert pixel coordinates to a 3D ray direction in camera space.
    ///
    /// The returned vector points from the camera origin through the given pixel,
    /// with the camera looking down -Z (Bevy/OpenGL convention).
    ///
    /// This method handles lens distortion correction automatically using the
    /// configured distortion model.
    ///
    /// # Arguments
    /// * `u` - Horizontal pixel coordinate (0 = left edge)
    /// * `v` - Vertical pixel coordinate (0 = top edge)
    ///
    /// # Returns
    /// Normalized direction vector in camera space.
    pub fn pixel_to_ray(&self, u: f32, v: f32) -> Vec3 {
        // Step 1: Convert pixel to normalized image coordinates
        // These are distorted coordinates
        let x_distorted = (u - self.principal_point.x) / self.focal_length.x;
        let y_distorted = (v - self.principal_point.y) / self.focal_length.y;

        // Step 2: Remove lens distortion
        let (x, y) = self.distortion.undistort(x_distorted, y_distorted);

        // Step 3: Create ray direction
        // Camera looks down -Z in Bevy convention
        // X = right, Y = up, Z = back (toward viewer)
        Vec3::new(x, -y, -1.0).normalize()
    }

    /// Convert a 3D ray direction in camera space to pixel coordinates.
    ///
    /// This is the inverse of `pixel_to_ray`. Handles lens distortion.
    ///
    /// # Arguments
    /// * `direction` - Direction vector in camera space (will be normalized internally)
    ///
    /// # Returns
    /// Pixel coordinates (u, v), or None if the ray points behind the camera.
    pub fn ray_to_pixel(&self, direction: Vec3) -> Option<(f32, f32)> {
        // Camera looks down -Z, so valid rays have negative Z
        if direction.z >= 0.0 {
            return None;
        }

        // Step 1: Project to normalized image coordinates
        let x = -direction.x / direction.z;
        let y = direction.y / direction.z;

        // Step 2: Apply lens distortion
        let (x_distorted, y_distorted) = self.distortion.distort(x, y);

        // Step 3: Convert to pixel coordinates
        let u = x_distorted * self.focal_length.x + self.principal_point.x;
        let v = y_distorted * self.focal_length.y + self.principal_point.y;

        Some((u, v))
    }

    /// Check if pixel coordinates are within image bounds.
    pub fn in_bounds(&self, u: f32, v: f32) -> bool {
        u >= 0.0 && v >= 0.0 && u < self.resolution.x as f32 && v < self.resolution.y as f32
    }

    /// Returns true if this camera has calibration data (non-trivial distortion).
    pub fn is_calibrated(&self) -> bool {
        self.distortion.has_distortion()
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Fov {
    pub horizontal: f32,
    pub vertical: f32,
}

/// Lens distortion model for camera calibration.
///
/// Different lenses exhibit different types of distortion. The most common model
/// is the OpenCV 5-parameter model (radial + tangential), which handles typical
/// wide-angle lenses well. For fisheye lenses (>120° FOV), use KannalaBrandt4.
///
/// # Distortion Mathematics
///
/// ## OpenCV5 Model (Brown-Conrady)
///
/// Given normalized image coordinates `(x, y)` where:
/// ```text
/// x = (X_camera / Z_camera)
/// y = (Y_camera / Z_camera)
/// ```
///
/// The distorted coordinates are computed as:
/// ```text
/// r² = x² + y²
/// radial = 1 + k1*r² + k2*r⁴ + k3*r⁶
/// x_distorted = x * radial + 2*p1*x*y + p2*(r² + 2*x²)
/// y_distorted = y * radial + p1*(r² + 2*y²) + 2*p2*x*y
/// ```
///
/// Then pixel coordinates are:
/// ```text
/// u = fx * x_distorted + cx
/// v = fy * y_distorted + cy
/// ```
///
/// ## Undistortion (pixel → ray)
///
/// The inverse mapping (from distorted pixel to 3D ray) requires iterative
/// solution since the distortion model is only defined in the forward direction.
/// We use Newton-Raphson iteration, which typically converges in 5-10 iterations.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub enum DistortionModel {
    /// No distortion correction - use simple pinhole model.
    /// This is the default and should be used when:
    /// - No calibration data is available
    /// - The lens has negligible distortion
    /// - Speed is critical and accuracy can be sacrificed
    #[default]
    None,

    /// OpenCV's 5-parameter distortion model (Brown-Conrady).
    /// Parameters: k1, k2 (radial), p1, p2 (tangential), k3 (radial)
    ///
    /// This is the most common model, compatible with:
    /// - OpenCV's `calibrateCamera()` output
    /// - ROS camera_calibration package (`plumb_bob` model)
    /// - Most machine vision software
    ///
    /// Suitable for lenses with <120° field of view.
    OpenCV5 {
        /// First radial distortion coefficient. Negative = barrel, positive = pincushion.
        k1: f32,
        /// Second radial distortion coefficient.
        k2: f32,
        /// First tangential distortion coefficient.
        p1: f32,
        /// Second tangential distortion coefficient.
        p2: f32,
        /// Third radial distortion coefficient (higher-order correction).
        k3: f32,
    },

    /// Kannala-Brandt 4-parameter fisheye model.
    /// Used for ultra-wide-angle and fisheye lenses (>120° FOV).
    ///
    /// Compatible with:
    /// - OpenCV's fisheye module
    /// - camera-intrinsic-calibration's KB4 model
    KannalaBrandt4 { k1: f32, k2: f32, k3: f32, k4: f32 },
}

impl DistortionModel {
    /// Apply distortion to normalized image coordinates.
    ///
    /// Takes undistorted normalized coordinates (x, y) and returns
    /// distorted normalized coordinates (x', y').
    ///
    /// Normalized coordinates are: x = X/Z, y = Y/Z in camera frame.
    pub fn distort(&self, x: f32, y: f32) -> (f32, f32) {
        match self {
            Self::None => (x, y),

            Self::OpenCV5 { k1, k2, p1, p2, k3 } => {
                let r2 = x * x + y * y;
                let r4 = r2 * r2;
                let r6 = r4 * r2;

                // Radial distortion
                let radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

                // Tangential distortion
                let xy2 = 2.0 * x * y;
                let x_tangential = xy2 * p1 + p2 * (r2 + 2.0 * x * x);
                let y_tangential = p1 * (r2 + 2.0 * y * y) + xy2 * p2;

                (x * radial + x_tangential, y * radial + y_tangential)
            }

            Self::KannalaBrandt4 { k1, k2, k3, k4 } => {
                let r = (x * x + y * y).sqrt();
                if r < 1e-8 {
                    return (x, y);
                }

                let theta = r.atan();
                let theta2 = theta * theta;
                let theta4 = theta2 * theta2;
                let theta6 = theta4 * theta2;
                let theta8 = theta4 * theta4;

                let theta_d = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
                let scale = theta_d / r;

                (x * scale, y * scale)
            }
        }
    }

    /// Remove distortion from normalized image coordinates.
    ///
    /// Takes distorted normalized coordinates and returns undistorted
    /// normalized coordinates using iterative Newton-Raphson refinement.
    ///
    /// This is the inverse of `distort()`.
    pub fn undistort(&self, x_distorted: f32, y_distorted: f32) -> (f32, f32) {
        match self {
            Self::None => (x_distorted, y_distorted),

            Self::OpenCV5 { k1, k2, p1, p2, k3 } => {
                // Fixed-point iteration to invert the distortion model
                // Start with the distorted point as initial guess
                let mut x = x_distorted;
                let mut y = y_distorted;

                // Typically converges in 5-15 iterations for reasonable distortion
                for _ in 0..15 {
                    let r2 = x * x + y * y;
                    let r4 = r2 * r2;
                    let r6 = r4 * r2;

                    let radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

                    // Guard against degenerate cases where radial factor is too small/negative
                    if radial.abs() < 0.1 {
                        // Distortion model is invalid for this point, return best guess
                        break;
                    }

                    let xy2 = 2.0 * x * y;

                    let x_tangential = xy2 * p1 + p2 * (r2 + 2.0 * x * x);
                    let y_tangential = p1 * (r2 + 2.0 * y * y) + xy2 * p2;

                    // Forward model: distorted = undistorted * radial + tangential
                    // We want to solve for undistorted given distorted
                    // Rearranging: undistorted ≈ (distorted - tangential) / radial
                    let x_new = (x_distorted - x_tangential) / radial;
                    let y_new = (y_distorted - y_tangential) / radial;

                    // Check for convergence
                    let dx = x_new - x;
                    let dy = y_new - y;
                    if dx * dx + dy * dy < 1e-12 {
                        return (x_new, y_new);
                    }

                    x = x_new;
                    y = y_new;
                }

                (x, y)
            }

            Self::KannalaBrandt4 { k1, k2, k3, k4 } => {
                let theta_d = (x_distorted * x_distorted + y_distorted * y_distorted).sqrt();
                if theta_d < 1e-8 {
                    return (x_distorted, y_distorted);
                }

                // Newton-Raphson to find theta from theta_d
                let mut theta = theta_d;
                for _ in 0..10 {
                    let theta2 = theta * theta;
                    let theta4 = theta2 * theta2;
                    let theta6 = theta4 * theta2;
                    let theta8 = theta4 * theta4;

                    let f = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8)
                        - theta_d;
                    let f_prime = 1.0
                        + 3.0 * k1 * theta2
                        + 5.0 * k2 * theta4
                        + 7.0 * k3 * theta6
                        + 9.0 * k4 * theta8;

                    theta -= f / f_prime;
                }

                let scale = if theta_d > 1e-8 {
                    theta.tan() / theta_d
                } else {
                    1.0
                };

                (x_distorted * scale, y_distorted * scale)
            }
        }
    }

    /// Returns true if this model has non-trivial distortion coefficients.
    pub fn has_distortion(&self) -> bool {
        match self {
            Self::None => false,
            Self::OpenCV5 { k1, k2, p1, p2, k3 } => {
                k1.abs() > 1e-10
                    || k2.abs() > 1e-10
                    || p1.abs() > 1e-10
                    || p2.abs() > 1e-10
                    || k3.abs() > 1e-10
            }
            Self::KannalaBrandt4 { k1, k2, k3, k4 } => {
                k1.abs() > 1e-10 || k2.abs() > 1e-10 || k3.abs() > 1e-10 || k4.abs() > 1e-10
            }
        }
    }
}

/// Camera calibration data loaded from a JSON file.
///
/// This structure is designed to be compatible with the output of
/// `camera-intrinsic-calibration` (ccrs) and similar calibration tools.
///
/// # Example JSON
///
/// ```json
/// {
///   "model_type": "OPENCV5",
///   "width": 1920,
///   "height": 1080,
///   "fx": 800.0,
///   "fy": 800.0,
///   "cx": 960.0,
///   "cy": 540.0,
///   "distortion": [-0.2, 0.1, 0.001, 0.001, 0.0]
/// }
/// ```
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CalibrationData {
    /// Camera model type (e.g., "OPENCV5", "KB4")
    pub model_type: String,
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Horizontal focal length in pixels
    pub fx: f64,
    /// Vertical focal length in pixels
    pub fy: f64,
    /// Principal point x-coordinate in pixels
    pub cx: f64,
    /// Principal point y-coordinate in pixels
    pub cy: f64,
    /// Distortion coefficients (interpretation depends on model_type)
    pub distortion: Vec<f64>,
}

/// Error type for calibration loading failures.
#[derive(Debug, Clone)]
pub enum CalibrationError {
    /// Failed to read the calibration file
    IoError(String),
    /// Failed to parse JSON
    ParseError(String),
    /// Unsupported camera model type
    UnsupportedModel(String),
    /// Wrong number of distortion coefficients
    WrongDistortionCount { expected: usize, got: usize },
}

impl std::fmt::Display for CalibrationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IoError(e) => write!(f, "failed to read calibration file: {e}"),
            Self::ParseError(e) => write!(f, "failed to parse calibration JSON: {e}"),
            Self::UnsupportedModel(m) => write!(f, "unsupported camera model: {m}"),
            Self::WrongDistortionCount { expected, got } => {
                write!(f, "expected {expected} distortion coefficients, got {got}")
            }
        }
    }
}

impl std::error::Error for CalibrationError {}

impl CalibrationData {
    /// Load calibration data from a JSON file.
    pub fn load(path: &Path) -> Result<Self, CalibrationError> {
        let contents =
            std::fs::read_to_string(path).map_err(|e| CalibrationError::IoError(e.to_string()))?;
        serde_json::from_str(&contents).map_err(|e| CalibrationError::ParseError(e.to_string()))
    }

    /// Convert to CameraIntrinsics.
    ///
    /// Validates that the model type is supported and that the distortion
    /// coefficients are the correct length for the specified model.
    pub fn to_intrinsics(&self) -> Result<CameraIntrinsics, CalibrationError> {
        let distortion = match self.model_type.to_uppercase().as_str() {
            "OPENCV5" | "PLUMB_BOB" | "BROWN_CONRADY" => {
                if self.distortion.len() != 5 {
                    return Err(CalibrationError::WrongDistortionCount {
                        expected: 5,
                        got: self.distortion.len(),
                    });
                }
                DistortionModel::OpenCV5 {
                    k1: self.distortion[0] as f32,
                    k2: self.distortion[1] as f32,
                    p1: self.distortion[2] as f32,
                    p2: self.distortion[3] as f32,
                    k3: self.distortion[4] as f32,
                }
            }
            "KB4" | "KANNALA_BRANDT4" | "FISHEYE" => {
                if self.distortion.len() != 4 {
                    return Err(CalibrationError::WrongDistortionCount {
                        expected: 4,
                        got: self.distortion.len(),
                    });
                }
                DistortionModel::KannalaBrandt4 {
                    k1: self.distortion[0] as f32,
                    k2: self.distortion[1] as f32,
                    k3: self.distortion[2] as f32,
                    k4: self.distortion[3] as f32,
                }
            }
            "NONE" | "PINHOLE" => DistortionModel::None,
            other => return Err(CalibrationError::UnsupportedModel(other.to_string())),
        };

        Ok(CameraIntrinsics::from_calibration(
            self.fx as f32,
            self.fy as f32,
            self.cx as f32,
            self.cy as f32,
            self.width,
            self.height,
            distortion,
        ))
    }
}

/// Error type for Ray validation failures.
#[derive(Debug, Clone, PartialEq)]
pub enum RayError {
    /// Origin contains NaN or infinity
    OriginNotFinite(Vec3),
    /// Direction is zero-length or contains NaN/infinity
    DirectionInvalid(Vec3),
    /// Intensity is NaN or infinity
    IntensityNotFinite(f32),
}

impl fmt::Display for RayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RayError::OriginNotFinite(v) => {
                write!(
                    f,
                    "ray origin {:?} is not finite (contains NaN or infinity)",
                    v
                )
            }
            RayError::DirectionInvalid(v) => {
                write!(
                    f,
                    "ray direction {:?} is invalid (zero-length or contains NaN/infinity)",
                    v
                )
            }
            RayError::IntensityNotFinite(v) => {
                write!(f, "ray intensity {} is not finite (NaN or infinity)", v)
            }
        }
    }
}

impl std::error::Error for RayError {}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Ray {
    pub origin: Vec3,
    /// Direction vector. Must be normalized for correct DDA raymarching step sizes.
    pub direction: Vec3,
    pub intensity: f32,
}

impl Ray {
    /// Creates a new ray, normalizing the direction vector.
    ///
    /// # Panics
    /// Panics if:
    /// - The origin contains NaN or infinity
    /// - The direction vector has zero length or contains NaN/infinity
    /// - The intensity is NaN or infinity
    ///
    /// For fallible construction, use [`Ray::try_new`].
    pub fn new(origin: Vec3, direction: Vec3, intensity: f32) -> Self {
        Self::try_new(origin, direction, intensity).expect("invalid ray parameters")
    }

    /// Creates a new ray, returning an error if validation fails.
    ///
    /// # Errors
    /// Returns [`RayError`] if:
    /// - The origin contains NaN or infinity
    /// - The direction vector has zero length or contains NaN/infinity
    /// - The intensity is NaN or infinity
    pub fn try_new(origin: Vec3, direction: Vec3, intensity: f32) -> Result<Self, RayError> {
        if !is_finite_vec3(origin) {
            return Err(RayError::OriginNotFinite(origin));
        }

        if !is_finite_f32(intensity) {
            return Err(RayError::IntensityNotFinite(intensity));
        }

        let normalized = direction.normalize();
        // After normalization, a valid direction will have length ~1.0
        // NaN/Inf inputs or zero-length vectors will produce NaN after normalize()
        if !is_finite_vec3(normalized) || (normalized.length() - 1.0).abs() >= 1e-5 {
            return Err(RayError::DirectionInvalid(direction));
        }

        Ok(Self {
            origin,
            direction: normalized,
            intensity,
        })
    }

    /// Check if this ray has valid (finite) values.
    ///
    /// Returns `true` if all components are finite and the direction is normalized.
    pub fn is_valid(&self) -> bool {
        is_finite_vec3(self.origin)
            && is_finite_vec3(self.direction)
            && is_finite_f32(self.intensity)
            && (self.direction.length() - 1.0).abs() < 1e-5
    }
}

/// Error type for VoxelContribution validation failures.
#[derive(Debug, Clone, PartialEq)]
pub enum VoxelContributionError {
    /// Intensity is NaN or infinity
    IntensityNotFinite(f32),
    /// Intensity is negative
    IntensityNegative(f32),
}

impl fmt::Display for VoxelContributionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VoxelContributionError::IntensityNotFinite(v) => {
                write!(
                    f,
                    "voxel contribution intensity {} is not finite (NaN or infinity)",
                    v
                )
            }
            VoxelContributionError::IntensityNegative(v) => {
                write!(f, "voxel contribution intensity {} is negative", v)
            }
        }
    }
}

impl std::error::Error for VoxelContributionError {}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct VoxelContribution {
    pub index: UVec3,
    pub intensity: f32,
}

impl VoxelContribution {
    /// Creates a new VoxelContribution with validation.
    ///
    /// # Panics
    /// Panics if intensity is NaN, infinity, or negative.
    ///
    /// For fallible construction, use [`VoxelContribution::try_new`].
    pub fn new(index: UVec3, intensity: f32) -> Self {
        Self::try_new(index, intensity).expect("invalid voxel contribution parameters")
    }

    /// Creates a new VoxelContribution, returning an error if validation fails.
    ///
    /// # Errors
    /// Returns [`VoxelContributionError`] if:
    /// - The intensity is NaN or infinity
    /// - The intensity is negative
    pub fn try_new(index: UVec3, intensity: f32) -> Result<Self, VoxelContributionError> {
        if !is_finite_f32(intensity) {
            return Err(VoxelContributionError::IntensityNotFinite(intensity));
        }
        if intensity < 0.0 {
            return Err(VoxelContributionError::IntensityNegative(intensity));
        }
        Ok(Self { index, intensity })
    }

    /// Check if this contribution has valid values.
    ///
    /// Returns `true` if the intensity is finite and non-negative.
    #[inline]
    pub fn is_valid(&self) -> bool {
        is_finite_f32(self.intensity) && self.intensity >= 0.0
    }
}

/// Validate a slice of VoxelContributions, returning only valid ones.
///
/// This function filters out any contributions with NaN/Inf or negative intensity values.
/// Use this at protocol boundaries to sanitize incoming data.
pub fn filter_valid_contributions(contributions: &[VoxelContribution]) -> Vec<VoxelContribution> {
    contributions
        .iter()
        .filter(|c| c.is_valid())
        .copied()
        .collect()
}

/// Count how many invalid contributions are in a slice.
///
/// Useful for logging/metrics when rejecting bad data from cameras.
pub fn count_invalid_contributions(contributions: &[VoxelContribution]) -> usize {
    contributions.iter().filter(|c| !c.is_valid()).count()
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl BoundingBox {
    /// Creates a new bounding box. In debug builds, panics if min > max on any axis.
    pub fn new(min: Vec3, max: Vec3) -> Self {
        debug_assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "Invalid bounding box: min ({:?}) must be <= max ({:?})",
            min,
            max
        );
        Self { min, max }
    }

    /// Creates a new bounding box, returning None if min > max on any axis.
    pub fn new_checked(min: Vec3, max: Vec3) -> Option<Self> {
        if min.x <= max.x && min.y <= max.y && min.z <= max.z {
            Some(Self { min, max })
        } else {
            None
        }
    }

    pub fn contains(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    pub fn center(&self) -> Vec3 {
        (self.min + self.max) / 2.0
    }

    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackedObject {
    pub id: ObjectId,
    pub centroid: Vec3,
    pub bounding_box: BoundingBox,
    pub point_count: u32,
    pub total_intensity: f32,
    pub velocity: Option<Vec3>,
    pub confidence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedPoint {
    pub position: Vec3,
    pub intensity: f32,
    pub confidence: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tracked_object_serialization_roundtrip() {
        let obj = TrackedObject {
            id: 42,
            centroid: Vec3::new(1.0, 2.0, 3.0),
            bounding_box: BoundingBox::new(Vec3::ZERO, Vec3::ONE),
            point_count: 100,
            total_intensity: 50.0,
            velocity: Some(Vec3::new(1.0, 0.0, 0.0)),
            confidence: 0.95,
        };

        let bytes = postcard::to_allocvec(&obj).expect("serialization failed");
        let decoded: TrackedObject = postcard::from_bytes(&bytes).expect("deserialization failed");

        assert_eq!(decoded.id, obj.id);
        assert_eq!(decoded.point_count, obj.point_count);
    }

    #[test]
    fn test_distortion_none_roundtrip() {
        let model = DistortionModel::None;
        let (x, y) = (0.5, -0.3);
        let (xd, yd) = model.distort(x, y);
        assert!((xd - x).abs() < 1e-6);
        assert!((yd - y).abs() < 1e-6);

        let (xu, yu) = model.undistort(xd, yd);
        assert!((xu - x).abs() < 1e-6);
        assert!((yu - y).abs() < 1e-6);
    }

    #[test]
    fn test_distortion_opencv5_roundtrip() {
        // Typical barrel distortion coefficients
        let model = DistortionModel::OpenCV5 {
            k1: -0.2,
            k2: 0.1,
            p1: 0.001,
            p2: -0.001,
            k3: 0.0,
        };

        // Test at various points across the image
        let test_points = [
            (0.0, 0.0),  // center
            (0.5, 0.0),  // right
            (0.0, 0.3),  // top
            (-0.4, 0.2), // upper left
            (0.3, -0.4), // lower right
        ];

        for (x, y) in test_points {
            let (xd, yd) = model.distort(x, y);
            let (xu, yu) = model.undistort(xd, yd);

            assert!(
                (xu - x).abs() < 1e-5,
                "x roundtrip failed: {x} -> {xd} -> {xu}"
            );
            assert!(
                (yu - y).abs() < 1e-5,
                "y roundtrip failed: {y} -> {yd} -> {yu}"
            );
        }
    }

    #[test]
    fn test_distortion_opencv5_barrel() {
        // Negative k1 = barrel distortion (points move toward center)
        let model = DistortionModel::OpenCV5 {
            k1: -0.3,
            k2: 0.0,
            p1: 0.0,
            p2: 0.0,
            k3: 0.0,
        };

        // Point at radius 0.5 from center
        let (x, y) = (0.5, 0.0);
        let (xd, _yd) = model.distort(x, y);

        // With barrel distortion, distorted point should be closer to center
        assert!(
            xd.abs() < x.abs(),
            "barrel distortion should move point toward center"
        );
    }

    #[test]
    fn test_distortion_opencv5_pincushion() {
        // Positive k1 = pincushion distortion (points move away from center)
        let model = DistortionModel::OpenCV5 {
            k1: 0.3,
            k2: 0.0,
            p1: 0.0,
            p2: 0.0,
            k3: 0.0,
        };

        let (x, y) = (0.5, 0.0);
        let (xd, _yd) = model.distort(x, y);

        // With pincushion distortion, distorted point should be farther from center
        assert!(
            xd.abs() > x.abs(),
            "pincushion distortion should move point away from center"
        );
    }

    #[test]
    fn test_distortion_kannala_brandt_roundtrip() {
        let model = DistortionModel::KannalaBrandt4 {
            k1: 0.1,
            k2: -0.02,
            k3: 0.005,
            k4: -0.001,
        };

        let test_points = [
            (0.0, 0.0),
            (0.3, 0.0),
            (0.0, 0.4),
            (-0.2, 0.2),
            (0.25, -0.3),
        ];

        for (x, y) in test_points {
            let (xd, yd) = model.distort(x, y);
            let (xu, yu) = model.undistort(xd, yd);

            assert!(
                (xu - x).abs() < 1e-5,
                "x roundtrip failed: {x} -> {xd} -> {xu}"
            );
            assert!(
                (yu - y).abs() < 1e-5,
                "y roundtrip failed: {y} -> {yd} -> {yu}"
            );
        }
    }

    #[test]
    fn test_intrinsics_pixel_to_ray_center() {
        let intrinsics = CameraIntrinsics::from_fov(1920, 1080, std::f32::consts::FRAC_PI_2);

        // Center pixel should produce ray pointing straight ahead (-Z)
        let ray = intrinsics.pixel_to_ray(960.0, 540.0);
        assert!((ray.x).abs() < 1e-5, "center ray should have x≈0");
        assert!((ray.y).abs() < 1e-5, "center ray should have y≈0");
        assert!(ray.z < 0.0, "center ray should point down -Z");
    }

    #[test]
    fn test_intrinsics_pixel_to_ray_corners() {
        let intrinsics = CameraIntrinsics::from_fov(1920, 1080, std::f32::consts::FRAC_PI_2);

        // Top-left corner
        let ray_tl = intrinsics.pixel_to_ray(0.0, 0.0);
        assert!(ray_tl.x < 0.0, "top-left ray should point left");
        assert!(ray_tl.y > 0.0, "top-left ray should point up");

        // Bottom-right corner
        let ray_br = intrinsics.pixel_to_ray(1920.0, 1080.0);
        assert!(ray_br.x > 0.0, "bottom-right ray should point right");
        assert!(ray_br.y < 0.0, "bottom-right ray should point down");
    }

    #[test]
    fn test_intrinsics_ray_to_pixel_roundtrip() {
        let intrinsics = CameraIntrinsics::from_calibration(
            800.0,
            800.0,
            960.0,
            540.0,
            1920,
            1080,
            DistortionModel::OpenCV5 {
                k1: -0.1,
                k2: 0.05,
                p1: 0.001,
                p2: -0.001,
                k3: 0.0,
            },
        );

        // Test pixel -> ray -> pixel roundtrip
        let test_pixels = [
            (960.0, 540.0),  // center
            (480.0, 270.0),  // upper left
            (1440.0, 810.0), // lower right
            (200.0, 500.0),  // left edge area
            (1700.0, 300.0), // upper right area
        ];

        for (u, v) in test_pixels {
            let ray = intrinsics.pixel_to_ray(u, v);
            let (u2, v2) = intrinsics.ray_to_pixel(ray).expect("ray should project");

            assert!(
                (u2 - u).abs() < 0.1,
                "u roundtrip failed at ({u}, {v}): {u} -> ray -> {u2}"
            );
            assert!(
                (v2 - v).abs() < 0.1,
                "v roundtrip failed at ({u}, {v}): {v} -> ray -> {v2}"
            );
        }
    }

    #[test]
    fn test_intrinsics_with_strong_distortion() {
        // Test with significant but realistic distortion (typical for wide-angle lens)
        // These values are similar to what you'd get from a GoPro or wide-angle webcam
        let intrinsics = CameraIntrinsics::from_calibration(
            500.0,
            500.0,
            640.0,
            360.0,
            1280,
            720,
            DistortionModel::OpenCV5 {
                k1: -0.28, // moderate barrel distortion
                k2: 0.08,
                p1: 0.001,
                p2: -0.001,
                k3: 0.0,
            },
        );

        // Test at edge of image where distortion is strongest
        let (u, v) = (100.0, 100.0);
        let ray = intrinsics.pixel_to_ray(u, v);
        let (u2, v2) = intrinsics.ray_to_pixel(ray).expect("ray should project");

        assert!(
            (u2 - u).abs() < 0.5,
            "strong distortion roundtrip failed for u: {u} -> {u2}"
        );
        assert!(
            (v2 - v).abs() < 0.5,
            "strong distortion roundtrip failed for v: {v} -> {v2}"
        );
    }

    #[test]
    fn test_calibration_data_parsing() {
        let json = r#"{
            "model_type": "OPENCV5",
            "width": 1920,
            "height": 1080,
            "fx": 800.0,
            "fy": 800.0,
            "cx": 960.0,
            "cy": 540.0,
            "distortion": [-0.2, 0.1, 0.001, -0.001, 0.0]
        }"#;

        let data: CalibrationData = serde_json::from_str(json).expect("parse failed");
        assert_eq!(data.width, 1920);
        assert_eq!(data.height, 1080);
        assert!((data.fx - 800.0).abs() < 1e-6);
        assert_eq!(data.distortion.len(), 5);

        let intrinsics = data.to_intrinsics().expect("conversion failed");
        assert_eq!(intrinsics.resolution.x, 1920);
        assert!(matches!(
            intrinsics.distortion,
            DistortionModel::OpenCV5 { .. }
        ));
    }

    #[test]
    fn test_calibration_data_fisheye() {
        let json = r#"{
            "model_type": "KB4",
            "width": 1280,
            "height": 720,
            "fx": 400.0,
            "fy": 400.0,
            "cx": 640.0,
            "cy": 360.0,
            "distortion": [0.1, -0.05, 0.01, -0.002]
        }"#;

        let data: CalibrationData = serde_json::from_str(json).expect("parse failed");
        let intrinsics = data.to_intrinsics().expect("conversion failed");
        assert!(matches!(
            intrinsics.distortion,
            DistortionModel::KannalaBrandt4 { .. }
        ));
    }

    #[test]
    fn test_calibration_data_wrong_distortion_count() {
        let json = r#"{
            "model_type": "OPENCV5",
            "width": 1920,
            "height": 1080,
            "fx": 800.0,
            "fy": 800.0,
            "cx": 960.0,
            "cy": 540.0,
            "distortion": [-0.2, 0.1]
        }"#;

        let data: CalibrationData = serde_json::from_str(json).expect("parse failed");
        let result = data.to_intrinsics();
        assert!(matches!(
            result,
            Err(CalibrationError::WrongDistortionCount { .. })
        ));
    }

    #[test]
    fn test_has_distortion() {
        assert!(!DistortionModel::None.has_distortion());

        let zero = DistortionModel::OpenCV5 {
            k1: 0.0,
            k2: 0.0,
            p1: 0.0,
            p2: 0.0,
            k3: 0.0,
        };
        assert!(!zero.has_distortion());

        let nonzero = DistortionModel::OpenCV5 {
            k1: -0.1,
            k2: 0.0,
            p1: 0.0,
            p2: 0.0,
            k3: 0.0,
        };
        assert!(nonzero.has_distortion());
    }

    // ==================== NaN/Inf Validation Tests ====================

    #[test]
    fn test_is_finite_vec3() {
        // Valid finite vectors
        assert!(is_finite_vec3(Vec3::ZERO));
        assert!(is_finite_vec3(Vec3::ONE));
        assert!(is_finite_vec3(Vec3::new(-100.0, 200.0, 0.001)));

        // NaN in various positions
        assert!(!is_finite_vec3(Vec3::new(f32::NAN, 0.0, 0.0)));
        assert!(!is_finite_vec3(Vec3::new(0.0, f32::NAN, 0.0)));
        assert!(!is_finite_vec3(Vec3::new(0.0, 0.0, f32::NAN)));

        // Infinity in various positions
        assert!(!is_finite_vec3(Vec3::new(f32::INFINITY, 0.0, 0.0)));
        assert!(!is_finite_vec3(Vec3::new(0.0, f32::NEG_INFINITY, 0.0)));
        assert!(!is_finite_vec3(Vec3::new(0.0, 0.0, f32::INFINITY)));
    }

    #[test]
    fn test_ray_valid_construction() {
        let ray = Ray::try_new(Vec3::new(1.0, 2.0, 3.0), Vec3::new(1.0, 0.0, 0.0), 0.5);
        assert!(ray.is_ok());
        let ray = ray.unwrap();
        assert!(ray.is_valid());
        assert!((ray.direction.length() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_ray_rejects_nan_origin() {
        let result = Ray::try_new(Vec3::new(f32::NAN, 0.0, 0.0), Vec3::X, 1.0);
        assert!(matches!(result, Err(RayError::OriginNotFinite(_))));
    }

    #[test]
    fn test_ray_rejects_inf_origin() {
        let result = Ray::try_new(Vec3::new(f32::INFINITY, 0.0, 0.0), Vec3::X, 1.0);
        assert!(matches!(result, Err(RayError::OriginNotFinite(_))));
    }

    #[test]
    fn test_ray_rejects_nan_direction() {
        let result = Ray::try_new(Vec3::ZERO, Vec3::new(f32::NAN, 1.0, 0.0), 1.0);
        assert!(matches!(result, Err(RayError::DirectionInvalid(_))));
    }

    #[test]
    fn test_ray_rejects_zero_direction() {
        let result = Ray::try_new(Vec3::ZERO, Vec3::ZERO, 1.0);
        assert!(matches!(result, Err(RayError::DirectionInvalid(_))));
    }

    #[test]
    fn test_ray_rejects_nan_intensity() {
        let result = Ray::try_new(Vec3::ZERO, Vec3::X, f32::NAN);
        assert!(matches!(result, Err(RayError::IntensityNotFinite(_))));
    }

    #[test]
    fn test_ray_rejects_inf_intensity() {
        let result = Ray::try_new(Vec3::ZERO, Vec3::X, f32::INFINITY);
        assert!(matches!(result, Err(RayError::IntensityNotFinite(_))));
    }

    #[test]
    #[should_panic(expected = "invalid ray parameters")]
    fn test_ray_new_panics_on_nan() {
        let _ = Ray::new(Vec3::new(f32::NAN, 0.0, 0.0), Vec3::X, 1.0);
    }

    #[test]
    fn test_voxel_contribution_valid_construction() {
        let contrib = VoxelContribution::try_new(UVec3::new(10, 20, 30), 0.5);
        assert!(contrib.is_ok());
        let contrib = contrib.unwrap();
        assert!(contrib.is_valid());
    }

    #[test]
    fn test_voxel_contribution_rejects_nan_intensity() {
        let result = VoxelContribution::try_new(UVec3::ZERO, f32::NAN);
        assert!(matches!(
            result,
            Err(VoxelContributionError::IntensityNotFinite(_))
        ));
    }

    #[test]
    fn test_voxel_contribution_rejects_inf_intensity() {
        let result = VoxelContribution::try_new(UVec3::ZERO, f32::INFINITY);
        assert!(matches!(
            result,
            Err(VoxelContributionError::IntensityNotFinite(_))
        ));
    }

    #[test]
    fn test_voxel_contribution_rejects_negative_intensity() {
        let result = VoxelContribution::try_new(UVec3::ZERO, -1.0);
        assert!(matches!(
            result,
            Err(VoxelContributionError::IntensityNegative(_))
        ));
    }

    #[test]
    #[should_panic(expected = "invalid voxel contribution parameters")]
    fn test_voxel_contribution_new_panics_on_nan() {
        let _ = VoxelContribution::new(UVec3::ZERO, f32::NAN);
    }

    #[test]
    fn test_filter_valid_contributions() {
        let contributions = vec![
            VoxelContribution {
                index: UVec3::new(1, 2, 3),
                intensity: 1.0,
            },
            VoxelContribution {
                index: UVec3::new(4, 5, 6),
                intensity: f32::NAN,
            },
            VoxelContribution {
                index: UVec3::new(7, 8, 9),
                intensity: 2.0,
            },
            VoxelContribution {
                index: UVec3::new(10, 11, 12),
                intensity: f32::INFINITY,
            },
            VoxelContribution {
                index: UVec3::new(13, 14, 15),
                intensity: -1.0,
            },
        ];

        let valid = filter_valid_contributions(&contributions);
        assert_eq!(valid.len(), 2);
        assert_eq!(valid[0].index, UVec3::new(1, 2, 3));
        assert_eq!(valid[1].index, UVec3::new(7, 8, 9));
    }

    #[test]
    fn test_count_invalid_contributions() {
        let contributions = vec![
            VoxelContribution {
                index: UVec3::ZERO,
                intensity: 1.0,
            },
            VoxelContribution {
                index: UVec3::ZERO,
                intensity: f32::NAN,
            },
            VoxelContribution {
                index: UVec3::ZERO,
                intensity: f32::NEG_INFINITY,
            },
        ];

        assert_eq!(count_invalid_contributions(&contributions), 2);
    }
}
