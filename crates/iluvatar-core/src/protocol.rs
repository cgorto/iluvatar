use serde::{Deserialize, Serialize};

use crate::{CameraId, CameraIntrinsics, CameraPose, TrackedObject, VoxelContribution};

/// Current protocol version. Increment when making breaking changes.
/// v1: Initial protocol with camera-side raymarching
/// v2: Added MotionFrame for server-side raymarching
pub const PROTOCOL_VERSION: u8 = 2;

pub const MAX_CONTRIBUTIONS_PER_FRAME: usize = 65536;

/// Maximum motion pixels per frame. At 1920x1080, this is ~3% of pixels which
/// represents significant motion. Higher counts indicate either extreme motion
/// or a threshold tuning issue.
pub const MAX_MOTION_PIXELS_PER_FRAME: usize = 65536;

// ============================================================================
// Motion Pixel Types
// ============================================================================

/// A single pixel with detected motion.
///
/// Uses u16 coordinates to support up to 65535x65535 resolution while keeping
/// the type compact. Intensity is stored as u8 (0-255) matching the difference
/// mask output directly, avoiding lossy f32 conversion.
///
/// # Wire Format (postcard)
///
/// With postcard's varint encoding:
/// - `x`: 1-3 bytes (varint, typically 2 for 1920 width)
/// - `y`: 1-3 bytes (varint, typically 2 for 1080 height)
/// - `intensity`: 1 byte (fixed u8)
///
/// Typical size: 5 bytes per pixel for 1080p, versus 5 bytes fixed if we used
/// a packed u32 representation. The varint approach is more flexible for
/// different resolutions and serializes identically for common cases.
///
/// # Design Rationale
///
/// **Why u16 coordinates?**
/// - Matches common camera resolutions (up to 8K: 7680x4320)
/// - postcard encodes small values efficiently via varint
/// - Simpler than bit-packing, no masking/shifting overhead on decode
///
/// **Why u8 intensity?**
/// - Matches `DifferenceMask` output directly (no conversion)
/// - Sufficient precision: 256 levels captures motion magnitude well
/// - Server can normalize to f32 if needed for aggregation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotionPixel {
    pub x: u16,
    pub y: u16,
    pub intensity: u8,
}

impl MotionPixel {
    #[inline]
    pub const fn new(x: u16, y: u16, intensity: u8) -> Self {
        Self { x, y, intensity }
    }
}

/// Encoded motion data supporting multiple compression strategies.
///
/// The camera chooses the encoding based on motion characteristics:
/// - `Sparse`: Best for scattered motion (few pixels, spread out)
/// - `RunLength`: Best for clustered motion (many adjacent pixels)
///
/// # Encoding Selection Heuristic
///
/// A simple heuristic the camera can use:
/// ```ignore
/// if motion_pixels.len() < 1000 || cluster_ratio < 0.3 {
///     MotionData::Sparse(pixels)
/// } else {
///     MotionData::RunLength(encode_rle(pixels))
/// }
/// ```
///
/// The server accepts either format transparently.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MotionData {
    /// Direct list of motion pixels. Optimal for sparse, scattered motion.
    ///
    /// Wire size: ~5 bytes per pixel (with varint overhead for Vec length)
    Sparse(Vec<MotionPixel>),

    /// Run-length encoded motion data. Optimal for clustered motion.
    ///
    /// Each run represents a horizontal span of motion pixels at a given
    /// intensity. Consecutive pixels in the same row with the same intensity
    /// are collapsed into a single run.
    ///
    /// Wire size: 6 bytes per run (y: u16, x_start: u16, length: u16, intensity: u8)
    /// plus varint overhead
    RunLength(Vec<MotionRun>),
}

impl MotionData {
    /// Returns the number of motion pixels represented by this data.
    pub fn pixel_count(&self) -> usize {
        match self {
            MotionData::Sparse(pixels) => pixels.len(),
            MotionData::RunLength(runs) => runs.iter().map(|r| r.length as usize).sum(),
        }
    }

    /// Returns true if this motion data is empty.
    pub fn is_empty(&self) -> bool {
        match self {
            MotionData::Sparse(pixels) => pixels.is_empty(),
            MotionData::RunLength(runs) => runs.is_empty(),
        }
    }

    /// Iterate over all motion pixels, regardless of encoding.
    ///
    /// This allows the server to process both formats uniformly.
    pub fn pixels(&self) -> MotionPixelIter<'_> {
        match self {
            MotionData::Sparse(pixels) => MotionPixelIter::Sparse(pixels.iter()),
            MotionData::RunLength(runs) => MotionPixelIter::RunLength {
                runs: runs.iter(),
                current_run: None,
                current_offset: 0,
            },
        }
    }

    /// Create sparse motion data from a difference mask's motion pixels.
    ///
    /// Utility constructor for cameras that don't implement RLE.
    pub fn from_motion_pixels(pixels: impl IntoIterator<Item = (u32, u32, u8)>) -> Self {
        let sparse: Vec<MotionPixel> = pixels
            .into_iter()
            .take(MAX_MOTION_PIXELS_PER_FRAME)
            .map(|(x, y, intensity)| MotionPixel::new(x as u16, y as u16, intensity))
            .collect();
        MotionData::Sparse(sparse)
    }
}

/// A horizontal run of motion pixels with uniform intensity.
///
/// Represents pixels at coordinates (x_start, y) through (x_start + length - 1, y),
/// all with the same intensity value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotionRun {
    /// Row (vertical position)
    pub y: u16,
    /// Starting column (horizontal position)
    pub x_start: u16,
    /// Number of consecutive pixels in this run (1-65535)
    pub length: u16,
    /// Motion intensity for all pixels in this run
    pub intensity: u8,
}

impl MotionRun {
    pub const fn new(y: u16, x_start: u16, length: u16, intensity: u8) -> Self {
        Self {
            y,
            x_start,
            length,
            intensity,
        }
    }
}

/// Iterator over motion pixels from any encoding.
pub enum MotionPixelIter<'a> {
    Sparse(std::slice::Iter<'a, MotionPixel>),
    RunLength {
        runs: std::slice::Iter<'a, MotionRun>,
        current_run: Option<&'a MotionRun>,
        current_offset: u16,
    },
}

impl<'a> Iterator for MotionPixelIter<'a> {
    type Item = MotionPixel;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            MotionPixelIter::Sparse(iter) => iter.next().copied(),
            MotionPixelIter::RunLength {
                runs,
                current_run,
                current_offset,
            } => {
                loop {
                    if let Some(run) = current_run {
                        if *current_offset < run.length {
                            let pixel = MotionPixel::new(
                                run.x_start + *current_offset,
                                run.y,
                                run.intensity,
                            );
                            *current_offset += 1;
                            return Some(pixel);
                        }
                    }
                    // Move to next run
                    *current_run = runs.next();
                    *current_offset = 0;
                    if current_run.is_none() {
                        return None;
                    }
                }
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        match self {
            MotionPixelIter::Sparse(iter) => iter.size_hint(),
            MotionPixelIter::RunLength { runs, .. } => {
                // Can't know exact size without summing all remaining run lengths
                let remaining_runs = runs.len();
                (remaining_runs, None)
            }
        }
    }
}

// ============================================================================
// Motion Frame Type
// ============================================================================

/// A frame containing raw motion pixels for server-side raymarching.
///
/// This is the new lightweight alternative to `CameraFrame`. Instead of
/// sending pre-computed voxel contributions, the camera sends motion pixels
/// and the server performs raymarching. This shifts compute to the server
/// and significantly reduces bandwidth for cameras.
///
/// # Bandwidth Comparison
///
/// For a typical frame with 1000 motion pixels hitting 5000 voxels:
/// - `CameraFrame` (old): 1000 * (12 + 4) = 16KB (UVec3 + f32 per contribution)
/// - `MotionFrame` (new): 1000 * 5 = 5KB (sparse motion pixels)
/// - Savings: ~70% bandwidth reduction
///
/// # Server Requirements
///
/// To process `MotionFrame`, the server needs:
/// 1. Camera intrinsics (from `CameraRegistration`)
/// 2. Camera pose (included in frame)
/// 3. Grid configuration (server-owned)
///
/// The server reconstructs rays using `CameraIntrinsics::pixel_to_ray()` and
/// marches them through the voxel grid.
///
/// # When to Use
///
/// Use `MotionFrame` when:
/// - Camera has limited compute (embedded devices)
/// - Network bandwidth is constrained
/// - Server has spare capacity for raymarching
///
/// Use `CameraFrame` when:
/// - Camera has strong compute capabilities
/// - Minimizing server load is important
/// - Running many cameras per server
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionFrame {
    /// Camera identifier (must match registration)
    pub camera_id: CameraId,

    /// Frame sequence number for ordering and loss detection
    pub sequence: u64,

    /// Capture timestamp in microseconds since epoch
    pub timestamp: u64,

    /// Camera pose at capture time. Essential for correct ray generation.
    pub pose: CameraPose,

    /// Encoded motion pixel data
    pub motion: MotionData,
}

// ============================================================================
// Capability Negotiation
// ============================================================================

/// Capabilities advertised during camera registration.
///
/// This enables graceful protocol evolution and mixed deployments where
/// some cameras support new features and others don't.
///
/// # Versioning Strategy
///
/// Rather than bumping the protocol version for every new feature, we use
/// capability flags. The server can:
/// 1. Check if a camera supports a feature before requesting it
/// 2. Fall back to legacy behavior for older cameras
/// 3. Enable features incrementally during rollout
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CameraCapabilities {
    /// Camera can send `MotionFrame` (server-side raymarching).
    /// If false, camera will only send `CameraFrame` with pre-computed contributions.
    #[serde(default)]
    pub motion_frames: bool,

    /// Camera supports RLE encoding in `MotionData`.
    /// If false, server should expect only `MotionData::Sparse`.
    #[serde(default)]
    pub rle_encoding: bool,

    /// Reserved for future capabilities. Using a flags field allows adding
    /// boolean capabilities without breaking wire compatibility.
    #[serde(default)]
    pub flags: u32,
}

impl CameraCapabilities {
    /// Create capabilities for a basic camera (voxel contributions only)
    pub fn basic() -> Self {
        Self::default()
    }

    /// Create capabilities for a camera that supports motion frames
    pub fn with_motion_frames() -> Self {
        Self {
            motion_frames: true,
            ..Default::default()
        }
    }

    /// Create full capabilities (motion frames + RLE)
    pub fn full() -> Self {
        Self {
            motion_frames: true,
            rle_encoding: true,
            flags: 0,
        }
    }
}

/// Server preferences sent after registration to configure camera behavior.
///
/// The server can request specific frame formats based on its current load
/// and the camera's capabilities.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerPreferences {
    /// Preferred frame format. Camera should honor this if capable.
    pub preferred_format: FrameFormat,

    /// Hint for target frame rate (frames per second).
    /// Camera can throttle if server is overloaded.
    #[serde(default)]
    pub target_fps: Option<f32>,

    /// Maximum motion pixels per frame the server wants to receive.
    /// Camera can downsample if this limit would be exceeded.
    #[serde(default)]
    pub max_motion_pixels: Option<u32>,
}

impl Default for ServerPreferences {
    fn default() -> Self {
        Self {
            preferred_format: FrameFormat::VoxelContributions,
            target_fps: None,
            max_motion_pixels: None,
        }
    }
}

/// Frame format preference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum FrameFormat {
    /// Camera computes voxel contributions (original format)
    #[default]
    VoxelContributions,
    /// Camera sends motion pixels, server raymarches
    MotionPixels,
}

// ============================================================================
// Frame Types
// ============================================================================

/// A frame containing pre-computed voxel contributions (original format).
///
/// The camera performs raymarching locally and sends the resulting voxel
/// contributions. This format is bandwidth-heavy but compute-light on the server.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraFrame {
    pub camera_id: CameraId,
    pub sequence: u64,
    pub timestamp: u64,
    pub pose: CameraPose,
    pub contributions: Vec<VoxelContribution>,
}

// ============================================================================
// Registration
// ============================================================================

/// Camera registration message sent on connection.
///
/// # Protocol Evolution
///
/// The `capabilities` field was added in protocol v2. When deserializing
/// messages from v1 cameras, it will default to `CameraCapabilities::default()`
/// (basic capabilities only). This maintains backward compatibility.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraRegistration {
    /// Protocol version the camera implements
    pub version: u8,
    pub camera_id: CameraId,
    pub intrinsics: CameraIntrinsics,
    pub initial_pose: CameraPose,

    /// Capabilities this camera supports. Added in v2.
    /// Defaults to basic capabilities for backward compatibility with v1 cameras.
    #[serde(default)]
    pub capabilities: CameraCapabilities,
}

// ============================================================================
// Message Enums
// ============================================================================

/// Messages sent from camera to server.
///
/// # Variant Ordering
///
/// The enum variant order affects the discriminant byte in postcard encoding.
/// New variants MUST be added at the end to maintain wire compatibility with
/// older servers that don't recognize new message types (they'll fail cleanly
/// on unknown discriminants rather than misinterpreting data).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CameraMessage {
    /// Initial registration (must be first message)
    Register(CameraRegistration),

    /// Frame with pre-computed voxel contributions (v1 format)
    Frame(CameraFrame),

    /// Keepalive signal
    Heartbeat { camera_id: CameraId, timestamp: u64 },

    /// Time synchronization request
    TimeSync { timestamp: u64 },

    /// Frame with motion pixels for server-side raymarching (v2 format)
    ///
    /// Cameras should only send this if they advertised `motion_frames` capability
    /// and received `FrameFormat::MotionPixels` preference from server.
    Motion(MotionFrame),
}

/// Messages sent from server to camera.
///
/// # Variant Ordering
///
/// Same ordering constraints as `CameraMessage` - new variants at the end only.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ServerMessage {
    /// Registration acknowledgment (v1)
    Registered { camera_id: CameraId },

    /// Grid configuration for raymarching
    GridConfig(GridConfigMessage),

    /// Error response
    Error { code: u32, message: String },

    /// Registration acknowledgment with preferences (v2)
    ///
    /// Sent instead of `Registered` when the camera advertises v2 capabilities.
    /// Includes server preferences that may influence camera behavior.
    RegisteredWithPrefs {
        camera_id: CameraId,
        preferences: ServerPreferences,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GridConfigMessage {
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub origin_alt: f64,
    pub dimensions: [u32; 3],
    pub voxel_size: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ClientMessage {
    Subscribe,
    Unsubscribe,
    RequestSnapshot,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ClientUpdate {
    Snapshot(SnapshotMessage),
    Update(UpdateMessage),
    CameraStatus(CameraStatusMessage),
    SystemStatus(SystemStatusMessage),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SnapshotMessage {
    pub timestamp: u64,
    pub objects: Vec<TrackedObject>,
    pub grid_bounds: crate::BoundingBox,
    pub camera_states: Vec<CameraStateInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpdateMessage {
    pub timestamp: u64,
    pub objects: Vec<TrackedObject>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraStatusMessage {
    pub cameras: Vec<CameraStateInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraStateInfo {
    pub camera_id: CameraId,
    pub connected: bool,
    pub last_frame_time: Option<u64>,
    pub frames_per_second: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemStatusMessage {
    pub active_cameras: u32,
    pub total_cameras: u32,
    pub tracked_objects: u32,
    pub voxels_active: u64,
    pub uptime_seconds: u64,
}

pub fn serialize<T: Serialize>(msg: &T) -> Result<Vec<u8>, postcard::Error> {
    postcard::to_allocvec(msg)
}

pub fn deserialize<'a, T: Deserialize<'a>>(data: &'a [u8]) -> Result<T, postcard::Error> {
    postcard::from_bytes(data)
}

// Framing utilities for QUIC streams
pub const MAX_MESSAGE_SIZE: usize = 10 * 1024 * 1024; // 10MB

#[derive(Debug)]
pub enum FrameError {
    Io(std::io::Error),
    TooLarge(usize),
    Truncated,
    /// Data length exceeds u32::MAX and cannot be encoded in a 4-byte length prefix
    LengthOverflow(usize),
}

impl std::fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameError::Io(e) => write!(f, "IO error: {}", e),
            FrameError::TooLarge(size) => {
                write!(
                    f,
                    "Message too large: {} bytes (max {})",
                    size, MAX_MESSAGE_SIZE
                )
            }
            FrameError::Truncated => write!(f, "Stream ended before message complete"),
            FrameError::LengthOverflow(size) => {
                write!(
                    f,
                    "Data length {} exceeds maximum encodable size ({})",
                    size,
                    u32::MAX
                )
            }
        }
    }
}

impl std::error::Error for FrameError {}

impl From<std::io::Error> for FrameError {
    fn from(e: std::io::Error) -> Self {
        FrameError::Io(e)
    }
}

/// Read a length-prefixed frame from a QUIC receive stream.
/// Format: 4-byte big-endian length prefix followed by payload.
pub async fn read_framed(recv: &mut quinn::RecvStream) -> Result<Vec<u8>, FrameError> {
    // Read 4-byte length prefix
    let mut len_buf = [0u8; 4];
    recv.read_exact(&mut len_buf)
        .await
        .map_err(|_| FrameError::Truncated)?;

    let len = u32::from_be_bytes(len_buf) as usize;

    if len > MAX_MESSAGE_SIZE {
        return Err(FrameError::TooLarge(len));
    }

    // Read payload
    let mut buf = vec![0u8; len];
    recv.read_exact(&mut buf)
        .await
        .map_err(|_| FrameError::Truncated)?;

    Ok(buf)
}

/// Create a length-prefixed frame for writing.
/// Format: 4-byte big-endian length prefix followed by payload.
///
/// Returns an error if the data length exceeds `u32::MAX`.
pub fn write_framed(data: &[u8]) -> Result<Vec<u8>, FrameError> {
    let len: u32 = data
        .len()
        .try_into()
        .map_err(|_| FrameError::LengthOverflow(data.len()))?;
    let mut buf = Vec::with_capacity(4 + data.len());
    buf.extend_from_slice(&len.to_be_bytes());
    buf.extend_from_slice(data);
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CameraIntrinsics, CameraPose, DistortionModel, Fov, GeoPosition, LocalizationStatus,
        PoseUncertainty, VoxelContribution,
    };
    use glam::{Quat, UVec2, UVec3, Vec2};

    fn test_pose() -> CameraPose {
        CameraPose {
            position: GeoPosition::new(37.7749, -122.4194, 100.0),
            orientation: Quat::IDENTITY,
            timestamp: 12345678,
            uncertainty: PoseUncertainty::default(),
            status: LocalizationStatus::Nominal,
        }
    }

    fn test_intrinsics() -> CameraIntrinsics {
        CameraIntrinsics {
            focal_length: Vec2::new(500.0, 500.0),
            principal_point: Vec2::new(320.0, 240.0),
            resolution: UVec2::new(640, 480),
            fov: Fov {
                horizontal: 60.0,
                vertical: 45.0,
            },
            distortion: DistortionModel::None,
        }
    }

    #[test]
    fn test_write_framed_format() {
        let data = b"hello";
        let framed = write_framed(data).unwrap();

        // Check length prefix (4 bytes, big-endian)
        assert_eq!(&framed[..4], &[0, 0, 0, 5]);
        // Check payload
        assert_eq!(&framed[4..], b"hello");
    }

    #[test]
    fn test_write_framed_empty() {
        let data = b"";
        let framed = write_framed(data).unwrap();

        assert_eq!(&framed[..4], &[0, 0, 0, 0]);
        assert_eq!(framed.len(), 4);
    }

    #[test]
    fn test_write_framed_large() {
        let data = vec![0xAB; 1000];
        let framed = write_framed(&data).unwrap();

        // 1000 = 0x03E8
        assert_eq!(&framed[..4], &[0, 0, 0x03, 0xE8]);
        assert_eq!(framed.len(), 1004);
        assert!(framed[4..].iter().all(|&b| b == 0xAB));
    }

    #[test]
    fn test_serialize_deserialize_camera_message() {
        let reg = CameraRegistration {
            version: PROTOCOL_VERSION,
            camera_id: 42,
            intrinsics: test_intrinsics(),
            initial_pose: test_pose(),
            capabilities: CameraCapabilities::basic(),
        };

        let msg = CameraMessage::Register(reg);
        let data = serialize(&msg).unwrap();
        let decoded: CameraMessage = deserialize(&data).unwrap();

        match decoded {
            CameraMessage::Register(r) => {
                assert_eq!(r.version, PROTOCOL_VERSION);
                assert_eq!(r.camera_id, 42);
                assert_eq!(r.intrinsics.resolution, UVec2::new(640, 480));
            }
            _ => panic!("Expected Register message"),
        }
    }

    #[test]
    fn test_serialize_deserialize_frame() {
        let contributions = vec![
            VoxelContribution {
                index: UVec3::new(10, 20, 30),
                intensity: 0.5,
            },
            VoxelContribution {
                index: UVec3::new(20, 30, 40),
                intensity: 0.8,
            },
        ];

        let frame = CameraFrame {
            camera_id: 1,
            sequence: 100,
            timestamp: 12345678,
            pose: test_pose(),
            contributions,
        };

        let msg = CameraMessage::Frame(frame);
        let data = serialize(&msg).unwrap();
        let decoded: CameraMessage = deserialize(&data).unwrap();

        match decoded {
            CameraMessage::Frame(f) => {
                assert_eq!(f.camera_id, 1);
                assert_eq!(f.sequence, 100);
                assert_eq!(f.contributions.len(), 2);
                assert_eq!(f.contributions[0].index, UVec3::new(10, 20, 30));
            }
            _ => panic!("Expected Frame message"),
        }
    }

    #[test]
    fn test_serialize_deserialize_grid_config() {
        let config = GridConfigMessage {
            origin_lat: 37.7749,
            origin_lon: -122.4194,
            origin_alt: 0.0,
            dimensions: [1000, 1000, 100],
            voxel_size: 1.0,
        };

        let msg = ServerMessage::GridConfig(config);
        let data = serialize(&msg).unwrap();
        let decoded: ServerMessage = deserialize(&data).unwrap();

        match decoded {
            ServerMessage::GridConfig(c) => {
                assert!((c.origin_lat - 37.7749).abs() < 1e-6);
                assert!((c.origin_lon - (-122.4194)).abs() < 1e-6);
                assert_eq!(c.dimensions, [1000, 1000, 100]);
                assert!((c.voxel_size - 1.0).abs() < 1e-6);
            }
            _ => panic!("Expected GridConfig message"),
        }
    }

    #[test]
    fn test_framing_roundtrip() {
        // Simulate a complete framing roundtrip by encoding, then parsing the framed format
        let original = b"test payload with binary \x00\xFF data";
        let framed = write_framed(original).unwrap();

        // Parse the framed data manually (simulating what read_framed does)
        let len = u32::from_be_bytes([framed[0], framed[1], framed[2], framed[3]]) as usize;
        let payload = &framed[4..4 + len];

        assert_eq!(payload, original);
    }

    #[test]
    fn test_framing_roundtrip_with_serialized_message() {
        // Full roundtrip: message -> serialize -> frame -> parse frame -> deserialize -> message
        let msg = CameraMessage::Heartbeat {
            camera_id: 123,
            timestamp: 9999999,
        };

        // Serialize
        let serialized = serialize(&msg).unwrap();

        // Frame
        let framed = write_framed(&serialized).unwrap();

        // Parse frame (extract payload)
        let len = u32::from_be_bytes([framed[0], framed[1], framed[2], framed[3]]) as usize;
        assert!(len <= MAX_MESSAGE_SIZE);
        let payload = &framed[4..4 + len];

        // Deserialize
        let decoded: CameraMessage = deserialize(payload).unwrap();

        match decoded {
            CameraMessage::Heartbeat {
                camera_id,
                timestamp,
            } => {
                assert_eq!(camera_id, 123);
                assert_eq!(timestamp, 9999999);
            }
            _ => panic!("Expected Heartbeat message"),
        }
    }

    // =========================================================================
    // Motion Pixel Tests
    // =========================================================================

    #[test]
    fn test_motion_pixel_serialization() {
        let pixel = MotionPixel::new(1920, 1080, 255);
        let data = serialize(&pixel).unwrap();
        let decoded: MotionPixel = deserialize(&data).unwrap();

        assert_eq!(decoded.x, 1920);
        assert_eq!(decoded.y, 1080);
        assert_eq!(decoded.intensity, 255);
    }

    #[test]
    fn test_motion_pixel_wire_size() {
        // Verify postcard's varint encoding produces expected sizes
        let pixel_small = MotionPixel::new(100, 50, 128);
        let pixel_large = MotionPixel::new(1920, 1080, 255);

        let small_bytes = serialize(&pixel_small).unwrap();
        let large_bytes = serialize(&pixel_large).unwrap();

        // Small coordinates (< 128) encode in 1 byte each, intensity is always 1 byte
        // 100 and 50 both fit in 1 byte varint, so 1 + 1 + 1 = 3 bytes
        assert_eq!(small_bytes.len(), 3, "small pixel should be 3 bytes");

        // 1920 requires 2 bytes (varint), 1080 requires 2 bytes, intensity 1 byte
        // So 2 + 2 + 1 = 5 bytes
        assert_eq!(large_bytes.len(), 5, "1080p pixel should be 5 bytes");
    }

    #[test]
    fn test_motion_data_sparse_roundtrip() {
        let pixels = vec![
            MotionPixel::new(100, 200, 50),
            MotionPixel::new(300, 400, 100),
            MotionPixel::new(500, 600, 150),
        ];
        let data = MotionData::Sparse(pixels);

        let bytes = serialize(&data).unwrap();
        let decoded: MotionData = deserialize(&bytes).unwrap();

        assert_eq!(decoded.pixel_count(), 3);

        let decoded_pixels: Vec<_> = decoded.pixels().collect();
        assert_eq!(decoded_pixels.len(), 3);
        assert_eq!(decoded_pixels[0].x, 100);
        assert_eq!(decoded_pixels[1].intensity, 100);
    }

    #[test]
    fn test_motion_data_rle_roundtrip() {
        let runs = vec![
            MotionRun::new(100, 50, 20, 128),  // 20 pixels at row 100
            MotionRun::new(101, 48, 25, 200),  // 25 pixels at row 101
        ];
        let data = MotionData::RunLength(runs);

        let bytes = serialize(&data).unwrap();
        let decoded: MotionData = deserialize(&bytes).unwrap();

        assert_eq!(decoded.pixel_count(), 45); // 20 + 25
    }

    #[test]
    fn test_motion_data_rle_iteration() {
        let runs = vec![
            MotionRun::new(10, 5, 3, 100),  // 3 pixels: (5,10), (6,10), (7,10)
            MotionRun::new(11, 0, 2, 200),  // 2 pixels: (0,11), (1,11)
        ];
        let data = MotionData::RunLength(runs);

        let pixels: Vec<_> = data.pixels().collect();
        assert_eq!(pixels.len(), 5);

        // First run
        assert_eq!(pixels[0], MotionPixel::new(5, 10, 100));
        assert_eq!(pixels[1], MotionPixel::new(6, 10, 100));
        assert_eq!(pixels[2], MotionPixel::new(7, 10, 100));

        // Second run
        assert_eq!(pixels[3], MotionPixel::new(0, 11, 200));
        assert_eq!(pixels[4], MotionPixel::new(1, 11, 200));
    }

    #[test]
    fn test_motion_data_from_motion_pixels() {
        let input = vec![
            (100u32, 200u32, 50u8),
            (300, 400, 100),
        ];
        let data = MotionData::from_motion_pixels(input);

        match data {
            MotionData::Sparse(pixels) => {
                assert_eq!(pixels.len(), 2);
                assert_eq!(pixels[0].x, 100);
                assert_eq!(pixels[0].y, 200);
            }
            _ => panic!("Expected Sparse encoding"),
        }
    }

    #[test]
    fn test_motion_data_empty() {
        let sparse = MotionData::Sparse(vec![]);
        let rle = MotionData::RunLength(vec![]);

        assert!(sparse.is_empty());
        assert!(rle.is_empty());
        assert_eq!(sparse.pixel_count(), 0);
        assert_eq!(rle.pixel_count(), 0);
    }

    #[test]
    fn test_motion_frame_roundtrip() {
        let frame = MotionFrame {
            camera_id: 42,
            sequence: 1000,
            timestamp: 1234567890,
            pose: test_pose(),
            motion: MotionData::Sparse(vec![
                MotionPixel::new(100, 200, 50),
                MotionPixel::new(300, 400, 100),
            ]),
        };

        let msg = CameraMessage::Motion(frame);
        let bytes = serialize(&msg).unwrap();
        let decoded: CameraMessage = deserialize(&bytes).unwrap();

        match decoded {
            CameraMessage::Motion(f) => {
                assert_eq!(f.camera_id, 42);
                assert_eq!(f.sequence, 1000);
                assert_eq!(f.motion.pixel_count(), 2);
            }
            _ => panic!("Expected Motion message"),
        }
    }

    #[test]
    fn test_motion_frame_bandwidth_comparison() {
        // Compare wire sizes between CameraFrame and MotionFrame for equivalent data
        // Scenario: 1000 motion pixels that would raymarch to 5000 voxel contributions

        let contributions: Vec<_> = (0..5000)
            .map(|i| VoxelContribution {
                index: UVec3::new(i % 100, (i / 100) % 100, i / 10000),
                intensity: 1.0,
            })
            .collect();

        let old_frame = CameraFrame {
            camera_id: 1,
            sequence: 1,
            timestamp: 1,
            pose: test_pose(),
            contributions,
        };

        let motion_pixels: Vec<_> = (0..1000)
            .map(|i| MotionPixel::new((i % 1920) as u16, (i / 1920) as u16, 128))
            .collect();

        let new_frame = MotionFrame {
            camera_id: 1,
            sequence: 1,
            timestamp: 1,
            pose: test_pose(),
            motion: MotionData::Sparse(motion_pixels),
        };

        let old_bytes = serialize(&CameraMessage::Frame(old_frame)).unwrap();
        let new_bytes = serialize(&CameraMessage::Motion(new_frame)).unwrap();

        // MotionFrame should be significantly smaller
        assert!(
            new_bytes.len() < old_bytes.len() / 2,
            "MotionFrame ({} bytes) should be less than half of CameraFrame ({} bytes)",
            new_bytes.len(),
            old_bytes.len()
        );

        println!(
            "Bandwidth comparison: CameraFrame={} bytes, MotionFrame={} bytes, savings={:.1}%",
            old_bytes.len(),
            new_bytes.len(),
            (1.0 - new_bytes.len() as f64 / old_bytes.len() as f64) * 100.0
        );
    }

    // =========================================================================
    // Capability Negotiation Tests
    // =========================================================================

    #[test]
    fn test_capabilities_default() {
        let caps = CameraCapabilities::default();
        assert!(!caps.motion_frames);
        assert!(!caps.rle_encoding);
        assert_eq!(caps.flags, 0);
    }

    #[test]
    fn test_capabilities_constructors() {
        let basic = CameraCapabilities::basic();
        assert!(!basic.motion_frames);

        let motion = CameraCapabilities::with_motion_frames();
        assert!(motion.motion_frames);
        assert!(!motion.rle_encoding);

        let full = CameraCapabilities::full();
        assert!(full.motion_frames);
        assert!(full.rle_encoding);
    }

    #[test]
    fn test_registration_with_capabilities() {
        let reg = CameraRegistration {
            version: PROTOCOL_VERSION,
            camera_id: 1,
            intrinsics: test_intrinsics(),
            initial_pose: test_pose(),
            capabilities: CameraCapabilities::full(),
        };

        let bytes = serialize(&reg).unwrap();
        let decoded: CameraRegistration = deserialize(&bytes).unwrap();

        assert!(decoded.capabilities.motion_frames);
        assert!(decoded.capabilities.rle_encoding);
    }

    #[test]
    fn test_server_preferences_roundtrip() {
        let prefs = ServerPreferences {
            preferred_format: FrameFormat::MotionPixels,
            target_fps: Some(30.0),
            max_motion_pixels: Some(10000),
        };

        let msg = ServerMessage::RegisteredWithPrefs {
            camera_id: 42,
            preferences: prefs,
        };

        let bytes = serialize(&msg).unwrap();
        let decoded: ServerMessage = deserialize(&bytes).unwrap();

        match decoded {
            ServerMessage::RegisteredWithPrefs { camera_id, preferences } => {
                assert_eq!(camera_id, 42);
                assert_eq!(preferences.preferred_format, FrameFormat::MotionPixels);
                assert_eq!(preferences.target_fps, Some(30.0));
                assert_eq!(preferences.max_motion_pixels, Some(10000));
            }
            _ => panic!("Expected RegisteredWithPrefs"),
        }
    }

    #[test]
    fn test_frame_format_default() {
        let format = FrameFormat::default();
        assert_eq!(format, FrameFormat::VoxelContributions);
    }

    // =========================================================================
    // Backward Compatibility Tests
    // =========================================================================

    #[test]
    fn test_enum_discriminants_stable() {
        // Verify that enum discriminants match expected values for wire compatibility
        // This test will fail if someone reorders the enum variants

        let register = CameraMessage::Register(CameraRegistration {
            version: 1,
            camera_id: 0,
            intrinsics: test_intrinsics(),
            initial_pose: test_pose(),
            capabilities: CameraCapabilities::default(),
        });
        let frame = CameraMessage::Frame(CameraFrame {
            camera_id: 0,
            sequence: 0,
            timestamp: 0,
            pose: test_pose(),
            contributions: vec![],
        });
        let heartbeat = CameraMessage::Heartbeat { camera_id: 0, timestamp: 0 };
        let timesync = CameraMessage::TimeSync { timestamp: 0 };
        let motion = CameraMessage::Motion(MotionFrame {
            camera_id: 0,
            sequence: 0,
            timestamp: 0,
            pose: test_pose(),
            motion: MotionData::Sparse(vec![]),
        });

        // First byte of postcard enum encoding is the discriminant
        let reg_bytes = serialize(&register).unwrap();
        let frame_bytes = serialize(&frame).unwrap();
        let hb_bytes = serialize(&heartbeat).unwrap();
        let ts_bytes = serialize(&timesync).unwrap();
        let motion_bytes = serialize(&motion).unwrap();

        assert_eq!(reg_bytes[0], 0, "Register should be discriminant 0");
        assert_eq!(frame_bytes[0], 1, "Frame should be discriminant 1");
        assert_eq!(hb_bytes[0], 2, "Heartbeat should be discriminant 2");
        assert_eq!(ts_bytes[0], 3, "TimeSync should be discriminant 3");
        assert_eq!(motion_bytes[0], 4, "Motion should be discriminant 4");
    }
}
