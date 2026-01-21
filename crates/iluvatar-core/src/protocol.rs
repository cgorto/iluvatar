use serde::{Deserialize, Serialize};

use crate::{CameraId, CameraIntrinsics, CameraPose, TrackedObject, VoxelContribution};

pub const PROTOCOL_VERSION: u8 = 1;
pub const MAX_CONTRIBUTIONS_PER_FRAME: usize = 65536;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraFrame {
    pub camera_id: CameraId,
    pub sequence: u64,
    pub timestamp: u64,
    pub pose: CameraPose,
    pub contributions: Vec<VoxelContribution>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraRegistration {
    pub version: u8,
    pub camera_id: CameraId,
    pub intrinsics: CameraIntrinsics,
    pub initial_pose: CameraPose,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CameraMessage {
    Register(CameraRegistration),
    Frame(CameraFrame),
    Heartbeat { camera_id: CameraId, timestamp: u64 },
    TimeSync { timestamp: u64 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ServerMessage {
    Registered { camera_id: CameraId },
    GridConfig(GridConfigMessage),
    Error { code: u32, message: String },
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
}
