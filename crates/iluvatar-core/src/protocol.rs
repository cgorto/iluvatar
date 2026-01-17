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

#[derive(Debug, Clone, Serialize, Deserialize)]
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
