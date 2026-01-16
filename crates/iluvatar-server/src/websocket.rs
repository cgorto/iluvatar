use iluvatar_core::{
    BoundingBox, CameraStateInfo, ClientUpdate, SnapshotMessage, SystemStatusMessage,
    TrackedObject, UpdateMessage,
};
use std::time::Instant;

/// State for a connected WebSocket client
pub struct ClientConnection {
    pub id: u64,
    pub connected_at: Instant,
    pub subscribed: bool,
    pub last_update: Instant,
}

impl ClientConnection {
    pub fn new(id: u64) -> Self {
        Self {
            id,
            connected_at: Instant::now(),
            subscribed: false,
            last_update: Instant::now(),
        }
    }
}

/// Build a snapshot message for clients
pub fn build_snapshot(
    timestamp: u64,
    objects: Vec<TrackedObject>,
    grid_bounds: BoundingBox,
    camera_states: Vec<CameraStateInfo>,
) -> ClientUpdate {
    ClientUpdate::Snapshot(SnapshotMessage {
        timestamp,
        objects,
        grid_bounds,
        camera_states,
    })
}

/// Build an update message for clients
pub fn build_update(timestamp: u64, objects: Vec<TrackedObject>) -> ClientUpdate {
    ClientUpdate::Update(UpdateMessage { timestamp, objects })
}

/// Build a system status message
pub fn build_system_status(
    active_cameras: u32,
    total_cameras: u32,
    tracked_objects: u32,
    voxels_active: u64,
    uptime: Instant,
) -> ClientUpdate {
    ClientUpdate::SystemStatus(SystemStatusMessage {
        active_cameras,
        total_cameras,
        tracked_objects,
        voxels_active,
        uptime_seconds: uptime.elapsed().as_secs(),
    })
}

/// Serialize a client update to JSON for WebSocket transport
pub fn serialize_update(update: &ClientUpdate) -> Result<String, serde_json::Error> {
    serde_json::to_string(update)
}
