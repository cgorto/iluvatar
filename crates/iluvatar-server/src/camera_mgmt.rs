use iluvatar_core::{CameraId, CameraIntrinsics, CameraPose, CameraRegistration, CameraStateInfo};
use std::collections::HashMap;
use std::time::Instant;

#[derive(Debug, Clone)]
pub enum ConnectionState {
    Connected { since: Instant },
    Disconnected { since: Instant },
}

pub struct CameraState {
    pub id: CameraId,
    pub intrinsics: CameraIntrinsics,
    pub last_pose: CameraPose,
    pub connection: ConnectionState,
    pub last_frame_time: Option<Instant>,
    pub frame_count: u64,
    frames_in_last_second: u32,
    last_fps_update: Instant,
    pub fps: f32,
}

impl CameraState {
    fn new(registration: CameraRegistration) -> Self {
        Self {
            id: registration.camera_id,
            intrinsics: registration.intrinsics,
            last_pose: registration.initial_pose,
            connection: ConnectionState::Connected {
                since: Instant::now(),
            },
            last_frame_time: None,
            frame_count: 0,
            frames_in_last_second: 0,
            last_fps_update: Instant::now(),
            fps: 0.0,
        }
    }

    fn record_frame(&mut self, pose: CameraPose) {
        self.last_pose = pose;
        self.last_frame_time = Some(Instant::now());
        self.frame_count += 1;
        self.frames_in_last_second += 1;

        // Update FPS every second
        let now = Instant::now();
        if now.duration_since(self.last_fps_update).as_secs_f32() >= 1.0 {
            self.fps = self.frames_in_last_second as f32;
            self.frames_in_last_second = 0;
            self.last_fps_update = now;
        }
    }

    pub fn to_info(&self) -> CameraStateInfo {
        CameraStateInfo {
            camera_id: self.id,
            connected: matches!(self.connection, ConnectionState::Connected { .. }),
            last_frame_time: self.last_frame_time.map(|t| t.elapsed().as_micros() as u64),
            frames_per_second: self.fps,
        }
    }
}

/// Registry of all cameras
pub struct CameraRegistry {
    cameras: HashMap<CameraId, CameraState>,
}

impl CameraRegistry {
    pub fn new() -> Self {
        Self {
            cameras: HashMap::new(),
        }
    }

    /// Register a camera. Allows re-registration after reconnect by updating
    /// the existing entry (insert-or-update semantics).
    pub fn register(&mut self, registration: CameraRegistration) -> bool {
        // Verify protocol version.
        if registration.version != iluvatar_core::PROTOCOL_VERSION {
            return false;
        }

        // Insert or update — allows re-registration after reconnect.
        self.cameras
            .insert(registration.camera_id, CameraState::new(registration));
        true
    }

    /// Mark camera as connected
    pub fn connect(&mut self, camera_id: CameraId) {
        if let Some(state) = self.cameras.get_mut(&camera_id) {
            state.connection = ConnectionState::Connected {
                since: Instant::now(),
            };
        }
    }

    /// Mark camera as disconnected
    pub fn disconnect(&mut self, camera_id: CameraId) {
        if let Some(state) = self.cameras.get_mut(&camera_id) {
            state.connection = ConnectionState::Disconnected {
                since: Instant::now(),
            };
        }
    }

    /// Record a frame from a camera
    pub fn record_frame(&mut self, camera_id: CameraId, pose: CameraPose) {
        if let Some(state) = self.cameras.get_mut(&camera_id) {
            state.record_frame(pose);
        }
    }

    /// Get camera state
    pub fn get(&self, camera_id: CameraId) -> Option<&CameraState> {
        self.cameras.get(&camera_id)
    }

    /// Get all camera states
    pub fn all(&self) -> impl Iterator<Item = &CameraState> {
        self.cameras.values()
    }

    /// Get camera count
    pub fn count(&self) -> usize {
        self.cameras.len()
    }

    /// Get connected camera count
    pub fn connected_count(&self) -> usize {
        self.cameras
            .values()
            .filter(|c| matches!(c.connection, ConnectionState::Connected { .. }))
            .count()
    }

    /// Get all camera info for client
    pub fn all_info(&self) -> Vec<CameraStateInfo> {
        self.cameras.values().map(|c| c.to_info()).collect()
    }
}

impl Default for CameraRegistry {
    fn default() -> Self {
        Self::new()
    }
}
