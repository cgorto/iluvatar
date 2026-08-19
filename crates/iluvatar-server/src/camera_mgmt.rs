use glam::EulerRot;
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
    session_id: u64,
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
    fn new(registration: CameraRegistration, session_id: u64) -> Self {
        Self {
            id: registration.camera_id,
            session_id,
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
    next_session_id: u64,
}

impl CameraRegistry {
    pub fn new() -> Self {
        Self {
            cameras: HashMap::new(),
            next_session_id: 1,
        }
    }

    /// Register a camera and return the generation assigned to this connection.
    /// A reconnect replaces the current generation; an older handler may then
    /// finish without disconnecting the replacement.
    pub fn register(&mut self, registration: CameraRegistration) -> Option<u64> {
        if registration.version != iluvatar_core::PROTOCOL_VERSION {
            return None;
        }

        let session_id = self.next_session_id;
        self.next_session_id = self.next_session_id.wrapping_add(1).max(1);
        self.cameras.insert(
            registration.camera_id,
            CameraState::new(registration, session_id),
        );
        Some(session_id)
    }

    /// Mark camera as connected
    pub fn connect(&mut self, camera_id: CameraId) {
        if let Some(state) = self.cameras.get_mut(&camera_id) {
            state.connection = ConnectionState::Connected {
                since: Instant::now(),
            };
        }
    }

    /// Mark a camera disconnected only if this is still its active connection.
    pub fn disconnect(&mut self, camera_id: CameraId, session_id: u64) {
        if let Some(state) = self.cameras.get_mut(&camera_id)
            && state.session_id == session_id
        {
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

    /// Check if a camera is currently connected.
    pub fn is_connected(&self, camera_id: CameraId) -> bool {
        self.cameras
            .get(&camera_id)
            .map(|c| matches!(c.connection, ConnectionState::Connected { .. }))
            .unwrap_or(false)
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

    /// Get camera info with positions for the viewer.
    pub fn viewer_info(&self) -> Vec<crate::websocket::CameraInfo> {
        self.cameras
            .values()
            .map(|c| {
                let pos = c.last_pose.position;
                // Convert quaternion back to [yaw, pitch, roll] degrees.
                let (yaw, pitch, roll) = c.last_pose.orientation.to_euler(EulerRot::YXZ);
                crate::websocket::CameraInfo {
                    camera_id: c.id,
                    name: format!("cam-{}", c.id),
                    position: [
                        pos.longitude as f32,
                        pos.latitude as f32,
                        pos.altitude as f32,
                    ],
                    orientation: Some([yaw.to_degrees(), pitch.to_degrees(), roll.to_degrees()]),
                    connected: matches!(c.connection, ConnectionState::Connected { .. }),
                    fps: c.fps,
                }
            })
            .collect()
    }
}

impl Default for CameraRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, UVec2, Vec2};
    use iluvatar_core::{
        CameraCapabilities, DistortionModel, Fov, GeoPosition, LocalizationStatus,
        PROTOCOL_VERSION, PoseUncertainty,
    };

    fn registration() -> CameraRegistration {
        CameraRegistration {
            version: PROTOCOL_VERSION,
            camera_id: 1,
            intrinsics: CameraIntrinsics {
                focal_length: Vec2::splat(600.0),
                principal_point: Vec2::new(640.0, 360.0),
                resolution: UVec2::new(1280, 720),
                fov: Fov {
                    horizontal: 1.2,
                    vertical: 0.7,
                },
                distortion: DistortionModel::None,
            },
            initial_pose: CameraPose {
                position: GeoPosition::new(47.6, -122.3, 10.0),
                orientation: Quat::IDENTITY,
                timestamp: 0,
                uncertainty: PoseUncertainty::default(),
                status: LocalizationStatus::Nominal,
            },
            capabilities: CameraCapabilities::with_motion_frames(),
        }
    }

    #[test]
    fn stale_connection_cannot_disconnect_replacement() {
        let mut registry = CameraRegistry::new();
        let old_session = registry.register(registration()).unwrap();
        let new_session = registry.register(registration()).unwrap();

        registry.disconnect(1, old_session);
        assert!(registry.is_connected(1));

        registry.disconnect(1, new_session);
        assert!(!registry.is_connected(1));
    }
}
