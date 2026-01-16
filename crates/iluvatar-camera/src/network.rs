use iluvatar_core::{
    CameraFrame, CameraId, CameraMessage, CameraRegistration, ServerMessage, protocol,
};
use std::time::{SystemTime, UNIX_EPOCH};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum NetworkError {
    #[error("Connection failed: {0}")]
    Connection(String),
    #[error("Send failed: {0}")]
    Send(String),
    #[error("Receive failed: {0}")]
    Receive(String),
    #[error("Serialization error: {0}")]
    Serialization(#[from] postcard::Error),
}

/// QUIC client connection state
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting { attempts: u32 },
}

fn now_micros() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}

/// Network client for camera-to-server communication
pub struct NetworkClient {
    server_addr: String,
    camera_id: CameraId,
    state: ConnectionState,
    sequence: u64,
}

impl NetworkClient {
    pub fn new(server_addr: String, camera_id: CameraId) -> Self {
        Self {
            server_addr,
            camera_id,
            state: ConnectionState::Disconnected,
            sequence: 0,
        }
    }

    pub fn state(&self) -> &ConnectionState {
        &self.state
    }

    pub fn is_connected(&self) -> bool {
        matches!(self.state, ConnectionState::Connected)
    }

    /// Connect to the server (async placeholder)
    pub async fn connect(&mut self) -> Result<(), NetworkError> {
        // TODO: Implement QUIC connection using quinn
        self.state = ConnectionState::Connecting;

        // Placeholder: pretend we connected
        self.state = ConnectionState::Connected;
        Ok(())
    }

    /// Send camera registration
    pub async fn register(
        &mut self,
        mut registration: CameraRegistration,
    ) -> Result<(), NetworkError> {
        registration.version = protocol::PROTOCOL_VERSION;
        let msg = CameraMessage::Register(registration);
        self.send_message(&msg).await
    }

    /// Send a frame to the server
    pub async fn send_frame(&mut self, frame: CameraFrame) -> Result<(), NetworkError> {
        self.sequence += 1;
        let msg = CameraMessage::Frame(frame);
        self.send_message(&msg).await
    }

    /// Send heartbeat
    pub async fn send_heartbeat(&mut self) -> Result<(), NetworkError> {
        let msg = CameraMessage::Heartbeat {
            camera_id: self.camera_id,
            timestamp: now_micros(),
        };
        self.send_message(&msg).await
    }

    async fn send_message(&self, msg: &CameraMessage) -> Result<(), NetworkError> {
        let _bytes = protocol::serialize(msg)?;
        // TODO: Actually send over QUIC
        Ok(())
    }

    /// Receive a message from the server
    pub async fn receive(&mut self) -> Result<ServerMessage, NetworkError> {
        // TODO: Actually receive from QUIC
        Err(NetworkError::Receive("Not implemented".to_string()))
    }
}
