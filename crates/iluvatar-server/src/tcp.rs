use async_channel::Sender;
use futures_lite::io::AsyncReadExt;
use futures_lite::io::AsyncWriteExt;
use iluvatar_core::{
    CameraMessage, FrameFormat, GridConfigMessage, ServerMessage, ServerPreferences,
    protocol::{self, FrameError, MAX_MESSAGE_SIZE},
};
use parking_lot::RwLock;
use smol::net::{TcpListener, TcpStream};
use std::net::SocketAddr;
use std::sync::Arc;
use thiserror::Error;
use tracing::{debug, error, info, warn};

use crate::{
    camera_mgmt::CameraRegistry,
    validation::{self, ValidationError},
};

#[derive(Debug, Error)]
pub enum TcpError {
    #[error("Failed to bind: {0}")]
    Bind(#[from] std::io::Error),
}

pub struct TcpServer {
    listener: TcpListener,
}

impl TcpServer {
    pub async fn bind(addr: SocketAddr) -> Result<Self, TcpError> {
        let listener = TcpListener::bind(addr).await?;
        info!(addr = %addr, "TCP server bound");
        Ok(Self { listener })
    }

    /// Returns the local address this server is bound to.
    /// Useful for tests that bind to port 0 (ephemeral).
    pub fn local_addr(&self) -> SocketAddr {
        self.listener
            .local_addr()
            .expect("bound listener has local addr")
    }

    /// Accept camera connections and forward messages to the processing
    /// pipeline. Mirrors the QUIC server contract: thin deserialize-and-forward
    /// loop, no heavy computation.
    pub async fn run(
        self,
        msg_tx: Sender<CameraMessage>,
        registry: Arc<RwLock<CameraRegistry>>,
        grid_config: GridConfigMessage,
    ) -> Result<(), TcpError> {
        info!("TCP server running, waiting for camera connections...");

        loop {
            let (stream, remote) = self.listener.accept().await?;
            let msg_tx = msg_tx.clone();
            let registry = registry.clone();
            let grid_config = grid_config.clone();

            smol::spawn(async move {
                match handle_connection(stream, remote, msg_tx, registry, grid_config).await {
                    Ok(()) => debug!("TCP camera disconnected normally"),
                    Err(e) => warn!(remote = %remote, "TCP connection error: {}", e),
                }
            })
            .detach();
        }
    }
}

// ---------------------------------------------------------------------------
// TCP framing: 4-byte big-endian length prefix + postcard payload.
// Identical wire format to QUIC framing, but reads from a TcpStream
// instead of a quinn::RecvStream. Kept here rather than generalising
// protocol.rs — the two callers use different async I/O traits and
// sharing would require a trait bound that adds complexity for no gain.
// ---------------------------------------------------------------------------

/// Read one length-prefixed message from a TCP stream.
async fn read_framed_tcp(stream: &mut TcpStream) -> Result<Vec<u8>, FrameError> {
    let mut len_buf = [0u8; 4];
    stream
        .read_exact(&mut len_buf)
        .await
        .map_err(|_| FrameError::Truncated)?;

    let len = u32::from_be_bytes(len_buf) as usize;
    if len > MAX_MESSAGE_SIZE {
        return Err(FrameError::TooLarge(len));
    }

    let mut buf = vec![0u8; len];
    stream
        .read_exact(&mut buf)
        .await
        .map_err(|_| FrameError::Truncated)?;

    Ok(buf)
}

/// Write one length-prefixed message to a TCP stream.
async fn write_framed_tcp(stream: &mut TcpStream, data: &[u8]) -> Result<(), ConnectionError> {
    let framed = protocol::write_framed(data)?;
    stream
        .write_all(&framed)
        .await
        .map_err(ConnectionError::Io)?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Per-connection handler
// ---------------------------------------------------------------------------

async fn handle_connection(
    mut stream: TcpStream,
    remote: SocketAddr,
    msg_tx: Sender<CameraMessage>,
    registry: Arc<RwLock<CameraRegistry>>,
    grid_config: GridConfigMessage,
) -> Result<(), ConnectionError> {
    info!(remote = %remote, "New TCP camera connection");

    // First message must be Register.
    let reg_data = read_framed_tcp(&mut stream).await?;
    let msg: CameraMessage =
        protocol::deserialize(&reg_data).map_err(ConnectionError::Deserialize)?;

    let registration = match msg {
        CameraMessage::Register(reg) => reg,
        _ => {
            error!(remote = %remote, "Expected Register, got something else");
            return Err(ConnectionError::Protocol("Expected Register message"));
        }
    };

    validation::validate_registration(&registration, grid_config.coordinate_mode)?;

    let camera_id = registration.camera_id;
    let supports_motion = registration.capabilities.motion_frames;
    info!(
        camera_id = camera_id,
        remote = %remote,
        motion_frames = supports_motion,
        "TCP camera registered"
    );

    // Add to registry.
    let session_id = registry
        .write()
        .register(registration.clone())
        .ok_or(ConnectionError::Protocol("Registration was rejected"))?;

    // Respond with preferences and grid config.
    send_registration_response(&mut stream, camera_id, supports_motion, &grid_config).await?;

    // Forward registration to main loop (creates per-camera raymarcher).
    let _ = msg_tx
        .send(CameraMessage::Register(registration.clone()))
        .await;

    // Receive loop: read length-prefixed messages until disconnect.
    loop {
        let frame_data = match read_framed_tcp(&mut stream).await {
            Ok(data) => data,
            Err(FrameError::Truncated) => {
                info!(camera_id = camera_id, "TCP camera disconnected");
                break;
            }
            Err(e) => {
                warn!(camera_id = camera_id, error = %e, "Error reading TCP frame");
                break;
            }
        };

        let msg: CameraMessage = match protocol::deserialize(&frame_data) {
            Ok(m) => m,
            Err(e) => {
                warn!(
                    camera_id = camera_id,
                    error = %e,
                    "Failed to deserialize TCP frame"
                );
                continue;
            }
        };

        if let Err(error) = validation::validate_message(&msg, &registration, &grid_config) {
            warn!(camera_id, %error, "Rejecting invalid camera message");
            break;
        }

        if let Err(e) = msg_tx.try_send(msg) {
            warn!(
                camera_id = camera_id,
                "Channel full, dropping message: {}", e
            );
        }
    }

    // Mark camera as disconnected.
    registry.write().disconnect(camera_id, session_id);
    Ok(())
}

/// Send negotiated preferences for motion cameras or only the grid for
/// contribution cameras. Both modes use protocol version 2.
async fn send_registration_response(
    stream: &mut TcpStream,
    camera_id: u64,
    supports_motion: bool,
    grid_config: &GridConfigMessage,
) -> Result<(), ConnectionError> {
    if supports_motion {
        let prefs = ServerMessage::RegisteredWithPrefs {
            camera_id,
            preferences: ServerPreferences {
                preferred_format: FrameFormat::MotionPixels,
                target_fps: None,
                max_motion_pixels: None,
            },
        };
        let prefs_data = protocol::serialize(&prefs).map_err(ConnectionError::Serialize)?;
        write_framed_tcp(stream, &prefs_data).await?;

        let grid_msg = ServerMessage::GridConfig(grid_config.clone());
        let grid_data = protocol::serialize(&grid_msg).map_err(ConnectionError::Serialize)?;
        write_framed_tcp(stream, &grid_data).await?;

        debug!(
            camera_id = camera_id,
            "Sent RegisteredWithPrefs + GridConfig (TCP)"
        );
    } else {
        let grid_msg = ServerMessage::GridConfig(grid_config.clone());
        let grid_data = protocol::serialize(&grid_msg).map_err(ConnectionError::Serialize)?;
        write_framed_tcp(stream, &grid_data).await?;

        debug!(camera_id = camera_id, "Sent GridConfig (TCP)");
    }

    Ok(())
}

#[derive(Debug, Error)]
enum ConnectionError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Frame error: {0}")]
    Frame(#[from] FrameError),
    #[error("Deserialize error: {0}")]
    Deserialize(postcard::Error),
    #[error("Serialize error: {0}")]
    Serialize(postcard::Error),
    #[error("Validation error: {0}")]
    Validation(#[from] ValidationError),
    #[error("Protocol error: {0}")]
    Protocol(&'static str),
}
