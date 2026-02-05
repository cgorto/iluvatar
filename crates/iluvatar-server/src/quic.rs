use async_channel::Sender;
use async_compat::Compat;
use glam::Vec3;
use iluvatar_core::{
    BoundingBox, CameraFrame, CameraMessage, CameraRegistration, FrameFormat, GeoPosition,
    GridConfigMessage, RaymarchConfig, ServerMessage, ServerPreferences,
    protocol::{self, FrameError},
    raymarch::Raymarcher,
};
use parking_lot::RwLock;
use quinn::{Endpoint, ServerConfig};
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;
use thiserror::Error;
use tracing::{debug, error, info, warn};

use crate::camera_mgmt::CameraRegistry;

#[derive(Debug, Error)]
pub enum QuicError {
    #[error("Failed to bind endpoint: {0}")]
    Bind(#[from] std::io::Error),
    #[error("TLS configuration error: {0}")]
    Tls(String),
    #[error("Connection error: {0}")]
    Connection(#[from] quinn::ConnectionError),
    #[error("Certificate error: {0}")]
    Certificate(String),
}

pub struct QuicServer {
    endpoint: Endpoint,
}

impl QuicServer {
    /// Bind a QUIC server to the given address.
    /// If cert_dir is provided, certificates will be loaded from or saved to that directory.
    pub async fn bind(addr: SocketAddr, cert_dir: Option<PathBuf>) -> Result<Self, QuicError> {
        let (cert_chain, key) = load_or_generate_certs(cert_dir)?;

        let mut server_crypto = rustls::ServerConfig::builder()
            .with_no_client_auth()
            .with_single_cert(cert_chain, key)
            .map_err(|e| QuicError::Tls(e.to_string()))?;

        server_crypto.alpn_protocols = vec![b"iluvatar".to_vec()];

        let server_config = ServerConfig::with_crypto(Arc::new(
            quinn::crypto::rustls::QuicServerConfig::try_from(server_crypto)
                .map_err(|e| QuicError::Tls(e.to_string()))?,
        ));

        // Use async-compat to ensure tokio runtime is available for quinn.
        let endpoint = Compat::new(async move {
            Endpoint::server(server_config, addr)
        })
        .await?;
        info!(addr = %addr, "QUIC server bound");

        Ok(Self { endpoint })
    }

    /// Run the server, accepting camera connections and forwarding messages.
    pub async fn run(
        self,
        msg_tx: Sender<CameraMessage>,
        registry: Arc<RwLock<CameraRegistry>>,
        grid_config: GridConfigMessage,
        raymarch_config: RaymarchConfig,
    ) -> Result<(), QuicError> {
        info!("QUIC server running, waiting for camera connections...");

        // All quinn async operations need tokio runtime via Compat.
        Compat::new(async move {
            while let Some(incoming) = self.endpoint.accept().await {
                let msg_tx = msg_tx.clone();
                let registry = registry.clone();
                let grid_config = grid_config.clone();
                let raymarch_config = raymarch_config.clone();

                // Spawn connection handler inside tokio-compatible context.
                smol::spawn(Compat::new(async move {
                    match handle_connection(
                        incoming,
                        msg_tx,
                        registry,
                        grid_config,
                        raymarch_config,
                    )
                    .await
                    {
                        Ok(()) => debug!("Camera connection closed normally"),
                        Err(e) => warn!("Camera connection error: {}", e),
                    }
                }))
                .detach();
            }
        })
        .await;

        Ok(())
    }
}

/// Build a Raymarcher from a registration's intrinsics and the server's config.
fn create_server_raymarcher(
    registration: &CameraRegistration,
    grid_config: &GridConfigMessage,
    raymarch_config: &RaymarchConfig,
) -> Raymarcher {
    let half_dim = Vec3::new(
        grid_config.dimensions[0] as f32 * grid_config.voxel_size * 0.5,
        grid_config.dimensions[1] as f32 * grid_config.voxel_size * 0.5,
        grid_config.dimensions[2] as f32 * grid_config.voxel_size * 0.5,
    );
    let grid_bounds = BoundingBox::new(-half_dim, half_dim);
    let world_origin = GeoPosition::new(
        grid_config.origin_lat,
        grid_config.origin_lon,
        grid_config.origin_alt,
    );

    // No contribution limit on the server — the grid capacity is the real bound.
    Raymarcher::new(
        registration.intrinsics,
        raymarch_config.clone(),
        grid_bounds,
        grid_config.voxel_size,
        world_origin,
        usize::MAX,
    )
}

async fn handle_connection(
    incoming: quinn::Incoming,
    msg_tx: Sender<CameraMessage>,
    registry: Arc<RwLock<CameraRegistry>>,
    grid_config: GridConfigMessage,
    raymarch_config: RaymarchConfig,
) -> Result<(), ConnectionHandlerError> {
    let connection = incoming.await?;
    let remote = connection.remote_address();
    info!(remote = %remote, "New camera connection");

    // Wait for registration on a bidirectional stream.
    let (mut send, mut recv) = connection.accept_bi().await?;

    // Read registration message.
    let reg_data = protocol::read_framed(&mut recv).await?;
    let msg: CameraMessage =
        protocol::deserialize(&reg_data).map_err(ConnectionHandlerError::Deserialize)?;

    let registration = match msg {
        CameraMessage::Register(reg) => reg,
        _ => {
            error!(remote = %remote, "Expected Register message, got something else");
            return Err(ConnectionHandlerError::Protocol(
                "Expected Register message".into(),
            ));
        }
    };

    let camera_id = registration.camera_id;
    let supports_motion = registration.capabilities.motion_frames;
    info!(
        camera_id = camera_id,
        remote = %remote,
        motion_frames = supports_motion,
        "Camera registered"
    );

    // Add to registry.
    {
        let mut reg = registry.write();
        reg.register(CameraRegistration {
            version: registration.version,
            camera_id: registration.camera_id,
            intrinsics: registration.intrinsics,
            initial_pose: registration.initial_pose,
            capabilities: registration.capabilities.clone(),
        });
    }

    // Decide frame format and create per-connection raymarcher if needed.
    let raymarcher: Option<Raymarcher>;

    if supports_motion {
        // v2 camera: tell it to send MotionPixels, create server-side raymarcher.
        raymarcher = Some(create_server_raymarcher(
            &registration,
            &grid_config,
            &raymarch_config,
        ));

        // Send RegisteredWithPrefs first, then GridConfig.
        let prefs_response = ServerMessage::RegisteredWithPrefs {
            camera_id,
            preferences: ServerPreferences {
                preferred_format: FrameFormat::MotionPixels,
                target_fps: None,
                max_motion_pixels: None,
            },
        };
        let prefs_data =
            protocol::serialize(&prefs_response).map_err(ConnectionHandlerError::Serialize)?;
        let prefs_framed = protocol::write_framed(&prefs_data)?;
        send.write_all(&prefs_framed)
            .await
            .map_err(ConnectionHandlerError::Write)?;

        let grid_response = ServerMessage::GridConfig(grid_config.clone());
        let grid_data =
            protocol::serialize(&grid_response).map_err(ConnectionHandlerError::Serialize)?;
        let grid_framed = protocol::write_framed(&grid_data)?;
        send.write_all(&grid_framed)
            .await
            .map_err(ConnectionHandlerError::Write)?;

        send.finish()
            .map_err(|_| ConnectionHandlerError::Protocol("Failed to finish stream".into()))?;

        debug!(
            camera_id = camera_id,
            "Sent RegisteredWithPrefs + GridConfig (MotionPixels mode)"
        );
    } else {
        // v1 camera: send GridConfig only (existing behavior).
        raymarcher = None;

        let response = ServerMessage::GridConfig(grid_config.clone());
        let response_data =
            protocol::serialize(&response).map_err(ConnectionHandlerError::Serialize)?;
        let framed = protocol::write_framed(&response_data)?;
        send.write_all(&framed)
            .await
            .map_err(ConnectionHandlerError::Write)?;
        send.finish()
            .map_err(|_| ConnectionHandlerError::Protocol("Failed to finish stream".into()))?;

        debug!(
            camera_id = camera_id,
            "Sent GridConfig (VoxelContributions mode)"
        );
    }

    // Forward the registration message to the channel.
    let _ = msg_tx
        .send(CameraMessage::Register(CameraRegistration {
            version: registration.version,
            camera_id: registration.camera_id,
            intrinsics: registration.intrinsics,
            initial_pose: registration.initial_pose,
            capabilities: registration.capabilities.clone(),
        }))
        .await;

    // Main receive loop: accept unidirectional streams for frame data.
    loop {
        let recv_result = connection.accept_uni().await;
        let mut recv = match recv_result {
            Ok(r) => r,
            Err(quinn::ConnectionError::ApplicationClosed(_)) => {
                info!(camera_id = camera_id, "Camera disconnected gracefully");
                break;
            }
            Err(quinn::ConnectionError::ConnectionClosed(_)) => {
                info!(camera_id = camera_id, "Camera connection closed");
                break;
            }
            Err(e) => {
                warn!(camera_id = camera_id, error = %e, "Error accepting stream");
                break;
            }
        };

        // Read frame from unidirectional stream.
        let frame_data = match protocol::read_framed(&mut recv).await {
            Ok(data) => data,
            Err(FrameError::Truncated) => {
                debug!(camera_id = camera_id, "Stream truncated");
                continue;
            }
            Err(e) => {
                warn!(camera_id = camera_id, error = %e, "Error reading frame");
                continue;
            }
        };

        let msg: CameraMessage = match protocol::deserialize(&frame_data) {
            Ok(m) => m,
            Err(e) => {
                warn!(camera_id = camera_id, error = %e, "Failed to deserialize frame");
                continue;
            }
        };

        // If we have a per-connection raymarcher, convert MotionFrames to CameraFrames.
        let msg_to_forward = match (&raymarcher, msg) {
            (Some(rm), CameraMessage::Motion(motion_frame)) => {
                // Server-side raymarching: convert motion pixels to voxel contributions.
                let contributions =
                    rm.raymarch_motion_pixels(&motion_frame.pose, &motion_frame.motion.pixels().collect::<Vec<_>>());

                CameraMessage::Frame(CameraFrame {
                    camera_id: motion_frame.camera_id,
                    sequence: motion_frame.sequence,
                    timestamp: motion_frame.timestamp,
                    pose: motion_frame.pose,
                    contributions,
                })
            }
            (_, other) => {
                // Pass through: v1 CameraFrame, heartbeats, etc.
                other
            }
        };

        // Forward to processing pipeline.
        if let Err(e) = msg_tx.try_send(msg_to_forward) {
            warn!(
                camera_id = camera_id,
                "Channel full, dropping message: {}", e
            );
        }
    }

    // Mark camera as disconnected. Raymarcher is dropped automatically.
    {
        let mut reg = registry.write();
        reg.disconnect(camera_id);
    }

    Ok(())
}

#[derive(Debug, Error)]
enum ConnectionHandlerError {
    #[error("Connection error: {0}")]
    Connection(#[from] quinn::ConnectionError),
    #[error("Write error: {0}")]
    Write(#[from] quinn::WriteError),
    #[error("Frame error: {0}")]
    Frame(#[from] FrameError),
    #[error("Deserialize error: {0}")]
    Deserialize(postcard::Error),
    #[error("Serialize error: {0}")]
    Serialize(postcard::Error),
    #[error("Protocol error: {0}")]
    Protocol(String),
}

fn load_or_generate_certs(
    cert_dir: Option<PathBuf>,
) -> Result<
    (
        Vec<rustls::pki_types::CertificateDer<'static>>,
        rustls::pki_types::PrivateKeyDer<'static>,
    ),
    QuicError,
> {
    let cert_dir = cert_dir.unwrap_or_else(|| {
        let home = std::env::var("HOME").unwrap_or_else(|_| ".".to_string());
        PathBuf::from(home).join(".iluvatar").join("certs")
    });

    let cert_path = cert_dir.join("cert.der");
    let key_path = cert_dir.join("key.der");

    // Try to load existing certificates.
    if cert_path.exists() && key_path.exists() {
        info!(path = %cert_dir.display(), "Loading existing certificates");
        let cert_der = std::fs::read(&cert_path)
            .map_err(|e| QuicError::Certificate(format!("Failed to read cert: {}", e)))?;
        let key_der = std::fs::read(&key_path)
            .map_err(|e| QuicError::Certificate(format!("Failed to read key: {}", e)))?;

        let cert = rustls::pki_types::CertificateDer::from(cert_der);
        let key = rustls::pki_types::PrivateKeyDer::try_from(key_der)
            .map_err(|e| QuicError::Certificate(format!("Failed to parse key: {}", e)))?;

        return Ok((vec![cert], key));
    }

    // Generate new self-signed certificate.
    info!(path = %cert_dir.display(), "Generating new self-signed certificate");
    std::fs::create_dir_all(&cert_dir)
        .map_err(|e| QuicError::Certificate(format!("Failed to create cert dir: {}", e)))?;

    let cert = rcgen::generate_simple_self_signed(vec!["localhost".into(), "iluvatar".into()])
        .map_err(|e| QuicError::Certificate(format!("Failed to generate cert: {}", e)))?;

    let cert_der = cert.cert.der().to_vec();
    let key_der = cert.key_pair.serialize_der();

    // Save to disk.
    std::fs::write(&cert_path, &cert_der)
        .map_err(|e| QuicError::Certificate(format!("Failed to write cert: {}", e)))?;
    std::fs::write(&key_path, &key_der)
        .map_err(|e| QuicError::Certificate(format!("Failed to write key: {}", e)))?;

    info!(
        cert = %cert_path.display(),
        key = %key_path.display(),
        "Certificates saved"
    );

    let cert = rustls::pki_types::CertificateDer::from(cert_der);
    let key = rustls::pki_types::PrivateKeyDer::try_from(key_der)
        .map_err(|e| QuicError::Certificate(format!("Failed to parse generated key: {}", e)))?;

    Ok((vec![cert], key))
}
