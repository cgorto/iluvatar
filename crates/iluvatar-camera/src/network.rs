use async_compat::Compat;
use iluvatar_core::{
    CameraFrame, CameraId, CameraMessage, CameraRegistration, FrameFormat, GridConfigMessage,
    MotionFrame, ServerMessage, protocol,
};
use quinn::{ClientConfig, Connection, Endpoint};
use rustls::pki_types::{CertificateDer, ServerName, UnixTime};
use sha2::{Digest, Sha256};
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tracing::{debug, error, info, warn};

use crate::config::TlsConfig;

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
    #[error("Protocol error: {0}")]
    Protocol(String),
    #[error("Frame error: {0}")]
    Frame(#[from] protocol::FrameError),
    #[error("TLS configuration error: {0}")]
    TlsConfig(String),
}

#[derive(Debug, Clone, Copy)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting { attempts: u32 },
}

/// Response from server registration, containing grid config and format preference.
#[derive(Debug, Clone)]
pub struct RegistrationResponse {
    pub grid_config: GridConfigMessage,
    pub format: FrameFormat,
}

/// Network client for camera-to-server communication over QUIC.
#[derive(Debug)]
pub struct NetworkClient {
    endpoint: Option<Endpoint>,
    connection: Option<Connection>,
    server_addr: SocketAddr,
    server_addr_str: String,
    camera_id: CameraId,
    state: ConnectionState,
    sequence: u64,
    tls_config: TlsConfig,
}

impl NetworkClient {
    /// Create a new network client.
    ///
    /// # Arguments
    /// * `server_addr` - Server address in "host:port" format (supports hostnames and IPs)
    /// * `camera_id` - Unique identifier for this camera
    /// * `tls_config` - TLS configuration for certificate verification
    ///
    /// # Errors
    /// Returns an error if the TLS configuration is invalid.
    pub fn new(
        server_addr: String,
        camera_id: CameraId,
        tls_config: TlsConfig,
    ) -> Result<Self, NetworkError> {
        // Validate TLS configuration
        validate_tls_config(&tls_config)?;

        // Try to parse as SocketAddr directly; if that fails, resolve at connect time
        let addr = server_addr
            .parse::<SocketAddr>()
            .or_else(|_| resolve_hostname(&server_addr))
            .map_err(|e| {
                NetworkError::Connection(format!(
                    "Invalid server address '{}': {}",
                    server_addr, e
                ))
            })?;

        Ok(Self {
            endpoint: None,
            connection: None,
            server_addr: addr,
            server_addr_str: server_addr,
            camera_id,
            state: ConnectionState::Disconnected,
            sequence: 0,
            tls_config,
        })
    }

    pub fn state(&self) -> &ConnectionState {
        &self.state
    }

    pub fn is_connected(&self) -> bool {
        matches!(self.state, ConnectionState::Connected)
    }

    /// Connect to the server using QUIC.
    pub async fn connect(&mut self) -> Result<(), NetworkError> {
        self.state = ConnectionState::Connecting;

        // Re-resolve hostname on each connection attempt (supports DNS changes/Docker)
        if self.server_addr_str.parse::<SocketAddr>().is_err() {
            match resolve_hostname(&self.server_addr_str) {
                Ok(addr) => {
                    if addr != self.server_addr {
                        info!(old = %self.server_addr, new = %addr, "Server address resolved to new IP");
                    }
                    self.server_addr = addr;
                }
                Err(e) => {
                    return Err(NetworkError::Connection(format!(
                        "Failed to resolve '{}': {}",
                        self.server_addr_str, e
                    )));
                }
            }
        }

        // Create endpoint if we don't have one (needs tokio runtime via Compat)
        if self.endpoint.is_none() {
            let client_config = configure_client(&self.tls_config)?;
            let endpoint = Compat::new(async move {
                let mut ep = Endpoint::client("0.0.0.0:0".parse().unwrap())?;
                ep.set_default_client_config(client_config);
                Ok::<_, std::io::Error>(ep)
            })
            .await
            .map_err(|e| NetworkError::Connection(format!("Failed to create endpoint: {}", e)))?;
            self.endpoint = Some(endpoint);
        }

        let endpoint = self.endpoint.as_ref().unwrap().clone();
        let server_addr = self.server_addr;

        // Connect to server (needs tokio runtime via Compat)
        let connection = Compat::new(async move {
            let connecting = endpoint
                .connect(server_addr, "iluvatar")
                .map_err(|e| format!("Failed to initiate connection: {}", e))?;
            connecting
                .await
                .map_err(|e| format!("Connection failed: {}", e))
        })
        .await
        .map_err(NetworkError::Connection)?;

        info!(
            remote = %connection.remote_address(),
            "Connected to server"
        );

        self.connection = Some(connection);
        self.state = ConnectionState::Connected;
        Ok(())
    }

    /// Send camera registration and receive grid configuration and format preference.
    ///
    /// The server may respond with either:
    /// - `ServerMessage::GridConfig` (v1 server): defaults to VoxelContributions format
    /// - `ServerMessage::RegisteredWithPrefs` (v2 server): includes format preference
    ///
    /// Both responses are normalized into a `RegistrationResponse`.
    pub async fn register(
        &mut self,
        mut registration: CameraRegistration,
    ) -> Result<RegistrationResponse, NetworkError> {
        registration.version = protocol::PROTOCOL_VERSION;

        let conn = self
            .connection
            .as_ref()
            .ok_or_else(|| NetworkError::Connection("Not connected".into()))?;

        // Open bidirectional stream for registration.
        let (mut send, mut recv) = conn
            .open_bi()
            .await
            .map_err(|e| NetworkError::Connection(format!("Failed to open stream: {}", e)))?;

        // Send registration message.
        let msg = CameraMessage::Register(registration);
        let data = protocol::serialize(&msg)?;
        let framed = protocol::write_framed(&data)?;

        send.write_all(&framed)
            .await
            .map_err(|e| NetworkError::Send(format!("Failed to write registration: {}", e)))?;
        send.finish()
            .map_err(|_| NetworkError::Send("Failed to finish send stream".into()))?;

        debug!("Registration sent, waiting for server response...");

        // Receive response (may be GridConfig or RegisteredWithPrefs).
        let response_data = protocol::read_framed(&mut recv).await?;
        let response: ServerMessage = protocol::deserialize(&response_data)?;

        match response {
            ServerMessage::GridConfig(config) => {
                // v1 server: no format preference, default to VoxelContributions.
                info!(
                    origin = format!("{:.6}, {:.6}", config.origin_lat, config.origin_lon),
                    dimensions = ?config.dimensions,
                    voxel_size = config.voxel_size,
                    format = ?FrameFormat::VoxelContributions,
                    "Received grid configuration (v1 response)"
                );
                Ok(RegistrationResponse {
                    grid_config: config,
                    format: FrameFormat::VoxelContributions,
                })
            }
            ServerMessage::RegisteredWithPrefs { camera_id: _, preferences } => {
                // v2 server: read a second message for grid config.
                let grid_data = protocol::read_framed(&mut recv).await?;
                let grid_msg: ServerMessage = protocol::deserialize(&grid_data)?;

                let config = match grid_msg {
                    ServerMessage::GridConfig(config) => config,
                    _ => {
                        return Err(NetworkError::Protocol(
                            "Expected GridConfig after RegisteredWithPrefs".into(),
                        ));
                    }
                };

                info!(
                    origin = format!("{:.6}, {:.6}", config.origin_lat, config.origin_lon),
                    dimensions = ?config.dimensions,
                    voxel_size = config.voxel_size,
                    format = ?preferences.preferred_format,
                    "Received grid configuration (v2 response)"
                );
                Ok(RegistrationResponse {
                    grid_config: config,
                    format: preferences.preferred_format,
                })
            }
            ServerMessage::Error { code, message } => Err(NetworkError::Protocol(format!(
                "Server error {}: {}",
                code, message
            ))),
            _ => Err(NetworkError::Protocol(
                "Unexpected response to registration".into(),
            )),
        }
    }

    /// Send a frame to the server using a unidirectional stream.
    pub async fn send_frame(&mut self, mut frame: CameraFrame) -> Result<(), NetworkError> {
        let conn = self
            .connection
            .as_ref()
            .ok_or_else(|| NetworkError::Connection("Not connected".into()))?;

        frame.sequence = self.sequence;
        self.sequence += 1;

        let msg = CameraMessage::Frame(frame);
        let data = protocol::serialize(&msg)?;
        let framed = protocol::write_framed(&data)?;

        // Open unidirectional stream (fire-and-forget)
        let mut send = conn
            .open_uni()
            .await
            .map_err(|e| NetworkError::Send(format!("Failed to open stream: {}", e)))?;

        send.write_all(&framed)
            .await
            .map_err(|e| NetworkError::Send(format!("Failed to write frame: {}", e)))?;

        send.finish()
            .map_err(|_| NetworkError::Send("Failed to finish stream".into()))?;

        Ok(())
    }

    /// Send a motion frame to the server using a unidirectional stream.
    pub async fn send_motion_frame(
        &mut self,
        mut frame: MotionFrame,
    ) -> Result<(), NetworkError> {
        let conn = self
            .connection
            .as_ref()
            .ok_or_else(|| NetworkError::Connection("Not connected".into()))?;

        frame.sequence = self.sequence;
        self.sequence += 1;

        let msg = CameraMessage::Motion(frame);
        let data = protocol::serialize(&msg)?;
        let framed = protocol::write_framed(&data)?;

        // Open unidirectional stream (fire-and-forget).
        let mut send = conn
            .open_uni()
            .await
            .map_err(|e| NetworkError::Send(format!("Failed to open stream: {}", e)))?;

        send.write_all(&framed)
            .await
            .map_err(|e| NetworkError::Send(format!("Failed to write motion frame: {}", e)))?;

        send.finish()
            .map_err(|_| NetworkError::Send("Failed to finish stream".into()))?;

        Ok(())
    }

    /// Send heartbeat to keep connection alive.
    pub async fn send_heartbeat(&mut self) -> Result<(), NetworkError> {
        let conn = self
            .connection
            .as_ref()
            .ok_or_else(|| NetworkError::Connection("Not connected".into()))?;

        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;

        let msg = CameraMessage::Heartbeat {
            camera_id: self.camera_id,
            timestamp,
        };
        let data = protocol::serialize(&msg)?;
        let framed = protocol::write_framed(&data)?;

        let mut send = conn
            .open_uni()
            .await
            .map_err(|e| NetworkError::Send(format!("Failed to open stream: {}", e)))?;

        send.write_all(&framed)
            .await
            .map_err(|e| NetworkError::Send(format!("Failed to write heartbeat: {}", e)))?;

        send.finish()
            .map_err(|_| NetworkError::Send("Failed to finish stream".into()))?;

        Ok(())
    }

    /// Attempt to reconnect with exponential backoff.
    ///
    /// # Arguments
    /// * `max_attempts` - Maximum number of reconnection attempts. None means unlimited.
    /// * `timeout` - Maximum total time to spend reconnecting. None means unlimited.
    ///
    /// # Returns
    /// Ok(()) on successful reconnection, Err if limits exceeded or unrecoverable error.
    pub async fn reconnect_with_backoff(
        &mut self,
        max_attempts: Option<u32>,
        timeout: Option<Duration>,
    ) -> Result<(), NetworkError> {
        const MIN_DELAY: Duration = Duration::from_millis(100);
        const MAX_DELAY: Duration = Duration::from_secs(30);
        const JITTER_FACTOR: f64 = 0.3;

        let start = std::time::Instant::now();
        let mut attempts = 0u32;
        let mut delay = MIN_DELAY;

        loop {
            // Check attempt limit
            if let Some(max) = max_attempts
                && attempts >= max
            {
                return Err(NetworkError::Connection(format!(
                    "Max reconnect attempts ({}) exceeded",
                    max
                )));
            }

            // Check timeout
            if let Some(t) = timeout
                && start.elapsed() > t
            {
                return Err(NetworkError::Connection(format!(
                    "Reconnect timeout ({:?}) exceeded after {} attempts",
                    t, attempts
                )));
            }

            self.state = ConnectionState::Reconnecting { attempts };
            attempts += 1;

            // Close existing connection
            if let Some(conn) = self.connection.take() {
                conn.close(0u32.into(), b"reconnecting");
            }

            warn!(
                attempt = attempts,
                max_attempts = ?max_attempts,
                elapsed_secs = start.elapsed().as_secs(),
                delay_ms = delay.as_millis(),
                "Attempting to reconnect..."
            );

            // Add jitter
            let jitter = (rand_jitter() * JITTER_FACTOR * delay.as_secs_f64()).max(0.0);
            let jittered_delay = delay + Duration::from_secs_f64(jitter);

            smol::Timer::after(jittered_delay).await;

            match self.connect().await {
                Ok(()) => {
                    info!(attempts = attempts, "Reconnected successfully");
                    return Ok(());
                }
                Err(e) => {
                    warn!(error = %e, "Reconnection attempt failed");
                    delay = (delay * 2).min(MAX_DELAY);
                }
            }
        }
    }

    /// Close the connection gracefully.
    pub fn close(&mut self) {
        if let Some(conn) = self.connection.take() {
            conn.close(0u32.into(), b"shutdown");
        }
        self.state = ConnectionState::Disconnected;
    }

    /// Check if the connection is still alive.
    pub fn check_connection(&self) -> bool {
        if let Some(conn) = &self.connection {
            conn.close_reason().is_none()
        } else {
            false
        }
    }
}

/// Resolve a hostname:port string to a SocketAddr.
///
/// Supports both `IP:port` and `hostname:port` formats. For hostnames,
/// uses the system resolver (which handles Docker service names, /etc/hosts, DNS, etc.).
fn resolve_hostname(addr: &str) -> Result<SocketAddr, String> {
    use std::net::ToSocketAddrs;
    addr.to_socket_addrs()
        .map_err(|e| format!("DNS resolution failed for '{}': {}", addr, e))?
        .next()
        .ok_or_else(|| format!("No addresses found for '{}'", addr))
}

/// Validate TLS configuration.
fn validate_tls_config(config: &TlsConfig) -> Result<(), NetworkError> {
    // Check fingerprint format if provided
    if let Some(ref fp) = config.certificate_fingerprint {
        let normalized = fp.replace(':', "").to_lowercase();
        if normalized.len() != 64 {
            return Err(NetworkError::TlsConfig(format!(
                "Certificate fingerprint must be 64 hex characters (SHA-256), got {}",
                normalized.len()
            )));
        }
        if !normalized.chars().all(|c| c.is_ascii_hexdigit()) {
            return Err(NetworkError::TlsConfig(
                "Certificate fingerprint must contain only hex characters".into(),
            ));
        }
    }

    // Warn if skipping verification
    if config.dangerous_skip_verification {
        warn!(
            "⚠️  TLS CERTIFICATE VERIFICATION IS DISABLED! This is a critical security risk. \
             Do NOT use this in production. Configure certificate_fingerprint or ca_cert_path instead."
        );
    }

    // Require some form of verification in non-skip mode
    if !config.dangerous_skip_verification
        && config.certificate_fingerprint.is_none()
        && config.ca_cert_path.is_none()
    {
        return Err(NetworkError::TlsConfig(
            "TLS configuration requires either certificate_fingerprint, ca_cert_path, \
             or dangerous_skip_verification (not recommended). See config documentation."
                .into(),
        ));
    }

    Ok(())
}

/// Configure TLS client based on the provided TLS configuration.
fn configure_client(tls_config: &TlsConfig) -> Result<ClientConfig, NetworkError> {
    let crypto = if tls_config.dangerous_skip_verification {
        // Insecure mode for development only
        warn!("Using insecure TLS configuration - skipping all certificate verification");
        rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(Arc::new(SkipServerVerification))
            .with_no_client_auth()
    } else {
        // Secure mode with proper verification
        let verifier = build_certificate_verifier(tls_config)?;
        rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(verifier)
            .with_no_client_auth()
    };

    let mut crypto = crypto;
    // Set ALPN protocol to match server
    crypto.alpn_protocols = vec![b"iluvatar".to_vec()];

    let mut config = ClientConfig::new(Arc::new(
        quinn::crypto::rustls::QuicClientConfig::try_from(crypto)
            .expect("Failed to create QUIC client config"),
    ));

    // Configure transport
    let mut transport = quinn::TransportConfig::default();
    transport.max_idle_timeout(Some(Duration::from_secs(60).try_into().unwrap()));
    transport.keep_alive_interval(Some(Duration::from_secs(15)));
    config.transport_config(Arc::new(transport));

    Ok(config)
}

/// Build a certificate verifier based on configuration.
fn build_certificate_verifier(
    tls_config: &TlsConfig,
) -> Result<Arc<dyn rustls::client::danger::ServerCertVerifier>, NetworkError> {
    // Parse fingerprint if provided
    let fingerprint = if let Some(ref fp) = tls_config.certificate_fingerprint {
        let normalized = fp.replace(':', "").to_lowercase();
        let bytes = hex::decode(&normalized).map_err(|e| {
            NetworkError::TlsConfig(format!("Invalid certificate fingerprint hex: {}", e))
        })?;
        let mut arr = [0u8; 32];
        arr.copy_from_slice(&bytes);
        Some(arr)
    } else {
        None
    };

    // Load CA certificate if provided
    let ca_certs = if let Some(ref ca_path) = tls_config.ca_cert_path {
        let ca_data = std::fs::read(ca_path)
            .map_err(|e| NetworkError::TlsConfig(format!("Failed to read CA cert '{}': {}", ca_path, e)))?;
        
        let certs: Vec<CertificateDer<'static>> = if ca_path.ends_with(".pem") || ca_path.ends_with(".crt") {
            rustls_pemfile::certs(&mut ca_data.as_slice())
                .filter_map(|c| c.ok())
                .collect()
        } else {
            // Assume DER format
            vec![CertificateDer::from(ca_data)]
        };

        if certs.is_empty() {
            return Err(NetworkError::TlsConfig(format!(
                "No valid certificates found in CA file: {}",
                ca_path
            )));
        }

        info!(ca_path = %ca_path, cert_count = certs.len(), "Loaded CA certificates");
        Some(certs)
    } else {
        None
    };

    Ok(Arc::new(PinnedCertificateVerifier {
        expected_fingerprint: fingerprint,
        ca_certs,
    }))
}

/// Certificate verifier that performs pinning and/or CA chain verification.
#[derive(Debug)]
struct PinnedCertificateVerifier {
    /// Expected SHA-256 fingerprint of the server certificate (optional).
    expected_fingerprint: Option<[u8; 32]>,
    /// CA certificates for chain verification (optional).
    ca_certs: Option<Vec<CertificateDer<'static>>>,
}

impl rustls::client::danger::ServerCertVerifier for PinnedCertificateVerifier {
    fn verify_server_cert(
        &self,
        end_entity: &CertificateDer<'_>,
        intermediates: &[CertificateDer<'_>],
        server_name: &ServerName<'_>,
        _ocsp_response: &[u8],
        _now: UnixTime,
    ) -> Result<rustls::client::danger::ServerCertVerified, rustls::Error> {
        // Verify fingerprint if configured
        if let Some(expected) = &self.expected_fingerprint {
            let actual = Sha256::digest(end_entity.as_ref());
            if actual.as_slice() != expected {
                error!(
                    expected = %hex::encode(expected),
                    actual = %hex::encode(actual),
                    "Server certificate fingerprint mismatch"
                );
                return Err(rustls::Error::General(
                    "Server certificate fingerprint does not match configured value".into(),
                ));
            }
            debug!(
                fingerprint = %hex::encode(expected),
                "Server certificate fingerprint verified"
            );
        }

        // Verify CA chain if configured
        if let Some(ref ca_certs) = self.ca_certs {
            verify_certificate_chain(end_entity, intermediates, ca_certs, server_name)?;
            debug!("Server certificate chain verified against configured CA");
        }

        Ok(rustls::client::danger::ServerCertVerified::assertion())
    }

    fn verify_tls12_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls12_signature(
            message,
            cert,
            dss,
            &rustls::crypto::ring::default_provider().signature_verification_algorithms,
        )
    }

    fn verify_tls13_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls13_signature(
            message,
            cert,
            dss,
            &rustls::crypto::ring::default_provider().signature_verification_algorithms,
        )
    }

    fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
        rustls::crypto::ring::default_provider()
            .signature_verification_algorithms
            .supported_schemes()
    }
}

/// Verify a certificate chain against trusted CA certificates.
fn verify_certificate_chain(
    end_entity: &CertificateDer<'_>,
    intermediates: &[CertificateDer<'_>],
    ca_certs: &[CertificateDer<'static>],
    server_name: &ServerName<'_>,
) -> Result<(), rustls::Error> {
    use rustls::RootCertStore;
    use rustls::client::WebPkiServerVerifier;
    use rustls::client::danger::ServerCertVerifier;

    // Build root cert store from CA certs
    let mut root_store = RootCertStore::empty();
    for ca_cert in ca_certs {
        root_store.add(ca_cert.clone()).map_err(|e| {
            rustls::Error::General(format!("Failed to add CA certificate: {}", e))
        })?;
    }

    // Create a WebPKI verifier
    let verifier = WebPkiServerVerifier::builder(Arc::new(root_store))
        .build()
        .map_err(|e| rustls::Error::General(format!("Failed to build verifier: {}", e)))?;

    // Verify using WebPKI
    verifier.verify_server_cert(
        end_entity,
        intermediates,
        server_name,
        &[],
        UnixTime::now(),
    )?;

    Ok(())
}

/// Custom certificate verifier that skips verification (for development only).
/// WARNING: Using this in production is a critical security vulnerability!
#[derive(Debug)]
struct SkipServerVerification;

impl rustls::client::danger::ServerCertVerifier for SkipServerVerification {
    fn verify_server_cert(
        &self,
        _end_entity: &CertificateDer<'_>,
        _intermediates: &[CertificateDer<'_>],
        _server_name: &ServerName<'_>,
        _ocsp_response: &[u8],
        _now: UnixTime,
    ) -> Result<rustls::client::danger::ServerCertVerified, rustls::Error> {
        Ok(rustls::client::danger::ServerCertVerified::assertion())
    }

    fn verify_tls12_signature(
        &self,
        _message: &[u8],
        _cert: &CertificateDer<'_>,
        _dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        Ok(rustls::client::danger::HandshakeSignatureValid::assertion())
    }

    fn verify_tls13_signature(
        &self,
        _message: &[u8],
        _cert: &CertificateDer<'_>,
        _dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        Ok(rustls::client::danger::HandshakeSignatureValid::assertion())
    }

    fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
        vec![
            rustls::SignatureScheme::RSA_PKCS1_SHA256,
            rustls::SignatureScheme::RSA_PKCS1_SHA384,
            rustls::SignatureScheme::RSA_PKCS1_SHA512,
            rustls::SignatureScheme::ECDSA_NISTP256_SHA256,
            rustls::SignatureScheme::ECDSA_NISTP384_SHA384,
            rustls::SignatureScheme::ECDSA_NISTP521_SHA512,
            rustls::SignatureScheme::RSA_PSS_SHA256,
            rustls::SignatureScheme::RSA_PSS_SHA384,
            rustls::SignatureScheme::RSA_PSS_SHA512,
            rustls::SignatureScheme::ED25519,
        ]
    }
}

/// Generate a random jitter value between -0.5 and 0.5.
fn rand_jitter() -> f64 {
    fastrand::f64() - 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a TLS config for testing that skips verification.
    fn test_tls_config() -> TlsConfig {
        TlsConfig {
            dangerous_skip_verification: true,
            certificate_fingerprint: None,
            ca_cert_path: None,
        }
    }

    /// Create a TLS config with fingerprint pinning for testing.
    fn test_tls_config_with_fingerprint(fingerprint: &str) -> TlsConfig {
        TlsConfig {
            dangerous_skip_verification: false,
            certificate_fingerprint: Some(fingerprint.to_string()),
            ca_cert_path: None,
        }
    }

    #[test]
    fn test_network_client_new_valid_address() {
        let result = NetworkClient::new("127.0.0.1:8080".to_string(), 1, test_tls_config());
        assert!(result.is_ok());
        let client = result.unwrap();
        assert_eq!(client.server_addr, "127.0.0.1:8080".parse().unwrap());
        assert!(matches!(client.state, ConnectionState::Disconnected));
    }

    #[test]
    fn test_network_client_new_invalid_address() {
        let result = NetworkClient::new("not-a-valid-address".to_string(), 1, test_tls_config());
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, NetworkError::Connection(_)));
    }

    #[test]
    fn test_network_client_new_hostname() {
        // Hostnames with ports should be resolved (localhost always resolves)
        let result = NetworkClient::new("localhost:8080".to_string(), 1, test_tls_config());
        assert!(result.is_ok());
        let client = result.unwrap();
        assert_eq!(client.server_addr_str, "localhost:8080");
    }

    #[test]
    fn test_network_client_new_ipv6() {
        let result = NetworkClient::new("[::1]:8080".to_string(), 1, test_tls_config());
        assert!(result.is_ok());
    }

    #[test]
    fn test_connection_state_transitions() {
        let mut client = NetworkClient::new("127.0.0.1:8080".to_string(), 1, test_tls_config()).unwrap();

        // Initial state
        assert!(matches!(client.state, ConnectionState::Disconnected));

        // Can't check Connected without a real server, but we can check the state field
        client.state = ConnectionState::Connected;
        assert!(client.is_connected());

        client.state = ConnectionState::Reconnecting { attempts: 5 };
        assert!(!client.is_connected());
        if let ConnectionState::Reconnecting { attempts } = client.state {
            assert_eq!(attempts, 5);
        } else {
            panic!("Expected Reconnecting state");
        }
    }

    #[test]
    fn test_check_connection_no_connection() {
        let client = NetworkClient::new("127.0.0.1:8080".to_string(), 1, test_tls_config()).unwrap();
        assert!(!client.check_connection());
    }

    #[test]
    fn test_close_without_connection() {
        let mut client = NetworkClient::new("127.0.0.1:8080".to_string(), 1, test_tls_config()).unwrap();
        // Should not panic
        client.close();
        assert!(matches!(client.state, ConnectionState::Disconnected));
    }

    #[test]
    fn test_rand_jitter_range() {
        // Test that jitter is in expected range
        for _ in 0..100 {
            let jitter = rand_jitter();
            assert!(
                jitter >= -0.5 && jitter <= 0.5,
                "jitter {} out of range",
                jitter
            );
        }
    }

    #[test]
    fn test_tls_config_no_verification_method_fails() {
        let config = TlsConfig {
            dangerous_skip_verification: false,
            certificate_fingerprint: None,
            ca_cert_path: None,
        };
        let result = NetworkClient::new("127.0.0.1:8080".to_string(), 1, config);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, NetworkError::TlsConfig(_)));
    }

    #[test]
    fn test_tls_config_valid_fingerprint() {
        let valid_fingerprint = "a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2";
        let result = NetworkClient::new(
            "127.0.0.1:8080".to_string(),
            1,
            test_tls_config_with_fingerprint(valid_fingerprint),
        );
        assert!(result.is_ok());
    }

    #[test]
    fn test_tls_config_invalid_fingerprint_length() {
        let invalid_fingerprint = "a1b2c3d4"; // Too short
        let result = NetworkClient::new(
            "127.0.0.1:8080".to_string(),
            1,
            test_tls_config_with_fingerprint(invalid_fingerprint),
        );
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, NetworkError::TlsConfig(_)));
    }

    #[test]
    fn test_tls_config_invalid_fingerprint_chars() {
        let invalid_fingerprint = "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz";
        let result = NetworkClient::new(
            "127.0.0.1:8080".to_string(),
            1,
            test_tls_config_with_fingerprint(invalid_fingerprint),
        );
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, NetworkError::TlsConfig(_)));
    }

    #[test]
    fn test_tls_config_fingerprint_with_colons() {
        // Fingerprints with colons should be accepted and normalized
        let fingerprint_with_colons = "a1:b2:c3:d4:e5:f6:a1:b2:c3:d4:e5:f6:a1:b2:c3:d4:e5:f6:a1:b2:c3:d4:e5:f6:a1:b2:c3:d4:e5:f6:a1:b2";
        let result = NetworkClient::new(
            "127.0.0.1:8080".to_string(),
            1,
            test_tls_config_with_fingerprint(fingerprint_with_colons),
        );
        assert!(result.is_ok());
    }
}
