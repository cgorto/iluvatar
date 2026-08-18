use iluvatar_core::{
    BoundingBox, CameraStateInfo, ClientUpdate, SnapshotMessage, SystemStatusMessage,
    TrackedObject, UpdateMessage,
};
use smol::io::{AsyncReadExt, AsyncWriteExt};
use smol::net::TcpStream;
use std::pin::Pin;
use std::task::{Context, Poll};
use std::time::Instant;

/// Embedded HTML viewer — served to browsers on plain HTTP GET.
const INDEX_HTML: &str = include_str!("viewer.html");

// ============================================================================
// Client State
// ============================================================================

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

// ============================================================================
// Message Builders
// ============================================================================

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

// ============================================================================
// Broadcast Message
// ============================================================================

/// Camera info for the viewer overlay.
#[derive(Debug, Clone, serde::Serialize)]
pub struct CameraInfo {
    pub camera_id: u64,
    pub name: String,
    pub position: [f32; 3],
    /// Orientation as [yaw, pitch, roll] in degrees. Null if unknown.
    pub orientation: Option<[f32; 3]>,
    pub connected: bool,
    pub fps: f32,
}

/// Message to broadcast to all connected clients
#[derive(Debug, Clone)]
pub struct BroadcastMessage {
    pub timestamp: u64,
    pub objects: Vec<TrackedObject>,
    pub cameras: Vec<CameraInfo>,
    /// Active voxel positions and intensities for grid visualization.
    /// Each entry is [x, y, z, intensity].
    pub voxels: Vec<[f32; 4]>,
    pub camera_count: u32,
    pub active_voxels: u64,
}

// ============================================================================
// ReplayStream — feeds buffered bytes then delegates to inner stream
// ============================================================================

/// A stream wrapper that replays buffered bytes before reading from the
/// underlying stream. Used to re-feed the HTTP request to tungstenite
/// after we have already read it to distinguish HTTP from WebSocket.
struct ReplayStream {
    buffer: Vec<u8>,
    position: usize,
    inner: TcpStream,
}

impl ReplayStream {
    fn new(buffer: Vec<u8>, inner: TcpStream) -> Self {
        Self {
            buffer,
            position: 0,
            inner,
        }
    }
}

impl smol::io::AsyncRead for ReplayStream {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut [u8],
    ) -> Poll<std::io::Result<usize>> {
        // Drain the replay buffer first.
        if self.position < self.buffer.len() {
            let remaining = &self.buffer[self.position..];
            let count = remaining.len().min(buf.len());
            buf[..count].copy_from_slice(&remaining[..count]);
            self.position += count;
            return Poll::Ready(Ok(count));
        }
        // Buffer exhausted — read from the real stream.
        Pin::new(&mut self.inner).poll_read(cx, buf)
    }
}

impl smol::io::AsyncWrite for ReplayStream {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<std::io::Result<usize>> {
        Pin::new(&mut self.inner).poll_write(cx, buf)
    }

    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        Pin::new(&mut self.inner).poll_flush(cx)
    }

    fn poll_close(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        Pin::new(&mut self.inner).poll_close(cx)
    }
}

// Unpin is safe because all fields are Unpin (Vec, usize, TcpStream).
impl Unpin for ReplayStream {}

// ============================================================================
// Server
// ============================================================================

/// Maximum size of an HTTP request we will buffer (8 KB).
/// Any request larger than this is rejected.
const MAX_REQUEST_SIZE: usize = 8192;

/// Read bytes from the stream until we see the HTTP header terminator
/// `\r\n\r\n`. Returns the raw request bytes including the terminator.
async fn read_http_request(stream: &mut TcpStream) -> std::io::Result<Vec<u8>> {
    let mut buf = Vec::with_capacity(2048);
    let mut tmp = [0u8; 1024];

    loop {
        let n = stream.read(&mut tmp).await?;
        if n == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "connection closed before headers complete",
            ));
        }
        buf.extend_from_slice(&tmp[..n]);

        // Check for end-of-headers marker.
        if buf.windows(4).any(|w| w == b"\r\n\r\n") {
            return Ok(buf);
        }
        if buf.len() > MAX_REQUEST_SIZE {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "HTTP request too large",
            ));
        }
    }
}

/// Check if an HTTP request contains a WebSocket upgrade header.
fn is_websocket_upgrade(request: &[u8]) -> bool {
    // Case-insensitive search for "upgrade: websocket" in the headers.
    let lowercase = request.to_ascii_lowercase();
    lowercase
        .windows(b"upgrade: websocket".len())
        .any(|w| w == b"upgrade: websocket")
}

/// Serve the embedded HTML viewer over plain HTTP.
async fn serve_html(mut stream: TcpStream) -> std::io::Result<()> {
    let response = format!(
        "HTTP/1.1 200 OK\r\n\
         Content-Type: text/html; charset=utf-8\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         Cache-Control: no-cache\r\n\
         \r\n\
         {}",
        INDEX_HTML.len(),
        INDEX_HTML,
    );
    stream.write_all(response.as_bytes()).await?;
    stream.flush().await?;
    Ok(())
}

/// Main WebSocket server loop.
///
/// Serves two roles on a single port:
/// - Plain HTTP GET → embedded HTML viewer
/// - WebSocket upgrade → real-time data stream
pub async fn run_server(
    port: u16,
    receiver: async_channel::Receiver<BroadcastMessage>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let addr = format!("0.0.0.0:{}", port);
    let listener = smol::net::TcpListener::bind(&addr).await?;
    tracing::info!("HTTP + WebSocket server listening on {}", addr);

    let clients: std::sync::Arc<dashmap::DashMap<u64, async_channel::Sender<String>>> =
        std::sync::Arc::new(dashmap::DashMap::new());

    // Spawn the broadcaster task.
    let clients_clone = clients.clone();
    smol::spawn(async move {
        while let Ok(msg) = receiver.recv().await {
            // Build a flat JSON object with objects, cameras, and voxels for the viewer.
            let payload = serde_json::json!({
                "Update": {
                    "timestamp": msg.timestamp,
                    "objects": msg.objects,
                },
                "cameras": msg.cameras,
                "voxels": msg.voxels,
                "camera_count": msg.camera_count,
                "active_voxels": msg.active_voxels,
            });
            if let Ok(json) = serde_json::to_string(&payload) {
                for client in clients_clone.iter() {
                    let _ = client.value().try_send(json.clone());
                }
            }
        }
    })
    .detach();

    let mut client_id_seq: u64 = 0;

    loop {
        let (stream, addr) = listener.accept().await?;
        let client_id = client_id_seq;
        client_id_seq += 1;
        let clients_map = clients.clone();

        smol::spawn(async move {
            if let Err(e) = handle_connection(stream, addr, client_id, clients_map).await {
                tracing::warn!(
                    client_id,
                    %addr,
                    "Connection error: {}",
                    e,
                );
            }
        })
        .detach();
    }
}

async fn handle_connection(
    mut stream: TcpStream,
    addr: std::net::SocketAddr,
    client_id: u64,
    clients: std::sync::Arc<dashmap::DashMap<u64, async_channel::Sender<String>>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // 1. Read the HTTP request headers.
    let request_bytes = read_http_request(&mut stream).await?;

    // 2. Dispatch: WebSocket upgrade or plain HTTP.
    if !is_websocket_upgrade(&request_bytes) {
        tracing::info!(%addr, "Serving HTML viewer");
        serve_html(stream).await?;
        return Ok(());
    }

    // 3. WebSocket upgrade — replay the request bytes for tungstenite.
    tracing::info!(client_id, %addr, "WebSocket upgrade");
    let replay = ReplayStream::new(request_bytes, stream);
    let ws_stream = async_tungstenite::accept_async(replay).await?;

    use futures::{SinkExt, StreamExt};
    let (mut write, mut read) = ws_stream.split();

    // Channel for sending messages to this client.
    let (tx, rx) = async_channel::bounded::<String>(100);
    clients.insert(client_id, tx);

    // Task to write messages to the socket.
    let write_task = smol::spawn(async move {
        while let Ok(msg) = rx.recv().await {
            if write
                .send(async_tungstenite::tungstenite::Message::Text(msg))
                .await
                .is_err()
            {
                break;
            }
        }
    });

    // Read loop: handle pings and disconnects.
    while let Some(msg) = read.next().await {
        match msg {
            Ok(async_tungstenite::tungstenite::Message::Close(_)) => break,
            Ok(_) => {}
            Err(_) => break,
        }
    }

    // Cleanup.
    clients.remove(&client_id);
    write_task.cancel().await;
    tracing::info!(client_id, "Client disconnected");

    Ok(())
}
