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

/// Message to broadcast to all connected clients
#[derive(Debug, Clone)]
pub struct BroadcastMessage {
    pub timestamp: u64,
    pub objects: Vec<TrackedObject>,
    pub camera_count: u32,
    pub active_voxels: u64,
}

/// Main WebSocket server loop
pub async fn run_server(
    port: u16,
    receiver: async_channel::Receiver<BroadcastMessage>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let addr = format!("0.0.0.0:{}", port);
    let listener = smol::net::TcpListener::bind(&addr).await?;
    tracing::info!("WebSocket server listening on {}", addr);

    // Map of connected clients: ID -> Sender<String>
    // We send JSON strings to the client actors
    let clients: std::sync::Arc<dashmap::DashMap<u64, async_channel::Sender<String>>> =
        std::sync::Arc::new(dashmap::DashMap::new());

    // Spawn the broadcaster task
    let clients_clone = clients.clone();
    smol::spawn(async move {
        while let Ok(msg) = receiver.recv().await {
            // 1. Send Update
            let update_msg = build_update(msg.timestamp, msg.objects.clone());
            if let Ok(json) = serialize_update(&update_msg) {
                for client in clients_clone.iter() {
                    let _ = client.value().try_send(json.clone());
                }
            }
        }
    })
    .detach();

    let mut client_id_seq = 0;

    loop {
        let (stream, addr) = listener.accept().await?;
        tracing::info!("Incoming WebSocket connection from {}", addr);

        let client_id = client_id_seq;
        client_id_seq += 1;

        let clients_map = clients.clone();

        smol::spawn(async move {
            if let Err(e) = handle_connection(stream, client_id, clients_map).await {
                tracing::warn!("WebSocket connection error: {}", e);
            }
        })
        .detach();
    }
}

async fn handle_connection(
    stream: smol::net::TcpStream,
    client_id: u64,
    clients: std::sync::Arc<dashmap::DashMap<u64, async_channel::Sender<String>>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let ws_stream = async_tungstenite::accept_async(stream).await?;
    tracing::info!("WebSocket handshake successful for client {}", client_id);

    use futures::{SinkExt, StreamExt};
    let (mut write, mut read) = ws_stream.split();

    // Channel for sending messages to this client
    let (tx, rx) = async_channel::bounded::<String>(100);
    clients.insert(client_id, tx);

    // Task to write messages to the socket
    let write_task = smol::spawn(async move {
        while let Ok(msg) = rx.recv().await {
            if let Err(_) = write
                .send(async_tungstenite::tungstenite::Message::Text(msg))
                .await
            {
                break;
            }
        }
    });

    // Main loop: read messages from client (handle pings, disconnects)
    while let Some(msg) = read.next().await {
        match msg {
            Ok(async_tungstenite::tungstenite::Message::Close(_)) => break,
            Ok(_) => {} // Ignore other messages for now
            Err(_) => break,
        }
    }

    // Cleanup
    clients.remove(&client_id);
    write_task.cancel().await;
    tracing::info!("Client {} disconnected", client_id);

    Ok(())
}
