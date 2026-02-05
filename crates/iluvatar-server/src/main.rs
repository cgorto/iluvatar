use iluvatar_core::CameraMessage;
use iluvatar_server::{
    aggregator::FrameAggregator,
    camera_mgmt::CameraRegistry,
    config::{ConfigError, ServerConfig},
    detector::ObjectDetector,
    grid::SparseVoxelGrid,
    quic::QuicServer,
    time::Clock,
    tracker::ObjectTracker,
};
use parking_lot::RwLock;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tracing::{error, info};

#[derive(Debug, Error)]
enum ServerError {
    #[error("Configuration error: {0}")]
    Config(#[from] ConfigError),
    #[error("Network error: {0}")]
    Network(String),
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive(tracing::Level::INFO.into()),
        )
        .init();

    info!("Iluvatar Server starting...");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/server.toml".to_string());

    if let Err(e) = smol::block_on(run(Path::new(&config_path))) {
        error!("Fatal error: {e}");
        std::process::exit(1);
    }
}

async fn run(config_path: &Path) -> Result<(), ServerError> {
    let config = ServerConfig::load(config_path)?;
    info!(
        quic_addr = %config.server.listen_address,
        ws_port = config.server.websocket_port,
        "Configuration loaded"
    );

    // Shared state
    let clock = Clock::new();
    let registry = Arc::new(RwLock::new(CameraRegistry::new()));
    let grid = Arc::new(SparseVoxelGrid::with_max_voxels(
        config.grid_origin(),
        config.grid_dimensions(),
        config.grid.voxel_size,
        config.decay.rate,
        clock.clone(),
        config.grid.max_voxels,
    ));

    // Frame channel from cameras to processing
    let (msg_tx, msg_rx) = async_channel::bounded::<CameraMessage>(1000);

    // Update channel to WebSocket clients
    let (update_tx, update_rx) =
        async_channel::bounded::<iluvatar_server::websocket::BroadcastMessage>(100);

    // Start QUIC server for cameras
    let listen_addr: std::net::SocketAddr = config
        .server
        .listen_address
        .parse()
        .map_err(|e| ServerError::Network(format!("Invalid listen address: {}", e)))?;
    let grid_config = config.to_grid_config_message();
    let raymarch_config = config.to_raymarch_config();
    let registry_clone = registry.clone();
    let msg_tx_clone = msg_tx.clone();
    smol::spawn(async move {
        match QuicServer::bind(listen_addr, None).await {
            Ok(server) => {
                if let Err(e) = server.run(msg_tx_clone, registry_clone, grid_config, raymarch_config).await {
                    error!("QUIC server error: {}", e);
                }
            }
            Err(e) => {
                error!("Failed to start QUIC server: {}", e);
            }
        }
    })
    .detach();

    // Start WebSocket server for clients
    let ws_port = config.server.websocket_port;
    smol::spawn(async move {
        if let Err(e) = iluvatar_server::websocket::run_server(ws_port, update_rx).await {
            error!("WebSocket server failed: {}", e);
        }
    })
    .detach();

    // Initialize detection and tracking
    // Note: Only the tracker generates object IDs - detections are anonymous until tracked
    let mut detector = ObjectDetector::new(config.to_detection_config());
    let mut tracker = ObjectTracker::new(
        config.tracking.association_threshold,
        config.tracking.max_missing_frames,
        config.server.broadcast_rate_hz, // Use broadcast rate as effective frame rate
    );

    let mut aggregator = FrameAggregator::new(Duration::from_millis(50), 10_000, clock.clone()); // 50ms latency, 10ms window

    // Timing
    let decay_interval = config.decay_interval();
    let broadcast_interval = config.broadcast_interval();
    let mut last_decay = clock.now();
    let mut last_broadcast = clock.now();
    let mut frames_received = 0u64;
    let mut last_stats = clock.now();

    info!(
        decay_interval_ms = decay_interval.as_millis(),
        broadcast_hz = config.server.broadcast_rate_hz,
        grid_dims = ?config.grid_dimensions(),
        voxel_size = config.grid.voxel_size,
        "Starting main processing loop"
    );

    loop {
        // Try to receive frames with a short timeout
        let timeout = decay_interval.min(Duration::from_millis(10));

        match smol::future::or(async { msg_rx.recv().await.ok() }, async {
            smol::Timer::after(timeout).await;
            None
        })
        .await
        {
            Some(msg) => match msg {
                CameraMessage::Frame(frame) => {
                    // Record frame in registry
                    {
                        let mut reg = registry.write();
                        reg.record_frame(frame.camera_id, frame.pose);
                    }

                    // Add to aggregator
                    aggregator.add_frame(frame);
                    frames_received += 1;
                }
                CameraMessage::TimeSync { timestamp } => {
                    clock.set_simulated_time(timestamp);
                }
                _ => {}
            },
            None => {
                // Timeout - continue to decay check
            }
        }

        // Process aggregated batches
        while let Some(batch) = aggregator.try_get_batch() {
            // Reset camera masks before processing each batch to ensure camera_count()
            // reflects only cameras that contributed in THIS batch, not accumulated
            // across time due to voxel decay persistence.
            grid.reset_camera_masks();
            for frame in batch {
                grid.add_frame(&frame);
            }
        }

        // Decay and detection cycle (tied together per design decision)
        if clock.now().duration_since(last_decay) >= decay_interval {
            // Apply decay to grid (removes low-intensity voxels)
            grid.apply_decay();

            // Extract active points
            let points = grid.extract_points(&config.to_detection_config());

            // Run detection
            let detections = detector.detect(&points);

            // Update tracking
            let now = clock.now();
            let dt = now.duration_since(last_decay).as_secs_f32();
            let tracked = tracker.update(detections, dt);

            // Queue update for broadcast (if any clients)
            if !tracked.is_empty() {
                tracing::debug!(
                    object_count = tracked.len(),
                    ids = ?tracked.iter().map(|o| o.id).collect::<Vec<_>>(),
                    "Broadcasting objects"
                );
                let update = iluvatar_server::websocket::BroadcastMessage {
                    timestamp: clock.now_micros(),
                    objects: tracked,
                    camera_count: registry.read().connected_count() as u32,
                    active_voxels: grid.active_count() as u64,
                };
                let _ = update_tx.try_send(update);
            }

            last_decay = clock.now();
        }

        // Broadcast to clients at fixed rate
        if clock.now().duration_since(last_broadcast) >= broadcast_interval {
            // Broadcasting is now handled by the async task consuming update_rx
            last_broadcast = clock.now();
        }

        // Periodic stats logging (every 10 seconds)
        if clock.now().duration_since(last_stats).as_secs() >= 10 {
            let elapsed = clock.now().duration_since(last_stats).as_secs_f64();
            let fps = frames_received as f64 / elapsed;
            let stats = grid.get_stats();
            info!(
                frames_per_sec = format!("{:.1}", fps),
                active_voxels = stats.active_voxels,
                memory_mb = stats.memory_usage_bytes / 1024 / 1024,
                max_intensity = format!("{:.1}", stats.max_intensity),
                cameras = registry.read().connected_count(),
                tracks = tracker.track_count(),
                "Stats"
            );
            frames_received = 0;
            last_stats = clock.now();
        }
    }
}
