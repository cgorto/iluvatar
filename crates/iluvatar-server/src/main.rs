use iluvatar_core::{CameraFrame, TrackedObject};
use iluvatar_server::{
    camera_mgmt::CameraRegistry,
    config::{ConfigError, ServerConfig},
    detector::ObjectDetector,
    grid::SparseVoxelGrid,
    tracker::ObjectTracker,
};
use parking_lot::RwLock;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};
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
    let registry = Arc::new(RwLock::new(CameraRegistry::new()));
    let grid = Arc::new(SparseVoxelGrid::new(
        config.grid_origin(),
        config.grid_dimensions(),
        config.grid.voxel_size,
        config.decay.rate,
    ));

    // Frame channel from cameras to processing
    let (_frame_tx, frame_rx) = async_channel::bounded::<CameraFrame>(1000);

    // Update channel to WebSocket clients
    let (update_tx, _update_rx) = async_channel::bounded::<WorldUpdate>(100);

    // TODO: Start QUIC server for cameras
    // For now, we'll just log that it would start
    info!(
        addr = %config.server.listen_address,
        "QUIC server would start (not implemented)"
    );

    // TODO: Start WebSocket server for clients
    info!(
        port = config.server.websocket_port,
        "WebSocket server would start (not implemented)"
    );

    // Initialize detection and tracking
    // Note: Only the tracker generates object IDs - detections are anonymous until tracked
    let mut detector = ObjectDetector::new(config.to_detection_config());
    let mut tracker = ObjectTracker::new(
        config.tracking.association_threshold,
        config.tracking.max_missing_frames,
        config.server.broadcast_rate_hz, // Use broadcast rate as effective frame rate
    );

    // Timing
    let decay_interval = config.decay_interval();
    let broadcast_interval = config.broadcast_interval();
    let mut last_decay = Instant::now();
    let mut last_broadcast = Instant::now();
    let mut frames_received = 0u64;
    let mut last_stats = Instant::now();

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

        match smol::future::or(async { frame_rx.recv().await.ok() }, async {
            smol::Timer::after(timeout).await;
            None
        })
        .await
        {
            Some(frame) => {
                // Record frame in registry
                {
                    let mut reg = registry.write();
                    reg.record_frame(frame.camera_id, frame.pose);
                }

                // Add contributions to grid
                grid.add_frame(&frame);

                frames_received += 1;
            }
            None => {
                // Timeout - continue to decay check
            }
        }

        // Decay and detection cycle (tied together per design decision)
        if last_decay.elapsed() >= decay_interval {
            // Apply decay to grid (removes low-intensity voxels)
            grid.apply_decay();

            // Extract active points
            let points = grid.extract_points(&config.to_detection_config());

            // Run detection
            let detections = detector.detect(&points);

            // Update tracking
            let tracked = tracker.update(detections);

            // Queue update for broadcast (if any clients)
            if !tracked.is_empty() {
                let update = WorldUpdate {
                    timestamp: now_micros(),
                    objects: tracked,
                    camera_count: registry.read().connected_count() as u32,
                    active_voxels: grid.active_count() as u64,
                };
                let _ = update_tx.try_send(update);
            }

            last_decay = Instant::now();
        }

        // Broadcast to clients at fixed rate
        if last_broadcast.elapsed() >= broadcast_interval {
            // TODO: Actually broadcast to WebSocket clients
            last_broadcast = Instant::now();
        }

        // Periodic stats logging (every 10 seconds)
        if last_stats.elapsed().as_secs() >= 10 {
            let elapsed = last_stats.elapsed().as_secs_f64();
            let fps = frames_received as f64 / elapsed;
            info!(
                frames_per_sec = format!("{:.1}", fps),
                active_voxels = grid.active_count(),
                cameras = registry.read().connected_count(),
                tracks = tracker.track_count(),
                "Stats"
            );
            frames_received = 0;
            last_stats = Instant::now();
        }
    }
}

fn now_micros() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}

/// Update sent to WebSocket clients
#[derive(Debug, Clone)]
struct WorldUpdate {
    timestamp: u64,
    objects: Vec<TrackedObject>,
    camera_count: u32,
    active_voxels: u64,
}
