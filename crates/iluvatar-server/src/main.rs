use glam::Vec3;
use iluvatar_core::{
    BoundingBox, CameraId, CameraMessage, CameraRegistration, GeoPosition, GridConfigMessage,
    RaymarchConfig, raymarch::Raymarcher,
};
#[allow(unused_imports)]
use iluvatar_server::{
    aggregator::FrameAggregator,
    camera_mgmt::CameraRegistry,
    config::{ConfigError, ServerConfig},
    detector::ObjectDetector,
    flat_grid::FlatVoxelGrid,
    profile_frame, profile_plot, profile_scope,
    tcp::TcpServer,
    time::Clock,
    tracker::ObjectTracker,
};
use parking_lot::RwLock;
use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tracing::{error, info, warn};

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
        tcp_addr = %config.server.listen_address,
        ws_port = config.server.websocket_port,
        "Configuration loaded"
    );

    // Build static camera list from config (for viewer display).
    let config_cameras: Vec<iluvatar_server::websocket::CameraInfo> = config
        .cameras
        .iter()
        .map(|c| iluvatar_server::websocket::CameraInfo {
            camera_id: c.id,
            name: c.name.clone().unwrap_or_else(|| format!("cam-{}", c.id)),
            position: [
                c.position[0] as f32,
                c.position[1] as f32,
                c.position[2] as f32,
            ],
            orientation: c.orientation,
            connected: false,
            fps: 0.0,
        })
        .collect();

    // Shared state
    let clock = Clock::new();
    let registry = Arc::new(RwLock::new(CameraRegistry::new()));
    let mut grid = FlatVoxelGrid::with_max_voxels(
        config.grid_origin(),
        config.grid_dimensions(),
        config.grid.voxel_size,
        config.decay.rate,
        clock.clone(),
        config.grid.max_voxels,
    );

    // Frame channel from cameras to processing
    let (msg_tx, msg_rx) = async_channel::bounded::<CameraMessage>(1000);

    // Update channel to WebSocket clients
    let (update_tx, update_rx) =
        async_channel::bounded::<iluvatar_server::websocket::BroadcastMessage>(100);

    // Start the validated K230 camera transport. Bind before detaching the
    // accept loop so startup fails visibly if the configured port is unavailable.
    let tcp_addr: std::net::SocketAddr = config
        .server
        .listen_address
        .parse()
        .map_err(|e| ServerError::Network(format!("Invalid TCP address: {e}")))?;
    let grid_config = config.to_grid_config_message();
    let raymarch_config = config.to_raymarch_config();
    let tcp_server = TcpServer::bind(tcp_addr)
        .await
        .map_err(|e| ServerError::Network(e.to_string()))?;
    let msg_tx_tcp = msg_tx.clone();
    let registry_tcp = registry.clone();
    let grid_config_tcp = grid_config.clone();
    smol::spawn(async move {
        if let Err(e) = tcp_server
            .run(msg_tx_tcp, registry_tcp, grid_config_tcp)
            .await
        {
            error!("TCP server error: {e}");
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

    // Mutable processing state bundled for process_message.
    let mut state = ProcessingState {
        aggregator: FrameAggregator::new(Duration::from_millis(50), 10_000, clock.clone()),
        raymarchers: HashMap::new(),
        frames_received: 0,
        contribution_limit: config.raymarch.contribution_limit,
    };

    // Timing
    let decay_interval = config.decay_interval();
    let broadcast_interval = config.broadcast_interval();
    let mut last_decay = clock.now();
    let mut last_broadcast = clock.now();
    let mut last_stats = clock.now();

    info!(
        decay_interval_ms = decay_interval.as_millis(),
        broadcast_hz = config.server.broadcast_rate_hz,
        grid_dims = ?config.grid_dimensions(),
        voxel_size = config.grid.voxel_size,
        "Starting main processing loop"
    );

    /// Maximum messages to drain per loop iteration. Prevents starving the
    /// decay timer when the channel has hundreds of buffered messages.
    const DRAIN_LIMIT: u32 = 256;

    /// Maximum wall-clock time (ms) to spend in the drain loop before
    /// yielding to decay/detect/track. With server-side raymarching, each
    /// motion frame can take tens of milliseconds, so a pure message-count
    /// limit is insufficient.
    const DRAIN_BUDGET: Duration = Duration::from_millis(50);

    loop {
        profile_frame!();
        profile_plot!("channel_depth", msg_rx.len());

        // Wait for at least one message, with a short timeout.
        let timeout = decay_interval.min(Duration::from_millis(10));
        let first_msg = smol::future::or(async { msg_rx.recv().await.ok() }, async {
            smol::Timer::after(timeout).await;
            None
        })
        .await;

        // Reset camera masks once per cycle so camera_count reflects only
        // the current cycle's contributions (both direct-drain and batched).
        grid.reset_camera_masks();

        // Process the first message, then drain up to DRAIN_LIMIT more without
        // blocking. This amortizes the per-iteration overhead (timer checks,
        // profile calls) across many messages when the channel has a backlog.
        if let Some(msg) = first_msg {
            process_message(
                msg,
                &mut state,
                &registry,
                &grid_config,
                &raymarch_config,
                &mut grid,
            );

            let drain_start = std::time::Instant::now();
            let mut drained: u32 = 0;
            while drained < DRAIN_LIMIT && drain_start.elapsed() < DRAIN_BUDGET {
                match msg_rx.try_recv() {
                    Ok(msg) => {
                        process_message(
                            msg,
                            &mut state,
                            &registry,
                            &grid_config,
                            &raymarch_config,
                            &mut grid,
                        );
                        drained += 1;
                    }
                    Err(_) => break,
                }
            }
        }

        // Process aggregated batches (VoxelContributions path from cameras
        // that do their own raymarching).
        while let Some(batch) = state.aggregator.try_get_batch() {
            profile_scope!("batch_process");
            for frame in batch {
                grid.add_frame(&frame);
            }
        }

        // Decay and detection cycle (tied together per design decision).
        if clock.now().duration_since(last_decay) >= decay_interval {
            let decay_points = {
                profile_scope!("decay");
                grid.apply_decay();
                grid.extract_points(&config.to_detection_config())
            };
            profile_plot!("active_voxels", grid.active_count());

            let detections = {
                profile_scope!("detect");
                detector.detect(&decay_points)
            };

            let now = clock.now();
            let dt = now.duration_since(last_decay).as_secs_f32();
            let tracked = {
                profile_scope!("track");
                tracker.update(detections, dt)
            };

            // Queue update for broadcast.
            if !tracked.is_empty() {
                tracing::debug!(
                    object_count = tracked.len(),
                    ids = ?tracked.iter().map(|o| o.id).collect::<Vec<_>>(),
                    "Broadcasting objects"
                );
            }
            // Merge config cameras with live camera data. Live data
            // overrides config entries for the same camera_id.
            let reg = registry.read();
            let live = reg.viewer_info();
            let live_ids: std::collections::HashSet<u64> =
                live.iter().map(|c| c.camera_id).collect();
            let mut cameras: Vec<_> = config_cameras
                .iter()
                .filter(|c| !live_ids.contains(&c.camera_id))
                .cloned()
                .collect();
            cameras.extend(live);

            // Pack active voxels as [x, y, z, intensity] for the viewer.
            // The grid's voxel_to_world returns 0-based positions, but the
            // raymarcher uses a centered grid (-half_dim to +half_dim).
            // Subtract half_dim so viewer positions match camera positions.
            let half_dim = Vec3::new(
                config.grid_dimensions().x as f32 * config.grid.voxel_size * 0.5,
                config.grid_dimensions().y as f32 * config.grid.voxel_size * 0.5,
                config.grid_dimensions().z as f32 * config.grid.voxel_size * 0.5,
            );
            let max_viewer_voxels: usize = 5000;
            let voxels: Vec<[f32; 4]> = decay_points
                .iter()
                .take(max_viewer_voxels)
                .map(|p| {
                    [
                        p.position.x - half_dim.x,
                        p.position.y - half_dim.y,
                        p.position.z - half_dim.z,
                        p.intensity,
                    ]
                })
                .collect();

            // Center tracked object positions to match camera coordinate space.
            let centered_objects: Vec<_> = tracked
                .into_iter()
                .map(|mut obj| {
                    obj.centroid -= half_dim;
                    obj
                })
                .collect();

            let update = iluvatar_server::websocket::BroadcastMessage {
                timestamp: clock.now_micros(),
                objects: centered_objects,
                cameras,
                voxels,
                camera_count: reg.connected_count() as u32,
                active_voxels: grid.active_count() as u64,
            };
            drop(reg);
            let _ = update_tx.try_send(update);

            last_decay = clock.now();
        }

        // Broadcast to clients at fixed rate.
        if clock.now().duration_since(last_broadcast) >= broadcast_interval {
            // Broadcasting is now handled by the async task consuming update_rx.
            last_broadcast = clock.now();
        }

        // Periodic stats logging (every 10 seconds).
        if clock.now().duration_since(last_stats).as_secs() >= 10 {
            let elapsed = clock.now().duration_since(last_stats).as_secs_f64();
            let fps = state.frames_received as f64 / elapsed;
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
            state.frames_received = 0;
            last_stats = clock.now();
        }
    }
}

/// Mutable state used during message processing, bundled to keep the
/// `process_message` signature under the 7-argument clippy limit.
struct ProcessingState {
    aggregator: FrameAggregator,
    raymarchers: HashMap<CameraId, Raymarcher>,
    frames_received: u64,
    contribution_limit: usize,
}

/// Dispatch a single camera message. Extracted from the main loop to keep
/// the loop body under 70 lines and to allow reuse in the drain path.
fn process_message(
    msg: CameraMessage,
    state: &mut ProcessingState,
    registry: &Arc<RwLock<CameraRegistry>>,
    grid_config: &GridConfigMessage,
    raymarch_config: &RaymarchConfig,
    grid: &mut FlatVoxelGrid,
) {
    match msg {
        CameraMessage::Register(registration) => {
            // Create a server-side raymarcher for cameras that send
            // motion pixels. This runs in the main loop so raymarching
            // doesn't block the TCP handler.
            if registration.capabilities.motion_frames {
                let rm = create_server_raymarcher(
                    &registration,
                    grid_config,
                    raymarch_config,
                    state.contribution_limit,
                );
                state.raymarchers.insert(registration.camera_id, rm);
                info!(
                    camera_id = registration.camera_id,
                    "Created server-side raymarcher"
                );
            }
        }
        CameraMessage::Frame(frame) => {
            // Skip stale frames from disconnected cameras.
            if registry.read().is_connected(frame.camera_id) {
                registry.write().record_frame(frame.camera_id, frame.pose);
                state.aggregator.add_frame(frame);
                state.frames_received += 1;
            }
        }
        CameraMessage::Motion(motion_frame) => {
            process_motion_frame(motion_frame, state, registry, grid);
        }
        CameraMessage::Heartbeat { .. } | CameraMessage::TimeSync { .. } => {}
    }
}

/// Handle a motion frame: skip if disconnected, raymarch pixels directly into grid.
///
/// This is the hot path. Instead of collecting voxel contributions into a Vec
/// and routing through the aggregator, we drain the raymarcher's internal
/// HashMap directly into the FlatVoxelGrid via `raymarch_into`. This eliminates
/// the Vec allocation, the CameraFrame wrapper, and the aggregator's batch
/// overhead for motion frames.
fn process_motion_frame(
    motion_frame: iluvatar_core::MotionFrame,
    state: &mut ProcessingState,
    registry: &Arc<RwLock<CameraRegistry>>,
    grid: &mut FlatVoxelGrid,
) {
    if !registry.read().is_connected(motion_frame.camera_id) {
        return;
    }
    registry
        .write()
        .record_frame(motion_frame.camera_id, motion_frame.pose);

    if let Some(rm) = state.raymarchers.get_mut(&motion_frame.camera_id) {
        let pixels: Vec<_> = motion_frame.motion.pixels().collect();
        profile_plot!("motion_pixels", pixels.len());

        let Some(camera_bit) = 1u64.checked_shl(motion_frame.camera_id as u32) else {
            warn!(
                camera_id = motion_frame.camera_id,
                "Camera id exceeds bitmask"
            );
            return;
        };
        {
            profile_scope!("raymarch");
            rm.raymarch_into(&motion_frame.pose, &pixels, &mut |key, intensity| {
                grid.accumulate(key, intensity, camera_bit);
            });
        }

        state.frames_received += 1;
    } else {
        warn!(
            camera_id = motion_frame.camera_id,
            "No raymarcher for camera, dropping motion frame"
        );
    }
}

/// Build a Raymarcher from a camera's intrinsics and the server's config.
///
/// The contribution limit controls how many unique voxels a single motion
/// frame can write to the grid. Keeping this low (16K default) prevents
/// the grid from filling faster than decay can clear it, which would push
/// the hash table past L3 cache and cause multi-second decay stalls.
fn create_server_raymarcher(
    registration: &CameraRegistration,
    grid_config: &GridConfigMessage,
    raymarch_config: &RaymarchConfig,
    contribution_limit: usize,
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

    Raymarcher::new(
        registration.intrinsics,
        raymarch_config.clone(),
        grid_bounds,
        grid_config.voxel_size,
        world_origin,
        grid_config.coordinate_mode,
        contribution_limit,
    )
}
