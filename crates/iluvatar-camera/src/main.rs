use async_signal::{Signal, Signals};
use futures_lite::StreamExt;
use glam::Vec3;
#[cfg(feature = "k230")]
use iluvatar_camera::capture_k230::{K230Camera, K230SensorType};
#[cfg(feature = "real")]
use iluvatar_camera::localization::{GpsFallbackMode, LocalizerBuilder};
use iluvatar_camera::{
    arena::FrameArena,
    capture::{CameraCapture, DummyCamera},
    channel::{DropOldestChannel, OutboundFrame},
    config::{CameraConfig, ConfigError},
    difference::FrameProcessor,
    localization::{DummyLocalizer, Localizer},
    network::{NetworkClient, NetworkError, RegistrationResponse},
    profile_frame, profile_plot, profile_scope,
    raymarch::{Raymarcher, raymarch_from_mask},
};
use iluvatar_core::{
    BoundingBox, CameraFrame, CameraRegistration, FrameFormat, GeoPosition, GridConfigMessage,
    MotionData, MotionFrame, MAX_CONTRIBUTIONS_PER_FRAME,
};
use std::path::Path;
use std::time::{Duration, Instant};
use thiserror::Error;
use tracing::{debug, error, info, warn};

#[derive(Debug, Error)]
enum CameraError {
    #[error("Configuration error: {0}")]
    Config(#[from] ConfigError),
    #[error("Localization error: {0}")]
    Localization(#[from] iluvatar_camera::localization::LocalizationError),
    #[error("Capture error: {0}")]
    Capture(#[from] iluvatar_camera::capture::CaptureError),
    #[error("Network error: {0}")]
    Network(#[from] NetworkError),
    #[error("GPS timeout: no fix within {0} seconds")]
    GpsTimeout(u64),
    #[error("Server connection timeout")]
    ConnectionTimeout,
    #[error("System error: {0}")]
    System(String),
}

/// Metrics tracked during camera operation.
#[derive(Debug, Default)]
struct CameraMetrics {
    frames_processed: u64,
    frames_sent: u64,
    reconnect_count: u32,
    last_stats_time: Option<Instant>,
}

impl CameraMetrics {
    fn new() -> Self {
        Self {
            last_stats_time: Some(Instant::now()),
            ..Default::default()
        }
    }

    fn log_stats(&mut self, frame_buffer: &DropOldestChannel<OutboundFrame>) {
        if let Some(last_time) = self.last_stats_time
            && last_time.elapsed().as_secs() >= 10
        {
            let elapsed = last_time.elapsed().as_secs_f64();
            let fps = self.frames_processed as f64 / elapsed;
            info!(
                fps = format!("{:.1}", fps),
                frames_processed = self.frames_processed,
                frames_sent = self.frames_sent,
                dropped = frame_buffer.dropped_count(),
                reconnects = self.reconnect_count,
                "Stats"
            );
            self.frames_processed = 0;
            self.frames_sent = 0;
            self.last_stats_time = Some(Instant::now());
        }
    }
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive(tracing::Level::INFO.into()),
        )
        .init();

    info!("Iluvatar Camera starting...");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/camera.toml".to_string());

    let exit_code = match smol::block_on(run(Path::new(&config_path))) {
        Ok(()) => {
            info!("Camera shutdown complete");
            0
        }
        Err(e) => {
            error!("Fatal error: {e}");
            1
        }
    };

    std::process::exit(exit_code);
}

async fn run(config_path: &Path) -> Result<(), CameraError> {
    let config = CameraConfig::load(config_path)?;
    info!(
        camera_id = config.identity.camera_id,
        "Configuration loaded"
    );

    // Set up signal handler for graceful shutdown.
    let mut signals = Signals::new([Signal::Term, Signal::Int])
        .map_err(|e| CameraError::System(format!("Failed to set up signal handler: {}", e)))?;

    // Initialize components.
    let mut camera = create_camera(&config);
    let mut processor = FrameProcessor::new(config.processing.difference_threshold);
    let mut localizer = create_localizer(&config);
    let mut network = NetworkClient::new(
        config.network.server_address.clone(),
        config.identity.camera_id,
        config.network.tls.clone(),
    )?;

    // Wait for GPS fix.
    info!("Waiting for GPS fix...");
    let initial_pose = wait_for_gps(&mut localizer, config.gps_timeout()).await?;
    info!("GPS fix acquired");

    // Connect to server with timeout.
    info!("Connecting to server...");
    connect_with_timeout(&mut network, config.connection_timeout()).await?;
    info!("Connected to server");

    // Create registration. Advertise motion_frames capability so the server
    // can choose to have us send raw pixels instead of voxel contributions.
    let registration = CameraRegistration {
        version: iluvatar_core::PROTOCOL_VERSION,
        camera_id: config.identity.camera_id,
        intrinsics: config.to_intrinsics()?,
        initial_pose,
        capabilities: iluvatar_core::CameraCapabilities::with_motion_frames(),
    };

    // Register with server and receive grid configuration + format preference.
    let mut reg_response = network.register(registration.clone()).await?;
    let mut frame_format = reg_response.format;
    info!(format = ?frame_format, "Registered with server");

    // Configure raymarcher with server-provided grid.
    // Only needed for VoxelContributions mode, but we create it unconditionally
    // so it's ready if the format changes on reconnect.
    let mut raymarcher = create_raymarcher(&config, &reg_response.grid_config);

    // Create frame buffer for backpressure.
    let frame_buffer: DropOldestChannel<OutboundFrame> =
        DropOldestChannel::new(config.network.frame_buffer_size);

    // Calculate thresholds.
    let motion_threshold = config.motion_pixel_threshold();
    let frame_interval = config.frame_interval();
    let heartbeat_interval = config.heartbeat_interval();
    let max_reconnect_attempts = config.network.max_reconnect_attempts;
    let reconnect_timeout = config.reconnect_timeout();

    info!(
        fps = config.hardware.fps,
        motion_threshold = motion_threshold,
        buffer_size = config.network.frame_buffer_size,
        heartbeat_interval_secs = heartbeat_interval.as_secs(),
        max_reconnect_attempts = max_reconnect_attempts,
        format = ?frame_format,
        "Starting main loop"
    );

    // Allocate arena with capacity for typical frame size.
    // ~2MB should be plenty for 1080p grayscale + mask + contributions.
    let mut arena = FrameArena::with_capacity(2 * 1024 * 1024);

    // Warmup: capture and discard frames to let the ISP auto-exposure stabilize.
    // Without this, the first real diff is against a black frame from ISP init,
    // causing 100% motion detection and a huge raymarch allocation.
    let warmup_frames = 30;
    info!(warmup_frames, "Warming up camera (discarding initial frames)");
    let mut processor_warmup = FrameProcessor::new(config.processing.difference_threshold);
    for i in 0..warmup_frames {
        let pose = localizer.get_pose()?;
        let frame = camera.capture_grayscale(&arena, &pose)?;
        // Feed frames into a throwaway processor so the real processor
        // starts with a clean slate and stable "previous frame".
        processor_warmup.compute_difference(&frame, &arena);
        arena.reset();
        if i == 0 {
            info!("First frame captured successfully from V4L2");
        }
    }
    // Seed the real processor with the last stable frame.
    {
        let pose = localizer.get_pose()?;
        let frame = camera.capture_grayscale(&arena, &pose)?;
        processor.compute_difference(&frame, &arena);
        arena.reset();
    }
    info!("Camera warmup complete, starting processing");

    let mut metrics = CameraMetrics::new();
    let mut last_heartbeat = Instant::now();

    loop {
        profile_frame!();
        let frame_start = Instant::now();

        // Check for shutdown signal (non-blocking).
        if let Some(result) = futures_lite::future::poll_fn(|cx| {
            use std::pin::Pin;
            match Pin::new(&mut signals).poll_next(cx) {
                std::task::Poll::Ready(item) => std::task::Poll::Ready(Some(item)),
                std::task::Poll::Pending => std::task::Poll::Ready(None),
            }
        })
        .await
        {
            match result {
                Some(Ok(signal)) => {
                    info!(?signal, "Shutdown signal received, cleaning up...");
                    break;
                }
                Some(Err(e)) => {
                    warn!("Error receiving signal: {e}");
                }
                None => {
                    warn!("Signal stream ended unexpectedly");
                }
            }
        }

        // Get current pose (fail fast on error).
        let pose = {
            profile_scope!("localizer");
            localizer.get_pose()?
        };

        // Capture frame.
        let grayscale = {
            profile_scope!("capture");
            camera.capture_grayscale(&arena, &pose)?
        };

        // Compute difference mask.
        let mask = {
            profile_scope!("difference");
            processor.compute_difference(&grayscale, &arena)
        };

        if let Some(mask) = mask {
            let motion_count = mask.motion_count();
            profile_plot!("motion_pixels", motion_count as f64);

            // Only process and send if motion exceeds threshold.
            if motion_count as u32 > motion_threshold {
                match frame_format {
                    FrameFormat::MotionPixels => {
                        // Server-side raymarching: send raw motion pixels.
                        let motion_data =
                            MotionData::from_motion_pixels(mask.motion_pixels());
                        profile_plot!(
                            "motion_pixel_count",
                            motion_data.pixel_count() as f64
                        );

                        let frame = MotionFrame {
                            camera_id: config.identity.camera_id,
                            sequence: 0, // Placeholder; set by network layer.
                            timestamp: pose.timestamp,
                            pose,
                            motion: motion_data,
                        };

                        let dropped =
                            frame_buffer.push(OutboundFrame::Motion(frame));
                        if dropped {
                            warn!("Dropped oldest frame due to backpressure");
                        }
                    }
                    FrameFormat::VoxelContributions => {
                        // Camera-side raymarching: compute voxel contributions.
                        let contributions = {
                            profile_scope!("raymarch");
                            raymarch_from_mask(&raymarcher, &pose, &mask)
                        };
                        profile_plot!(
                            "contributions",
                            contributions.len() as f64
                        );

                        let frame = CameraFrame {
                            camera_id: config.identity.camera_id,
                            sequence: 0, // Placeholder; set by network layer.
                            timestamp: pose.timestamp,
                            pose,
                            contributions,
                        };

                        let dropped =
                            frame_buffer.push(OutboundFrame::Voxel(frame));
                        if dropped {
                            warn!("Dropped oldest frame due to backpressure");
                        }
                    }
                }
            }
        }

        // Try to send buffered frames.
        let send_result = {
            profile_scope!("network_send");
            send_buffered_frames(&mut network, &frame_buffer, &mut metrics).await
        };
        if let Err(e) = send_result {
            warn!("Network error during send: {e}");

            // Attempt reconnection.
            match handle_reconnect(
                &mut network,
                &registration,
                max_reconnect_attempts,
                reconnect_timeout,
            )
            .await
            {
                Ok(new_response) => {
                    metrics.reconnect_count += 1;
                    // Server might have different grid config after restart.
                    if new_response.grid_config != reg_response.grid_config {
                        info!("Grid configuration changed, updating raymarcher");
                        raymarcher =
                            create_raymarcher(&config, &new_response.grid_config);
                    }
                    if new_response.format != reg_response.format {
                        info!(
                            old = ?reg_response.format,
                            new = ?new_response.format,
                            "Frame format changed after reconnect"
                        );
                    }
                    frame_format = new_response.format;
                    reg_response = new_response;
                    last_heartbeat = Instant::now();
                    info!("Reconnected and re-registered successfully");
                }
                Err(e) => {
                    error!("Failed to reconnect: {e}");
                    return Err(CameraError::Network(e));
                }
            }
        }

        // Periodic heartbeat.
        if last_heartbeat.elapsed() > heartbeat_interval {
            if let Err(e) = network.send_heartbeat().await {
                debug!("Heartbeat failed: {e}");
                // Don't immediately reconnect — wait for next frame send to fail.
                // This avoids unnecessary reconnects on transient issues.
            }
            last_heartbeat = Instant::now();
        }

        metrics.frames_processed += 1;

        // Reset arena for next frame.
        arena.reset();

        // Periodic stats logging.
        metrics.log_stats(&frame_buffer);

        // Maintain frame rate.
        let elapsed = frame_start.elapsed();
        if elapsed < frame_interval {
            smol::Timer::after(frame_interval - elapsed).await;
        }
    }

    // Graceful shutdown.
    info!("Closing network connection...");
    network.close();
    info!("Shutdown complete");

    Ok(())
}

/// Send all buffered frames to the server, dispatching by frame type.
async fn send_buffered_frames(
    network: &mut NetworkClient,
    buffer: &DropOldestChannel<OutboundFrame>,
    metrics: &mut CameraMetrics,
) -> Result<(), NetworkError> {
    while let Some(frame) = buffer.pop() {
        match frame {
            OutboundFrame::Voxel(f) => network.send_frame(f).await?,
            OutboundFrame::Motion(f) => network.send_motion_frame(f).await?,
        }
        metrics.frames_sent += 1;
    }
    Ok(())
}

/// Handle reconnection with re-registration.
async fn handle_reconnect(
    network: &mut NetworkClient,
    registration: &CameraRegistration,
    max_attempts: u32,
    timeout: Duration,
) -> Result<RegistrationResponse, NetworkError> {
    info!(
        max_attempts = max_attempts,
        timeout_secs = timeout.as_secs(),
        "Attempting to reconnect..."
    );

    // Reconnect with backoff.
    network
        .reconnect_with_backoff(Some(max_attempts), Some(timeout))
        .await?;

    // Re-register with server.
    info!("Reconnected, re-registering with server...");
    network.register(registration.clone()).await
}

/// Create a raymarcher configured for the server's grid.
fn create_raymarcher(config: &CameraConfig, grid_config: &GridConfigMessage) -> Raymarcher {
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
        config.to_intrinsics_or_default(),
        config.to_raymarch_config(),
        grid_bounds,
        grid_config.voxel_size,
        world_origin,
        MAX_CONTRIBUTIONS_PER_FRAME,
    )
}

fn create_camera(config: &CameraConfig) -> Box<dyn CameraCapture> {
    let (width, height) = config.resolution();
    #[cfg(feature = "k230")]
    let fps = config.hardware.fps;
    let device = &config.hardware.device;

    // K230 SDK VICAP backend (preferred on K230 hardware).
    if device == "k230" {
        #[cfg(feature = "k230")]
        {
            match K230Camera::new(width, height, fps, K230SensorType::Ov5647) {
                Ok(cam) => {
                    info!(
                        width,
                        height,
                        fps,
                        "Initialized K230 VICAP camera"
                    );
                    return Box::new(cam);
                }
                Err(e) => {
                    error!(
                        "Failed to initialize K230 VICAP camera: {}. Falling back.",
                        e
                    );
                }
            }
        }
        #[cfg(not(feature = "k230"))]
        warn!(
            "K230 backend requested but 'k230' feature not enabled. Using fallback."
        );
    }

    // V4L2 backend.
    if device.starts_with("/dev/video") {
        #[cfg(all(target_os = "linux", feature = "real"))]
        {
            match iluvatar_camera::capture::V4L2Camera::new(device, width, height) {
                Ok(cam) => {
                    info!(
                        "Initialized V4L2 camera at {} ({}x{})",
                        device, width, height
                    );
                    return Box::new(cam);
                }
                Err(e) => {
                    error!(
                        "Failed to initialize V4L2 camera: {}. Falling back to dummy.",
                        e
                    );
                }
            }
        }
        #[cfg(not(all(target_os = "linux", feature = "real")))]
        warn!(
            "V4L2 not supported on this platform or 'real' feature disabled. Using dummy camera."
        );
    } else if device != "k230" {
        info!("Using dummy camera as requested (device: {})", device);
    }

    Box::new(DummyCamera::new(
        width,
        height,
        Some(config.to_intrinsics_or_default()),
    ))
}

fn create_localizer(config: &CameraConfig) -> Box<dyn Localizer> {
    let gps_device = &config.localization.gps_device;

    // Check if gpsd address is configured (contains ':' like "localhost:2947").
    let use_gpsd = gps_device.contains(':') && !gps_device.starts_with("/dev/");

    if use_gpsd {
        #[cfg(feature = "real")]
        {
            let dead_reckoning_timeout = config.dead_reckoning_timeout();

            // Determine orientation from config (as quaternion).
            let orientation = config
                .localization
                .fixed_orientation
                .map(|[yaw, pitch, roll]| {
                    glam::Quat::from_euler(
                        glam::EulerRot::YXZ,
                        yaw.to_radians(),
                        pitch.to_radians(),
                        roll.to_radians(),
                    )
                });

            let mut builder = LocalizerBuilder::new()
                .with_gpsd(gps_device.clone())
                .with_fallback_mode(GpsFallbackMode::DeadReckoning)
                .with_dead_reckoning_timeout(dead_reckoning_timeout);

            if let Some(orient) = orientation {
                builder = builder.with_fixed_orientation(orient);
            }

            match builder.build_gpsd_mock_imu() {
                Ok(localizer) => {
                    info!(
                        gps_address = gps_device,
                        timeout_secs = dead_reckoning_timeout.as_secs(),
                        "Initialized GPS+IMU localizer with gpsd"
                    );
                    return Box::new(localizer);
                }
                Err(e) => {
                    warn!(
                        ?e,
                        "Failed to connect to gpsd at {}, falling back to dummy localizer",
                        gps_device
                    );
                }
            }
        }
        #[cfg(not(feature = "real"))]
        {
            warn!(
                gps_address = gps_device,
                "gpsd support requires the 'real' feature. Using dummy localizer."
            );
        }
    } else if gps_device != "none" && gps_device != "dummy" {
        warn!(
            gps_device = gps_device,
            "Serial GPS devices not yet supported. Using dummy localizer at grid_origin."
        );
    } else {
        info!(
            "GPS disabled (device: {}), using dummy localizer at grid_origin",
            gps_device
        );
    }

    // Fallback: use dummy localizer at configured grid_origin.
    let origin = &config.processing.grid_origin;
    Box::new(DummyLocalizer::with_position(
        origin.latitude,
        origin.longitude,
        origin.altitude,
    ))
}

async fn wait_for_gps(
    localizer: &mut Box<dyn Localizer>,
    timeout: Duration,
) -> Result<iluvatar_core::CameraPose, CameraError> {
    let start = Instant::now();

    loop {
        match localizer.get_pose() {
            Ok(pose) => return Ok(pose),
            Err(iluvatar_camera::localization::LocalizationError::NoFix) => {
                if start.elapsed() > timeout {
                    return Err(CameraError::GpsTimeout(timeout.as_secs()));
                }
                smol::Timer::after(Duration::from_millis(100)).await;
            }
            Err(e) => return Err(CameraError::Localization(e)),
        }
    }
}

async fn connect_with_timeout(
    network: &mut NetworkClient,
    timeout: Duration,
) -> Result<(), CameraError> {
    let start = Instant::now();
    let mut delay = Duration::from_millis(100);
    let max_delay = Duration::from_secs(5);

    loop {
        match network.connect().await {
            Ok(()) => return Ok(()),
            Err(e) => {
                if start.elapsed() > timeout {
                    error!(
                        "Connection timeout after {} seconds",
                        start.elapsed().as_secs()
                    );
                    return Err(CameraError::ConnectionTimeout);
                }
                warn!("Connection failed: {e}, retrying in {:?}", delay);
                smol::Timer::after(delay).await;
                delay = (delay * 2).min(max_delay);
            }
        }
    }
}
