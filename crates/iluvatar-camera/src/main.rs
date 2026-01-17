use glam::Vec3;
use iluvatar_camera::{
    arena::FrameArena,
    capture::{CameraCapture, DummyCamera},
    channel::DropOldestChannel,
    config::{CameraConfig, ConfigError},
    difference::FrameProcessor,
    localization::{DummyLocalizer, Localizer},
    network::NetworkClient,
    raymarch::Raymarcher,
};
use iluvatar_core::{BoundingBox, CameraFrame, CameraRegistration, GeoPosition};
use std::path::Path;
use std::time::Instant;
use thiserror::Error;
use tracing::{error, info, warn};

#[derive(Debug, Error)]
enum CameraError {
    #[error("Configuration error: {0}")]
    Config(#[from] ConfigError),
    #[error("Localization error: {0}")]
    Localization(#[from] iluvatar_camera::localization::LocalizationError),
    #[error("Capture error: {0}")]
    Capture(#[from] iluvatar_camera::capture::CaptureError),
    #[error("Network error: {0}")]
    Network(#[from] iluvatar_camera::network::NetworkError),
    #[error("GPS timeout: no fix within {0} seconds")]
    GpsTimeout(u64),
    #[error("Server connection timeout")]
    ConnectionTimeout,
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

    if let Err(e) = smol::block_on(run(Path::new(&config_path))) {
        error!("Fatal error: {e}");
        std::process::exit(1);
    }
}

async fn run(config_path: &Path) -> Result<(), CameraError> {
    let config = CameraConfig::load(config_path)?;
    info!(
        camera_id = config.identity.camera_id,
        "Configuration loaded"
    );

    // Initialize components
    let mut camera = create_camera(&config);
    let mut processor = FrameProcessor::new(config.processing.difference_threshold);
    let mut localizer = create_localizer(&config);
    let mut network = NetworkClient::new(
        config.network.server_address.clone(),
        config.identity.camera_id,
    );

    // Wait for GPS fix
    info!("Waiting for GPS fix...");
    let initial_pose = wait_for_gps(&mut localizer, config.gps_timeout()).await?;
    info!("GPS fix acquired");

    // Connect to server with timeout
    info!("Connecting to server...");
    connect_with_timeout(&mut network, config.connection_timeout()).await?;
    info!("Connected to server");

    // Register with server
    let registration = CameraRegistration {
        version: iluvatar_core::PROTOCOL_VERSION,
        camera_id: config.identity.camera_id,
        intrinsics: config.to_intrinsics(),
        initial_pose,
    };
    network.register(registration).await?;
    info!("Registered with server");

    // TODO: Receive grid config from server
    // For now, use a default grid
    let grid_bounds = BoundingBox::new(Vec3::splat(-500.0), Vec3::splat(500.0));
    let voxel_size = 1.0;

    let origin = &config.processing.grid_origin;
    let world_origin = GeoPosition::new(origin.latitude, origin.longitude, origin.altitude);

    let raymarcher = Raymarcher::new(
        config.to_intrinsics(),
        config.to_raymarch_config(),
        grid_bounds,
        voxel_size,
        world_origin,
    );

    // Create frame buffer for backpressure
    let frame_buffer: DropOldestChannel<CameraFrame> =
        DropOldestChannel::new(config.network.frame_buffer_size);

    // Calculate thresholds
    let motion_threshold = config.motion_pixel_threshold();
    let frame_interval = config.frame_interval();

    info!(
        fps = config.hardware.fps,
        motion_threshold = motion_threshold,
        buffer_size = config.network.frame_buffer_size,
        "Starting main loop"
    );

    // Allocate arena with capacity for typical frame size
    // ~2MB should be plenty for 1080p grayscale + mask + contributions
    let mut arena = FrameArena::with_capacity(2 * 1024 * 1024);

    let mut sequence = 0u64;
    let mut frames_processed = 0u64;
    let mut last_stats_time = Instant::now();

    loop {
        let frame_start = Instant::now();

        // Get current pose (fail fast on error)
        let pose = localizer.get_pose()?;

        // Capture frame
        let grayscale = camera.capture_grayscale(&arena, &pose)?;

        // Compute difference mask
        if let Some(mask) = processor.compute_difference(&grayscale, &arena) {
            let motion_count = mask.motion_count();

            // Only raymarch and send if motion exceeds threshold
            if motion_count as u32 > motion_threshold {
                // Raymarch to get voxel contributions
                let contributions = raymarcher.raymarch(&pose, &mask);

                // Create frame message
                let frame = CameraFrame {
                    camera_id: config.identity.camera_id,
                    sequence,
                    timestamp: pose.timestamp,
                    pose,
                    contributions,
                };

                // Push to buffer (may drop oldest if full)
                let dropped = frame_buffer.push(frame);
                if dropped {
                    warn!("Dropped oldest frame due to backpressure");
                }

                sequence += 1;
            }
        }

        // Try to send buffered frames
        while let Some(frame) = frame_buffer.pop() {
            if let Err(e) = network.send_frame(frame).await {
                error!("Failed to send frame: {e}");
                return Err(CameraError::Network(e));
            }
        }

        frames_processed += 1;

        // Reset arena for next frame
        arena.reset();

        // Periodic stats logging (every 10 seconds)
        if last_stats_time.elapsed().as_secs() >= 10 {
            let elapsed = last_stats_time.elapsed().as_secs_f64();
            let fps = frames_processed as f64 / elapsed;
            info!(
                fps = format!("{:.1}", fps),
                frames = frames_processed,
                dropped = frame_buffer.dropped_count(),
                "Stats"
            );
            frames_processed = 0;
            last_stats_time = Instant::now();
        }

        // Maintain frame rate
        let elapsed = frame_start.elapsed();
        if elapsed < frame_interval {
            smol::Timer::after(frame_interval - elapsed).await;
        }
    }
}

fn create_camera(config: &CameraConfig) -> Box<dyn CameraCapture> {
    let (width, height) = config.resolution();
    let device = &config.hardware.device;

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
    } else {
        info!("Using dummy camera as requested (device: {})", device);
    }

    Box::new(DummyCamera::new(
        width,
        height,
        Some(config.to_intrinsics()),
    ))
}

fn create_localizer(config: &CameraConfig) -> Box<dyn Localizer> {
    // TODO: Use GpsImuLocalizer when GPS device is available
    // For now, use dummy localizer at configured grid_origin
    let origin = &config.processing.grid_origin;
    Box::new(DummyLocalizer::with_position(
        origin.latitude,
        origin.longitude,
        origin.altitude,
    ))
}

async fn wait_for_gps(
    localizer: &mut Box<dyn Localizer>,
    timeout: std::time::Duration,
) -> Result<iluvatar_core::CameraPose, CameraError> {
    let start = Instant::now();

    loop {
        match localizer.get_pose() {
            Ok(pose) => return Ok(pose),
            Err(iluvatar_camera::localization::LocalizationError::NoFix) => {
                if start.elapsed() > timeout {
                    return Err(CameraError::GpsTimeout(timeout.as_secs()));
                }
                smol::Timer::after(std::time::Duration::from_millis(100)).await;
            }
            Err(e) => return Err(CameraError::Localization(e)),
        }
    }
}

async fn connect_with_timeout(
    network: &mut NetworkClient,
    timeout: std::time::Duration,
) -> Result<(), CameraError> {
    let start = Instant::now();
    let mut delay = std::time::Duration::from_millis(100);
    let max_delay = std::time::Duration::from_secs(5);

    loop {
        match network.connect().await {
            Ok(()) => return Ok(()),
            Err(e) => {
                if start.elapsed() > timeout {
                    error!(
                        "Connection timeout after {} attempts",
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
