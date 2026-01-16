# iluvatar-camera Implementation Guide

This crate implements the camera unit that captures video, detects motion, raymarches into the voxel grid, and sends contributions to the server.

## Overview

```
iluvatar-camera/
├── src/
│   ├── lib.rs           # Module exports
│   ├── main.rs          # Binary entry point
│   ├── capture.rs       # Camera capture abstraction
│   ├── difference.rs    # Motion detection
│   ├── localization.rs  # GPS + IMU fusion
│   ├── raymarch.rs      # Voxel raymarching
│   └── network.rs       # QUIC client
└── Cargo.toml
```

## Data Flow

```
┌─────────────┐   ┌──────────────┐   ┌────────────────┐   ┌───────────────┐
│  Capture    │──▶│  Difference  │──▶│  Raymarch      │──▶│  Network      │
│  (60 FPS)   │   │  Detection   │   │  (sparse rays) │   │  (QUIC)       │
└─────────────┘   └──────────────┘   └────────────────┘   └───────────────┘
       │                 │                   │                    │
       ▼                 ▼                   ▼                    ▼
   GrayscaleFrame   DifferenceMask    VoxelContribution[]    CameraFrame
```

---

## Module Implementation Details

### capture.rs - Video Capture

This module abstracts camera hardware access.

#### Current State
- `CameraCapture` trait defined
- `DummyCamera` implementation for testing
- `GrayscaleFrame` data structure

#### TODO: V4L2 Implementation (Linux)

```rust
use v4l::buffer::Type;
use v4l::io::mmap::Stream;
use v4l::video::Capture;
use v4l::Device;
use v4l::FourCC;

pub struct V4L2Camera {
    stream: Stream<'static>,
    width: u32,
    height: u32,
}

impl V4L2Camera {
    pub fn open(device_path: &str, width: u32, height: u32, fps: u32) -> Result<Self, CaptureError> {
        let device = Device::with_path(device_path)
            .map_err(|e| CaptureError::DeviceOpen(e.to_string()))?;

        // Set format to grayscale or YUV (we'll extract Y channel)
        let mut format = device.format()?;
        format.width = width;
        format.height = height;
        format.fourcc = FourCC::new(b"YUYV"); // Common format, extract Y
        device.set_format(&format)?;

        // Set frame rate
        let mut params = device.params()?;
        params.interval = v4l::Fraction::new(1, fps);
        device.set_params(&params)?;

        // Create memory-mapped stream
        let stream = Stream::with_buffers(&device, Type::VideoCapture, 4)?;

        Ok(Self { stream, width, height })
    }

    fn yuyv_to_grayscale(&self, yuyv: &[u8]) -> Vec<u8> {
        // YUYV format: Y0 U0 Y1 V0 Y2 U1 Y3 V1 ...
        // Extract every other byte (Y values)
        yuyv.iter().step_by(2).copied().collect()
    }
}

impl CameraCapture for V4L2Camera {
    fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn capture_grayscale(&mut self) -> Result<GrayscaleFrame, CaptureError> {
        let (buffer, _metadata) = self.stream.next()
            .map_err(|e| CaptureError::Capture(e.to_string()))?;

        let grayscale = self.yuyv_to_grayscale(buffer);

        Ok(GrayscaleFrame {
            width: self.width,
            height: self.height,
            data: grayscale,
        })
    }
}
```

#### TODO: Frame Rate Control

Implement precise timing for consistent frame rates:

```rust
pub struct TimedCapture<C: CameraCapture> {
    camera: C,
    target_interval: Duration,
    last_capture: Instant,
}

impl<C: CameraCapture> TimedCapture<C> {
    pub fn new(camera: C, fps: u32) -> Self {
        Self {
            camera,
            target_interval: Duration::from_secs_f64(1.0 / fps as f64),
            last_capture: Instant::now(),
        }
    }

    pub async fn capture_at_rate(&mut self) -> Result<GrayscaleFrame, CaptureError> {
        let elapsed = self.last_capture.elapsed();
        if elapsed < self.target_interval {
            smol::Timer::after(self.target_interval - elapsed).await;
        }
        self.last_capture = Instant::now();
        self.camera.capture_grayscale()
    }
}
```

---

### difference.rs - Motion Detection

This module detects motion by comparing consecutive frames.

#### Current State
- `DifferenceMask` stores per-pixel motion values
- `FrameProcessor` computes frame differences with fixed threshold

#### TODO: Adaptive Thresholding

Handle varying lighting conditions:

```rust
pub struct AdaptiveFrameProcessor {
    previous_frame: Option<GrayscaleFrame>,
    base_threshold: u8,
    adaptation_rate: f32,
    scene_brightness: f32,
}

impl AdaptiveFrameProcessor {
    pub fn new(base_threshold: u8, adaptation_rate: f32) -> Self {
        Self {
            previous_frame: None,
            base_threshold,
            adaptation_rate,
            scene_brightness: 128.0,
        }
    }

    pub fn compute_difference(&mut self, current: GrayscaleFrame) -> Option<DifferenceMask> {
        // Update scene brightness estimate (exponential moving average)
        let current_brightness = current.pixels().iter()
            .map(|&p| p as f32)
            .sum::<f32>() / current.pixels().len() as f32;

        self.scene_brightness = self.scene_brightness * (1.0 - self.adaptation_rate)
            + current_brightness * self.adaptation_rate;

        // Adjust threshold based on brightness
        // Darker scenes need lower thresholds, brighter scenes higher
        let brightness_factor = (self.scene_brightness / 128.0).clamp(0.5, 2.0);
        let adaptive_threshold = (self.base_threshold as f32 * brightness_factor) as u8;

        let result = if let Some(ref previous) = self.previous_frame {
            let mut mask = DifferenceMask::new(current.width, current.height);

            for (i, (curr, prev)) in current.pixels().iter()
                .zip(previous.pixels().iter())
                .enumerate()
            {
                let diff = curr.abs_diff(*prev);
                if diff > adaptive_threshold {
                    mask.set(i, diff);
                }
            }

            Some(mask)
        } else {
            None
        };

        self.previous_frame = Some(current);
        result
    }
}
```

#### TODO: Noise Filtering

Add morphological operations to reduce noise:

```rust
impl DifferenceMask {
    /// Apply erosion to remove isolated noise pixels
    pub fn erode(&mut self) {
        let mut eroded = vec![0u8; self.data.len()];

        for y in 1..self.height - 1 {
            for x in 1..self.width - 1 {
                let idx = (y * self.width + x) as usize;

                // Check 3x3 neighborhood - pixel survives only if all neighbors are active
                let min_neighbor = [
                    self.get(x - 1, y - 1), self.get(x, y - 1), self.get(x + 1, y - 1),
                    self.get(x - 1, y),     self.get(x, y),     self.get(x + 1, y),
                    self.get(x - 1, y + 1), self.get(x, y + 1), self.get(x + 1, y + 1),
                ].into_iter().min().unwrap();

                eroded[idx] = min_neighbor;
            }
        }

        self.data = eroded;
    }

    /// Apply dilation to fill small gaps
    pub fn dilate(&mut self) {
        let mut dilated = vec![0u8; self.data.len()];

        for y in 1..self.height - 1 {
            for x in 1..self.width - 1 {
                let idx = (y * self.width + x) as usize;

                let max_neighbor = [
                    self.get(x - 1, y - 1), self.get(x, y - 1), self.get(x + 1, y - 1),
                    self.get(x - 1, y),     self.get(x, y),     self.get(x + 1, y),
                    self.get(x - 1, y + 1), self.get(x, y + 1), self.get(x + 1, y + 1),
                ].into_iter().max().unwrap();

                dilated[idx] = max_neighbor;
            }
        }

        self.data = dilated;
    }

    /// Opening operation (erode then dilate) - removes noise while preserving shape
    pub fn open(&mut self) {
        self.erode();
        self.dilate();
    }
}
```

---

### localization.rs - GPS + IMU Fusion

This module provides camera position and orientation.

#### Current State
- `Localizer` trait defined
- `DummyLocalizer` for testing
- `GpsImuLocalizer` placeholder

#### TODO: GPS Reader

```rust
use std::io::{BufRead, BufReader};
use std::fs::File;

pub struct GpsReader {
    reader: BufReader<File>,
    last_fix: Option<GpsFix>,
}

#[derive(Debug, Clone)]
pub struct GpsFix {
    pub position: GeoPosition,
    pub timestamp: u64,
    pub fix_quality: u8,
    pub satellites: u8,
    pub hdop: f32,
}

impl GpsReader {
    pub fn open(device: &str) -> Result<Self, LocalizationError> {
        let file = File::open(device)
            .map_err(|_| LocalizationError::GpsUnavailable)?;

        Ok(Self {
            reader: BufReader::new(file),
            last_fix: None,
        })
    }

    pub fn read_fix(&mut self) -> Result<Option<GpsFix>, LocalizationError> {
        let mut line = String::new();
        self.reader.read_line(&mut line)
            .map_err(|_| LocalizationError::GpsUnavailable)?;

        // Parse NMEA sentences
        if line.starts_with("$GPGGA") || line.starts_with("$GNGGA") {
            if let Some(fix) = self.parse_gga(&line) {
                self.last_fix = Some(fix.clone());
                return Ok(Some(fix));
            }
        }

        Ok(None)
    }

    fn parse_gga(&self, line: &str) -> Option<GpsFix> {
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 15 {
            return None;
        }

        let lat = parse_nmea_coord(parts[2], parts[3])?;
        let lon = parse_nmea_coord(parts[4], parts[5])?;
        let alt: f64 = parts[9].parse().ok()?;
        let fix_quality: u8 = parts[6].parse().ok()?;
        let satellites: u8 = parts[7].parse().ok()?;
        let hdop: f32 = parts[8].parse().ok()?;

        Some(GpsFix {
            position: GeoPosition::new(lat, lon, alt),
            timestamp: now_micros(),
            fix_quality,
            satellites,
            hdop,
        })
    }
}

fn parse_nmea_coord(value: &str, direction: &str) -> Option<f64> {
    let deg_len = if value.len() > 10 { 3 } else { 2 }; // Lon has 3 digits
    let degrees: f64 = value[..deg_len].parse().ok()?;
    let minutes: f64 = value[deg_len..].parse().ok()?;
    let mut coord = degrees + minutes / 60.0;

    if direction == "S" || direction == "W" {
        coord = -coord;
    }

    Some(coord)
}
```

#### TODO: IMU Reader

```rust
pub struct ImuReader {
    // I2C device handle
    device: I2cDevice,
    // Calibration offsets
    accel_offset: Vec3,
    gyro_offset: Vec3,
}

#[derive(Debug, Clone)]
pub struct ImuReading {
    pub acceleration: Vec3,  // m/s²
    pub angular_velocity: Vec3,  // rad/s
    pub timestamp: u64,
}

impl ImuReader {
    pub fn open(bus: &str, address: u8) -> Result<Self, LocalizationError> {
        // Open I2C device and configure IMU
        // This is hardware-specific (MPU6050, BNO055, etc.)
        todo!("Implement for specific IMU hardware")
    }

    pub fn read(&mut self) -> Result<ImuReading, LocalizationError> {
        // Read accelerometer and gyroscope registers
        // Apply calibration offsets
        // Convert to physical units
        todo!("Implement for specific IMU hardware")
    }

    pub fn calibrate(&mut self) -> Result<(), LocalizationError> {
        // Collect samples while stationary
        // Compute offsets for zero readings
        todo!()
    }
}
```

#### TODO: Sensor Fusion

Implement complementary filter or Kalman filter:

```rust
pub struct SensorFusion {
    // Current orientation estimate
    orientation: Quat,
    // Complementary filter weight (0.98 = trust gyro, 0.02 = trust accel)
    alpha: f32,
    last_update: Instant,
}

impl SensorFusion {
    pub fn new() -> Self {
        Self {
            orientation: Quat::IDENTITY,
            alpha: 0.98,
            last_update: Instant::now(),
        }
    }

    pub fn update(&mut self, imu: &ImuReading) -> Quat {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update).as_secs_f32();
        self.last_update = now;

        // Integrate gyroscope
        let gyro_delta = Quat::from_scaled_axis(imu.angular_velocity * dt);
        let gyro_orientation = self.orientation * gyro_delta;

        // Calculate orientation from accelerometer (gravity direction)
        let accel_normalized = imu.acceleration.normalize();
        let accel_orientation = orientation_from_gravity(accel_normalized);

        // Complementary filter: blend gyro (fast, drifts) with accel (slow, stable)
        self.orientation = gyro_orientation.slerp(accel_orientation, 1.0 - self.alpha);
        self.orientation = self.orientation.normalize();

        self.orientation
    }
}

fn orientation_from_gravity(gravity: Vec3) -> Quat {
    // Calculate rotation from world-up to measured gravity direction
    let up = Vec3::Y;
    let axis = up.cross(gravity).normalize();
    let angle = up.dot(gravity).acos();

    if axis.is_finite() {
        Quat::from_axis_angle(axis, angle)
    } else {
        Quat::IDENTITY
    }
}
```

#### TODO: Complete GPS+IMU Localizer

```rust
impl GpsImuLocalizer {
    pub fn new(gps_device: &str, imu_device: &str, fallback_mode: GpsFallbackMode) -> Result<Self, LocalizationError> {
        Ok(Self {
            gps: GpsReader::open(gps_device)?,
            imu: ImuReader::open(imu_device)?,
            fusion: SensorFusion::new(),
            last_gps_fix: None,
            fallback_mode,
            status: LocalizationStatus::Unavailable,
        })
    }
}

impl Localizer for GpsImuLocalizer {
    fn get_pose(&mut self) -> Result<CameraPose, LocalizationError> {
        // Try to get GPS fix
        if let Ok(Some(fix)) = self.gps.read_fix() {
            self.last_gps_fix = Some(fix.clone());
            self.status = LocalizationStatus::Nominal;
        }

        // Always read IMU for orientation
        let imu_reading = self.imu.read()?;
        let orientation = self.fusion.update(&imu_reading);

        // Determine position based on GPS status
        let (position, status) = match (&self.last_gps_fix, self.fallback_mode) {
            (Some(fix), _) if fix.timestamp > now_micros() - 5_000_000 => {
                // GPS fix is recent (< 5 seconds old)
                (fix.position, LocalizationStatus::Nominal)
            }
            (Some(fix), GpsFallbackMode::DeadReckoning) => {
                // TODO: Integrate IMU to estimate movement since last fix
                (fix.position, LocalizationStatus::DeadReckoning {
                    duration_ms: (now_micros() - fix.timestamp) / 1000,
                })
            }
            (Some(fix), GpsFallbackMode::MarkUncertain) => {
                (fix.position, LocalizationStatus::DeadReckoning {
                    duration_ms: (now_micros() - fix.timestamp) / 1000,
                })
            }
            (_, GpsFallbackMode::Pause) | (None, _) => {
                return Err(LocalizationError::NoFix);
            }
        };

        self.status = status;

        Ok(CameraPose {
            position,
            orientation,
            timestamp: now_micros(),
            uncertainty: self.compute_uncertainty(),
            status,
        })
    }

    fn status(&self) -> LocalizationStatus {
        self.status
    }
}
```

---

### raymarch.rs - Voxel Raymarching

This module projects motion rays into the voxel grid.

#### Current State
- `Raymarcher` struct with basic ray generation
- Fixed-step raymarching algorithm
- Voxel contribution collection

#### TODO: Subpixel Sampling

For better accuracy, sample multiple rays per motion pixel:

```rust
impl Raymarcher {
    pub fn raymarch_subpixel(
        &self,
        pose: &CameraPose,
        mask: &DifferenceMask,
        samples_per_pixel: u32,
    ) -> Vec<VoxelContribution> {
        let mut contributions: HashMap<(u32, u32, u32), f32> = HashMap::new();

        for (x, y, intensity) in mask.motion_pixels() {
            // Generate multiple rays per pixel with jittered offsets
            for _ in 0..samples_per_pixel {
                let jitter_x = rand::random::<f32>() - 0.5;
                let jitter_y = rand::random::<f32>() - 0.5;

                let ray = self.pixel_to_ray(
                    pose,
                    x as f32 + jitter_x,
                    y as f32 + jitter_y,
                    intensity as f32 / samples_per_pixel as f32,
                );

                self.march_ray(&ray, &mut contributions);
            }
        }

        // Convert to output format
        contributions.into_iter()
            .map(|((x, y, z), intensity)| VoxelContribution {
                index: UVec3::new(x, y, z),
                intensity,
            })
            .collect()
    }
}
```

#### TODO: Cone Casting

Instead of single rays, cast cones that spread with distance:

```rust
pub struct ConeCaster {
    base_config: RaymarchConfig,
    cone_angle: f32,  // radians, spread per unit distance
}

impl ConeCaster {
    /// Cast a cone and distribute intensity across voxels it intersects
    pub fn cast_cone(
        &self,
        ray: &Ray,
        contributions: &mut HashMap<(u32, u32, u32), f32>,
    ) {
        let mut t = 0.0;

        while t < self.base_config.max_distance {
            let center = ray.origin + ray.direction * t;

            // Cone radius at this distance
            let radius = t * self.cone_angle.tan();

            // Sample voxels within cone radius
            let attenuation = self.base_config.attenuation.compute(t);
            let voxel_count = self.sample_cone_voxels(center, radius, ray.intensity * attenuation, contributions);

            // Adjust step size based on cone radius
            let step = (self.base_config.step_size).max(radius * 0.5);
            t += step;
        }
    }

    fn sample_cone_voxels(
        &self,
        center: Vec3,
        radius: f32,
        intensity: f32,
        contributions: &mut HashMap<(u32, u32, u32), f32>,
    ) -> usize {
        // Sample voxels in a sphere around the center point
        // Distribute intensity based on distance from center
        todo!()
    }
}
```

---

### network.rs - QUIC Client

This module handles communication with the server.

#### Current State
- `NetworkClient` placeholder structure
- Message type definitions

#### TODO: Implement QUIC Connection

```rust
use quinn::{ClientConfig, Endpoint, Connection};
use std::sync::Arc;

pub struct QuicClient {
    endpoint: Endpoint,
    connection: Option<Connection>,
    server_addr: SocketAddr,
    camera_id: CameraId,
    config: ClientConfig,
}

impl QuicClient {
    pub fn new(server_addr: &str, camera_id: CameraId) -> Result<Self, NetworkError> {
        let server_addr: SocketAddr = server_addr.parse()
            .map_err(|e| NetworkError::Connection(format!("Invalid address: {e}")))?;

        // Configure TLS (for now, accept any certificate)
        let crypto = rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(Arc::new(SkipServerVerification))
            .with_no_client_auth();

        let config = ClientConfig::new(Arc::new(
            quinn::crypto::rustls::QuicClientConfig::try_from(crypto)
                .map_err(|e| NetworkError::Connection(e.to_string()))?
        ));

        let mut endpoint = Endpoint::client("0.0.0.0:0".parse().unwrap())
            .map_err(|e| NetworkError::Connection(e.to_string()))?;

        endpoint.set_default_client_config(config.clone());

        Ok(Self {
            endpoint,
            connection: None,
            server_addr,
            camera_id,
            config,
        })
    }

    pub async fn connect(&mut self) -> Result<(), NetworkError> {
        let connection = self.endpoint
            .connect(self.server_addr, "iluvatar-server")
            .map_err(|e| NetworkError::Connection(e.to_string()))?
            .await
            .map_err(|e| NetworkError::Connection(e.to_string()))?;

        self.connection = Some(connection);
        Ok(())
    }

    pub async fn send_frame(&self, frame: CameraFrame) -> Result<(), NetworkError> {
        let conn = self.connection.as_ref()
            .ok_or_else(|| NetworkError::Send("Not connected".to_string()))?;

        let msg = CameraMessage::Frame(frame);
        let data = protocol::serialize(&msg)?;

        // Open a unidirectional stream and send
        let mut send = conn.open_uni().await
            .map_err(|e| NetworkError::Send(e.to_string()))?;

        send.write_all(&data).await
            .map_err(|e| NetworkError::Send(e.to_string()))?;

        send.finish()
            .map_err(|e| NetworkError::Send(e.to_string()))?;

        Ok(())
    }
}
```

#### TODO: Reconnection Logic

```rust
impl QuicClient {
    pub async fn ensure_connected(&mut self) -> Result<(), NetworkError> {
        if self.is_connected() {
            return Ok(());
        }

        let mut delay = Duration::from_secs(1);
        let max_delay = Duration::from_secs(30);

        loop {
            match self.connect().await {
                Ok(()) => {
                    tracing::info!("Connected to server");
                    return Ok(());
                }
                Err(e) => {
                    tracing::warn!("Connection failed: {e}, retrying in {:?}", delay);
                    smol::Timer::after(delay).await;
                    delay = (delay * 2).min(max_delay);
                }
            }
        }
    }

    fn is_connected(&self) -> bool {
        self.connection.as_ref()
            .map(|c| !c.close_reason().is_some())
            .unwrap_or(false)
    }
}
```

---

## Main Loop Implementation

The main binary ties everything together:

```rust
// main.rs

async fn run_camera(config: CameraConfig) -> Result<(), Box<dyn std::error::Error>> {
    // Initialize components
    let mut camera = V4L2Camera::open(&config.hardware.device, ...)?;
    let mut processor = AdaptiveFrameProcessor::new(config.processing.difference_threshold, 0.01);
    let mut localizer = GpsImuLocalizer::new(...)?;
    let mut network = QuicClient::new(&config.network.server_address, config.identity.camera_id)?;

    // Get grid configuration from server
    network.ensure_connected().await?;
    network.register(make_registration(&config, &localizer)).await?;
    let grid_config = network.receive_config().await?;

    let raymarcher = Raymarcher::new(
        config.to_intrinsics(),
        config.processing.to_raymarch_config(),
        grid_config.bounds(),
        grid_config.voxel_size,
    );

    // Main processing loop
    let mut sequence = 0u64;
    let target_interval = Duration::from_secs_f64(1.0 / config.hardware.fps as f64);

    loop {
        let frame_start = Instant::now();

        // Capture frame
        let grayscale = camera.capture_grayscale()?;

        // Get current pose
        let pose = match localizer.get_pose() {
            Ok(pose) => pose,
            Err(LocalizationError::NoFix) if config.localization.fallback_mode == "pause" => {
                tracing::warn!("No GPS fix, skipping frame");
                continue;
            }
            Err(e) => return Err(e.into()),
        };

        // Compute difference mask
        if let Some(mut mask) = processor.compute_difference(grayscale) {
            // Filter noise
            mask.open();

            // Only raymarch if there's significant motion
            if mask.motion_count() > 100 {
                // Raymarch
                let contributions = raymarcher.raymarch(&pose, &mask);

                // Send to server
                let frame = CameraFrame {
                    camera_id: config.identity.camera_id,
                    sequence,
                    timestamp: pose.timestamp,
                    pose,
                    contributions,
                };

                if let Err(e) = network.send_frame(frame).await {
                    tracing::error!("Failed to send frame: {e}");
                    network.ensure_connected().await?;
                }

                sequence += 1;
            }
        }

        // Maintain frame rate
        let elapsed = frame_start.elapsed();
        if elapsed < target_interval {
            smol::Timer::after(target_interval - elapsed).await;
        } else {
            tracing::warn!("Frame took {:?}, exceeding target {:?}", elapsed, target_interval);
        }
    }
}

fn main() {
    tracing_subscriber::fmt::init();

    let config = CameraConfig::load_or_default(Path::new("config/camera.toml"));

    smol::block_on(async {
        if let Err(e) = run_camera(config).await {
            tracing::error!("Fatal error: {e}");
            std::process::exit(1);
        }
    });
}
```

---

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_difference_detection() {
        let mut processor = FrameProcessor::new(10);

        let frame1 = GrayscaleFrame { width: 10, height: 10, data: vec![100; 100] };
        assert!(processor.compute_difference(frame1).is_none());

        let mut frame2_data = vec![100u8; 100];
        frame2_data[50..].fill(150); // Change bottom half
        let frame2 = GrayscaleFrame { width: 10, height: 10, data: frame2_data };

        let mask = processor.compute_difference(frame2).unwrap();
        assert_eq!(mask.motion_count(), 50);
    }

    #[test]
    fn test_raymarch_bounds() {
        // Ensure rays don't contribute outside grid bounds
        let raymarcher = Raymarcher::new(...);
        // Test edge cases
    }
}
```

### Integration Tests

```rust
#[test]
fn test_full_pipeline() {
    // Create dummy camera with known pattern
    // Process through entire pipeline
    // Verify expected voxel contributions
}
```

---

## Implementation Priority

1. **Phase 1: Basic Functionality**
   - V4L2 camera capture
   - Fixed-threshold difference detection
   - Basic raymarching
   - QUIC connection (without TLS)

2. **Phase 2: Robustness**
   - GPS + IMU integration
   - Adaptive thresholding
   - Reconnection handling
   - Error recovery

3. **Phase 3: Optimization**
   - Subpixel sampling
   - Cone casting
   - SIMD-optimized difference computation
   - Frame rate optimization
