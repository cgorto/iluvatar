# Iluvatar: Multi-Camera Volumetric Motion Detection System

## Technical Specification v1.0

---

## 1. Executive Summary

Iluvatar is a distributed motion detection system that uses multiple cameras to detect and localize moving objects in 3D space. By projecting motion-detected pixels as rays into a shared voxel grid, the system achieves accurate 3D positioning of moving targets through the intersection of rays from multiple viewpoints.

### Key Characteristics
- **Scale**: 1km+ outdoor coverage
- **Resolution**: 1 meter voxel size
- **Latency**: <100ms target
- **Cameras**: 5-20 per deployment
- **Frame Rate**: 60 FPS

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                           DEPLOYMENT                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   ┌──────────┐  ┌──────────┐  ┌──────────┐       ┌──────────┐      │
│   │ Camera 1 │  │ Camera 2 │  │ Camera 3 │  ...  │ Camera N │      │
│   │          │  │          │  │          │       │          │      │
│   │ GPS+IMU  │  │ GPS+IMU  │  │ GPS+IMU  │       │ GPS+IMU  │      │
│   │ CMOS     │  │ CMOS     │  │ CMOS     │       │ CMOS     │      │
│   │ Raymarch │  │ Raymarch │  │ Raymarch │       │ Raymarch │      │
│   └────┬─────┘  └────┬─────┘  └────┬─────┘       └────┬─────┘      │
│        │             │             │                  │             │
│        └──────────┬──┴─────────────┴────┬─────────────┘             │
│                   │    QUIC/Cellular    │                           │
│                   ▼                     ▼                           │
│        ┌─────────────────────────────────────────┐                  │
│        │            MAIN SERVER                  │                  │
│        │  ┌─────────────────────────────────┐   │                  │
│        │  │    Voxel Grid Aggregator        │   │                  │
│        │  │    (Sparse Storage + Decay)     │   │                  │
│        │  └─────────────────────────────────┘   │                  │
│        │  ┌─────────────────────────────────┐   │                  │
│        │  │    Object Detector/Clusterer    │   │                  │
│        │  └─────────────────────────────────┘   │                  │
│        │  ┌─────────────────────────────────┐   │                  │
│        │  │    Data Persistence Layer       │   │                  │
│        │  └─────────────────────────────────┘   │                  │
│        └────────────────┬────────────────────────┘                  │
│                         │ WebSocket                                 │
│                         ▼                                           │
│        ┌─────────────────────────────────────────┐                  │
│        │           WEB CLIENT                    │                  │
│        │    (CesiumJS + Satellite Imagery)       │                  │
│        └─────────────────────────────────────────┘                  │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.1 Component Overview

| Component | Language | Runtime | Purpose |
|-----------|----------|---------|---------|
| Camera Unit | Rust | smol | Capture, difference detection, raymarching |
| Main Server | Rust | smol (or tokio) | Aggregation, object detection, client serving |
| Web Client | TypeScript | Browser | Visualization, monitoring |
| Simulator | Rust | Bevy | Testing and validation |

---

## 3. Camera Unit

### 3.1 Hardware Requirements

- Custom embedded platform
- GPS module with antenna
- IMU (accelerometer + gyroscope + magnetometer)
- Standard CMOS camera sensor (60-120° FOV)
- Cellular modem (LTE/5G)
- Sufficient compute for 60 FPS raymarching

### 3.2 Localization

**Primary**: GPS + IMU fusion

```rust
pub struct CameraPose {
    /// Position in WGS84 coordinates
    pub position: GeoPosition,
    /// Orientation as quaternion (relative to ENU frame)
    pub orientation: Quaternion,
    /// Timestamp of pose measurement (GPS time)
    pub timestamp: Timestamp,
    /// Uncertainty estimate
    pub uncertainty: PoseUncertainty,
    /// Localization status
    pub status: LocalizationStatus,
}

pub struct GeoPosition {
    pub latitude: f64,   // degrees
    pub longitude: f64,  // degrees
    pub altitude: f64,   // meters above WGS84 ellipsoid
}

pub struct PoseUncertainty {
    pub position_stddev: Vec3,     // meters
    pub orientation_stddev: Vec3,  // radians
}

pub enum LocalizationStatus {
    /// Full GPS + IMU fusion
    Nominal,
    /// GPS degraded, IMU dead-reckoning
    DeadReckoning { duration: Duration },
    /// No reliable position
    Unavailable,
}
```

**GPS Fallback Behavior** (configurable):
1. **Dead-reckoning**: Continue with IMU-only, accept drift
2. **Uncertainty marking**: Tag contributions with high uncertainty
3. **Pause contribution**: Stop contributing until GPS recovers

### 3.3 Frame Processing Pipeline

```
┌─────────────┐   ┌──────────────┐   ┌────────────────┐   ┌───────────────┐
│ Capture     │──▶│ Difference   │──▶│ Threshold &    │──▶│ Raymarch &    │
│ Frame       │   │ Computation  │   │ Filter         │   │ Transmit      │
└─────────────┘   └──────────────┘   └────────────────┘   └───────────────┘
     60 FPS           ~16ms              ~16ms                 ~16ms
```

#### 3.3.1 Difference Computation

```rust
pub struct FrameProcessor {
    previous_frame: GrayscaleFrame,
    threshold: u8,  // Fixed threshold for MVP
}

impl FrameProcessor {
    /// Compute difference mask between current and previous frame
    pub fn compute_difference(&mut self, current: &GrayscaleFrame) -> DifferenceMask {
        let mut mask = DifferenceMask::new(current.width(), current.height());

        for (i, (curr, prev)) in current.pixels()
            .zip(self.previous_frame.pixels())
            .enumerate()
        {
            let diff = curr.abs_diff(*prev);
            if diff > self.threshold {
                mask.set(i, diff);
            }
        }

        self.previous_frame = current.clone();
        mask
    }
}
```

#### 3.3.2 Ray Generation

Only rays for pixels with detected motion are generated (sparse output).

```rust
pub struct Ray {
    /// Ray origin in world coordinates (camera position)
    pub origin: Vec3,
    /// Normalized ray direction
    pub direction: Vec3,
    /// Motion intensity (difference value)
    pub intensity: f32,
}

pub struct CameraIntrinsics {
    pub focal_length: Vec2,
    pub principal_point: Vec2,
    pub resolution: UVec2,
    pub fov: Fov,
}

pub struct Fov {
    pub horizontal: f32,  // radians
    pub vertical: f32,    // radians
}

impl Camera {
    /// Generate ray for a pixel with detected motion
    pub fn pixel_to_ray(&self, pixel: UVec2, intensity: f32) -> Ray {
        let ndc = self.pixel_to_ndc(pixel);
        let direction_camera = Vec3::new(
            ndc.x * (self.intrinsics.fov.horizontal / 2.0).tan(),
            ndc.y * (self.intrinsics.fov.vertical / 2.0).tan(),
            1.0,
        ).normalize();

        let direction_world = self.pose.orientation * direction_camera;

        Ray {
            origin: self.pose.position.to_local_coords(&self.grid_origin),
            direction: direction_world,
            intensity,
        }
    }
}
```

#### 3.3.3 Local Raymarching

Each camera marches rays into a local fixed-resolution grid, then sends sparse voxel contributions.

```rust
pub struct RaymarchConfig {
    /// Maximum ray distance in meters
    pub max_distance: f32,
    /// Step size in meters
    pub step_size: f32,
    /// Distance attenuation mode
    pub attenuation: AttenuationMode,
}

pub enum AttenuationMode {
    None,
    Linear { max_distance: f32 },
    InverseSquare { reference_distance: f32 },
}

pub struct VoxelContribution {
    /// Voxel index in the grid
    pub index: UVec3,
    /// Intensity contribution
    pub intensity: f32,
}

impl Camera {
    pub fn raymarch(&self, rays: &[Ray], config: &RaymarchConfig) -> Vec<VoxelContribution> {
        let mut contributions = HashMap::new();

        for ray in rays {
            let mut t = 0.0;
            while t < config.max_distance {
                let point = ray.origin + ray.direction * t;
                let voxel_idx = self.world_to_voxel(point);

                if self.grid_bounds.contains(voxel_idx) {
                    let attenuation = config.attenuation.compute(t);
                    let contribution = ray.intensity * attenuation;

                    contributions
                        .entry(voxel_idx)
                        .and_modify(|v| *v += contribution)
                        .or_insert(contribution);
                }

                t += config.step_size;
            }
        }

        contributions
            .into_iter()
            .map(|(index, intensity)| VoxelContribution { index, intensity })
            .collect()
    }
}
```

### 3.4 Network Protocol

**Protocol**: QUIC

```rust
/// Message sent from camera to server
pub struct CameraFrame {
    /// Unique camera identifier
    pub camera_id: CameraId,
    /// Frame sequence number
    pub sequence: u64,
    /// Timestamp (GPS time, microseconds since epoch)
    pub timestamp: u64,
    /// Camera pose at capture time
    pub pose: CameraPose,
    /// Sparse voxel contributions
    pub contributions: Vec<VoxelContribution>,
}

/// Wire format (for serialization)
/// Using postcard or similar compact binary format
pub const MAX_CONTRIBUTIONS_PER_FRAME: usize = 65536;
```

**Connection Management**:
- Automatic reconnection on disconnect
- Exponential backoff with jitter
- Heartbeat every 1 second

---

## 4. Main Server

### 4.1 Voxel Grid

#### 4.1.1 Coordinate System

Fixed world grid with origin computed from camera positions.

```rust
pub struct GridConfig {
    /// Grid origin in WGS84
    pub origin: GeoPosition,
    /// Grid dimensions in voxels
    pub dimensions: UVec3,
    /// Voxel size in meters
    pub voxel_size: f32,  // 1.0 for MVP
}

impl GridConfig {
    /// Compute grid bounds from camera positions and FOVs
    pub fn from_cameras(cameras: &[CameraRegistration]) -> Self {
        // Find bounding box of all camera frustums
        let mut min = Vec3::splat(f32::MAX);
        let mut max = Vec3::splat(f32::MIN);

        for camera in cameras {
            let frustum_bounds = camera.compute_frustum_bounds();
            min = min.min(frustum_bounds.min);
            max = max.max(frustum_bounds.max);
        }

        // Add margin and compute grid
        let margin = 100.0; // meters
        let size = max - min + Vec3::splat(margin * 2.0);
        let dimensions = (size / VOXEL_SIZE).ceil().as_uvec3();

        Self {
            origin: GeoPosition::from_local(min - Vec3::splat(margin)),
            dimensions,
            voxel_size: VOXEL_SIZE,
        }
    }
}
```

#### 4.1.2 Sparse Storage

Given 1m resolution over 1km³ (1 billion potential voxels), sparse storage is required.

```rust
use std::collections::HashMap;

pub struct SparseVoxelGrid {
    config: GridConfig,
    /// Only stores non-zero voxels
    voxels: HashMap<VoxelIndex, Voxel>,
    /// Decay rate per second
    decay_rate: f32,
    /// Last update timestamp
    last_update: Instant,
}

pub type VoxelIndex = u64;  // Packed UVec3

pub struct Voxel {
    /// Accumulated intensity
    pub intensity: f32,
    /// Number of cameras contributing
    pub contributor_count: u8,
    /// Last update time
    pub last_update: Instant,
}

impl SparseVoxelGrid {
    /// Pack 3D index into 64-bit key
    /// Supports up to 2^21 (~2 million) voxels per axis
    fn pack_index(x: u32, y: u32, z: u32) -> VoxelIndex {
        ((x as u64) << 42) | ((y as u64) << 21) | (z as u64)
    }

    fn unpack_index(idx: VoxelIndex) -> UVec3 {
        UVec3::new(
            ((idx >> 42) & 0x1FFFFF) as u32,
            ((idx >> 21) & 0x1FFFFF) as u32,
            (idx & 0x1FFFFF) as u32,
        )
    }

    /// Apply time decay to all voxels
    pub fn apply_decay(&mut self) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update).as_secs_f32();
        let decay_factor = (-self.decay_rate * dt).exp();

        self.voxels.retain(|_, voxel| {
            voxel.intensity *= decay_factor;
            voxel.intensity > INTENSITY_THRESHOLD
        });

        self.last_update = now;
    }

    /// Add contributions from a camera frame
    pub fn add_contributions(&mut self, frame: &CameraFrame) {
        for contrib in &frame.contributions {
            let idx = Self::pack_index(contrib.index.x, contrib.index.y, contrib.index.z);

            self.voxels
                .entry(idx)
                .and_modify(|v| {
                    v.intensity += contrib.intensity;
                    v.contributor_count = v.contributor_count.saturating_add(1);
                    v.last_update = Instant::now();
                })
                .or_insert(Voxel {
                    intensity: contrib.intensity,
                    contributor_count: 1,
                    last_update: Instant::now(),
                });
        }
    }
}
```

### 4.2 Frame Aggregation

Cameras operate asynchronously; the server interpolates based on timestamps.

```rust
pub struct FrameAggregator {
    /// Per-camera frame buffers for interpolation
    camera_buffers: HashMap<CameraId, FrameBuffer>,
    /// Time window for aggregation
    aggregation_window: Duration,
}

pub struct FrameBuffer {
    frames: VecDeque<CameraFrame>,
    max_frames: usize,
}

impl FrameAggregator {
    /// Aggregate all frames within the time window
    pub fn aggregate(&mut self, target_time: u64) -> Vec<&CameraFrame> {
        let window_start = target_time.saturating_sub(self.aggregation_window.as_micros() as u64);
        let window_end = target_time;

        self.camera_buffers
            .values()
            .filter_map(|buffer| {
                // Get most recent frame within window
                buffer.frames.iter().rev().find(|f| {
                    f.timestamp >= window_start && f.timestamp <= window_end
                })
            })
            .collect()
    }
}
```

### 4.3 Object Detection

#### 4.3.1 Point Cloud Extraction

```rust
pub struct DetectionConfig {
    /// Minimum intensity to consider a voxel "active"
    pub intensity_threshold: f32,
    /// Minimum contributor count for confidence
    pub min_contributors: u8,
}

impl SparseVoxelGrid {
    pub fn extract_point_cloud(&self, config: &DetectionConfig) -> Vec<DetectedPoint> {
        self.voxels
            .iter()
            .filter(|(_, v)| {
                v.intensity >= config.intensity_threshold
                    && v.contributor_count >= config.min_contributors
            })
            .map(|(idx, v)| {
                let pos = Self::unpack_index(*idx);
                DetectedPoint {
                    position: self.voxel_to_world(pos),
                    intensity: v.intensity,
                    confidence: v.contributor_count as f32 / self.active_cameras() as f32,
                }
            })
            .collect()
    }
}

pub struct DetectedPoint {
    pub position: Vec3,
    pub intensity: f32,
    pub confidence: f32,
}
```

#### 4.3.2 Clustering (DBSCAN)

```rust
pub struct ClusterConfig {
    /// Maximum distance between points in a cluster (meters)
    pub epsilon: f32,
    /// Minimum points to form a cluster
    pub min_points: usize,
}

pub struct TrackedObject {
    pub id: ObjectId,
    pub centroid: Vec3,
    pub bounding_box: BoundingBox,
    pub point_count: usize,
    pub total_intensity: f32,
    pub velocity: Option<Vec3>,  // Computed from history
}

pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl ObjectDetector {
    pub fn cluster_points(&self, points: &[DetectedPoint]) -> Vec<TrackedObject> {
        // DBSCAN clustering implementation
        let clusters = dbscan(points, self.config.epsilon, self.config.min_points);

        clusters.into_iter().map(|cluster| {
            let centroid = cluster.iter()
                .map(|p| p.position)
                .sum::<Vec3>() / cluster.len() as f32;

            let min = cluster.iter().map(|p| p.position).reduce(Vec3::min).unwrap();
            let max = cluster.iter().map(|p| p.position).reduce(Vec3::max).unwrap();

            TrackedObject {
                id: self.assign_or_track_id(centroid),
                centroid,
                bounding_box: BoundingBox { min, max },
                point_count: cluster.len(),
                total_intensity: cluster.iter().map(|p| p.intensity).sum(),
                velocity: None,  // Computed by tracker
            }
        }).collect()
    }
}
```

#### 4.3.3 Object Tracking

```rust
pub struct ObjectTracker {
    /// Active tracked objects
    objects: HashMap<ObjectId, TrackedObjectState>,
    /// Next available ID
    next_id: ObjectId,
    /// Maximum distance to associate detection with existing track
    association_threshold: f32,
    /// Frames before dropping unmatched track
    max_missing_frames: u32,
}

pub struct TrackedObjectState {
    pub object: TrackedObject,
    pub history: VecDeque<Vec3>,  // Position history for velocity
    pub missing_frames: u32,
}

impl ObjectTracker {
    pub fn update(&mut self, detections: Vec<TrackedObject>) -> Vec<TrackedObject> {
        // Hungarian algorithm or greedy matching
        let assignments = self.associate_detections(&detections);

        let mut output = Vec::new();

        for (detection_idx, object_id) in assignments {
            let detection = &detections[detection_idx];

            if let Some(state) = self.objects.get_mut(&object_id) {
                // Update existing track
                state.history.push_back(detection.centroid);
                if state.history.len() > 10 {
                    state.history.pop_front();
                }

                let velocity = if state.history.len() >= 2 {
                    let dt = 1.0 / 60.0;  // Assuming 60 FPS
                    Some((state.history.back().unwrap() - state.history.front().unwrap())
                         / (state.history.len() as f32 * dt))
                } else {
                    None
                };

                state.object = TrackedObject {
                    velocity,
                    ..detection.clone()
                };
                state.missing_frames = 0;

                output.push(state.object.clone());
            }
        }

        // Create new tracks for unmatched detections
        // Age out old tracks
        // ...

        output
    }
}
```

### 4.4 Data Persistence

```rust
pub struct PersistenceConfig {
    /// Enable persistence
    pub enabled: bool,
    /// Retention duration
    pub retention: Duration,
    /// Storage path
    pub path: PathBuf,
    /// Snapshot interval
    pub snapshot_interval: Duration,
}

pub struct PersistenceLayer {
    config: PersistenceConfig,
    /// Write-ahead log for crash recovery
    wal: WriteAheadLog,
    /// Periodic snapshots
    snapshots: SnapshotManager,
}

/// Stored frame for replay
pub struct StoredFrame {
    pub timestamp: u64,
    pub objects: Vec<TrackedObject>,
    pub raw_voxels: Option<CompressedVoxelData>,
}
```

### 4.5 Camera Management

```rust
pub struct CameraRegistry {
    cameras: HashMap<CameraId, CameraState>,
}

pub struct CameraState {
    pub id: CameraId,
    pub registration: CameraRegistration,
    pub connection_state: ConnectionState,
    pub last_frame_time: Option<Instant>,
    pub stats: CameraStats,
}

pub struct CameraRegistration {
    pub id: CameraId,
    pub intrinsics: CameraIntrinsics,
    pub initial_pose: CameraPose,
}

pub enum ConnectionState {
    Connected { since: Instant },
    Disconnected { since: Instant, reconnect_attempts: u32 },
}

pub struct CameraStats {
    pub frames_received: u64,
    pub bytes_received: u64,
    pub average_latency: Duration,
    pub contribution_rate: f32,  // contributions per frame
}
```

---

## 5. Web Client

### 5.1 Technology Stack

- **Framework**: TypeScript + modern bundler (Vite)
- **3D Visualization**: CesiumJS
- **Real-time Communication**: WebSocket
- **State Management**: Minimal (single site, view-only MVP)

### 5.2 WebSocket Protocol

```typescript
// Server -> Client messages
interface ServerMessage {
  type: 'snapshot' | 'update' | 'camera_status' | 'system_status';
}

interface SnapshotMessage extends ServerMessage {
  type: 'snapshot';
  timestamp: number;
  objects: TrackedObject[];
  pointCloud?: Point3D[];  // Optional raw point cloud
  gridBounds: BoundingBox;
  cameraStates: CameraStatusMap;
}

interface UpdateMessage extends ServerMessage {
  type: 'update';
  timestamp: number;
  objects: TrackedObject[];
  // Differential updates for efficiency
}

interface TrackedObject {
  id: string;
  position: [number, number, number];  // lat, lon, alt
  boundingBox: BoundingBox;
  velocity?: [number, number, number];
  confidence: number;
}
```

### 5.3 Visualization Features

```typescript
interface ViewerConfig {
  // Satellite imagery base layer
  imageryProvider: 'bing' | 'mapbox' | 'osm';

  // Voxel grid visualization
  showVoxelGrid: boolean;
  voxelOpacity: number;

  // Object rendering
  objectStyle: 'point' | 'box' | 'both';
  showTrails: boolean;
  trailLength: number;

  // Camera visualization
  showCameraFrustums: boolean;
  showCameraStatus: boolean;
}

class IluvatarViewer {
  private viewer: Cesium.Viewer;
  private objectEntities: Map<string, Cesium.Entity>;
  private cameraEntities: Map<string, Cesium.Entity>;

  constructor(container: HTMLElement, config: ViewerConfig) {
    this.viewer = new Cesium.Viewer(container, {
      terrainProvider: Cesium.createWorldTerrain(),
      imageryProvider: this.createImageryProvider(config.imageryProvider),
    });

    this.objectEntities = new Map();
    this.cameraEntities = new Map();
  }

  updateObjects(objects: TrackedObject[]): void {
    // Update or create entities for each tracked object
    for (const obj of objects) {
      if (this.objectEntities.has(obj.id)) {
        this.updateObjectEntity(obj);
      } else {
        this.createObjectEntity(obj);
      }
    }

    // Remove entities for objects no longer tracked
    // ...
  }

  updateCameras(cameras: CameraStatusMap): void {
    // Visualize camera positions, FOV frustums, and status
    // ...
  }
}
```

### 5.4 UI Components

```
┌─────────────────────────────────────────────────────────────────────┐
│  ILUVATAR CONTROL CENTER                          [Status: Online]  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│                    ┌──────────────────────────┐                     │
│                    │                          │                     │
│                    │      CESIUM 3D VIEW      │                     │
│                    │                          │                     │
│                    │   [Objects rendered as   │                     │
│                    │    points/boxes with     │                     │
│                    │    motion trails]        │                     │
│                    │                          │                     │
│                    └──────────────────────────┘                     │
│                                                                      │
├─────────────────────────────────────────────────────────────────────┤
│  TRACKED OBJECTS (3)                                                │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ OBJ-001  │ Position: 47.6062°N, 122.3321°W, 152m │ v=12m/s │   │
│  │ OBJ-002  │ Position: 47.6055°N, 122.3315°W, 89m  │ v=5m/s  │   │
│  │ OBJ-003  │ Position: 47.6070°N, 122.3300°W, 201m │ v=28m/s │   │
│  └─────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────┤
│  CAMERAS (5/5 Online)                                               │
│  ● CAM-01  ● CAM-02  ● CAM-03  ● CAM-04  ● CAM-05                  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6. Simulator

### 6.1 Purpose

The Bevy-based simulator validates the complete pipeline without physical hardware by:
1. Rendering a 3D scene with moving objects
2. Simulating camera captures at configurable positions
3. Feeding simulated frames through the same processing pipeline
4. Verifying detection accuracy against ground truth

### 6.2 Architecture

```rust
use bevy::prelude::*;

pub struct SimulatorPlugin;

impl Plugin for SimulatorPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_plugins(DefaultPlugins)
            .add_systems(Startup, setup_scene)
            .add_systems(Update, (
                move_targets,
                capture_camera_frames,
                send_to_pipeline,
                compare_with_ground_truth,
            ));
    }
}

#[derive(Component)]
pub struct SimulatedCamera {
    pub camera_id: CameraId,
    pub intrinsics: CameraIntrinsics,
}

#[derive(Component)]
pub struct SimulatedTarget {
    pub ground_truth_id: u32,
    pub velocity: Vec3,
}

fn setup_scene(mut commands: Commands) {
    // Create simulated cameras at configured positions
    // Create moving target objects
    // Set up ground plane and environment
}

fn capture_camera_frames(
    cameras: Query<(&SimulatedCamera, &Transform, &Camera)>,
    // ...
) {
    // For each simulated camera:
    // 1. Render scene from camera viewpoint
    // 2. Compute difference from previous frame
    // 3. Generate rays for motion pixels
    // 4. Package as CameraFrame
}
```

### 6.3 Test Scenarios

```rust
pub enum TestScenario {
    /// Single object moving in a straight line
    SingleLinear {
        start: Vec3,
        end: Vec3,
        duration: Duration,
    },
    /// Single object moving in a curve
    SingleCurved {
        path: BezierPath,
        duration: Duration,
    },
    /// Multiple objects, no occlusion
    MultipleSimple {
        count: usize,
        bounds: BoundingBox,
    },
    /// Objects passing behind occluders
    Occlusion {
        target_path: BezierPath,
        occluder_positions: Vec<Vec3>,
    },
    /// Random motion for stress testing
    Chaos {
        object_count: usize,
        duration: Duration,
    },
}
```

---

## 7. Crate Organization

```
iluvatar/
├── Cargo.toml                 # Workspace definition
├── crates/
│   ├── iluvatar-core/         # Shared types and utilities
│   │   ├── src/
│   │   │   ├── lib.rs
│   │   │   ├── types.rs       # CameraPose, VoxelContribution, etc.
│   │   │   ├── geo.rs         # Coordinate transformations
│   │   │   ├── protocol.rs    # Wire protocol definitions
│   │   │   └── config.rs      # Configuration types
│   │   └── Cargo.toml
│   │
│   ├── iluvatar-camera/       # Camera unit implementation
│   │   ├── src/
│   │   │   ├── lib.rs
│   │   │   ├── main.rs        # Camera binary entry point
│   │   │   ├── capture.rs     # Frame capture abstraction
│   │   │   ├── difference.rs  # Motion detection
│   │   │   ├── raymarch.rs    # Ray generation and marching
│   │   │   ├── localization.rs # GPS + IMU fusion
│   │   │   └── network.rs     # QUIC client
│   │   └── Cargo.toml
│   │
│   ├── iluvatar-server/       # Main server implementation
│   │   ├── src/
│   │   │   ├── lib.rs
│   │   │   ├── main.rs        # Server binary entry point
│   │   │   ├── grid.rs        # Sparse voxel grid
│   │   │   ├── aggregator.rs  # Frame aggregation
│   │   │   ├── detector.rs    # Object detection (clustering)
│   │   │   ├── tracker.rs     # Object tracking
│   │   │   ├── persistence.rs # Data storage
│   │   │   ├── camera_mgmt.rs # Camera registry
│   │   │   └── websocket.rs   # Client WebSocket server
│   │   └── Cargo.toml
│   │
│   ├── iluvatar-client/       # Web client
│   │   ├── src/
│   │   │   ├── main.ts
│   │   │   ├── viewer.ts      # CesiumJS integration
│   │   │   ├── websocket.ts   # Server connection
│   │   │   └── ui/
│   │   ├── index.html
│   │   ├── package.json
│   │   └── vite.config.ts
│   │
│   └── iluvatar-simulator/    # Bevy simulator
│       ├── src/
│       │   ├── lib.rs
│       │   ├── main.rs
│       │   ├── scene.rs       # Scene setup
│       │   ├── targets.rs     # Moving objects
│       │   ├── cameras.rs     # Simulated cameras
│       │   └── validation.rs  # Ground truth comparison
│       └── Cargo.toml
│
├── config/
│   ├── camera.example.toml
│   ├── server.example.toml
│   └── simulator.example.toml
│
└── docs/
    └── SPEC.md               # This document
```

---

## 8. Key Dependencies

### 8.1 Rust Crates

```toml
# iluvatar-core
[dependencies]
glam = "0.27"           # Vector math
serde = { version = "1.0", features = ["derive"] }
postcard = "1.0"        # Compact binary serialization

# iluvatar-camera
[dependencies]
smol = "2.0"            # Async runtime
quinn = "0.11"          # QUIC implementation
v4l = "0.14"            # Video4Linux camera capture (Linux)
image = "0.25"          # Image processing

# iluvatar-server
[dependencies]
smol = "2.0"
quinn = "0.11"
tokio-tungstenite = "0.21"  # WebSocket (may use smol equivalent)
dashmap = "5.5"         # Concurrent hashmap
parking_lot = "0.12"    # Better mutexes

# iluvatar-simulator
[dependencies]
bevy = "0.18"
```

### 8.2 TypeScript/Web

```json
{
  "dependencies": {
    "cesium": "^1.115",
    "vite": "^5.0"
  }
}
```

---

## 9. Configuration

### 9.1 Camera Configuration

```toml
# camera.toml

[identity]
camera_id = "cam-001"

[hardware]
device = "/dev/video0"
resolution = [1920, 1080]
fps = 60
fov_horizontal = 90.0  # degrees
fov_vertical = 60.0

[localization]
gps_device = "/dev/ttyUSB0"
imu_device = "/dev/i2c-1"
fallback_mode = "dead_reckoning"  # or "uncertain" or "pause"

[processing]
difference_threshold = 25
raymarch_max_distance = 500.0  # meters
raymarch_step_size = 0.5       # meters

[attenuation]
mode = "linear"  # or "none" or "inverse_square"
max_distance = 500.0

[network]
server_address = "server.example.com:4433"
reconnect_interval = 5  # seconds
```

### 9.2 Server Configuration

```toml
# server.toml

[server]
listen_address = "0.0.0.0:4433"
websocket_port = 8080

[grid]
voxel_size = 1.0  # meters
# Bounds auto-computed from cameras, or:
# origin = { lat = 47.6062, lon = -122.3321, alt = 0.0 }
# dimensions = [1000, 1000, 500]

[decay]
rate = 0.5  # per second
update_interval = 0.1  # seconds

[detection]
intensity_threshold = 10.0
min_contributors = 2
cluster_epsilon = 5.0  # meters
cluster_min_points = 3

[persistence]
enabled = true
path = "/var/lib/iluvatar/data"
retention = "24h"
snapshot_interval = "5m"
```

---

## 10. Performance Considerations

### 10.1 Bandwidth Estimation

With motion-only rays at 60 FPS:
- Assume 1% of pixels show motion: ~20,000 pixels (1920x1080)
- Each VoxelContribution: ~12 bytes (index + intensity)
- Raymarch produces ~10 voxels per ray on average
- Per frame: 20,000 × 10 × 12 = ~2.4 MB
- Per second: ~144 MB/s (uncompressed)

**Optimizations needed**:
- Run-length encoding for consecutive voxel indices
- Delta encoding between frames
- Lossy quantization of intensity values
- Adaptive frame rate based on motion amount

### 10.2 Memory Estimation

Sparse voxel grid at 1m resolution, 1km³:
- Typical active voxels: 0.001% = 10,000 voxels
- Per voxel storage: ~32 bytes (with metadata)
- Base memory: ~320 KB (manageable)
- Peak with motion: potentially 1M voxels = ~32 MB

### 10.3 Latency Budget

Target: <100ms end-to-end

| Stage | Budget |
|-------|--------|
| Camera capture | 16ms (60 FPS) |
| Difference + raymarch | 10ms |
| Network (cellular) | 40-60ms |
| Server aggregation | 5ms |
| Detection + tracking | 5ms |
| WebSocket to client | 10ms |
| **Total** | ~90-100ms |

---

## 11. Future Enhancements

### 11.1 Designed For, Implement Later

- **Authentication**: mTLS or token-based camera auth
- **Object Classification**: ML-based identification
- **Dynamic Resolution**: Octree voxel grid
- **Multi-modal Sensors**: Thermal, IR, neuromorphic cameras
- **Multi-site**: Support for multiple deployments

### 11.2 Potential Optimizations

- GPU-accelerated raymarching (CUDA/compute shaders)
- Predictive frame interpolation for latency hiding
- Adaptive grid resolution based on activity
- P2P camera mesh for reduced server load

---

## 12. Glossary

| Term | Definition |
|------|------------|
| **Voxel** | Volumetric pixel; a 3D grid cell |
| **Raymarch** | Stepping along a ray and sampling at regular intervals |
| **Difference Mask** | Per-pixel absolute difference between consecutive frames |
| **FOV** | Field of View; angular extent visible to camera |
| **DBSCAN** | Density-Based Spatial Clustering of Applications with Noise |
| **ENU** | East-North-Up local coordinate frame |
| **WGS84** | World Geodetic System 1984; standard GPS coordinate system |
| **IMU** | Inertial Measurement Unit |
| **PPS** | Pulse Per Second (GPS timing signal) |
| **QUIC** | Quick UDP Internet Connections protocol |

---

*Document generated for Project Iluvatar*
*Version 1.0 - Initial Specification*
