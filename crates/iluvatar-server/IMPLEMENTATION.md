# iluvatar-server Implementation Guide

This crate implements the central aggregation server that receives data from cameras, maintains the voxel grid, detects and tracks objects, and serves data to web clients.

## Overview

```
iluvatar-server/
├── src/
│   ├── lib.rs           # Module exports
│   ├── main.rs          # Binary entry point
│   ├── grid.rs          # Sparse voxel grid
│   ├── aggregator.rs    # Frame aggregation
│   ├── detector.rs      # Object detection (DBSCAN)
│   ├── tracker.rs       # Object tracking
│   ├── camera_mgmt.rs   # Camera registry
│   ├── persistence.rs   # Data storage
│   └── websocket.rs     # Client WebSocket server
└── Cargo.toml
```

## Architecture

```
                     ┌─────────────────────────────────────────────┐
                     │              QUIC Server (4433)              │
                     │  ┌─────────────────────────────────────────┐│
                     │  │          Camera Handler Tasks           ││
                     │  │   (one per connected camera)            ││
                     │  └──────────────────┬──────────────────────┘│
                     └────────────────────┬┴───────────────────────┘
                                          │
                                          ▼
┌─────────────────┐   ┌──────────────────────────────────────────────┐
│ Camera Registry │◀──│              Frame Aggregator                │
│  (camera_mgmt)  │   │  - Buffers frames from all cameras           │
└─────────────────┘   │  - Synchronizes by timestamp                 │
                      └──────────────────┬───────────────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────────────┐
                      │            Sparse Voxel Grid                 │
                      │  - Accumulates contributions                 │
                      │  - Applies time decay                        │
                      └──────────────────┬───────────────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────────────┐
                      │           Object Detector                    │
                      │  - Extracts active voxels                    │
                      │  - DBSCAN clustering                         │
                      └──────────────────┬───────────────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────────────┐
                      │           Object Tracker                     │
                      │  - Associates detections across frames       │
                      │  - Computes velocities                       │
                      └──────────────────┬───────────────────────────┘
                                         │
          ┌──────────────────────────────┼──────────────────────────────┐
          ▼                              ▼                              ▼
┌─────────────────┐           ┌─────────────────┐           ┌─────────────────┐
│   Persistence   │           │ WebSocket Server│           │   Metrics       │
│  (historical)   │           │    (8080)       │           │  (optional)     │
└─────────────────┘           └─────────────────┘           └─────────────────┘
```

---

## Module Implementation Details

### grid.rs - Sparse Voxel Grid

This is the core data structure that accumulates ray contributions from all cameras.

#### Current State
- `SparseVoxelGrid` with `DashMap` for concurrent access
- Pack/unpack functions for 64-bit voxel indices
- Basic decay and contribution accumulation

#### TODO: Improve Memory Efficiency

For very large grids, consider chunked storage:

```rust
const CHUNK_SIZE: u32 = 64;

pub struct ChunkedVoxelGrid {
    chunks: DashMap<ChunkIndex, Chunk>,
    origin: GeoPosition,
    voxel_size: f32,
    decay_rate: f32,
}

type ChunkIndex = (u32, u32, u32);

struct Chunk {
    voxels: HashMap<u16, Voxel>,  // Local index within chunk (16-bit)
    last_access: Instant,
}

impl ChunkedVoxelGrid {
    fn global_to_chunk(&self, index: UVec3) -> (ChunkIndex, u16) {
        let chunk = (
            index.x / CHUNK_SIZE,
            index.y / CHUNK_SIZE,
            index.z / CHUNK_SIZE,
        );
        let local = (
            (index.x % CHUNK_SIZE) * CHUNK_SIZE * CHUNK_SIZE +
            (index.y % CHUNK_SIZE) * CHUNK_SIZE +
            (index.z % CHUNK_SIZE)
        ) as u16;
        (chunk, local)
    }

    pub fn add_contribution(&self, index: UVec3, intensity: f32) {
        let (chunk_idx, local_idx) = self.global_to_chunk(index);

        self.chunks.entry(chunk_idx)
            .or_insert_with(|| Chunk {
                voxels: HashMap::new(),
                last_access: Instant::now(),
            })
            .voxels
            .entry(local_idx)
            .and_modify(|v| {
                v.intensity += intensity;
                v.contributor_count += 1;
                v.last_update = Instant::now();
            })
            .or_insert(Voxel {
                intensity,
                contributor_count: 1,
                last_update: Instant::now(),
            });
    }

    /// Remove empty chunks to save memory
    pub fn gc_empty_chunks(&self) {
        self.chunks.retain(|_, chunk| !chunk.voxels.is_empty());
    }
}
```

#### TODO: Parallel Decay

Use rayon for parallel decay processing:

```rust
use rayon::prelude::*;

impl SparseVoxelGrid {
    pub fn apply_decay_parallel(&self) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_decay.load()).as_secs_f32();
        let decay_factor = (-self.decay_rate * dt).exp();

        // Process chunks in parallel
        self.voxels.par_iter_mut().for_each(|mut entry| {
            entry.value_mut().intensity *= decay_factor;
        });

        // Remove dead voxels (can't do in parallel with DashMap easily)
        self.voxels.retain(|_, v| v.intensity > INTENSITY_THRESHOLD);

        self.last_decay.store(now);
    }
}
```

#### TODO: Grid Bounds Auto-Computation

```rust
impl SparseVoxelGrid {
    /// Compute grid dimensions from camera frustums
    pub fn compute_bounds_from_cameras(
        cameras: &[CameraRegistration],
        max_distance: f32,
        margin: f32,
    ) -> (GeoPosition, UVec3) {
        let mut min_local = Vec3::splat(f32::MAX);
        let mut max_local = Vec3::splat(f32::MIN);

        // Use first camera position as origin reference
        let origin = cameras.first()
            .map(|c| c.initial_pose.position)
            .unwrap_or(GeoPosition::new(0.0, 0.0, 0.0));

        let coord_system = LocalCoordinateSystem::new(origin);

        for camera in cameras {
            let cam_local = coord_system.geo_to_local(&camera.initial_pose.position);

            // Compute frustum corners at max distance
            let frustum_corners = compute_frustum_corners(
                &camera.initial_pose,
                &camera.intrinsics,
                max_distance,
            );

            for corner in frustum_corners {
                min_local = min_local.min(corner);
                max_local = max_local.max(corner);
            }

            // Include camera position
            min_local = min_local.min(cam_local);
            max_local = max_local.max(cam_local);
        }

        // Add margin
        min_local -= Vec3::splat(margin);
        max_local += Vec3::splat(margin);

        // Ensure origin is at min corner
        let adjusted_origin = coord_system.local_to_geo(min_local);
        let size = max_local - min_local;
        let dimensions = UVec3::new(
            size.x.ceil() as u32,
            size.y.ceil() as u32,
            size.z.ceil() as u32,
        );

        (adjusted_origin, dimensions)
    }
}
```

---

### aggregator.rs - Frame Aggregation

This module buffers and synchronizes frames from multiple cameras.

#### Current State
- `FrameAggregator` with per-camera buffers
- Timestamp-based frame selection

#### TODO: Interpolation for Better Synchronization

```rust
impl FrameAggregator {
    /// Get interpolated contributions at exact target time
    pub fn get_interpolated_contributions(
        &self,
        target_time: u64,
    ) -> Vec<InterpolatedContribution> {
        let mut all_contributions = Vec::new();

        for buffer in self.camera_buffers.values() {
            if let Some((before, after)) = buffer.get_bracketing_frames(target_time) {
                // Linear interpolation factor
                let t = (target_time - before.timestamp) as f32
                    / (after.timestamp - before.timestamp) as f32;

                // For position-based contributions, interpolate poses
                let interpolated_pose = interpolate_pose(&before.pose, &after.pose, t);

                // Use the frame closer to target time for contributions
                let contributions = if t < 0.5 {
                    &before.contributions
                } else {
                    &after.contributions
                };

                all_contributions.extend(contributions.iter().cloned());
            }
        }

        all_contributions
    }
}

fn interpolate_pose(a: &CameraPose, b: &CameraPose, t: f32) -> CameraPose {
    CameraPose {
        position: GeoPosition {
            latitude: a.position.latitude + (b.position.latitude - a.position.latitude) * t as f64,
            longitude: a.position.longitude + (b.position.longitude - a.position.longitude) * t as f64,
            altitude: a.position.altitude + (b.position.altitude - a.position.altitude) * t as f64,
        },
        orientation: a.orientation.slerp(b.orientation, t),
        timestamp: a.timestamp + ((b.timestamp - a.timestamp) as f32 * t) as u64,
        uncertainty: a.uncertainty, // Could interpolate this too
        status: a.status,
    }
}
```

---

### detector.rs - Object Detection

DBSCAN clustering to group active voxels into discrete objects.

#### Current State
- Basic DBSCAN implementation
- Cluster to TrackedObject conversion

#### TODO: Optimize DBSCAN with Spatial Index

```rust
use std::collections::BTreeMap;

/// Spatial hash grid for O(1) neighbor lookup
pub struct SpatialIndex {
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
    cell_size: f32,
}

impl SpatialIndex {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cells: HashMap::new(),
            cell_size,
        }
    }

    pub fn build(&mut self, points: &[DetectedPoint]) {
        self.cells.clear();

        for (i, point) in points.iter().enumerate() {
            let cell = self.position_to_cell(point.position);
            self.cells.entry(cell).or_default().push(i);
        }
    }

    fn position_to_cell(&self, pos: Vec3) -> (i32, i32, i32) {
        (
            (pos.x / self.cell_size).floor() as i32,
            (pos.y / self.cell_size).floor() as i32,
            (pos.z / self.cell_size).floor() as i32,
        )
    }

    /// Find all points within distance of a query point
    pub fn query_radius(&self, points: &[DetectedPoint], center: Vec3, radius: f32) -> Vec<usize> {
        let radius_sq = radius * radius;
        let cell_radius = (radius / self.cell_size).ceil() as i32;
        let center_cell = self.position_to_cell(center);

        let mut result = Vec::new();

        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                for dz in -cell_radius..=cell_radius {
                    let cell = (
                        center_cell.0 + dx,
                        center_cell.1 + dy,
                        center_cell.2 + dz,
                    );

                    if let Some(indices) = self.cells.get(&cell) {
                        for &idx in indices {
                            if points[idx].position.distance_squared(center) <= radius_sq {
                                result.push(idx);
                            }
                        }
                    }
                }
            }
        }

        result
    }
}

impl ObjectDetector {
    pub fn detect_optimized(&mut self, points: &[DetectedPoint]) -> Vec<TrackedObject> {
        if points.is_empty() {
            return Vec::new();
        }

        // Build spatial index
        let mut index = SpatialIndex::new(self.config.cluster_epsilon);
        index.build(points);

        // Run DBSCAN with spatial index
        let clusters = self.dbscan_with_index(points, &index);

        clusters.into_iter()
            .filter(|c| c.len() >= self.config.cluster_min_points)
            .map(|c| self.cluster_to_object(c))
            .collect()
    }

    fn dbscan_with_index<'a>(
        &self,
        points: &'a [DetectedPoint],
        index: &SpatialIndex,
    ) -> Vec<Vec<&'a DetectedPoint>> {
        let mut visited = vec![false; points.len()];
        let mut clusters = Vec::new();

        for i in 0..points.len() {
            if visited[i] {
                continue;
            }

            let neighbors = index.query_radius(points, points[i].position, self.config.cluster_epsilon);

            if neighbors.len() < self.config.cluster_min_points {
                continue;
            }

            visited[i] = true;
            let mut cluster = vec![&points[i]];
            let mut seeds: Vec<usize> = neighbors;

            while let Some(q) = seeds.pop() {
                if visited[q] {
                    continue;
                }

                visited[q] = true;
                cluster.push(&points[q]);

                let q_neighbors = index.query_radius(points, points[q].position, self.config.cluster_epsilon);
                if q_neighbors.len() >= self.config.cluster_min_points {
                    seeds.extend(q_neighbors);
                }
            }

            clusters.push(cluster);
        }

        clusters
    }
}
```

---

### tracker.rs - Object Tracking

Associates detections across frames and computes velocities.

#### Current State
- Simple distance-based association
- Position history for velocity computation

#### TODO: Hungarian Algorithm for Optimal Assignment

```rust
use std::collections::HashMap;

impl ObjectTracker {
    /// Optimal assignment using Hungarian algorithm
    fn associate_hungarian(
        &self,
        detections: &[TrackedObject],
    ) -> Vec<(usize, ObjectId)> {
        let track_ids: Vec<ObjectId> = self.tracks.keys().copied().collect();

        if track_ids.is_empty() || detections.is_empty() {
            return Vec::new();
        }

        // Build cost matrix
        let mut cost_matrix = vec![vec![f32::MAX; track_ids.len()]; detections.len()];

        for (d_idx, detection) in detections.iter().enumerate() {
            for (t_idx, &track_id) in track_ids.iter().enumerate() {
                let track = &self.tracks[&track_id];
                let predicted = self.predict_position(track);
                let distance = predicted.distance(detection.centroid);

                if distance <= self.association_threshold {
                    cost_matrix[d_idx][t_idx] = distance;
                }
            }
        }

        // Run Hungarian algorithm (using munkres crate or similar)
        let assignments = hungarian_algorithm(&cost_matrix);

        assignments.into_iter()
            .filter_map(|(d_idx, t_idx)| {
                if cost_matrix[d_idx][t_idx] < f32::MAX {
                    Some((d_idx, track_ids[t_idx]))
                } else {
                    None
                }
            })
            .collect()
    }
}
```

#### TODO: Kalman Filter for Better Prediction

```rust
pub struct KalmanTracker {
    // State: [x, y, z, vx, vy, vz]
    state: [f32; 6],
    // Covariance matrix
    covariance: [[f32; 6]; 6],
    // Process noise
    q: f32,
    // Measurement noise
    r: f32,
}

impl KalmanTracker {
    pub fn new(initial_position: Vec3) -> Self {
        let mut state = [0.0; 6];
        state[0] = initial_position.x;
        state[1] = initial_position.y;
        state[2] = initial_position.z;

        Self {
            state,
            covariance: identity_matrix_scaled(100.0), // High initial uncertainty
            q: 1.0,
            r: 1.0,
        }
    }

    pub fn predict(&mut self, dt: f32) -> Vec3 {
        // State transition: position += velocity * dt
        self.state[0] += self.state[3] * dt;
        self.state[1] += self.state[4] * dt;
        self.state[2] += self.state[5] * dt;

        // Update covariance (simplified)
        for i in 0..6 {
            self.covariance[i][i] += self.q * dt;
        }

        Vec3::new(self.state[0], self.state[1], self.state[2])
    }

    pub fn update(&mut self, measurement: Vec3) {
        // Kalman gain (simplified for position-only measurement)
        let k = self.covariance[0][0] / (self.covariance[0][0] + self.r);

        // Update state
        let innovation = [
            measurement.x - self.state[0],
            measurement.y - self.state[1],
            measurement.z - self.state[2],
        ];

        for i in 0..3 {
            self.state[i] += k * innovation[i];
        }

        // Update velocity estimate from innovation
        // (simplified - full Kalman would use measurement model)

        // Update covariance
        for i in 0..6 {
            self.covariance[i][i] *= 1.0 - k;
        }
    }

    pub fn position(&self) -> Vec3 {
        Vec3::new(self.state[0], self.state[1], self.state[2])
    }

    pub fn velocity(&self) -> Vec3 {
        Vec3::new(self.state[3], self.state[4], self.state[5])
    }
}
```

---

### camera_mgmt.rs - Camera Registry

Manages connected cameras and their state.

#### Current State
- `CameraRegistry` with basic registration
- Connection state tracking
- FPS calculation

#### TODO: Health Monitoring

```rust
impl CameraRegistry {
    /// Check for cameras that haven't sent data recently
    pub fn check_health(&mut self) -> Vec<CameraHealthIssue> {
        let mut issues = Vec::new();
        let now = Instant::now();

        for (id, state) in &self.cameras {
            // Check for stale data
            if let Some(last_frame) = state.last_frame_time {
                let age = now.duration_since(last_frame);
                if age > Duration::from_secs(5) {
                    issues.push(CameraHealthIssue::StaleData {
                        camera_id: *id,
                        last_frame_age: age,
                    });
                }
            }

            // Check for low FPS
            if state.fps < 30.0 && state.fps > 0.0 {
                issues.push(CameraHealthIssue::LowFrameRate {
                    camera_id: *id,
                    fps: state.fps,
                });
            }

            // Check localization status
            match state.last_pose.status {
                LocalizationStatus::DeadReckoning { duration_ms } if duration_ms > 30000 => {
                    issues.push(CameraHealthIssue::GpsDegradation {
                        camera_id: *id,
                        duration: Duration::from_millis(duration_ms),
                    });
                }
                LocalizationStatus::Unavailable => {
                    issues.push(CameraHealthIssue::NoLocalization {
                        camera_id: *id,
                    });
                }
                _ => {}
            }
        }

        issues
    }
}

pub enum CameraHealthIssue {
    StaleData { camera_id: CameraId, last_frame_age: Duration },
    LowFrameRate { camera_id: CameraId, fps: f32 },
    GpsDegradation { camera_id: CameraId, duration: Duration },
    NoLocalization { camera_id: CameraId },
    Disconnected { camera_id: CameraId },
}
```

---

### QUIC Server Implementation

#### TODO: Implement Camera Connection Handler

```rust
use quinn::{Endpoint, ServerConfig, Connection, RecvStream};

pub struct QuicServer {
    endpoint: Endpoint,
}

impl QuicServer {
    pub async fn bind(addr: SocketAddr) -> Result<Self, ServerError> {
        // Generate self-signed certificate for development
        let cert = rcgen::generate_simple_self_signed(vec!["localhost".to_string()])?;
        let key = rustls::pki_types::PrivateKeyDer::Pkcs8(cert.key_pair.serialize_der().into());
        let cert = rustls::pki_types::CertificateDer::from(cert.cert);

        let mut server_crypto = rustls::ServerConfig::builder()
            .with_no_client_auth()
            .with_single_cert(vec![cert], key)?;

        let server_config = ServerConfig::with_crypto(Arc::new(
            quinn::crypto::rustls::QuicServerConfig::try_from(server_crypto)?
        ));

        let endpoint = Endpoint::server(server_config, addr)?;

        Ok(Self { endpoint })
    }

    pub async fn run(
        &self,
        frame_tx: async_channel::Sender<CameraFrame>,
        registry: Arc<RwLock<CameraRegistry>>,
    ) {
        while let Some(connecting) = self.endpoint.accept().await {
            let frame_tx = frame_tx.clone();
            let registry = registry.clone();

            smol::spawn(async move {
                match connecting.await {
                    Ok(connection) => {
                        Self::handle_connection(connection, frame_tx, registry).await;
                    }
                    Err(e) => {
                        tracing::error!("Connection failed: {e}");
                    }
                }
            }).detach();
        }
    }

    async fn handle_connection(
        connection: Connection,
        frame_tx: async_channel::Sender<CameraFrame>,
        registry: Arc<RwLock<CameraRegistry>>,
    ) {
        let mut camera_id: Option<CameraId> = None;

        loop {
            match connection.accept_uni().await {
                Ok(stream) => {
                    if let Err(e) = Self::handle_stream(
                        stream,
                        &frame_tx,
                        &registry,
                        &mut camera_id,
                    ).await {
                        tracing::warn!("Stream error: {e}");
                    }
                }
                Err(e) => {
                    tracing::info!("Connection closed: {e}");
                    break;
                }
            }
        }

        // Mark camera as disconnected
        if let Some(id) = camera_id {
            registry.write().await.disconnect(id);
        }
    }

    async fn handle_stream(
        mut stream: RecvStream,
        frame_tx: &async_channel::Sender<CameraFrame>,
        registry: &Arc<RwLock<CameraRegistry>>,
        camera_id: &mut Option<CameraId>,
    ) -> Result<(), ServerError> {
        let data = stream.read_to_end(MAX_MESSAGE_SIZE).await?;
        let msg: CameraMessage = protocol::deserialize(&data)?;

        match msg {
            CameraMessage::Register(registration) => {
                *camera_id = Some(registration.camera_id);
                registry.write().await.register(registration);
                tracing::info!("Camera {} registered", camera_id.unwrap());
            }
            CameraMessage::Frame(frame) => {
                if let Some(id) = camera_id {
                    registry.write().await.record_frame(*id, frame.pose);
                    frame_tx.send(frame).await?;
                }
            }
            CameraMessage::Heartbeat { camera_id: id, .. } => {
                registry.write().await.connect(id);
            }
        }

        Ok(())
    }
}
```

---

### websocket.rs - WebSocket Server

Serves real-time updates to web clients.

#### Current State
- Message building functions
- JSON serialization

#### TODO: Full WebSocket Server

```rust
use async_tungstenite::tungstenite::Message;
use futures_lite::StreamExt;

pub struct WebSocketServer {
    clients: Arc<DashMap<u64, ClientConnection>>,
    next_client_id: AtomicU64,
}

impl WebSocketServer {
    pub async fn run(
        &self,
        addr: SocketAddr,
        mut update_rx: async_channel::Receiver<WorldUpdate>,
    ) -> Result<(), ServerError> {
        let listener = smol::net::TcpListener::bind(addr).await?;
        tracing::info!("WebSocket server listening on {}", addr);

        // Spawn broadcast task
        let clients = self.clients.clone();
        smol::spawn(async move {
            while let Ok(update) = update_rx.recv().await {
                Self::broadcast(&clients, update).await;
            }
        }).detach();

        // Accept connections
        loop {
            let (stream, peer_addr) = listener.accept().await?;
            let clients = self.clients.clone();
            let client_id = self.next_client_id.fetch_add(1, Ordering::Relaxed);

            smol::spawn(async move {
                if let Err(e) = Self::handle_client(stream, client_id, clients).await {
                    tracing::warn!("Client {} error: {e}", peer_addr);
                }
            }).detach();
        }
    }

    async fn handle_client(
        stream: smol::net::TcpStream,
        client_id: u64,
        clients: Arc<DashMap<u64, ClientConnection>>,
    ) -> Result<(), ServerError> {
        let ws_stream = async_tungstenite::accept_async(stream).await?;
        let (mut write, mut read) = ws_stream.split();

        clients.insert(client_id, ClientConnection::new(client_id));
        tracing::info!("Client {} connected", client_id);

        // Handle incoming messages
        while let Some(msg) = read.next().await {
            match msg? {
                Message::Text(text) => {
                    let client_msg: ClientMessage = serde_json::from_str(&text)?;
                    Self::handle_client_message(client_id, client_msg, &clients);
                }
                Message::Close(_) => break,
                _ => {}
            }
        }

        clients.remove(&client_id);
        tracing::info!("Client {} disconnected", client_id);

        Ok(())
    }

    async fn broadcast(clients: &DashMap<u64, ClientConnection>, update: WorldUpdate) {
        let json = serde_json::to_string(&ClientUpdate::Update(UpdateMessage {
            timestamp: update.timestamp,
            objects: update.objects,
        })).unwrap();

        let msg = Message::Text(json);

        for entry in clients.iter() {
            if entry.value().subscribed {
                // Send to client (need to store sender in ClientConnection)
                // entry.value().sender.send(msg.clone()).await;
            }
        }
    }
}
```

---

## Main Loop Implementation

```rust
// main.rs

async fn run_server(config: ServerConfig) -> Result<(), Box<dyn std::error::Error>> {
    // Shared state
    let registry = Arc::new(RwLock::new(CameraRegistry::new()));
    let grid = Arc::new(SparseVoxelGrid::new(
        config.grid.origin(),
        config.grid.dimensions(),
        config.grid.voxel_size,
        config.decay.rate,
    ));

    // Channels
    let (frame_tx, frame_rx) = async_channel::bounded::<CameraFrame>(1000);
    let (update_tx, update_rx) = async_channel::bounded::<WorldUpdate>(100);

    // Start QUIC server
    let quic_server = QuicServer::bind(config.server.listen_address.parse()?).await?;
    smol::spawn({
        let registry = registry.clone();
        async move {
            quic_server.run(frame_tx, registry).await;
        }
    }).detach();

    // Start WebSocket server
    let ws_server = WebSocketServer::new();
    smol::spawn({
        let addr = format!("0.0.0.0:{}", config.server.websocket_port).parse()?;
        async move {
            ws_server.run(addr, update_rx).await.unwrap();
        }
    }).detach();

    // Initialize detection and tracking
    let mut detector = ObjectDetector::new(config.detection.clone());
    let mut tracker = ObjectTracker::new(
        config.tracking.association_threshold,
        60.0, // Assume 60 FPS for now
    );
    let mut aggregator = FrameAggregator::new(
        Duration::from_millis(100),
        10,
    );

    // Decay timer
    let decay_interval = Duration::from_secs_f32(config.decay.update_interval);
    let mut last_decay = Instant::now();

    // Main processing loop
    loop {
        // Receive frames (with timeout for decay processing)
        match smol::future::or(
            async { Some(frame_rx.recv().await.ok()?) },
            async {
                smol::Timer::after(decay_interval).await;
                None
            },
        ).await {
            Some(frame) => {
                // Add to grid
                grid.add_frame(&frame);

                // Add to aggregator
                aggregator.add_frame(frame);
            }
            None => {
                // Timeout - do decay
            }
        }

        // Periodic processing
        if last_decay.elapsed() >= decay_interval {
            // Apply decay
            grid.apply_decay();

            // Extract points and detect objects
            let points = grid.extract_points(&config.detection);
            let detections = detector.detect_optimized(&points);

            // Track objects
            let tracked = tracker.update(detections);

            // Send update to clients
            let update = WorldUpdate {
                timestamp: now_micros(),
                objects: tracked,
            };
            let _ = update_tx.try_send(update);

            last_decay = Instant::now();
        }
    }
}
```

---

## Implementation Priority

1. **Phase 1: Core Functionality**
   - QUIC server accepting camera connections
   - Voxel grid accumulation
   - Basic DBSCAN detection
   - Simple tracking
   - WebSocket server for clients

2. **Phase 2: Robustness**
   - Grid bounds auto-computation
   - Camera health monitoring
   - Optimized DBSCAN with spatial index
   - Hungarian algorithm for tracking

3. **Phase 3: Scale & Performance**
   - Parallel decay processing
   - Chunked voxel storage
   - Kalman filtering
   - Persistence layer
   - Metrics and monitoring

---

## Testing Strategy

### Unit Tests
- Grid operations (add, decay, extract)
- DBSCAN clustering
- Tracker association

### Integration Tests
- Full pipeline with simulated camera data
- Multi-camera scenarios
- Disconnection/reconnection handling

### Load Tests
- Maximum cameras
- Maximum voxels
- Maximum clients
