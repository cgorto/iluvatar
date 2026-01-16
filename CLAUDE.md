# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Iluvatar is a distributed motion detection system using multiple cameras to detect and localize moving objects in 3D space. The core technique: project motion-detected pixels as rays into a shared voxel grid, where ray intersections from multiple viewpoints triangulate moving targets with high accuracy.

## Build Commands

```bash
# Build all crates
cargo build

# Build specific binary
cargo build -p iluvatar-server
cargo build -p iluvatar-camera
cargo build -p iluvatar-simulator

# Run binaries
cargo run -p iluvatar-server
cargo run -p iluvatar-camera
cargo run -p iluvatar-simulator

# Run tests
cargo test
cargo test -p iluvatar-core

# Check compilation without building
cargo check

# Format code
cargo fmt

# Lint
cargo clippy
```

## Architecture

### Workspace Structure

```
crates/
├── iluvatar-core       # Shared types, coordinate transforms, protocol definitions
├── iluvatar-camera     # Camera unit: capture → difference → raymarch → network
├── iluvatar-server     # Central server: aggregation → detection → tracking → websocket
└── iluvatar-simulator  # Bevy-based testing environment
```

### Data Flow

```
Camera Unit:
  Frame Capture → Difference Mask → Ray Generation → Raymarch → VoxelContribution[] → QUIC

Server:
  QUIC → Frame Aggregator → Sparse Voxel Grid (with decay) → DBSCAN Clustering → Tracker → WebSocket

Client:
  WebSocket → CesiumJS Visualization
```

### Coordinate Systems

The system uses 4 coordinate systems with transforms in `iluvatar-core/src/geo.rs`:
- **WGS84**: GPS coordinates (latitude, longitude, altitude)
- **ECEF**: Earth-Centered, Earth-Fixed Cartesian
- **ENU**: East-North-Up local tangent plane (camera operations)
- **Grid Local**: Voxel grid coordinates (meters from origin)

### Key Design Patterns

- **Async Runtime**: `smol` is used consistently across all crates
- **Serialization**: `postcard` for compact binary wire format (bandwidth-constrained cameras)
- **Concurrency**: `DashMap` for lock-free voxel grid access in the server
- **Sparse Storage**: HashMap-based voxel grid to handle 1km³ at 1m resolution efficiently

### Module Responsibilities

**iluvatar-core**:
- `types.rs`: CameraPose, GeoPosition, Ray, VoxelContribution, TrackedObject
- `geo.rs`: Coordinate system conversions
- `protocol.rs`: Wire format for camera↔server and server↔client
- `config.rs`: Configuration structures

**iluvatar-camera**:
- `capture.rs`: Camera hardware abstraction (DummyCamera impl, V4L2 planned)
- `difference.rs`: Frame difference computation with thresholding
- `raymarch.rs`: Ray generation and voxel contribution calculation
- `localization.rs`: GPS + IMU fusion
- `network.rs`: QUIC client

**iluvatar-server**:
- `grid.rs`: SparseVoxelGrid with 64-bit packed indices
- `aggregator.rs`: Frame synchronization from multiple cameras
- `detector.rs`: DBSCAN clustering for object detection
- `tracker.rs`: Object association and velocity computation
- `websocket.rs`: Client broadcast

**iluvatar-simulator**:
- `scene.rs`: Ground plane, lighting, environment setup
- `targets.rs`: Moving objects with configurable motion patterns
- `cameras.rs`: Simulated camera units with render-to-texture
- `validation.rs`: Ground truth comparison

## Configuration

Configuration files use TOML format. Examples in `config/`:
- `camera.example.toml`: Camera identity, hardware, processing, network settings
- `server.example.toml`: Listen address, grid config, decay rate, detection thresholds

## Technical Constraints

- **Target latency**: <100ms end-to-end
- **Frame rate**: 60 FPS processing
- **Voxel size**: 1 meter resolution
- **Scale**: 1km+ outdoor coverage, 5-20 cameras per deployment
