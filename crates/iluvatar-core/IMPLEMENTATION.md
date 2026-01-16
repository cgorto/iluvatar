# iluvatar-core Implementation Guide

This crate provides the shared types, coordinate transformations, and protocol definitions used across all Iluvatar components. It's the foundation that ensures consistency between the camera, server, and client.

## Overview

```
iluvatar-core/
├── src/
│   ├── lib.rs        # Re-exports all modules
│   ├── types.rs      # Core data structures
│   ├── geo.rs        # Coordinate transformations
│   ├── protocol.rs   # Wire protocol definitions
│   └── config.rs     # Configuration types
└── Cargo.toml
```

## Module Responsibilities

### types.rs - Core Data Structures

This module defines the fundamental types used throughout the system.

#### Already Implemented
- `CameraId`, `ObjectId`, `Timestamp` - Type aliases for identifiers
- `CameraPose` - Camera position and orientation
- `GeoPosition` - WGS84 coordinates (lat/lon/alt)
- `CameraIntrinsics` - Camera optical parameters
- `Ray`, `VoxelContribution` - Raymarching primitives
- `BoundingBox`, `TrackedObject`, `DetectedPoint` - Detection types

#### TODO: Add Validation

```rust
impl GeoPosition {
    /// Validate that coordinates are within valid ranges
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.latitude < -90.0 || self.latitude > 90.0 {
            return Err(ValidationError::InvalidLatitude(self.latitude));
        }
        if self.longitude < -180.0 || self.longitude > 180.0 {
            return Err(ValidationError::InvalidLongitude(self.longitude));
        }
        Ok(())
    }
}
```

#### TODO: Add Builder Patterns

For complex types like `CameraIntrinsics`, add builders:

```rust
pub struct CameraIntrinsicsBuilder {
    resolution: Option<UVec2>,
    fov: Option<Fov>,
    // ...
}

impl CameraIntrinsicsBuilder {
    pub fn new() -> Self { ... }
    pub fn resolution(mut self, width: u32, height: u32) -> Self { ... }
    pub fn fov_degrees(mut self, horizontal: f32, vertical: f32) -> Self { ... }
    pub fn build(self) -> Result<CameraIntrinsics, BuilderError> { ... }
}
```

---

### geo.rs - Coordinate Transformations

This module handles conversions between coordinate systems.

#### Coordinate Systems Used

1. **WGS84 (GeoPosition)** - Global GPS coordinates
2. **ECEF** - Earth-Centered, Earth-Fixed Cartesian
3. **ENU** - East-North-Up local tangent plane
4. **Grid Local** - Voxel grid coordinates (meters from origin)

#### Already Implemented
- `GeoPosition::to_ecef()` / `from_ecef()` - WGS84 ↔ ECEF
- `GeoPosition::to_local_enu()` / `from_local_enu()` - WGS84 ↔ ENU
- `LocalCoordinateSystem` - Cached rotation matrices for efficient batch transforms

#### TODO: Grid Coordinate Helpers

Add utilities for working with the voxel grid:

```rust
/// Convert between grid coordinates and geo positions
pub struct GridCoordinateSystem {
    local: LocalCoordinateSystem,
    voxel_size: f32,
}

impl GridCoordinateSystem {
    pub fn new(origin: GeoPosition, voxel_size: f32) -> Self {
        Self {
            local: LocalCoordinateSystem::new(origin),
            voxel_size,
        }
    }

    /// Convert geo position to voxel index
    pub fn geo_to_voxel(&self, pos: &GeoPosition) -> IVec3 {
        let local = self.local.geo_to_local(pos);
        IVec3::new(
            (local.x / self.voxel_size).floor() as i32,
            (local.y / self.voxel_size).floor() as i32,
            (local.z / self.voxel_size).floor() as i32,
        )
    }

    /// Convert voxel index to geo position (voxel center)
    pub fn voxel_to_geo(&self, voxel: UVec3) -> GeoPosition {
        let local = Vec3::new(
            (voxel.x as f32 + 0.5) * self.voxel_size,
            (voxel.y as f32 + 0.5) * self.voxel_size,
            (voxel.z as f32 + 0.5) * self.voxel_size,
        );
        self.local.local_to_geo(local)
    }

    /// Get the local position of a voxel center
    pub fn voxel_to_local(&self, voxel: UVec3) -> Vec3 {
        Vec3::new(
            (voxel.x as f32 + 0.5) * self.voxel_size,
            (voxel.y as f32 + 0.5) * self.voxel_size,
            (voxel.z as f32 + 0.5) * self.voxel_size,
        )
    }
}
```

#### TODO: Frustum Calculations

Add camera frustum geometry for grid bounds computation:

```rust
/// Calculate the bounding box of a camera's view frustum
pub fn compute_frustum_bounds(
    pose: &CameraPose,
    intrinsics: &CameraIntrinsics,
    max_distance: f32,
    coord_system: &LocalCoordinateSystem,
) -> BoundingBox {
    // Calculate the 8 corners of the frustum
    let corners = [
        // Near plane corners (at distance 0, so just camera position)
        // Far plane corners
        compute_ray_endpoint(pose, intrinsics, 0.0, 0.0, max_distance),     // center
        compute_ray_endpoint(pose, intrinsics, -1.0, -1.0, max_distance),   // bottom-left
        compute_ray_endpoint(pose, intrinsics, 1.0, -1.0, max_distance),    // bottom-right
        compute_ray_endpoint(pose, intrinsics, -1.0, 1.0, max_distance),    // top-left
        compute_ray_endpoint(pose, intrinsics, 1.0, 1.0, max_distance),     // top-right
    ];

    let camera_local = coord_system.geo_to_local(&pose.position);

    let mut min = camera_local;
    let mut max = camera_local;

    for corner in corners {
        min = min.min(corner);
        max = max.max(corner);
    }

    BoundingBox::new(min, max)
}
```

---

### protocol.rs - Wire Protocol

This module defines the message formats for camera↔server and server↔client communication.

#### Already Implemented
- `CameraMessage` - Messages from camera to server
- `ServerMessage` - Messages from server to camera
- `ClientUpdate` - Messages from server to web client
- `serialize()` / `deserialize()` - Postcard binary serialization

#### TODO: Message Versioning

Add protocol version checking for compatibility:

```rust
pub const PROTOCOL_VERSION: u8 = 1;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VersionedMessage<T> {
    pub version: u8,
    pub payload: T,
}

impl<T: Serialize> VersionedMessage<T> {
    pub fn new(payload: T) -> Self {
        Self {
            version: PROTOCOL_VERSION,
            payload,
        }
    }
}

pub fn serialize_versioned<T: Serialize>(msg: &T) -> Result<Vec<u8>, postcard::Error> {
    let versioned = VersionedMessage::new(msg);
    postcard::to_allocvec(&versioned)
}

pub fn deserialize_versioned<'a, T: Deserialize<'a>>(
    data: &'a [u8],
) -> Result<T, ProtocolError> {
    let versioned: VersionedMessage<T> = postcard::from_bytes(data)?;
    if versioned.version != PROTOCOL_VERSION {
        return Err(ProtocolError::VersionMismatch {
            expected: PROTOCOL_VERSION,
            got: versioned.version,
        });
    }
    Ok(versioned.payload)
}
```

#### TODO: Compression for Large Payloads

For frames with many voxel contributions, add optional compression:

```rust
pub fn serialize_compressed<T: Serialize>(msg: &T) -> Result<Vec<u8>, postcard::Error> {
    let raw = postcard::to_allocvec(msg)?;
    if raw.len() > COMPRESSION_THRESHOLD {
        // Use LZ4 or similar fast compression
        let compressed = lz4_flex::compress_prepend_size(&raw);
        // Prepend compression flag
        let mut result = vec![1u8]; // 1 = compressed
        result.extend(compressed);
        Ok(result)
    } else {
        let mut result = vec![0u8]; // 0 = uncompressed
        result.extend(raw);
        Ok(result)
    }
}
```

---

### config.rs - Configuration Types

This module defines configuration structures that can be loaded from TOML files.

#### Already Implemented
- `GridConfig` - Voxel grid parameters
- `DecayConfig` - Time decay settings
- `DetectionConfig` - Object detection thresholds
- `RaymarchConfig` - Raymarching parameters
- `AttenuationConfig` - Distance attenuation modes

#### TODO: Configuration Loading

Add TOML loading utilities:

```rust
use std::path::Path;

pub trait LoadableConfig: for<'de> Deserialize<'de> + Default {
    fn load_from_file(path: &Path) -> Result<Self, ConfigError> {
        let contents = std::fs::read_to_string(path)?;
        let config: Self = toml::from_str(&contents)?;
        Ok(config)
    }

    fn load_or_default(path: &Path) -> Self {
        Self::load_from_file(path).unwrap_or_default()
    }
}

// Implement for all config types
impl LoadableConfig for GridConfig {}
impl LoadableConfig for DecayConfig {}
impl LoadableConfig for DetectionConfig {}
impl LoadableConfig for RaymarchConfig {}
```

#### TODO: Configuration Validation

Add validation for configuration values:

```rust
impl DetectionConfig {
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.intensity_threshold < 0.0 {
            return Err(ConfigError::InvalidValue {
                field: "intensity_threshold",
                reason: "must be non-negative",
            });
        }
        if self.cluster_epsilon <= 0.0 {
            return Err(ConfigError::InvalidValue {
                field: "cluster_epsilon",
                reason: "must be positive",
            });
        }
        if self.cluster_min_points < 1 {
            return Err(ConfigError::InvalidValue {
                field: "cluster_min_points",
                reason: "must be at least 1",
            });
        }
        Ok(())
    }
}
```

---

## Testing Strategy

### Unit Tests

Each module should have comprehensive unit tests:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_geo_roundtrip() {
        let origin = GeoPosition::new(47.6062, -122.3321, 0.0);
        let point = GeoPosition::new(47.6072, -122.3311, 100.0);

        let local = point.to_local_enu(&origin);
        let back = GeoPosition::from_local_enu(local, &origin);

        assert!((point.latitude - back.latitude).abs() < 1e-9);
        assert!((point.longitude - back.longitude).abs() < 1e-9);
        assert!((point.altitude - back.altitude).abs() < 1e-6);
    }

    #[test]
    fn test_protocol_roundtrip() {
        let msg = CameraMessage::Heartbeat {
            camera_id: 42,
            timestamp: 1234567890,
        };

        let bytes = serialize(&msg).unwrap();
        let decoded: CameraMessage = deserialize(&bytes).unwrap();

        match decoded {
            CameraMessage::Heartbeat { camera_id, timestamp } => {
                assert_eq!(camera_id, 42);
                assert_eq!(timestamp, 1234567890);
            }
            _ => panic!("Wrong message type"),
        }
    }
}
```

### Property-Based Tests

Consider using `proptest` for coordinate transformations:

```rust
use proptest::prelude::*;

proptest! {
    #[test]
    fn ecef_roundtrip_property(
        lat in -90.0f64..90.0,
        lon in -180.0f64..180.0,
        alt in -1000.0f64..100000.0
    ) {
        let pos = GeoPosition::new(lat, lon, alt);
        let ecef = pos.to_ecef();
        let back = GeoPosition::from_ecef(ecef);

        prop_assert!((pos.latitude - back.latitude).abs() < 1e-9);
        prop_assert!((pos.longitude - back.longitude).abs() < 1e-9);
        prop_assert!((pos.altitude - back.altitude).abs() < 1e-3);
    }
}
```

---

## Implementation Priority

1. **High Priority** (needed for basic functionality)
   - Grid coordinate system helpers
   - Configuration validation
   - Protocol versioning

2. **Medium Priority** (improves robustness)
   - Frustum calculations
   - Input validation
   - Builder patterns

3. **Lower Priority** (optimization)
   - Message compression
   - Batch coordinate transforms
   - SIMD optimizations for geo math

---

## Dependencies

Current dependencies and their purposes:

| Crate | Purpose |
|-------|---------|
| `glam` | Vector/matrix math with serde support |
| `serde` | Serialization framework |
| `postcard` | Compact binary serialization |
| `thiserror` | Error type derivation |

### Potential Additions

- `proptest` - Property-based testing
- `lz4_flex` - Fast compression (if adding compression)
- `toml` - Configuration file parsing (move from individual crates)
