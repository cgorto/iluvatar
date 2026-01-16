use dashmap::DashMap;
use glam::{UVec3, Vec3};
use iluvatar_core::{
    BoundingBox, CameraFrame, CameraId, DetectedPoint, DetectionConfig, GeoPosition,
    VoxelContribution,
};
use parking_lot::Mutex;
use std::time::Instant;

const INTENSITY_THRESHOLD: f32 = 0.01;

#[derive(Debug, Clone)]
pub struct Voxel {
    pub intensity: f32,
    /// Bitmask of cameras that have contributed to this voxel (supports up to 64 cameras)
    pub camera_mask: u64,
    pub last_update: Instant,
}

impl Voxel {
    /// Returns the number of unique cameras that contributed to this voxel
    pub fn camera_count(&self) -> u8 {
        self.camera_mask.count_ones() as u8
    }
}

pub struct SparseVoxelGrid {
    voxels: DashMap<u64, Voxel>,
    pub origin: GeoPosition,
    pub dimensions: UVec3,
    pub voxel_size: f32,
    decay_rate: f32,
    /// Last decay time, wrapped in Mutex for interior mutability (allows apply_decay on &self)
    last_decay: Mutex<Instant>,
}

impl SparseVoxelGrid {
    pub fn new(origin: GeoPosition, dimensions: UVec3, voxel_size: f32, decay_rate: f32) -> Self {
        Self {
            voxels: DashMap::new(),
            origin,
            dimensions,
            voxel_size,
            decay_rate,
            last_decay: Mutex::new(Instant::now()),
        }
    }

    /// Pack 3D index into 64-bit key
    /// Supports up to 2^21 (~2 million) voxels per axis
    fn pack_index(x: u32, y: u32, z: u32) -> u64 {
        ((x as u64) << 42) | ((y as u64) << 21) | (z as u64)
    }

    fn unpack_index(idx: u64) -> UVec3 {
        UVec3::new(
            ((idx >> 42) & 0x1FFFFF) as u32,
            ((idx >> 21) & 0x1FFFFF) as u32,
            (idx & 0x1FFFFF) as u32,
        )
    }

    /// Check if a voxel index is within grid bounds
    fn in_bounds(&self, index: UVec3) -> bool {
        index.x < self.dimensions.x && index.y < self.dimensions.y && index.z < self.dimensions.z
    }

    /// Apply time decay to all voxels.
    /// This method takes &self (not &mut self) so it can be called on Arc<SparseVoxelGrid>.
    pub fn apply_decay(&self) {
        let now = Instant::now();
        let mut last_decay = self.last_decay.lock();
        let dt = now.duration_since(*last_decay).as_secs_f32();
        let decay_factor = (-self.decay_rate * dt).exp();

        // DashMap::retain takes &self, using interior mutability
        self.voxels.retain(|_, voxel| {
            voxel.intensity *= decay_factor;
            voxel.intensity > INTENSITY_THRESHOLD
        });

        *last_decay = now;
    }

    /// Add contributions from a specific camera.
    /// The camera_id is used to track unique camera contributors per voxel.
    ///
    /// # Panics
    /// Panics if `camera_id >= 64`. The camera bitmask is stored as a u64,
    /// limiting the system to 64 unique cameras. If more cameras are needed,
    /// the `camera_mask` field in `Voxel` must be changed to u128.
    pub fn add_camera_contributions(
        &self,
        camera_id: CameraId,
        contributions: &[VoxelContribution],
    ) {
        assert!(
            camera_id < 64,
            "Camera ID {} exceeds maximum of 63 (64-camera limit due to u64 bitmask)",
            camera_id
        );
        let now = Instant::now();
        let camera_bit = 1u64 << camera_id;

        for contrib in contributions {
            if !self.in_bounds(contrib.index) {
                continue;
            }

            let idx = Self::pack_index(contrib.index.x, contrib.index.y, contrib.index.z);

            self.voxels
                .entry(idx)
                .and_modify(|v| {
                    v.intensity += contrib.intensity;
                    v.camera_mask |= camera_bit;
                    v.last_update = now;
                })
                .or_insert(Voxel {
                    intensity: contrib.intensity,
                    camera_mask: camera_bit,
                    last_update: now,
                });
        }
    }

    /// Add contributions without camera tracking (legacy, uses camera_id=0)
    pub fn add_contributions(&self, contributions: &[VoxelContribution]) {
        self.add_camera_contributions(0, contributions);
    }

    /// Add a complete camera frame
    pub fn add_frame(&self, frame: &CameraFrame) {
        self.add_camera_contributions(frame.camera_id, &frame.contributions);
    }

    /// Convert voxel index to world position (center of voxel)
    pub fn voxel_to_world(&self, index: UVec3) -> Vec3 {
        Vec3::new(
            (index.x as f32 + 0.5) * self.voxel_size,
            (index.y as f32 + 0.5) * self.voxel_size,
            (index.z as f32 + 0.5) * self.voxel_size,
        )
    }

    /// Get grid bounds as a bounding box
    pub fn bounds(&self) -> BoundingBox {
        let size = Vec3::new(
            self.dimensions.x as f32 * self.voxel_size,
            self.dimensions.y as f32 * self.voxel_size,
            self.dimensions.z as f32 * self.voxel_size,
        );
        BoundingBox::new(Vec3::ZERO, size)
    }

    /// Extract point cloud of active voxels above threshold
    pub fn extract_points(&self, config: &DetectionConfig) -> Vec<DetectedPoint> {
        self.extract_points_with_camera_count(config, 64) // Default to max cameras
    }

    /// Extract point cloud with known camera count for confidence calculation
    pub fn extract_points_with_camera_count(
        &self,
        config: &DetectionConfig,
        active_cameras: u8,
    ) -> Vec<DetectedPoint> {
        self.voxels
            .iter()
            .filter(|entry| {
                let v = entry.value();
                v.intensity >= config.intensity_threshold
                    && v.camera_count() >= config.min_contributors
            })
            .map(|entry| {
                let idx = *entry.key();
                let v = entry.value();
                let pos = Self::unpack_index(idx);

                DetectedPoint {
                    position: self.voxel_to_world(pos),
                    intensity: v.intensity,
                    confidence: v.camera_count() as f32 / active_cameras as f32,
                }
            })
            .collect()
    }

    /// Get count of active voxels
    pub fn active_count(&self) -> usize {
        self.voxels.len()
    }

    /// Iterate over all active voxels for visualization purposes.
    ///
    /// Returns tuples of (world_position, intensity, camera_count).
    /// The `intensity_threshold` parameter filters out dim voxels.
    /// The `max_voxels` parameter limits the number of returned voxels for performance.
    pub fn iter_voxels_for_visualization(
        &self,
        intensity_threshold: f32,
        max_voxels: usize,
    ) -> Vec<(Vec3, f32, u8)> {
        self.voxels
            .iter()
            .filter(|entry| entry.value().intensity >= intensity_threshold)
            .take(max_voxels)
            .map(|entry| {
                let idx = *entry.key();
                let v = entry.value();
                let pos = Self::unpack_index(idx);
                (self.voxel_to_world(pos), v.intensity, v.camera_count())
            })
            .collect()
    }

    /// Get the maximum intensity among all active voxels (for normalization).
    /// Returns 0.0 if there are no active voxels.
    pub fn max_intensity(&self) -> f32 {
        self.voxels
            .iter()
            .map(|entry| entry.value().intensity)
            .fold(0.0f32, f32::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pack_unpack() {
        let x = 100u32;
        let y = 200u32;
        let z = 300u32;

        let packed = SparseVoxelGrid::pack_index(x, y, z);
        let unpacked = SparseVoxelGrid::unpack_index(packed);

        assert_eq!(unpacked.x, x);
        assert_eq!(unpacked.y, y);
        assert_eq!(unpacked.z, z);
    }

    #[test]
    fn test_add_contributions_same_camera() {
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.5,
        );

        // Two contributions from the same camera
        let contributions = vec![
            VoxelContribution {
                index: UVec3::new(10, 10, 10),
                intensity: 1.0,
            },
            VoxelContribution {
                index: UVec3::new(10, 10, 10),
                intensity: 0.5,
            },
        ];

        grid.add_camera_contributions(0, &contributions);

        assert_eq!(grid.active_count(), 1);

        let packed = SparseVoxelGrid::pack_index(10, 10, 10);
        let voxel = grid.voxels.get(&packed).unwrap();
        assert!((voxel.intensity - 1.5).abs() < 0.01);
        // Same camera contributing twice should only count as 1 unique camera
        assert_eq!(voxel.camera_count(), 1);
    }

    #[test]
    fn test_add_contributions_multiple_cameras() {
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.5,
        );

        let contribution = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: 1.0,
        }];

        // Three different cameras contribute to the same voxel
        grid.add_camera_contributions(0, &contribution);
        grid.add_camera_contributions(1, &contribution);
        grid.add_camera_contributions(2, &contribution);

        let packed = SparseVoxelGrid::pack_index(10, 10, 10);
        let voxel = grid.voxels.get(&packed).unwrap();
        assert!((voxel.intensity - 3.0).abs() < 0.01);
        // Three unique cameras
        assert_eq!(voxel.camera_count(), 3);
    }

    #[test]
    fn test_decay_removes_low_intensity_voxels() {
        // Use a high decay rate so voxels decay quickly
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            100.0, // Very high decay rate for testing
        );

        // Add a low-intensity voxel
        let contribution = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: 0.1,
        }];
        grid.add_camera_contributions(0, &contribution);

        assert_eq!(grid.active_count(), 1);

        // Wait a tiny bit and decay - high decay rate should remove it
        std::thread::sleep(std::time::Duration::from_millis(50));
        grid.apply_decay();

        // Voxel should be removed (intensity below threshold after decay)
        assert_eq!(grid.active_count(), 0);
    }

    #[test]
    fn test_decay_preserves_high_intensity_voxels() {
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.1, // Low decay rate
        );

        // Add a high-intensity voxel
        let contribution = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: 1000.0,
        }];
        grid.add_camera_contributions(0, &contribution);

        assert_eq!(grid.active_count(), 1);

        // Decay with low rate - voxel should survive
        std::thread::sleep(std::time::Duration::from_millis(10));
        grid.apply_decay();

        // Voxel should still exist (high intensity survives low decay)
        assert_eq!(grid.active_count(), 1);
    }

    #[test]
    fn test_concurrent_access() {
        use std::sync::Arc;
        use std::thread;

        let grid = Arc::new(SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(1000, 1000, 1000),
            1.0,
            0.5,
        ));

        let mut handles = vec![];

        // Writers
        for i in 0..10 {
            let grid = grid.clone();
            handles.push(thread::spawn(move || {
                for j in 0..100 {
                    let contrib = vec![VoxelContribution {
                        index: UVec3::new(j % 100, i, 0),
                        intensity: 1.0,
                    }];
                    grid.add_camera_contributions(i as u64, &contrib);
                    std::thread::yield_now();
                }
            }));
        }

        // Decayer
        let grid_decay = grid.clone();
        handles.push(thread::spawn(move || {
            for _ in 0..10 {
                grid_decay.apply_decay();
                std::thread::yield_now();
            }
        }));

        for h in handles {
            h.join().unwrap();
        }
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn test_pack_unpack_fuzz(
            x in 0u32..0x200000,
            y in 0u32..0x200000,
            z in 0u32..0x200000
        ) {
            let packed = SparseVoxelGrid::pack_index(x, y, z);
            let unpacked = SparseVoxelGrid::unpack_index(packed);
            assert_eq!(unpacked.x, x);
            assert_eq!(unpacked.y, y);
            assert_eq!(unpacked.z, z);
        }
    }
}
