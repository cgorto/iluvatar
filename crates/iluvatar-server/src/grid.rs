use dashmap::DashMap;
use glam::{UVec3, Vec3};
use iluvatar_core::{
    BoundingBox, CameraFrame, DetectedPoint, DetectionConfig, GeoPosition, VoxelContribution,
};
use std::time::Instant;

const INTENSITY_THRESHOLD: f32 = 0.01;

#[derive(Debug, Clone)]
pub struct Voxel {
    pub intensity: f32,
    pub contributor_count: u8,
    pub last_update: Instant,
}

pub struct SparseVoxelGrid {
    voxels: DashMap<u64, Voxel>,
    pub origin: GeoPosition,
    pub dimensions: UVec3,
    pub voxel_size: f32,
    decay_rate: f32,
    last_decay: Instant,
}

impl SparseVoxelGrid {
    pub fn new(origin: GeoPosition, dimensions: UVec3, voxel_size: f32, decay_rate: f32) -> Self {
        Self {
            voxels: DashMap::new(),
            origin,
            dimensions,
            voxel_size,
            decay_rate,
            last_decay: Instant::now(),
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

    /// Apply time decay to all voxels
    pub fn apply_decay(&mut self) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_decay).as_secs_f32();
        let decay_factor = (-self.decay_rate * dt).exp();

        self.voxels.retain(|_, voxel| {
            voxel.intensity *= decay_factor;
            voxel.intensity > INTENSITY_THRESHOLD
        });

        self.last_decay = now;
    }

    /// Add contributions from a camera frame
    pub fn add_contributions(&self, contributions: &[VoxelContribution]) {
        let now = Instant::now();

        for contrib in contributions {
            if !self.in_bounds(contrib.index) {
                continue;
            }

            let idx = Self::pack_index(contrib.index.x, contrib.index.y, contrib.index.z);

            self.voxels
                .entry(idx)
                .and_modify(|v| {
                    v.intensity += contrib.intensity;
                    v.contributor_count = v.contributor_count.saturating_add(1);
                    v.last_update = now;
                })
                .or_insert(Voxel {
                    intensity: contrib.intensity,
                    contributor_count: 1,
                    last_update: now,
                });
        }
    }

    /// Add a complete camera frame
    pub fn add_frame(&self, frame: &CameraFrame) {
        self.add_contributions(&frame.contributions);
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
        let active_cameras = 5; // TODO: Get actual count

        self.voxels
            .iter()
            .filter(|entry| {
                let v = entry.value();
                v.intensity >= config.intensity_threshold
                    && v.contributor_count >= config.min_contributors
            })
            .map(|entry| {
                let idx = *entry.key();
                let v = entry.value();
                let pos = Self::unpack_index(idx);

                DetectedPoint {
                    position: self.voxel_to_world(pos),
                    intensity: v.intensity,
                    confidence: v.contributor_count as f32 / active_cameras as f32,
                }
            })
            .collect()
    }

    /// Get count of active voxels
    pub fn active_count(&self) -> usize {
        self.voxels.len()
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
    fn test_add_contributions() {
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.5,
        );

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

        grid.add_contributions(&contributions);

        assert_eq!(grid.active_count(), 1);

        let packed = SparseVoxelGrid::pack_index(10, 10, 10);
        let voxel = grid.voxels.get(&packed).unwrap();
        assert!((voxel.intensity - 1.5).abs() < 0.01);
        assert_eq!(voxel.contributor_count, 2);
    }
}
