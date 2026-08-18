use crate::time::{Clock, TimePoint};
use dashmap::DashMap;
use glam::{UVec3, Vec3};
use iluvatar_core::{
    BoundingBox, CameraFrame, CameraId, DetectedPoint, DetectionConfig, GeoPosition,
    VoxelContribution,
};
use parking_lot::Mutex;
use std::sync::Arc;
use tracing::{debug, warn};

const INTENSITY_THRESHOLD: f32 = 0.01;

/// Default maximum number of voxels allowed in the grid.
/// 200K voxels keeps the hash table under ~10 MB (fits in L3 cache),
/// preventing the decay loop from becoming memory-bandwidth-bound.
pub const DEFAULT_MAX_VOXELS: usize = 200_000;

/// Compute the Nth percentile of a slice using quickselect.
/// Percentile should be 0.0 to 1.0 (e.g., 0.9 for 90th percentile).
/// Modifies the input slice (partial sort).
fn compute_percentile(values: &mut [f32], percentile: f32) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    let idx = ((values.len() - 1) as f32 * percentile).round() as usize;
    let idx = idx.min(values.len() - 1);

    // Use select_nth_unstable for O(n) percentile finding
    let (_, median, _) = values.select_nth_unstable_by(idx, |a, b| {
        a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
    });
    *median
}

#[derive(Debug, Clone)]
pub struct Voxel {
    pub intensity: f32,
    /// Bitmask of cameras that have contributed to this voxel (supports up to 64 cameras)
    pub camera_mask: u64,
    pub last_update: TimePoint,
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
    clock: Arc<Clock>,
    /// Last decay time, wrapped in Mutex for interior mutability (allows apply_decay on &self)
    last_decay: Mutex<TimePoint>,
    /// Maximum number of voxels allowed in the grid (memory protection)
    max_voxels: usize,
}

#[derive(Debug, Clone)]
pub struct GridStats {
    pub active_voxels: usize,
    pub memory_usage_bytes: usize,
    pub max_intensity: f32,
    pub non_zero_camera_masks: usize,
}

impl SparseVoxelGrid {
    pub fn get_stats(&self) -> GridStats {
        let mut max_intensity = 0.0f32;
        let mut non_zero_camera_masks = 0;

        for entry in self.voxels.iter() {
            let v = entry.value();
            if v.intensity > max_intensity {
                max_intensity = v.intensity;
            }
            if v.camera_mask != 0 {
                non_zero_camera_masks += 1;
            }
        }

        GridStats {
            active_voxels: self.voxels.len(),
            memory_usage_bytes: self.voxels.capacity() * std::mem::size_of::<Voxel>(),
            max_intensity,
            non_zero_camera_masks,
        }
    }
    pub fn new(
        origin: GeoPosition,
        dimensions: UVec3,
        voxel_size: f32,
        decay_rate: f32,
        clock: Arc<Clock>,
    ) -> Self {
        Self::with_max_voxels(
            origin,
            dimensions,
            voxel_size,
            decay_rate,
            clock,
            DEFAULT_MAX_VOXELS,
        )
    }

    /// Create a new sparse voxel grid with a custom maximum voxel limit.
    ///
    /// # Arguments
    /// * `origin` - The WGS84 origin of the grid
    /// * `dimensions` - Grid dimensions in voxels (x, y, z)
    /// * `voxel_size` - Size of each voxel in meters
    /// * `decay_rate` - Exponential decay rate for voxel intensities
    /// * `clock` - Shared clock for time tracking
    /// * `max_voxels` - Maximum number of voxels allowed (memory protection)
    pub fn with_max_voxels(
        origin: GeoPosition,
        dimensions: UVec3,
        voxel_size: f32,
        decay_rate: f32,
        clock: Arc<Clock>,
        max_voxels: usize,
    ) -> Self {
        Self {
            voxels: DashMap::new(),
            origin,
            dimensions,
            voxel_size,
            decay_rate,
            last_decay: Mutex::new(clock.now()),
            clock,
            max_voxels,
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
        let now = self.clock.now();
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
    /// If the grid exceeds `max_voxels`, new voxels will be rejected and a warning logged.
    /// Existing voxels can still be updated.
    ///
    /// **NaN/Inf validation**: Contributions with NaN or infinite intensity values are
    /// silently skipped as defense-in-depth. A warning is logged if any are detected.
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
        let now = self.clock.now();
        let camera_bit = 1u64 << camera_id;

        let mut rejected_count = 0usize;
        let mut invalid_count = 0usize;

        for contrib in contributions {
            // Defense-in-depth: validate intensity at the grid level
            // This catches any NaN/Inf that slipped through protocol deserialization
            if !contrib.intensity.is_finite() || contrib.intensity < 0.0 {
                invalid_count += 1;
                continue;
            }

            if !self.in_bounds(contrib.index) {
                continue;
            }

            let idx = Self::pack_index(contrib.index.x, contrib.index.y, contrib.index.z);

            // Fast path: update existing voxels with a single get_mut lookup.
            // This is the common case (same voxels hit repeatedly within a frame).
            if let Some(mut entry) = self.voxels.get_mut(&idx) {
                entry.intensity += contrib.intensity;
                entry.camera_mask |= camera_bit;
                entry.last_update = now;
                continue;
            }

            // Slow path: new voxel. Check capacity before inserting.
            if self.voxels.len() >= self.max_voxels {
                rejected_count += 1;
                continue;
            }

            // Insert new voxel. Another thread may have inserted between our
            // get_mut and this entry call, so and_modify handles the race.
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

        if invalid_count > 0 {
            warn!(
                "Camera {}: rejected {} contributions with invalid intensity (NaN/Inf/negative)",
                camera_id, invalid_count
            );
        }

        if rejected_count > 0 {
            debug!(
                "Grid at capacity ({} voxels): rejected {} new voxel contributions from camera {}",
                self.max_voxels, rejected_count, camera_id
            );
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

    /// Clear all voxels from the grid.
    /// Useful for frame-by-frame processing where you want a fresh slate
    /// instead of relying on decay.
    pub fn clear(&self) {
        self.voxels.clear();
    }

    /// Reset camera masks on all voxels without clearing intensity.
    /// Call this at the start of each frame to ensure camera_count
    /// reflects only the current frame's contributions.
    pub fn reset_camera_masks(&self) {
        for mut entry in self.voxels.iter_mut() {
            entry.value_mut().camera_mask = 0;
        }
    }

    /// Extract points using percentile-based filtering.
    ///
    /// Instead of a fixed intensity threshold, this keeps only the top N% of voxels
    /// by intensity. This is crucial for finding the true ray intersection hotspots
    /// when dealing with noisy data.
    ///
    /// # Arguments
    /// * `percentile` - Value between 0.0 and 1.0. 0.9 means keep top 10%.
    /// * `min_camera_count` - Minimum number of cameras that must contribute
    /// * `active_cameras` - Total number of active cameras (for confidence calc)
    ///
    /// # Returns
    /// Points above the percentile threshold that also meet the camera count requirement.
    pub fn extract_points_percentile(
        &self,
        percentile: f32,
        min_camera_count: u8,
        active_cameras: u8,
    ) -> Vec<DetectedPoint> {
        // First pass: collect all voxels meeting camera requirement
        let qualifying_voxels: Vec<_> = self
            .voxels
            .iter()
            .filter(|entry| entry.value().camera_count() >= min_camera_count)
            .map(|entry| (*entry.key(), entry.value().clone()))
            .collect();

        if qualifying_voxels.is_empty() {
            return Vec::new();
        }

        // Minimum points fallback: if we have very few qualifying voxels,
        // don't apply percentile filtering - just take them all.
        // This prevents the "0 detected points" issue when targets are in
        // sparse visibility regions.
        const MIN_POINTS_FOR_PERCENTILE: usize = 5;

        let points: Vec<DetectedPoint> = if qualifying_voxels.len() < MIN_POINTS_FOR_PERCENTILE {
            // Too few points - take them all
            qualifying_voxels
                .iter()
                .map(|(idx, v)| {
                    let pos = Self::unpack_index(*idx);
                    DetectedPoint {
                        position: self.voxel_to_world(pos),
                        intensity: v.intensity,
                        confidence: v.camera_count() as f32 / active_cameras as f32,
                    }
                })
                .collect()
        } else {
            // Enough points - apply percentile filtering
            let mut intensities: Vec<f32> =
                qualifying_voxels.iter().map(|(_, v)| v.intensity).collect();
            let threshold = compute_percentile(&mut intensities, percentile);

            qualifying_voxels
                .iter()
                .filter(|(_, v)| v.intensity >= threshold)
                .map(|(idx, v)| {
                    let pos = Self::unpack_index(*idx);
                    DetectedPoint {
                        position: self.voxel_to_world(pos),
                        intensity: v.intensity,
                        confidence: v.camera_count() as f32 / active_cameras as f32,
                    }
                })
                .collect()
        };

        points
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

    /// Get debug statistics about voxel distribution.
    /// Returns (multi_camera_count, high_intensity_count) where:
    /// - multi_camera_count: voxels seen by >= 2 cameras
    /// - high_intensity_count: voxels with intensity >= 5.0
    pub fn debug_stats(&self) -> (usize, usize) {
        let mut multi_cam = 0usize;
        let mut high_intensity = 0usize;

        for entry in self.voxels.iter() {
            let v = entry.value();
            if v.camera_count() >= 2 {
                multi_cam += 1;
            }
            if v.intensity >= 5.0 {
                high_intensity += 1;
            }
        }

        (multi_cam, high_intensity)
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
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.5,
            clock,
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
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.5,
            clock,
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
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            100.0, // Very high decay rate for testing
            clock,
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
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.1, // Low decay rate
            clock,
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

        let clock = Clock::new();
        let grid = Arc::new(SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(1000, 1000, 1000),
            1.0,
            0.5,
            clock,
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

    #[test]
    fn test_percentile_extraction() {
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.0, // No decay
            clock,
        );

        // Add 10 voxels with varying intensities from 2 cameras
        // Voxels at positions 0-9 with intensities 1.0-10.0
        for i in 0..10 {
            let contrib = vec![VoxelContribution {
                index: UVec3::new(i, 0, 0),
                intensity: (i + 1) as f32, // 1.0 to 10.0
            }];
            // Add from camera 0 and camera 1 for multi-camera requirement
            grid.add_camera_contributions(0, &contrib);
            grid.add_camera_contributions(1, &contrib);
        }

        assert_eq!(grid.active_count(), 10);

        // Extract top 10% (90th percentile) - should get the top voxels
        // With 10 items, 90th percentile is at index 9 (the max)
        // Due to rounding, we might get 1-2 voxels at or above threshold
        let points = grid.extract_points_percentile(0.9, 2, 2);
        assert!(!points.is_empty() && points.len() <= 2);
        // The brightest voxel has intensity 20.0 (10 + 10 from two cameras)
        let max_intensity = points.iter().map(|p| p.intensity).fold(0.0f32, f32::max);
        assert!(max_intensity >= 19.0);

        // Extract top 50% (50th percentile) - should get ~5 voxels
        let points = grid.extract_points_percentile(0.5, 2, 2);
        assert!(points.len() >= 4 && points.len() <= 6);

        // With min_camera_count = 3, should get nothing (we only have 2 cameras)
        let points = grid.extract_points_percentile(0.5, 3, 2);
        assert_eq!(points.len(), 0);
    }

    #[test]
    fn test_clear() {
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.0,
            clock,
        );

        let contrib = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: 5.0,
        }];
        grid.add_camera_contributions(0, &contrib);
        assert_eq!(grid.active_count(), 1);

        grid.clear();
        assert_eq!(grid.active_count(), 0);
    }

    #[test]
    fn test_reset_camera_masks() {
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.0,
            clock,
        );

        let contrib = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: 5.0,
        }];

        // Add from two cameras
        grid.add_camera_contributions(0, &contrib);
        grid.add_camera_contributions(1, &contrib);

        let packed = SparseVoxelGrid::pack_index(10, 10, 10);
        assert_eq!(grid.voxels.get(&packed).unwrap().camera_count(), 2);

        // Reset masks
        grid.reset_camera_masks();
        assert_eq!(grid.voxels.get(&packed).unwrap().camera_count(), 0);

        // Intensity should be preserved
        assert!((grid.voxels.get(&packed).unwrap().intensity - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_max_voxels_limit() {
        let clock = Clock::new();
        // Create grid with very low max_voxels limit
        let grid = SparseVoxelGrid::with_max_voxels(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.0, // No decay
            clock,
            5, // Only allow 5 voxels
        );

        // Try to add 10 voxels at different positions
        let contributions: Vec<VoxelContribution> = (0..10)
            .map(|i| VoxelContribution {
                index: UVec3::new(i, 0, 0),
                intensity: 1.0,
            })
            .collect();

        grid.add_camera_contributions(0, &contributions);

        // Should be capped at max_voxels
        assert_eq!(grid.active_count(), 5);
    }

    #[test]
    fn test_max_voxels_allows_updates_to_existing() {
        let clock = Clock::new();
        let grid = SparseVoxelGrid::with_max_voxels(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.0,
            clock,
            2, // Only allow 2 voxels
        );

        // Add 2 voxels - fills the grid
        let contributions = vec![
            VoxelContribution {
                index: UVec3::new(0, 0, 0),
                intensity: 1.0,
            },
            VoxelContribution {
                index: UVec3::new(1, 0, 0),
                intensity: 1.0,
            },
        ];
        grid.add_camera_contributions(0, &contributions);
        assert_eq!(grid.active_count(), 2);

        // Try to add 3 more: 1 existing (should update), 2 new (should be rejected)
        let contributions2 = vec![
            VoxelContribution {
                index: UVec3::new(0, 0, 0), // Existing - should update
                intensity: 5.0,
            },
            VoxelContribution {
                index: UVec3::new(2, 0, 0), // New - should be rejected
                intensity: 1.0,
            },
            VoxelContribution {
                index: UVec3::new(3, 0, 0), // New - should be rejected
                intensity: 1.0,
            },
        ];
        grid.add_camera_contributions(1, &contributions2);

        // Still only 2 voxels
        assert_eq!(grid.active_count(), 2);

        // But the existing voxel was updated
        let packed = SparseVoxelGrid::pack_index(0, 0, 0);
        let voxel = grid.voxels.get(&packed).unwrap();
        assert!((voxel.intensity - 6.0).abs() < 0.01); // 1.0 + 5.0
        assert_eq!(voxel.camera_count(), 2); // Both cameras contributed
    }

    #[test]
    fn test_nan_inf_contributions_rejected() {
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.0, // No decay
            clock,
        );

        // Mix of valid and invalid contributions
        let contributions = vec![
            VoxelContribution {
                index: UVec3::new(1, 0, 0),
                intensity: 1.0, // Valid
            },
            VoxelContribution {
                index: UVec3::new(2, 0, 0),
                intensity: f32::NAN, // Invalid - NaN
            },
            VoxelContribution {
                index: UVec3::new(3, 0, 0),
                intensity: f32::INFINITY, // Invalid - Infinity
            },
            VoxelContribution {
                index: UVec3::new(4, 0, 0),
                intensity: f32::NEG_INFINITY, // Invalid - Negative infinity
            },
            VoxelContribution {
                index: UVec3::new(5, 0, 0),
                intensity: -1.0, // Invalid - Negative
            },
            VoxelContribution {
                index: UVec3::new(6, 0, 0),
                intensity: 2.0, // Valid
            },
        ];

        grid.add_camera_contributions(0, &contributions);

        // Only 2 valid contributions should be added
        assert_eq!(grid.active_count(), 2);

        // Verify the correct voxels were added
        let packed1 = SparseVoxelGrid::pack_index(1, 0, 0);
        let packed6 = SparseVoxelGrid::pack_index(6, 0, 0);
        assert!(grid.voxels.get(&packed1).is_some());
        assert!(grid.voxels.get(&packed6).is_some());

        // Invalid ones should not exist
        let packed2 = SparseVoxelGrid::pack_index(2, 0, 0);
        let packed3 = SparseVoxelGrid::pack_index(3, 0, 0);
        let packed4 = SparseVoxelGrid::pack_index(4, 0, 0);
        let packed5 = SparseVoxelGrid::pack_index(5, 0, 0);
        assert!(grid.voxels.get(&packed2).is_none());
        assert!(grid.voxels.get(&packed3).is_none());
        assert!(grid.voxels.get(&packed4).is_none());
        assert!(grid.voxels.get(&packed5).is_none());
    }

    #[test]
    fn test_nan_contribution_does_not_corrupt_existing_voxel() {
        let clock = Clock::new();
        let grid = SparseVoxelGrid::new(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.0,
            clock,
        );

        // Add a valid contribution first
        let valid_contrib = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: 5.0,
        }];
        grid.add_camera_contributions(0, &valid_contrib);

        let packed = SparseVoxelGrid::pack_index(10, 10, 10);
        let initial_intensity = grid.voxels.get(&packed).unwrap().intensity;
        assert!((initial_intensity - 5.0).abs() < 0.01);

        // Now try to add a NaN contribution to the same voxel
        // This should be rejected, not corrupt the existing value
        let nan_contrib = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: f32::NAN,
        }];
        grid.add_camera_contributions(1, &nan_contrib);

        // Intensity should remain unchanged (NaN was rejected)
        let final_intensity = grid.voxels.get(&packed).unwrap().intensity;
        assert!(
            (final_intensity - 5.0).abs() < 0.01,
            "NaN contribution should not modify existing voxel"
        );
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
