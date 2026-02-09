use crate::grid::GridStats;
use crate::time::{Clock, TimePoint};
use glam::{UVec3, Vec3};
use iluvatar_core::{
    BoundingBox, CameraFrame, CameraId, DetectedPoint, DetectionConfig, GeoPosition,
    VoxelContribution,
};
use std::sync::Arc;
use tracing::{debug, info, warn};

/// Sentinel value marking an empty slot. No valid packed voxel key can equal
/// this because each axis uses only 21 bits — all-ones in 64 bits is impossible.
const EMPTY_KEY: u64 = u64::MAX;

/// Minimum intensity below which a voxel is considered dead during decay.
const INTENSITY_THRESHOLD: f32 = 0.01;

/// Default maximum number of voxels allowed in the grid.
pub const DEFAULT_MAX_VOXELS: usize = 1_000_000;

/// Initial table capacity. 8K slots × 24 bytes = 192 KB, fits in L2 cache.
/// The table grows by doubling when load factor exceeds 50%.
const INITIAL_CAPACITY: usize = 8192;

/// FxHash constant (golden-ratio fractional bits scaled to 64 bits).
const HASH_CONSTANT: u64 = 0x517cc1b727220a95;

/// A single slot in the hash table. Array-of-structs layout so that one
/// cache line load (~64 bytes) fetches 2 complete slots, which is ideal
/// for linear probing.
#[derive(Clone, Copy)]
struct Slot {
    key: u64,
    intensity: f32,
    camera_mask: u64,
}

impl Slot {
    const EMPTY: Slot = Slot {
        key: EMPTY_KEY,
        intensity: 0.0,
        camera_mask: 0,
    };
}

/// Grow-on-demand open-addressing hash table for voxel storage.
///
/// Replaces `SparseVoxelGrid` (DashMap-based) for single-threaded server use.
/// Linear probing with FxHash-style multiply gives O(1) amortized lookup
/// without lock acquisition. AoS layout means each probe touches one cache
/// line instead of four (the SoA approach scattered across separate arrays).
///
/// # Memory
///
/// Starts at 192 KB and doubles when load factor > 50%. The table is always
/// 2× the live entry count, keeping both probe chains short and the working
/// set cache-friendly. At 100K voxels the table is ~6 MB (fits in L3).
pub struct FlatVoxelGrid {
    slots: Vec<Slot>,
    capacity: usize,
    mask: usize,
    count: usize,
    max_voxels: usize,

    // Grid geometry (same fields as SparseVoxelGrid).
    pub origin: GeoPosition,
    pub dimensions: UVec3,
    pub voxel_size: f32,
    decay_rate: f32,
    clock: Arc<Clock>,
    last_decay: TimePoint,
}

impl FlatVoxelGrid {
    /// Create a new flat voxel grid with a custom maximum voxel limit.
    ///
    /// The table starts small (8K slots = 192 KB) and grows on demand.
    /// `max_voxels` is a hard cap on how many live voxels can exist at once.
    pub fn with_max_voxels(
        origin: GeoPosition,
        dimensions: UVec3,
        voxel_size: f32,
        decay_rate: f32,
        clock: Arc<Clock>,
        max_voxels: usize,
    ) -> Self {
        assert!(max_voxels > 0);
        let last_decay = clock.now();
        Self {
            slots: vec![Slot::EMPTY; INITIAL_CAPACITY],
            capacity: INITIAL_CAPACITY,
            mask: INITIAL_CAPACITY - 1,
            count: 0,
            max_voxels,
            origin,
            dimensions,
            voxel_size,
            decay_rate,
            clock,
            last_decay,
        }
    }

    // ========================================================================
    // Index packing (identical to SparseVoxelGrid / raymarch.rs)
    // ========================================================================

    /// Pack 3D index into 64-bit key.
    /// Supports up to 2^21 (~2 million) voxels per axis.
    #[inline]
    fn pack_index(x: u32, y: u32, z: u32) -> u64 {
        ((x as u64) << 42) | ((y as u64) << 21) | (z as u64)
    }

    #[inline]
    fn unpack_index(idx: u64) -> UVec3 {
        UVec3::new(
            ((idx >> 42) & 0x1FFFFF) as u32,
            ((idx >> 21) & 0x1FFFFF) as u32,
            (idx & 0x1FFFFF) as u32,
        )
    }

    // ========================================================================
    // Hashing + probing
    // ========================================================================

    /// FxHash-style multiply, masked to table size.
    #[inline]
    fn hash_slot(&self, key: u64) -> usize {
        (key.wrapping_mul(HASH_CONSTANT) as usize) & self.mask
    }

    /// Find the slot for `key`. Returns the index of either the matching
    /// slot or the first empty slot where `key` could be inserted.
    #[inline]
    fn find_slot(&self, key: u64) -> usize {
        debug_assert!(key != EMPTY_KEY);
        let mut slot = self.hash_slot(key);
        loop {
            let k = self.slots[slot].key;
            if k == key || k == EMPTY_KEY {
                return slot;
            }
            slot = (slot + 1) & self.mask;
        }
    }

    // ========================================================================
    // Growth
    // ========================================================================

    /// Double the table capacity and rehash all live entries.
    ///
    /// This is called when load factor exceeds 50%. Growth is amortized
    /// O(1) per insertion and happens at most log2(max_voxels) times over
    /// the lifetime of the grid.
    fn grow(&mut self) {
        let new_capacity = self.capacity * 2;
        let new_mask = new_capacity - 1;
        let mut new_slots = vec![Slot::EMPTY; new_capacity];

        for i in 0..self.capacity {
            let slot = &self.slots[i];
            if slot.key == EMPTY_KEY {
                continue;
            }
            // Find new home in the larger table.
            let mut dest = (slot.key.wrapping_mul(HASH_CONSTANT) as usize) & new_mask;
            loop {
                if new_slots[dest].key == EMPTY_KEY {
                    new_slots[dest] = *slot;
                    break;
                }
                dest = (dest + 1) & new_mask;
            }
        }

        let old_cap = self.capacity;
        self.slots = new_slots;
        self.capacity = new_capacity;
        self.mask = new_mask;
        info!(
            old_capacity = old_cap,
            new_capacity = new_capacity,
            live_entries = self.count,
            memory_kb = new_capacity * std::mem::size_of::<Slot>() / 1024,
            "Grid table grew"
        );
    }

    // ========================================================================
    // Core mutation
    // ========================================================================

    /// Insert or update a single voxel. This is the hot path for direct-drain
    /// raymarching: the closure passed to `raymarch_into` calls this for every
    /// (key, intensity) pair.
    ///
    /// If the voxel already exists, intensity is accumulated and the camera bit
    /// is OR'd in. If it's new and the grid is at `max_voxels`, the contribution
    /// is silently dropped.
    #[inline]
    pub fn accumulate(&mut self, key: u64, intensity: f32, camera_bit: u64) {
        debug_assert!(key != EMPTY_KEY);
        debug_assert!(intensity.is_finite());
        debug_assert!(intensity >= 0.0);

        let slot_idx = self.find_slot(key);
        let slot = &mut self.slots[slot_idx];

        if slot.key == key {
            // Update existing voxel.
            slot.intensity += intensity;
            slot.camera_mask |= camera_bit;
        } else {
            // New voxel — check hard cap.
            if self.count >= self.max_voxels {
                return;
            }
            // Grow if load factor would exceed 50%.
            if self.count * 2 >= self.capacity {
                self.grow();
                // Slot index is stale after grow — re-probe.
                let new_idx = self.find_slot(key);
                self.slots[new_idx] = Slot {
                    key,
                    intensity,
                    camera_mask: camera_bit,
                };
                self.count += 1;
                return;
            }
            *slot = Slot {
                key,
                intensity,
                camera_mask: camera_bit,
            };
            self.count += 1;
        }
    }

    // ========================================================================
    // Backward-compatible bulk insertion (for VoxelContributions path)
    // ========================================================================

    /// Add contributions from a specific camera.
    /// Validates intensity and bounds, same semantics as SparseVoxelGrid.
    pub fn add_camera_contributions(
        &mut self,
        camera_id: CameraId,
        contributions: &[VoxelContribution],
    ) {
        assert!(
            camera_id < 64,
            "Camera ID {} exceeds maximum of 63 (64-camera limit due to u64 bitmask)",
            camera_id
        );
        let camera_bit = 1u64 << camera_id;
        let mut rejected_count = 0usize;
        let mut invalid_count = 0usize;

        for contrib in contributions {
            if !contrib.intensity.is_finite() || contrib.intensity < 0.0 {
                invalid_count += 1;
                continue;
            }
            if !self.in_bounds(contrib.index) {
                continue;
            }

            let key = Self::pack_index(contrib.index.x, contrib.index.y, contrib.index.z);

            // Peek to check if we'd hit the capacity wall for a NEW entry.
            // This avoids calling accumulate (which might grow the table)
            // only to discover the max_voxels limit.
            if self.count >= self.max_voxels {
                let slot_idx = self.find_slot(key);
                if self.slots[slot_idx].key != key {
                    rejected_count += 1;
                    continue;
                }
            }

            self.accumulate(key, contrib.intensity, camera_bit);
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

    /// Add a complete camera frame.
    pub fn add_frame(&mut self, frame: &CameraFrame) {
        self.add_camera_contributions(frame.camera_id, &frame.contributions);
    }

    // ========================================================================
    // Bounds check
    // ========================================================================

    #[inline]
    fn in_bounds(&self, index: UVec3) -> bool {
        index.x < self.dimensions.x && index.y < self.dimensions.y && index.z < self.dimensions.z
    }

    // ========================================================================
    // Decay
    // ========================================================================

    /// Apply time decay to all voxels, removing dead ones.
    ///
    /// Dead slots are cleared to EMPTY_KEY, then survivors are rehashed
    /// in-place to repair probe chains broken by the removals.
    pub fn apply_decay(&mut self) {
        let now = self.clock.now();
        let dt = now.duration_since(self.last_decay).as_secs_f32();
        let decay_factor = (-self.decay_rate * dt).exp();

        // Pass 1: decay intensities, mark dead slots.
        let mut dead_count = 0usize;
        for i in 0..self.capacity {
            if self.slots[i].key == EMPTY_KEY {
                continue;
            }
            self.slots[i].intensity *= decay_factor;
            if self.slots[i].intensity <= INTENSITY_THRESHOLD {
                self.slots[i] = Slot::EMPTY;
                dead_count += 1;
            }
        }
        self.count -= dead_count;

        // Pass 2: rehash survivors to repair probe chains.
        if dead_count > 0 {
            self.rehash_in_place();
        }

        self.last_decay = now;
    }

    /// Rehash all entries in-place to repair probe chains after deletions.
    ///
    /// Algorithm: scan left to right. For each occupied slot, check if it
    /// is at its ideal position. If not, remove it and re-insert via
    /// find_slot. This is the standard backward-shift deletion repair.
    fn rehash_in_place(&mut self) {
        for i in 0..self.capacity {
            if self.slots[i].key == EMPTY_KEY {
                continue;
            }
            let ideal = self.hash_slot(self.slots[i].key);
            if ideal == i {
                continue;
            }

            // Remove from current slot.
            let entry = self.slots[i];
            self.slots[i] = Slot::EMPTY;

            // Find new slot (may end up back at i or somewhere better).
            let new_idx = self.find_slot(entry.key);
            self.slots[new_idx] = entry;
        }
    }

    // ========================================================================
    // Camera mask management
    // ========================================================================

    /// Reset all camera masks to zero. Called once per processing cycle so that
    /// camera_count reflects only the current cycle's contributions.
    pub fn reset_camera_masks(&mut self) {
        for i in 0..self.capacity {
            self.slots[i].camera_mask = 0;
        }
    }

    // ========================================================================
    // Point extraction (detection pipeline)
    // ========================================================================

    /// Extract point cloud of active voxels above threshold.
    pub fn extract_points(&self, config: &DetectionConfig) -> Vec<DetectedPoint> {
        self.extract_points_with_camera_count(config, 64)
    }

    /// Extract point cloud with known camera count for confidence calculation.
    pub fn extract_points_with_camera_count(
        &self,
        config: &DetectionConfig,
        active_cameras: u8,
    ) -> Vec<DetectedPoint> {
        let mut points = Vec::new();
        for slot in &self.slots {
            if slot.key == EMPTY_KEY {
                continue;
            }
            let camera_count = slot.camera_mask.count_ones() as u8;
            if slot.intensity >= config.intensity_threshold
                && camera_count >= config.min_contributors
            {
                let pos = Self::unpack_index(slot.key);
                points.push(DetectedPoint {
                    position: self.voxel_to_world(pos),
                    intensity: slot.intensity,
                    confidence: camera_count as f32 / active_cameras as f32,
                });
            }
        }
        points
    }

    /// Extract points using percentile-based filtering.
    pub fn extract_points_percentile(
        &self,
        percentile: f32,
        min_camera_count: u8,
        active_cameras: u8,
    ) -> Vec<DetectedPoint> {
        // First pass: collect qualifying voxels.
        let mut qualifying: Vec<(u64, f32, u8)> = Vec::new();
        for slot in &self.slots {
            if slot.key == EMPTY_KEY {
                continue;
            }
            let camera_count = slot.camera_mask.count_ones() as u8;
            if camera_count >= min_camera_count {
                qualifying.push((slot.key, slot.intensity, camera_count));
            }
        }

        if qualifying.is_empty() {
            return Vec::new();
        }

        const MIN_POINTS_FOR_PERCENTILE: usize = 5;

        if qualifying.len() < MIN_POINTS_FOR_PERCENTILE {
            return qualifying
                .iter()
                .map(|&(key, intensity, cc)| {
                    let pos = Self::unpack_index(key);
                    DetectedPoint {
                        position: self.voxel_to_world(pos),
                        intensity,
                        confidence: cc as f32 / active_cameras as f32,
                    }
                })
                .collect();
        }

        let mut intensities: Vec<f32> = qualifying.iter().map(|&(_, i, _)| i).collect();
        let threshold = compute_percentile(&mut intensities, percentile);

        qualifying
            .iter()
            .filter(|&&(_, intensity, _)| intensity >= threshold)
            .map(|&(key, intensity, cc)| {
                let pos = Self::unpack_index(key);
                DetectedPoint {
                    position: self.voxel_to_world(pos),
                    intensity,
                    confidence: cc as f32 / active_cameras as f32,
                }
            })
            .collect()
    }

    // ========================================================================
    // Geometry helpers
    // ========================================================================

    /// Convert voxel index to world position (center of voxel).
    pub fn voxel_to_world(&self, index: UVec3) -> Vec3 {
        Vec3::new(
            (index.x as f32 + 0.5) * self.voxel_size,
            (index.y as f32 + 0.5) * self.voxel_size,
            (index.z as f32 + 0.5) * self.voxel_size,
        )
    }

    /// Get grid bounds as a bounding box.
    pub fn bounds(&self) -> BoundingBox {
        let size = Vec3::new(
            self.dimensions.x as f32 * self.voxel_size,
            self.dimensions.y as f32 * self.voxel_size,
            self.dimensions.z as f32 * self.voxel_size,
        );
        BoundingBox::new(Vec3::ZERO, size)
    }

    // ========================================================================
    // Stats and diagnostics
    // ========================================================================

    /// Get count of active voxels.
    pub fn active_count(&self) -> usize {
        self.count
    }

    pub fn get_stats(&self) -> GridStats {
        let mut max_intensity = 0.0f32;
        let mut non_zero_camera_masks = 0usize;
        for slot in &self.slots {
            if slot.key == EMPTY_KEY {
                continue;
            }
            if slot.intensity > max_intensity {
                max_intensity = slot.intensity;
            }
            if slot.camera_mask != 0 {
                non_zero_camera_masks += 1;
            }
        }
        GridStats {
            active_voxels: self.count,
            memory_usage_bytes: self.capacity * std::mem::size_of::<Slot>(),
            max_intensity,
            non_zero_camera_masks,
        }
    }

    /// Get the maximum intensity among all active voxels (for normalization).
    pub fn max_intensity(&self) -> f32 {
        let mut max = 0.0f32;
        for slot in &self.slots {
            if slot.key != EMPTY_KEY && slot.intensity > max {
                max = slot.intensity;
            }
        }
        max
    }

    /// Iterate over all active voxels for visualization purposes.
    pub fn iter_voxels_for_visualization(
        &self,
        intensity_threshold: f32,
        max_voxels: usize,
    ) -> Vec<(Vec3, f32, u8)> {
        let mut result = Vec::new();
        for slot in &self.slots {
            if result.len() >= max_voxels {
                break;
            }
            if slot.key == EMPTY_KEY {
                continue;
            }
            if slot.intensity >= intensity_threshold {
                let pos = Self::unpack_index(slot.key);
                let camera_count = slot.camera_mask.count_ones() as u8;
                result.push((self.voxel_to_world(pos), slot.intensity, camera_count));
            }
        }
        result
    }

    /// Debug statistics: (multi_camera_count, high_intensity_count).
    pub fn debug_stats(&self) -> (usize, usize) {
        let mut multi_cam = 0usize;
        let mut high_intensity = 0usize;
        for slot in &self.slots {
            if slot.key == EMPTY_KEY {
                continue;
            }
            if slot.camera_mask.count_ones() >= 2 {
                multi_cam += 1;
            }
            if slot.intensity >= 5.0 {
                high_intensity += 1;
            }
        }
        (multi_cam, high_intensity)
    }

    /// Clear all voxels from the grid.
    pub fn clear(&mut self) {
        self.slots.fill(Slot::EMPTY);
        self.count = 0;
    }
}

/// Compute the Nth percentile of a slice using quickselect.
fn compute_percentile(values: &mut [f32], percentile: f32) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    let idx = ((values.len() - 1) as f32 * percentile).round() as usize;
    let idx = idx.min(values.len() - 1);
    let (_, median, _) = values.select_nth_unstable_by(idx, |a, b| {
        a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
    });
    *median
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn test_grid(max_voxels: usize) -> FlatVoxelGrid {
        let clock = Clock::new();
        FlatVoxelGrid::with_max_voxels(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.5,
            clock,
            max_voxels,
        )
    }

    #[test]
    fn test_slot_size() {
        // Verify AoS slot is compact: key(8) + intensity(4) + pad(4) + camera_mask(8) = 24.
        assert!(std::mem::size_of::<Slot>() <= 24);
    }

    #[test]
    fn test_pack_unpack() {
        let x = 100u32;
        let y = 200u32;
        let z = 300u32;
        let packed = FlatVoxelGrid::pack_index(x, y, z);
        let unpacked = FlatVoxelGrid::unpack_index(packed);
        assert_eq!(unpacked.x, x);
        assert_eq!(unpacked.y, y);
        assert_eq!(unpacked.z, z);
    }

    #[test]
    fn test_pack_unpack_boundary() {
        let max_val = 0x1FFFFF_u32;
        let packed = FlatVoxelGrid::pack_index(max_val, max_val, max_val);
        let unpacked = FlatVoxelGrid::unpack_index(packed);
        assert_eq!(unpacked.x, max_val);
        assert_eq!(unpacked.y, max_val);
        assert_eq!(unpacked.z, max_val);
    }

    #[test]
    fn test_insert_and_update() {
        let mut grid = test_grid(1000);
        let key = FlatVoxelGrid::pack_index(10, 20, 30);

        grid.accumulate(key, 1.0, 1u64 << 0);
        assert_eq!(grid.active_count(), 1);

        // Second accumulate to same key — should update, not insert.
        grid.accumulate(key, 2.5, 1u64 << 1);
        assert_eq!(grid.active_count(), 1);

        // Verify accumulated intensity.
        let slot = grid.find_slot(key);
        assert!((grid.slots[slot].intensity - 3.5).abs() < 0.01);
        assert_eq!(grid.slots[slot].camera_mask.count_ones(), 2);
    }

    #[test]
    fn test_capacity_limit() {
        let mut grid = test_grid(5);

        // Insert 10 distinct voxels — only 5 should survive.
        for i in 0..10u32 {
            let key = FlatVoxelGrid::pack_index(i, 0, 0);
            grid.accumulate(key, 1.0, 1);
        }
        assert_eq!(grid.active_count(), 5);
    }

    #[test]
    fn test_capacity_allows_updates() {
        let mut grid = test_grid(2);

        let k0 = FlatVoxelGrid::pack_index(0, 0, 0);
        let k1 = FlatVoxelGrid::pack_index(1, 0, 0);
        let k2 = FlatVoxelGrid::pack_index(2, 0, 0);

        grid.accumulate(k0, 1.0, 1);
        grid.accumulate(k1, 1.0, 1);
        assert_eq!(grid.active_count(), 2);

        // New key should be rejected.
        grid.accumulate(k2, 99.0, 1);
        assert_eq!(grid.active_count(), 2);

        // Existing key should still update.
        grid.accumulate(k0, 5.0, 1 << 1);
        let slot = grid.find_slot(k0);
        assert!((grid.slots[slot].intensity - 6.0).abs() < 0.01);
    }

    #[test]
    fn test_grow_on_demand() {
        // Start with default 8K capacity. Insert enough to trigger growth.
        let mut grid = test_grid(100_000);
        assert_eq!(grid.capacity, INITIAL_CAPACITY);

        // Insert 5000 entries — should trigger growth at 4096 (50% of 8192).
        for i in 0..5000u32 {
            let key = FlatVoxelGrid::pack_index(i, 0, 0);
            grid.accumulate(key, 1.0, 1);
        }

        assert_eq!(grid.active_count(), 5000);
        // Table should have grown at least once.
        assert!(grid.capacity > INITIAL_CAPACITY);

        // All entries should still be findable after growth.
        for i in 0..5000u32 {
            let key = FlatVoxelGrid::pack_index(i, 0, 0);
            let slot = grid.find_slot(key);
            assert_eq!(grid.slots[slot].key, key);
            assert!((grid.slots[slot].intensity - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_decay_removes_low_intensity() {
        let clock = Clock::new();
        let mut grid = FlatVoxelGrid::with_max_voxels(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            100.0, // Very high decay rate.
            clock,
            1000,
        );

        let key = FlatVoxelGrid::pack_index(10, 10, 10);
        grid.accumulate(key, 0.1, 1);
        assert_eq!(grid.active_count(), 1);

        std::thread::sleep(std::time::Duration::from_millis(50));
        grid.apply_decay();

        assert_eq!(grid.active_count(), 0);
    }

    #[test]
    fn test_decay_preserves_high_intensity() {
        let clock = Clock::new();
        let mut grid = FlatVoxelGrid::with_max_voxels(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            0.1, // Low decay rate.
            clock,
            1000,
        );

        let key = FlatVoxelGrid::pack_index(10, 10, 10);
        grid.accumulate(key, 1000.0, 1);
        assert_eq!(grid.active_count(), 1);

        std::thread::sleep(std::time::Duration::from_millis(10));
        grid.apply_decay();

        assert_eq!(grid.active_count(), 1);
    }

    #[test]
    fn test_decay_rehash_preserves_entries() {
        let clock = Clock::new();
        let mut grid = FlatVoxelGrid::with_max_voxels(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(100, 100, 100),
            1.0,
            100.0, // High decay rate.
            clock,
            1000,
        );

        let weak_key = FlatVoxelGrid::pack_index(1, 0, 0);
        let strong_key = FlatVoxelGrid::pack_index(2, 0, 0);
        grid.accumulate(weak_key, 0.02, 1);
        grid.accumulate(strong_key, 1000.0, 1);
        assert_eq!(grid.active_count(), 2);

        std::thread::sleep(std::time::Duration::from_millis(50));
        grid.apply_decay();

        assert_eq!(grid.active_count(), 1);

        // The strong entry must still be findable.
        let slot = grid.find_slot(strong_key);
        assert_eq!(grid.slots[slot].key, strong_key);
        assert!(grid.slots[slot].intensity > 0.0);
    }

    #[test]
    fn test_reset_camera_masks() {
        let mut grid = test_grid(1000);

        let key = FlatVoxelGrid::pack_index(10, 10, 10);
        grid.accumulate(key, 5.0, 1 << 0);
        grid.accumulate(key, 5.0, 1 << 1);

        let slot = grid.find_slot(key);
        assert_eq!(grid.slots[slot].camera_mask.count_ones(), 2);

        grid.reset_camera_masks();
        assert_eq!(grid.slots[slot].camera_mask, 0);
        // Intensity preserved.
        assert!((grid.slots[slot].intensity - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_add_camera_contributions() {
        let mut grid = test_grid(1000);

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
        let key = FlatVoxelGrid::pack_index(10, 10, 10);
        let slot = grid.find_slot(key);
        assert!((grid.slots[slot].intensity - 1.5).abs() < 0.01);
        assert_eq!(grid.slots[slot].camera_mask.count_ones(), 1);
    }

    #[test]
    fn test_add_camera_contributions_multiple_cameras() {
        let mut grid = test_grid(1000);

        let contribution = vec![VoxelContribution {
            index: UVec3::new(10, 10, 10),
            intensity: 1.0,
        }];
        grid.add_camera_contributions(0, &contribution);
        grid.add_camera_contributions(1, &contribution);
        grid.add_camera_contributions(2, &contribution);

        let key = FlatVoxelGrid::pack_index(10, 10, 10);
        let slot = grid.find_slot(key);
        assert!((grid.slots[slot].intensity - 3.0).abs() < 0.01);
        assert_eq!(grid.slots[slot].camera_mask.count_ones(), 3);
    }

    #[test]
    fn test_nan_inf_rejected() {
        let mut grid = test_grid(1000);

        let contributions = vec![
            VoxelContribution {
                index: UVec3::new(1, 0, 0),
                intensity: 1.0,
            },
            VoxelContribution {
                index: UVec3::new(2, 0, 0),
                intensity: f32::NAN,
            },
            VoxelContribution {
                index: UVec3::new(3, 0, 0),
                intensity: f32::INFINITY,
            },
            VoxelContribution {
                index: UVec3::new(4, 0, 0),
                intensity: -1.0,
            },
            VoxelContribution {
                index: UVec3::new(5, 0, 0),
                intensity: 2.0,
            },
        ];
        grid.add_camera_contributions(0, &contributions);

        assert_eq!(grid.active_count(), 2);
    }

    #[test]
    fn test_clear() {
        let mut grid = test_grid(1000);

        let key = FlatVoxelGrid::pack_index(10, 10, 10);
        grid.accumulate(key, 5.0, 1);
        assert_eq!(grid.active_count(), 1);

        grid.clear();
        assert_eq!(grid.active_count(), 0);
    }

    #[test]
    fn test_extract_points() {
        let mut grid = test_grid(1000);

        let key = FlatVoxelGrid::pack_index(5, 5, 5);
        grid.accumulate(key, 10.0, (1 << 0) | (1 << 1));

        let config = DetectionConfig {
            intensity_threshold: 1.0,
            min_contributors: 2,
            cluster_epsilon: 3.0,
            cluster_min_points: 1,
        };
        let points = grid.extract_points(&config);
        assert_eq!(points.len(), 1);
        assert!((points[0].intensity - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_max_voxels_limit_via_contributions() {
        let mut grid = test_grid(5);

        let contributions: Vec<VoxelContribution> = (0..10)
            .map(|i| VoxelContribution {
                index: UVec3::new(i, 0, 0),
                intensity: 1.0,
            })
            .collect();
        grid.add_camera_contributions(0, &contributions);

        assert_eq!(grid.active_count(), 5);
    }
}
