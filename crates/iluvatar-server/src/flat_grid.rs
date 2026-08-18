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

/// Sentinel value marking a tombstone slot. Dead entries become tombstones
/// instead of EMPTY so that probe chains remain intact — no rehash needed.
/// `find_slot` skips tombstones during probing; `accumulate` reuses them
/// for new entries.
const TOMBSTONE_KEY: u64 = u64::MAX - 1;

/// Minimum intensity below which a voxel is considered dead during decay.
const INTENSITY_THRESHOLD: f32 = 0.01;

/// Default maximum number of voxels allowed in the grid.
/// 200K voxels keeps the hash table under ~10 MB (fits in L3 cache),
/// preventing the decay loop from becoming memory-bandwidth-bound.
pub const DEFAULT_MAX_VOXELS: usize = 200_000;

/// Initial table capacity. 8K slots × 24 bytes = 192 KB, fits in L2 cache.
/// The table grows by doubling when load factor exceeds 50%.
const INITIAL_CAPACITY: usize = 8192;

/// FxHash constant (golden-ratio fractional bits scaled to 64 bits).
const HASH_CONSTANT: u64 = 0x517cc1b727220a95;

/// A single slot in the hash table. Array-of-structs layout so that one
/// cache line load (~64 bytes) fetches 2 complete slots, which is ideal
/// for linear probing.
///
/// Layout: key(8) + intensity(4) + epoch(4) + camera_mask(8) = 24 bytes.
/// The `epoch` field fills the 4-byte alignment padding that previously
/// existed between `intensity` (f32) and `camera_mask` (u64), so slot
/// size remains 24 bytes — zero memory cost.
#[derive(Clone, Copy)]
struct Slot {
    key: u64,
    intensity: f32,
    epoch: u32,
    camera_mask: u64,
}

impl Slot {
    const EMPTY: Slot = Slot {
        key: EMPTY_KEY,
        intensity: 0.0,
        epoch: 0,
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

    /// Number of tombstone slots (dead entries preserving probe chains).
    /// Tracked separately so the load factor check in `accumulate` accounts
    /// for physical occupancy (live + tombstones), not just live entries.
    tombstone_count: usize,

    /// Current frame epoch. Incremented each processing cycle instead of
    /// scanning the entire table to clear camera masks. Slots whose
    /// `epoch` field does not match this value have a stale camera_mask
    /// that is treated as zero.
    frame_epoch: u32,

    /// Indices of slots that contain live entries. Rebuilt during decay
    /// and grow; new entries are appended during accumulate. Enables
    /// O(live) iteration for extraction and stats instead of O(capacity).
    live_indices: Vec<u32>,

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
            tombstone_count: 0,
            frame_epoch: 1,
            live_indices: Vec::with_capacity(max_voxels),
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

    /// Find the slot for `key`. Returns either:
    /// - The index of the slot containing `key` (if present), or
    /// - The first available slot (tombstone or empty) for insertion.
    ///
    /// Tombstones are skipped during the search but remembered as
    /// insertion candidates. The probe always continues to EMPTY_KEY
    /// to ensure we don't miss a matching key hiding behind tombstones.
    #[inline]
    fn find_slot(&self, key: u64) -> usize {
        debug_assert!(key != EMPTY_KEY);
        debug_assert!(key != TOMBSTONE_KEY);
        let mut slot = self.hash_slot(key);
        let mut first_tombstone: usize = usize::MAX;
        loop {
            let k = self.slots[slot].key;
            if k == key {
                return slot;
            }
            if k == EMPTY_KEY {
                // Key not found. Return first tombstone for reuse, or
                // this empty slot.
                if first_tombstone != usize::MAX {
                    return first_tombstone;
                }
                return slot;
            }
            if k == TOMBSTONE_KEY && first_tombstone == usize::MAX {
                first_tombstone = slot;
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

        // Rebuild live_indices during the rehash into the new table.
        // Tombstones are discarded — they served their purpose.
        self.live_indices.clear();

        for i in 0..self.capacity {
            let slot = &self.slots[i];
            if slot.key == EMPTY_KEY || slot.key == TOMBSTONE_KEY {
                continue;
            }
            // Find new home in the larger table.
            let mut dest = (slot.key.wrapping_mul(HASH_CONSTANT) as usize) & new_mask;
            loop {
                if new_slots[dest].key == EMPTY_KEY {
                    new_slots[dest] = *slot;
                    self.live_indices.push(dest as u32);
                    break;
                }
                dest = (dest + 1) & new_mask;
            }
        }

        let old_cap = self.capacity;
        self.slots = new_slots;
        self.capacity = new_capacity;
        self.mask = new_mask;
        self.tombstone_count = 0;
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
        debug_assert!(key != TOMBSTONE_KEY);
        debug_assert!(intensity.is_finite());
        debug_assert!(intensity >= 0.0);

        let slot_idx = self.find_slot(key);
        let slot = &mut self.slots[slot_idx];

        if slot.key == key {
            // Update existing voxel. If epoch is stale, this is the first
            // touch this frame — reset camera_mask before OR'ing.
            slot.intensity += intensity;
            if slot.epoch != self.frame_epoch {
                slot.camera_mask = camera_bit;
                slot.epoch = self.frame_epoch;
            } else {
                slot.camera_mask |= camera_bit;
            }
        } else {
            // New voxel — check hard cap.
            if self.count >= self.max_voxels {
                return;
            }
            let reusing_tombstone = slot.key == TOMBSTONE_KEY;
            // Grow if physical occupancy (live + tombstones) would exceed
            // 50%. Tombstones occupy real slots and degrade probing if they
            // accumulate beyond the load factor threshold.
            if !reusing_tombstone && (self.count + self.tombstone_count) * 2 >= self.capacity {
                self.grow();
                // Slot index is stale after grow — re-probe.
                let new_idx = self.find_slot(key);
                self.slots[new_idx] = Slot {
                    key,
                    intensity,
                    epoch: self.frame_epoch,
                    camera_mask: camera_bit,
                };
                self.live_indices.push(new_idx as u32);
                self.count += 1;
                return;
            }
            *slot = Slot {
                key,
                intensity,
                epoch: self.frame_epoch,
                camera_mask: camera_bit,
            };
            self.live_indices.push(slot_idx as u32);
            self.count += 1;
            if reusing_tombstone {
                self.tombstone_count -= 1;
            }
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
    /// Dead entries become tombstones instead of being deleted. This
    /// preserves probe chain integrity — no rehash needed. The cost
    /// is O(live) per cycle instead of O(live) + O(live) rehash.
    ///
    /// Tombstones accumulate across cycles and are cleaned up by
    /// `compact()` when they exceed half the live count, amortizing
    /// the expensive rehash across many decay cycles.
    pub fn apply_decay(&mut self) {
        let now = self.clock.now();
        let dt = now.duration_since(self.last_decay).as_secs_f32();
        let decay_factor = (-self.decay_rate * dt).exp();

        // Decay intensities and compact live_indices in place. Dead
        // entries become tombstones: key set to TOMBSTONE_KEY so the
        // probe chain from their hash position is not broken.
        let mut dead_count = 0usize;
        let mut write = 0usize;

        for read in 0..self.live_indices.len() {
            let idx = self.live_indices[read] as usize;
            debug_assert!(self.slots[idx].key != EMPTY_KEY);
            debug_assert!(self.slots[idx].key != TOMBSTONE_KEY);

            self.slots[idx].intensity *= decay_factor;
            if self.slots[idx].intensity <= INTENSITY_THRESHOLD {
                self.slots[idx].key = TOMBSTONE_KEY;
                self.slots[idx].intensity = 0.0;
                dead_count += 1;
            } else {
                self.live_indices[write] = self.live_indices[read];
                write += 1;
            }
        }
        self.live_indices.truncate(write);
        self.count -= dead_count;
        self.tombstone_count += dead_count;

        // Compact when tombstones exceed the live count. This threshold
        // is high enough to keep compaction infrequent (~every 500+ decay
        // cycles at steady state) while still preventing probe chains from
        // degrading. Compaction allocates a fresh table — the old table is
        // freed immediately, avoiding the expensive in-place fill.
        if self.tombstone_count > self.count && self.tombstone_count > 0 {
            self.compact();
        }

        self.last_decay = now;
    }

    /// Remove all tombstones by rehashing live entries into a fresh table.
    ///
    /// Allocates a new table instead of zeroing in-place. The old table is
    /// freed immediately (O(1) deallocation), and the new table's pages are
    /// demand-faulted by the OS — we only pay for pages touched during
    /// reinsertion. This is significantly faster than `slots.fill()` which
    /// writes every byte of the old table.
    ///
    /// The table may shrink if it is oversized for the current live count,
    /// improving cache utilization after a peak subsides.
    fn compact(&mut self) {
        // Collect live entries via live_indices (already compacted by decay).
        let live: Vec<Slot> = self
            .live_indices
            .iter()
            .map(|&idx| {
                let slot = self.slots[idx as usize];
                debug_assert!(slot.key != EMPTY_KEY);
                debug_assert!(slot.key != TOMBSTONE_KEY);
                slot
            })
            .collect();
        debug_assert_eq!(live.len(), self.count);

        // Shrink the table if it is oversized for the current live count.
        // Target: 50% load factor, minimum INITIAL_CAPACITY.
        let target = (self.count * 2).next_power_of_two().max(INITIAL_CAPACITY);
        let shrinking = target < self.capacity;
        if shrinking {
            self.capacity = target;
            self.mask = target - 1;
        }

        // Allocate a fresh table. Dropping the old Vec frees its pages
        // without touching them — no expensive zeroing of stale data.
        self.slots = vec![Slot::EMPTY; self.capacity];
        self.live_indices.clear();
        self.tombstone_count = 0;

        for entry in &live {
            let mut idx = self.hash_slot(entry.key);
            loop {
                if self.slots[idx].key == EMPTY_KEY {
                    self.slots[idx] = *entry;
                    self.live_indices.push(idx as u32);
                    break;
                }
                idx = (idx + 1) & self.mask;
            }
        }

        debug_assert_eq!(self.live_indices.len(), self.count);

        if shrinking {
            info!(
                new_capacity = self.capacity,
                live_entries = self.count,
                memory_kb = self.capacity * std::mem::size_of::<Slot>() / 1024,
                "Grid table shrank during compaction"
            );
        }
    }

    // ========================================================================
    // Camera mask management
    // ========================================================================

    /// Advance the frame epoch. Slots with a stale epoch are treated as having
    /// camera_mask == 0 — the first `accumulate()` call that touches a stale
    /// slot will reset its mask and stamp the current epoch. This replaces
    /// an O(capacity) scan with O(1).
    pub fn reset_camera_masks(&mut self) {
        self.frame_epoch = self.frame_epoch.wrapping_add(1);
    }

    // ========================================================================
    // Epoch-aware camera mask
    // ========================================================================

    /// Return the effective camera mask for a slot. If the slot's epoch is
    /// stale (does not match `frame_epoch`), the mask is logically zero —
    /// no cameras contributed this frame.
    #[inline]
    fn effective_camera_mask(&self, slot: &Slot) -> u64 {
        if slot.epoch == self.frame_epoch {
            slot.camera_mask
        } else {
            0
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
    /// Iterates only live slots via `live_indices`.
    pub fn extract_points_with_camera_count(
        &self,
        config: &DetectionConfig,
        active_cameras: u8,
    ) -> Vec<DetectedPoint> {
        let mut points = Vec::new();
        for &idx in &self.live_indices {
            let slot = &self.slots[idx as usize];
            debug_assert!(slot.key != EMPTY_KEY);
            let camera_count = self.effective_camera_mask(slot).count_ones() as u8;
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
    /// Iterates only live slots via `live_indices`.
    pub fn extract_points_percentile(
        &self,
        percentile: f32,
        min_camera_count: u8,
        active_cameras: u8,
    ) -> Vec<DetectedPoint> {
        // First pass: collect qualifying voxels.
        let mut qualifying: Vec<(u64, f32, u8)> = Vec::new();
        for &idx in &self.live_indices {
            let slot = &self.slots[idx as usize];
            debug_assert!(slot.key != EMPTY_KEY);
            let camera_count = self.effective_camera_mask(slot).count_ones() as u8;
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

    /// Convert voxel index to centered world position.
    /// The raymarcher uses a grid centered at the origin (-half_dim to +half_dim).
    /// This returns positions in that centered coordinate system.
    pub fn voxel_to_centered(&self, index: UVec3) -> Vec3 {
        let half_dim = Vec3::new(
            self.dimensions.x as f32 * self.voxel_size * 0.5,
            self.dimensions.y as f32 * self.voxel_size * 0.5,
            self.dimensions.z as f32 * self.voxel_size * 0.5,
        );
        Vec3::new(
            (index.x as f32 + 0.5) * self.voxel_size - half_dim.x,
            (index.y as f32 + 0.5) * self.voxel_size - half_dim.y,
            (index.z as f32 + 0.5) * self.voxel_size - half_dim.z,
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
        for &idx in &self.live_indices {
            let slot = &self.slots[idx as usize];
            debug_assert!(slot.key != EMPTY_KEY);
            if slot.intensity > max_intensity {
                max_intensity = slot.intensity;
            }
            if self.effective_camera_mask(slot) != 0 {
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
        for &idx in &self.live_indices {
            let slot = &self.slots[idx as usize];
            debug_assert!(slot.key != EMPTY_KEY);
            if slot.intensity > max {
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
        for &idx in &self.live_indices {
            if result.len() >= max_voxels {
                break;
            }
            let slot = &self.slots[idx as usize];
            debug_assert!(slot.key != EMPTY_KEY);
            if slot.intensity >= intensity_threshold {
                let pos = Self::unpack_index(slot.key);
                let camera_count = self.effective_camera_mask(slot).count_ones() as u8;
                result.push((self.voxel_to_world(pos), slot.intensity, camera_count));
            }
        }
        result
    }

    /// Debug statistics: (multi_camera_count, high_intensity_count).
    pub fn debug_stats(&self) -> (usize, usize) {
        let mut multi_cam = 0usize;
        let mut high_intensity = 0usize;
        for &idx in &self.live_indices {
            let slot = &self.slots[idx as usize];
            debug_assert!(slot.key != EMPTY_KEY);
            if self.effective_camera_mask(slot).count_ones() >= 2 {
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
        self.tombstone_count = 0;
        self.live_indices.clear();
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
        // Verify AoS slot is compact: key(8) + intensity(4) + epoch(4) + camera_mask(8) = 24.
        // The epoch field fills the alignment padding — zero memory cost.
        assert_eq!(std::mem::size_of::<Slot>(), 24);
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

        let slot_idx = grid.find_slot(key);
        assert_eq!(grid.slots[slot_idx].camera_mask.count_ones(), 2);

        // After reset, the effective mask should be zero (epoch is stale),
        // but the raw bits in the slot are unchanged — only the epoch
        // comparison makes them logically zero.
        grid.reset_camera_masks();
        assert_eq!(grid.effective_camera_mask(&grid.slots[slot_idx]), 0);
        // Intensity preserved.
        assert!((grid.slots[slot_idx].intensity - 10.0).abs() < 0.01);
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

    #[test]
    fn test_epoch_stale_camera_mask() {
        // Verify that after reset_camera_masks (epoch advance), stale
        // camera masks read as zero, and new accumulations start fresh.
        let mut grid = test_grid(1000);
        let key = FlatVoxelGrid::pack_index(10, 10, 10);

        // Frame 1: two cameras contribute.
        grid.accumulate(key, 1.0, 1 << 0);
        grid.accumulate(key, 1.0, 1 << 1);
        let slot_idx = grid.find_slot(key);
        assert_eq!(
            grid.effective_camera_mask(&grid.slots[slot_idx])
                .count_ones(),
            2
        );

        // Advance epoch — effective mask should be zero.
        grid.reset_camera_masks();
        assert_eq!(
            grid.effective_camera_mask(&grid.slots[slot_idx])
                .count_ones(),
            0
        );

        // Frame 2: only camera 3 contributes.
        grid.accumulate(key, 1.0, 1 << 3);
        let slot_idx = grid.find_slot(key);
        assert_eq!(
            grid.effective_camera_mask(&grid.slots[slot_idx])
                .count_ones(),
            1
        );
        // Intensity accumulated across frames.
        assert!((grid.slots[slot_idx].intensity - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_live_indices_consistency() {
        // Verify live_indices matches actual live count after insert,
        // decay, and grow sequences.
        let clock = Clock::new();
        let mut grid = FlatVoxelGrid::with_max_voxels(
            GeoPosition::new(0.0, 0.0, 0.0),
            UVec3::new(1000, 1000, 1000),
            1.0,
            100.0, // High decay for quick removal.
            clock,
            100_000,
        );

        // Insert 200 entries (triggers grow from 8K to 16K at entry 4096).
        for i in 0..200u32 {
            let key = FlatVoxelGrid::pack_index(i, 0, 0);
            grid.accumulate(key, if i < 100 { 0.02 } else { 1000.0 }, 1);
        }
        assert_eq!(grid.active_count(), 200);
        assert_eq!(grid.live_indices.len(), 200);

        // Decay should kill the weak entries (intensity 0.02).
        std::thread::sleep(std::time::Duration::from_millis(50));
        grid.apply_decay();

        assert_eq!(grid.active_count(), 100);
        assert_eq!(grid.live_indices.len(), 100);

        // Every index in live_indices must point to a live slot.
        for &idx in &grid.live_indices {
            assert!(grid.slots[idx as usize].key != EMPTY_KEY);
        }

        // Insert enough to trigger a grow.
        for i in 200..5000u32 {
            let key = FlatVoxelGrid::pack_index(i, 0, 0);
            grid.accumulate(key, 500.0, 1);
        }
        assert_eq!(grid.live_indices.len(), grid.active_count());

        // Every index must still be valid.
        for &idx in &grid.live_indices {
            assert!((idx as usize) < grid.capacity);
            assert!(grid.slots[idx as usize].key != EMPTY_KEY);
        }
    }

    /// Verify that every live slot is reachable via `find_slot` from its
    /// hash position. Catches orphaned entries caused by broken probe chains.
    /// Tombstones are skipped (they are not findable by design).
    fn assert_all_findable(grid: &FlatVoxelGrid) {
        for i in 0..grid.capacity {
            let key = grid.slots[i].key;
            if key == EMPTY_KEY || key == TOMBSTONE_KEY {
                continue;
            }
            let found = grid.find_slot(key);
            assert_eq!(
                found,
                i,
                "Entry at slot {} (key={}) is orphaned: find_slot \
                 returns slot {} (key={})",
                i,
                key,
                found,
                if grid.slots[found].key == EMPTY_KEY {
                    "EMPTY".to_string()
                } else {
                    grid.slots[found].key.to_string()
                },
            );
        }
    }

    #[test]
    fn test_rehash_wraparound_probe_chains() {
        // Regression test: the original rehash_in_place used a left-to-right
        // scan to relocate displaced entries after decay. This failed when a
        // probe chain wrapped around the table boundary.
        //
        // With capacity=8 and FxHash (key × 0x517cc1b727220a95 & 7):
        //   key=1 → slot 5    key=9 → slot 5    key=6 → slot 6    key=3 → slot 7
        //
        // Insert order builds this chain:
        //   slot 5: key=1 (ideal=5)  — low intensity, will die
        //   slot 6: key=9 (ideal=5)  — displaced one step
        //   slot 7: key=6 (ideal=6)  — displaced one step
        //   slot 0: key=3 (ideal=7)  — displaced, WRAPS to slot 0
        //
        // After key=1 dies at slot 5, the old left-to-right scan visited
        // slot 0 first (key=3, ideal=7). It couldn't move key=3 because
        // key=6 still occupied slot 7. Later the scan moved key=6 from
        // slot 7→6 and key=9 from 6→5, leaving slot 7 empty. But slot 0
        // was already processed — key=3 was never relocated, and its probe
        // chain (starting at slot 7) hit EMPTY immediately. Orphaned.
        let clock = Clock::new();
        clock.set_simulated_time(1_000_000);

        let mut grid = FlatVoxelGrid {
            slots: vec![Slot::EMPTY; 8],
            capacity: 8,
            mask: 7,
            count: 0,
            max_voxels: 100,
            tombstone_count: 0,
            frame_epoch: 1,
            live_indices: Vec::with_capacity(100),
            origin: GeoPosition::new(0.0, 0.0, 0.0),
            dimensions: UVec3::new(1000, 1000, 1000),
            voxel_size: 1.0,
            decay_rate: 100.0,
            clock: clock.clone(),
            last_decay: clock.now(),
        };

        // Verify our hash slot assumptions.
        assert_eq!(grid.hash_slot(1), 5);
        assert_eq!(grid.hash_slot(9), 5);
        assert_eq!(grid.hash_slot(6), 6);
        assert_eq!(grid.hash_slot(3), 7);

        // Build the wrap-around chain.
        grid.accumulate(1, 0.02, 1); // slot 5, will die
        grid.accumulate(9, 1000.0, 1); // slot 6, survives
        grid.accumulate(6, 1000.0, 1); // slot 7, survives
        grid.accumulate(3, 1000.0, 1); // wraps to slot 0, survives
        assert_eq!(grid.active_count(), 4);

        // Verify placement before decay.
        assert_eq!(grid.slots[5].key, 1);
        assert_eq!(grid.slots[6].key, 9);
        assert_eq!(grid.slots[7].key, 6);
        assert_eq!(grid.slots[0].key, 3, "key=3 should wrap to slot 0");

        assert_all_findable(&grid);

        // Advance time. decay_factor = exp(-100 × 0.05) ≈ 0.0067.
        // key=1: 0.02 × 0.0067 ≈ 0.00013 < 0.01 → dead.
        // Others: 1000 × 0.0067 ≈ 6.7 > 0.01 → survive.
        clock.set_simulated_time(1_050_000);
        grid.apply_decay();

        assert_eq!(grid.active_count(), 3, "key=1 should have decayed");

        // The critical invariant: every surviving entry must be findable.
        // Before the fix, key=3 was orphaned at slot 0 with an empty slot 7
        // breaking its probe chain.
        assert_all_findable(&grid);

        // Explicit checks for each survivor.
        assert_eq!(grid.slots[grid.find_slot(9)].key, 9);
        assert_eq!(grid.slots[grid.find_slot(6)].key, 6);
        assert_eq!(grid.slots[grid.find_slot(3)].key, 3);
    }
}
