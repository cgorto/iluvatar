use crate::arena::FrameArena;
use crate::capture::GrayscaleFrame;
#[allow(unused_imports)]
use crate::profile_scope;

#[cfg(feature = "simd")]
mod simd {
    use core::simd::{prelude::*, Select};

    const LANES: usize = 32;

    /// SIMD-accelerated frame difference with threshold.
    ///
    /// Computes `|current[i] - previous[i]|` for each pixel and writes the
    /// difference to `mask[i]` if it exceeds `threshold`, otherwise writes 0.
    ///
    /// On RISC-V with the V extension, this compiles to vectorized
    /// vle8/vmaxu/vminu/vsub/vmsgtu/vmerge/vse8 instructions, processing
    /// 32 pixels per iteration (LMUL=2 on VLEN=128).
    ///
    /// # Safety
    /// The `target_feature` attribute is used to enable vector instructions
    /// only for this function, avoiding LLVM crashes in dependency crates.
    #[cfg_attr(target_arch = "riscv64", target_feature(enable = "v"))]
    pub unsafe fn diff_threshold(current: &[u8], previous: &[u8], mask: &mut [u8], threshold: u8) {
        let len = current.len().min(previous.len()).min(mask.len());
        let thresh = Simd::<u8, LANES>::splat(threshold);
        let zero = Simd::<u8, LANES>::splat(0);

        let chunks = len / LANES;
        let remainder = len % LANES;

        for i in 0..chunks {
            let offset = i * LANES;
            let va = Simd::<u8, LANES>::from_slice(&current[offset..]);
            let vb = Simd::<u8, LANES>::from_slice(&previous[offset..]);
            // abs_diff = max(a,b) - min(a,b)
            let diff = va.simd_max(vb) - va.simd_min(vb);
            let above = diff.simd_gt(thresh);
            let result = above.select(diff, zero);
            mask[offset..offset + LANES].copy_from_slice(result.as_array());
        }

        // Scalar remainder
        let start = chunks * LANES;
        for i in start..start + remainder {
            let diff = current[i].abs_diff(previous[i]);
            mask[i] = if diff > threshold { diff } else { 0 };
        }
    }
}

/// Difference mask storing motion detection results
pub struct DifferenceMask<S> {
    pub width: u32,
    pub height: u32,
    pub data: S,
}

impl<S> DifferenceMask<S>
where
    S: AsRef<[u8]>,
{
    pub fn get(&self, x: u32, y: u32) -> u8 {
        self.data.as_ref()[(y * self.width + x) as usize]
    }

    /// Iterate over pixels with motion (value > 0)
    pub fn motion_pixels(&self) -> impl Iterator<Item = (u32, u32, u8)> + '_ {
        self.data
            .as_ref()
            .iter()
            .enumerate()
            .filter_map(|(i, &value)| {
                if value > 0 {
                    let x = (i as u32) % self.width;
                    let y = (i as u32) / self.width;
                    Some((x, y, value))
                } else {
                    None
                }
            })
    }

    /// Count of pixels with detected motion
    pub fn motion_count(&self) -> usize {
        self.data.as_ref().iter().filter(|&&v| v > 0).count()
    }

    /// Max-pool motion pixels into a downsampled grid.
    ///
    /// Each `factor x factor` block is reduced to a single pixel holding the
    /// maximum intensity found anywhere in the block. Coordinates are mapped
    /// back to original resolution (`block_x * factor`, `block_y * factor`) so
    /// server intrinsics remain valid.
    ///
    /// The caller provides a pre-allocated `pool` buffer (sized for the
    /// downsampled dimensions) to avoid per-frame allocation. The buffer is
    /// cleared, filled, and then yielded as `(x, y, intensity)` tuples.
    ///
    /// A `factor` of 1 is a pass-through: the buffer is unused and the full-
    /// resolution `motion_pixels()` iterator is returned directly via the
    /// right variant of the enum.
    /// Max-pool motion pixels into a downsampled grid.
    ///
    /// Each `factor x factor` block is reduced to a single pixel holding the
    /// maximum intensity found anywhere in the block. Coordinates are mapped
    /// back to original resolution (`block_x * factor`, `block_y * factor`) so
    /// server intrinsics remain valid.
    ///
    /// The caller provides a pre-allocated `pool` buffer to avoid per-frame
    /// allocation. It is cleared and resized as needed.
    ///
    /// A `factor` of 1 is a valid identity operation (pool equals full mask).
    pub fn motion_pixels_pooled<'b>(
        &self,
        factor: u32,
        pool: &'b mut Vec<u8>,
    ) -> PooledMotionPixels<'b> {
        assert!(factor >= 1);

        let pooled_width  = self.width.div_ceil(factor);
        let pooled_height = self.height.div_ceil(factor);
        let pooled_len = (pooled_width * pooled_height) as usize;

        // Reuse the buffer, resizing only if the dimensions changed.
        pool.resize(pooled_len, 0);
        pool.fill(0);

        let data = self.data.as_ref();
        let width = self.width;

        // Single pass over motion pixels: assign each to its block, keep max.
        for (i, &value) in data.iter().enumerate() {
            if value > 0 {
                let x = (i as u32) % width;
                let y = (i as u32) / width;
                let bx = x / factor;
                let by = y / factor;
                let bi = (by * pooled_width + bx) as usize;
                if value > pool[bi] {
                    pool[bi] = value;
                }
            }
        }

        PooledMotionPixels {
            pool,
            pooled_width,
            factor,
            index: 0,
        }
    }

    /// Filter out isolated pixels (noise)
    /// Performs a single pass of erosion: pixels must have at least `min_neighbors` active neighbors
    /// (out of 8) to survive.
    ///
    /// The `buffer` parameter is a pre-allocated Vec that will be cleared and reused to avoid
    /// per-frame heap allocations.
    pub fn filter_noise(&mut self, min_neighbors: u8, buffer: &mut Vec<usize>)
    where
        S: AsMut<[u8]> + AsRef<[u8]>,
    {
        // Clear and reuse the provided buffer instead of allocating
        buffer.clear();

        let width = self.width;
        let height = self.height;

        {
            let data = self.data.as_ref();

            for y in 0..height {
                for x in 0..width {
                    let idx = (y * width + x) as usize;
                    if data[idx] > 0 {
                        let mut neighbors = 0;
                        for dy in -1..=1 {
                            for dx in -1..=1 {
                                if dx == 0 && dy == 0 {
                                    continue;
                                }

                                let nx = x as i32 + dx;
                                let ny = y as i32 + dy;

                                if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                                    if data[(ny as u32 * width + nx as u32) as usize] > 0 {
                                        neighbors += 1;
                                    }
                                }
                            }
                        }

                        if neighbors < min_neighbors {
                            buffer.push(idx);
                        }
                    }
                }
            }
        }

        let data_mut = self.data.as_mut();
        for &idx in buffer.iter() {
            data_mut[idx] = 0;
        }
    }
}

impl<S> DifferenceMask<S>
where
    S: AsMut<[u8]>,
{
    pub fn set(&mut self, index: usize, value: u8) {
        self.data.as_mut()[index] = value;
    }
}

impl DifferenceMask<Vec<u8>> {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            data: vec![0; (width * height) as usize],
        }
    }
}

impl<'a> DifferenceMask<&'a mut [u8]> {
    pub fn new_in(arena: &'a FrameArena, width: u32, height: u32) -> Self {
        let data = arena.alloc_slice((width * height) as usize);
        Self {
            width,
            height,
            data,
        }
    }
}

/// Iterator over max-pooled motion pixels.
///
/// Walks a downsampled buffer and emits non-zero cells with coordinates
/// scaled back to original resolution.
pub struct PooledMotionPixels<'a> {
    pool: &'a [u8],
    pooled_width: u32,
    factor: u32,
    index: usize,
}

impl Iterator for PooledMotionPixels<'_> {
    type Item = (u32, u32, u8);

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.pool.len() {
            let i = self.index;
            self.index += 1;
            let value = self.pool[i];
            if value > 0 {
                let bx = (i as u32) % self.pooled_width;
                let by = (i as u32) / self.pooled_width;
                let x = bx * self.factor;
                let y = by * self.factor;
                return Some((x, y, value));
            }
        }
        None
    }
}

/// Frame processor for computing difference masks
pub struct FrameProcessor {
    previous_frame: Option<GrayscaleFrame<Vec<u8>>>,
    threshold: u8,
    min_neighbors: u8,
    /// Pre-allocated buffer for noise filtering to avoid per-frame allocations.
    noise_filter_buffer: Vec<usize>,
    /// Pre-allocated buffer for motion pixel max-pooling.
    /// Sized to `ceil(width / factor) * ceil(height / factor)` on first use.
    pub pool_buffer: Vec<u8>,
}

impl FrameProcessor {
    pub fn new(threshold: u8) -> Self {
        Self {
            previous_frame: None,
            threshold,
            min_neighbors: 2, // Default to requiring 2 neighbors
            noise_filter_buffer: Vec::new(),
            pool_buffer: Vec::new(),
        }
    }

    pub fn set_threshold(&mut self, threshold: u8) {
        self.threshold = threshold;
    }

    pub fn set_min_neighbors(&mut self, min_neighbors: u8) {
        self.min_neighbors = min_neighbors;
    }

    /// Compute difference mask between current and previous frame
    pub fn compute_difference<'a, S>(
        &mut self,
        current: &GrayscaleFrame<S>,
        arena: &'a FrameArena,
    ) -> Option<DifferenceMask<&'a mut [u8]>>
    where
        S: AsRef<[u8]>,
    {
        let result = if let Some(ref previous) = self.previous_frame {
            if previous.width != current.width || previous.height != current.height {
                // Resolution changed, reset previous
                None
            } else {
                let mut mask = DifferenceMask::new_in(arena, current.width, current.height);

                {
                    profile_scope!("diff_pixels");
                    #[cfg(feature = "simd")]
                    {
                        // SAFETY: The `simd` feature is only enabled when building for
                        // a target with RISC-V V extension support (verified via /proc/cpuinfo).
                        unsafe {
                            simd::diff_threshold(
                                current.pixels(),
                                previous.pixels(),
                                mask.data.as_mut(),
                                self.threshold,
                            );
                        }
                    }

                    #[cfg(not(feature = "simd"))]
                    {
                        let mask_data = mask.data.as_mut();
                        for (i, (curr, prev)) in current
                            .pixels()
                            .iter()
                            .zip(previous.pixels().iter())
                            .enumerate()
                        {
                            let diff = curr.abs_diff(*prev);
                            mask_data[i] = if diff > self.threshold { diff } else { 0 };
                        }
                    }
                }

                if self.min_neighbors > 0 {
                    profile_scope!("noise_filter");
                    mask.filter_noise(self.min_neighbors, &mut self.noise_filter_buffer);
                }

                Some(mask)
            }
        } else {
            None
        };

        // Update previous frame
        // We need to store an owned copy of the current frame
        {
            profile_scope!("copy_frame");
            if let Some(ref mut prev) = self.previous_frame {
                if prev.width == current.width && prev.height == current.height {
                    // Reuse allocation
                    prev.data.copy_from_slice(current.pixels());
                } else {
                    // Reallocate
                    let mut new_prev = GrayscaleFrame::new(current.width, current.height);
                    new_prev.data.copy_from_slice(current.pixels());
                    self.previous_frame = Some(new_prev);
                }
            } else {
                // First frame
                let mut new_prev = GrayscaleFrame::new(current.width, current.height);
                new_prev.data.copy_from_slice(current.pixels());
                self.previous_frame = Some(new_prev);
            }
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_difference_detection() {
        let mut processor = FrameProcessor::new(10);
        let arena = FrameArena::new();

        let mut frame1 = GrayscaleFrame::new(10, 10);
        for i in 0..100 {
            frame1.data[i] = 100;
        }

        // First frame returns None (no previous)
        assert!(processor.compute_difference(&frame1, &arena).is_none());

        let mut frame2 = GrayscaleFrame::new(10, 10);
        for i in 0..100 {
            frame2.data[i] = if i < 50 { 100 } else { 150 };
        }

        let mask = processor.compute_difference(&frame2, &arena).unwrap();
        assert_eq!(mask.motion_count(), 50);
    }
}
