use crate::arena::FrameArena;
use crate::capture::GrayscaleFrame;

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

/// Frame processor for computing difference masks
pub struct FrameProcessor {
    previous_frame: Option<GrayscaleFrame<Vec<u8>>>,
    threshold: u8,
    min_neighbors: u8,
    /// Pre-allocated buffer for noise filtering to avoid per-frame allocations
    noise_filter_buffer: Vec<usize>,
}

impl FrameProcessor {
    pub fn new(threshold: u8) -> Self {
        Self {
            previous_frame: None,
            threshold,
            min_neighbors: 2, // Default to requiring 2 neighbors
            noise_filter_buffer: Vec::new(),
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
                let mask_data = mask.data.as_mut();

                for (i, (curr, prev)) in current
                    .pixels()
                    .iter()
                    .zip(previous.pixels().iter())
                    .enumerate()
                {
                    let diff = curr.abs_diff(*prev);
                    if diff > self.threshold {
                        mask_data[i] = diff;
                    }
                }

                if self.min_neighbors > 0 {
                    mask.filter_noise(self.min_neighbors, &mut self.noise_filter_buffer);
                }

                Some(mask)
            }
        } else {
            None
        };

        // Update previous frame
        // We need to store an owned copy of the current frame
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
