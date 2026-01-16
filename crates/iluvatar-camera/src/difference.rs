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
}

impl FrameProcessor {
    pub fn new(threshold: u8) -> Self {
        Self {
            previous_frame: None,
            threshold,
        }
    }

    pub fn set_threshold(&mut self, threshold: u8) {
        self.threshold = threshold;
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
                let mask = DifferenceMask::new_in(arena, current.width, current.height);
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
