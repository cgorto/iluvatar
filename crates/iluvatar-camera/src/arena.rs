use bumpalo::Bump;

/// Arena allocator for per-frame allocations.
///
/// This arena is reset at the end of each frame processing cycle,
/// providing efficient allocation without individual deallocations.
pub struct FrameArena {
    bump: Bump,
}

impl FrameArena {
    /// Create a new frame arena with default capacity.
    pub fn new() -> Self {
        Self { bump: Bump::new() }
    }

    /// Create a new frame arena with specified initial capacity in bytes.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            bump: Bump::with_capacity(capacity),
        }
    }

    /// Allocate a slice of bytes from the arena.
    pub fn alloc_slice(&self, len: usize) -> &mut [u8] {
        self.bump.alloc_slice_fill_default(len)
    }

    /// Allocate a slice and copy data into it.
    pub fn alloc_slice_copy(&self, data: &[u8]) -> &mut [u8] {
        self.bump.alloc_slice_copy(data)
    }

    /// Allocate a vector-like growable buffer backed by the arena.
    pub fn alloc_vec<T: Copy>(&self) -> bumpalo::collections::Vec<'_, T> {
        bumpalo::collections::Vec::new_in(&self.bump)
    }

    /// Allocate a vector with specified capacity.
    pub fn alloc_vec_with_capacity<T: Copy>(
        &self,
        capacity: usize,
    ) -> bumpalo::collections::Vec<'_, T> {
        bumpalo::collections::Vec::with_capacity_in(capacity, &self.bump)
    }

    /// Reset the arena, deallocating all allocations.
    /// Call this at the end of each frame.
    pub fn reset(&mut self) {
        self.bump.reset();
    }

    /// Get the number of bytes currently allocated.
    pub fn allocated_bytes(&self) -> usize {
        self.bump.allocated_bytes()
    }
}

impl Default for FrameArena {
    fn default() -> Self {
        Self::new()
    }
}

/// A grayscale frame that borrows its data from an arena.
pub struct ArenaGrayscaleFrame<'a> {
    pub width: u32,
    pub height: u32,
    pub data: &'a mut [u8],
}

impl<'a> ArenaGrayscaleFrame<'a> {
    /// Allocate a new grayscale frame from the arena.
    pub fn new(arena: &'a FrameArena, width: u32, height: u32) -> Self {
        let data = arena.alloc_slice((width * height) as usize);
        Self {
            width,
            height,
            data,
        }
    }

    pub fn pixels(&self) -> &[u8] {
        self.data
    }

    pub fn pixels_mut(&mut self) -> &mut [u8] {
        self.data
    }

    pub fn get(&self, x: u32, y: u32) -> u8 {
        self.data[(y * self.width + x) as usize]
    }

    pub fn set(&mut self, x: u32, y: u32, value: u8) {
        self.data[(y * self.width + x) as usize] = value;
    }
}

/// A difference mask that borrows its data from an arena.
pub struct ArenaDifferenceMask<'a> {
    pub width: u32,
    pub height: u32,
    pub data: &'a mut [u8],
}

impl<'a> ArenaDifferenceMask<'a> {
    /// Allocate a new difference mask from the arena.
    pub fn new(arena: &'a FrameArena, width: u32, height: u32) -> Self {
        let data = arena.alloc_slice((width * height) as usize);
        Self {
            width,
            height,
            data,
        }
    }

    pub fn set(&mut self, index: usize, value: u8) {
        self.data[index] = value;
    }

    pub fn get(&self, x: u32, y: u32) -> u8 {
        self.data[(y * self.width + x) as usize]
    }

    /// Iterate over pixels with motion (value > 0)
    pub fn motion_pixels(&self) -> impl Iterator<Item = (u32, u32, u8)> + '_ {
        self.data.iter().enumerate().filter_map(|(i, &value)| {
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
        self.data.iter().filter(|&&v| v > 0).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arena_allocation() {
        let arena = FrameArena::with_capacity(1024 * 1024);

        let frame = ArenaGrayscaleFrame::new(&arena, 640, 480);
        assert_eq!(frame.data.len(), 640 * 480);

        let mask = ArenaDifferenceMask::new(&arena, 640, 480);
        assert_eq!(mask.data.len(), 640 * 480);

        // Both allocations should be from the same arena
        assert!(arena.allocated_bytes() >= 640 * 480 * 2);
    }

    #[test]
    fn test_arena_reset() {
        let mut arena = FrameArena::with_capacity(1024);

        let _ = arena.alloc_slice(512);
        assert!(arena.allocated_bytes() >= 512);

        arena.reset();
        // After reset, we can allocate again from the beginning
        let _ = arena.alloc_slice(512);
    }
}
