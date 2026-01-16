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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::GrayscaleFrame;
    use crate::difference::DifferenceMask;

    #[test]
    fn test_arena_allocation() {
        let arena = FrameArena::with_capacity(1024 * 1024);

        let frame = GrayscaleFrame::new_in(&arena, 640, 480);
        assert_eq!(frame.data.len(), 640 * 480);

        let mask = DifferenceMask::new_in(&arena, 640, 480);
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
