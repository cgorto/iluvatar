use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// A bounded channel that drops the oldest item when full.
///
/// This is useful for backpressure handling where we prefer
/// to drop stale data rather than block production.
pub struct DropOldestChannel<T> {
    inner: Arc<Mutex<ChannelInner<T>>>,
    capacity: usize,
}

struct ChannelInner<T> {
    buffer: VecDeque<T>,
    dropped_count: u64,
}

impl<T> DropOldestChannel<T> {
    /// Create a new channel with the specified capacity.
    ///
    /// Capacity must be at least 1.
    pub fn new(capacity: usize) -> Self {
        assert!(capacity >= 1, "capacity must be at least 1");
        Self {
            inner: Arc::new(Mutex::new(ChannelInner {
                buffer: VecDeque::with_capacity(capacity),
                dropped_count: 0,
            })),
            capacity,
        }
    }

    /// Push an item to the channel.
    ///
    /// If the channel is full, the oldest item is dropped and
    /// this function returns `true` to indicate a drop occurred.
    pub fn push(&self, item: T) -> bool {
        let mut inner = self.inner.lock().unwrap();
        let dropped = if inner.buffer.len() >= self.capacity {
            inner.buffer.pop_front();
            inner.dropped_count += 1;
            true
        } else {
            false
        };
        inner.buffer.push_back(item);
        dropped
    }

    /// Try to pop the oldest item from the channel.
    ///
    /// Returns `None` if the channel is empty.
    pub fn pop(&self) -> Option<T> {
        let mut inner = self.inner.lock().unwrap();
        inner.buffer.pop_front()
    }

    /// Returns the number of items currently in the channel.
    pub fn len(&self) -> usize {
        self.inner.lock().unwrap().buffer.len()
    }

    /// Returns true if the channel is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the total number of items that have been dropped.
    pub fn dropped_count(&self) -> u64 {
        self.inner.lock().unwrap().dropped_count
    }

    /// Returns the capacity of the channel.
    pub fn capacity(&self) -> usize {
        self.capacity
    }
}

impl<T> Clone for DropOldestChannel<T> {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
            capacity: self.capacity,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_push_pop() {
        let channel = DropOldestChannel::new(3);

        assert!(!channel.push(1));
        assert!(!channel.push(2));
        assert!(!channel.push(3));

        assert_eq!(channel.len(), 3);
        assert_eq!(channel.pop(), Some(1));
        assert_eq!(channel.pop(), Some(2));
        assert_eq!(channel.pop(), Some(3));
        assert_eq!(channel.pop(), None);
    }

    #[test]
    fn test_drop_oldest() {
        let channel = DropOldestChannel::new(2);

        assert!(!channel.push(1));
        assert!(!channel.push(2));
        assert!(channel.push(3)); // Should drop 1

        assert_eq!(channel.dropped_count(), 1);
        assert_eq!(channel.pop(), Some(2));
        assert_eq!(channel.pop(), Some(3));
    }

    #[test]
    fn test_multiple_drops() {
        let channel = DropOldestChannel::new(2);

        for i in 0..10 {
            channel.push(i);
        }

        // Should have dropped 8 items, keeping 8 and 9
        assert_eq!(channel.dropped_count(), 8);
        assert_eq!(channel.pop(), Some(8));
        assert_eq!(channel.pop(), Some(9));
    }
}
