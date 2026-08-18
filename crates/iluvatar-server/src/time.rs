use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// A malleable time source that can switch between system time and simulated time.
#[derive(Debug)]
pub struct Clock {
    is_simulated: AtomicBool,
    sim_time: AtomicU64, // Microseconds since epoch
}

impl Clock {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            is_simulated: AtomicBool::new(false),
            sim_time: AtomicU64::new(0),
        })
    }

    /// Set the clock to use simulated time with the given timestamp (micros).
    pub fn set_simulated_time(&self, timestamp: u64) {
        self.sim_time.store(timestamp, Ordering::Release);
        self.is_simulated.store(true, Ordering::Release);
    }

    /// Set the clock to use real system time.
    pub fn set_real_time(&self) {
        self.is_simulated.store(false, Ordering::Release);
    }

    /// Get the current time as a TimePoint.
    pub fn now(&self) -> TimePoint {
        let micros = if self.is_simulated.load(Ordering::Acquire) {
            self.sim_time.load(Ordering::Acquire)
        } else {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64
        };
        TimePoint(micros)
    }

    /// Helper to get micros directly.
    pub fn now_micros(&self) -> u64 {
        self.now().0
    }
}

/// A point in time, abstracting over Real/Simulated origin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct TimePoint(u64);

impl TimePoint {
    pub fn duration_since(&self, earlier: TimePoint) -> Duration {
        Duration::from_micros(self.0.saturating_sub(earlier.0))
    }

    pub fn as_micros(&self) -> u64 {
        self.0
    }
}
