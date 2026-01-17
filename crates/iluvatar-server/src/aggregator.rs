use crate::time::{Clock, TimePoint};
use iluvatar_core::{CameraFrame, CameraId};
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::time::Duration;

/// Aggregates frames from multiple cameras to ensure time-synchronized processing.
///
/// It buffers frames and releases them in batches when:
/// 1. We have frames from all active cameras for a specific timestamp (ideal), OR
/// 2. A timeout elapses (latency limit).
pub struct FrameAggregator {
    /// Buffered frames organized by timestamp (bucketed)
    buffer: VecDeque<FrameBatch>,
    clock: Arc<Clock>,
    /// Set of currently active camera IDs (to know when a batch is "complete")
    active_cameras: HashMap<CameraId, TimePoint>,
    /// Maximum latency we are willing to tolerate waiting for stragglers
    max_latency: Duration,
    /// Time window to group frames together (microseconds)
    sync_window_micros: u64,
}

struct FrameBatch {
    /// The timestamp of the first frame in this batch (microseconds)
    timestamp: u64,
    /// The actual frames
    frames: Vec<CameraFrame>,
    /// Which cameras have contributed to this batch
    contributors: u64, // Bitmask
    /// When this batch was first created (for timeout)
    created_at: TimePoint,
}

impl FrameAggregator {
    pub fn new(max_latency: Duration, sync_window_micros: u64, clock: Arc<Clock>) -> Self {
        Self {
            buffer: VecDeque::new(),
            active_cameras: HashMap::new(),
            max_latency,
            sync_window_micros,
            clock,
        }
    }

    pub fn add_frame(&mut self, frame: CameraFrame) {
        let now = self.clock.now();
        let camera_id = frame.camera_id;
        let frame_ts = frame.timestamp;

        self.active_cameras.insert(camera_id, now);

        // Prune inactive cameras (haven't heard from in 2 seconds)
        self.active_cameras
            .retain(|_, last_seen| now.duration_since(*last_seen) < Duration::from_secs(2));

        // Try to add to an existing batch
        for batch in &mut self.buffer {
            // Check if frame timestamp is within window of batch timestamp
            let diff = if frame_ts > batch.timestamp {
                frame_ts - batch.timestamp
            } else {
                batch.timestamp - frame_ts
            };

            if diff <= self.sync_window_micros {
                // Check if we already have a frame from this camera in this batch
                if (batch.contributors & (1 << camera_id)) == 0 {
                    batch.frames.push(frame.clone());
                    batch.contributors |= 1 << camera_id;
                    return;
                } else {
                    // Duplicate frame from same camera for this timeslot? Update it?
                    // For now, ignore duplicates or newer frames for same slot
                }
            }
        }

        // Create new batch
        self.buffer.push_back(FrameBatch {
            timestamp: frame_ts,
            frames: vec![frame],
            contributors: 1 << camera_id,
            created_at: now,
        });
    }

    /// Returns a vector of frames if a synchronized batch is ready.
    pub fn try_get_batch(&mut self) -> Option<Vec<CameraFrame>> {
        if self.buffer.is_empty() {
            return None;
        }

        let now = self.clock.now();
        let active_mask = self.get_active_mask();

        // Check head of queue
        let is_ready = {
            let front = self.buffer.front().unwrap();
            let age = now.duration_since(front.created_at);

            // Ready if we have all active cameras OR we've waited too long
            // Also ensure we have at least one frame
            let has_all_active = (front.contributors & active_mask) == active_mask;
            has_all_active || age >= self.max_latency
        };

        if is_ready {
            if let Some(batch) = self.buffer.pop_front() {
                if !batch.frames.is_empty() {
                    return Some(batch.frames);
                }
            }
        }

        None
    }

    fn get_active_mask(&self) -> u64 {
        let mut mask = 0u64;
        for &id in self.active_cameras.keys() {
            if id < 64 {
                mask |= 1 << id;
            }
        }
        mask
    }
}
