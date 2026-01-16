use iluvatar_core::{CameraFrame, CameraId};
use std::collections::{HashMap, VecDeque};
use std::time::Duration;

/// Frame buffer for a single camera
struct FrameBuffer {
    frames: VecDeque<CameraFrame>,
    max_frames: usize,
}

impl FrameBuffer {
    fn new(max_frames: usize) -> Self {
        Self {
            frames: VecDeque::with_capacity(max_frames),
            max_frames,
        }
    }

    fn push(&mut self, frame: CameraFrame) {
        if self.frames.len() >= self.max_frames {
            self.frames.pop_front();
        }
        self.frames.push_back(frame);
    }

    fn get_frame_near(&self, target_time: u64) -> Option<&CameraFrame> {
        // Find frame closest to target time within window
        self.frames
            .iter()
            .min_by_key(|f| (f.timestamp as i64 - target_time as i64).abs())
    }
}

/// Aggregates frames from multiple cameras
pub struct FrameAggregator {
    camera_buffers: HashMap<CameraId, FrameBuffer>,
    aggregation_window: Duration,
    max_frames_per_camera: usize,
}

impl FrameAggregator {
    pub fn new(aggregation_window: Duration, max_frames_per_camera: usize) -> Self {
        Self {
            camera_buffers: HashMap::new(),
            aggregation_window,
            max_frames_per_camera,
        }
    }

    /// Add a frame from a camera
    pub fn add_frame(&mut self, frame: CameraFrame) {
        let buffer = self
            .camera_buffers
            .entry(frame.camera_id)
            .or_insert_with(|| FrameBuffer::new(self.max_frames_per_camera));

        buffer.push(frame);
    }

    /// Get frames from all cameras near the target time
    pub fn get_frames_near(&self, target_time: u64) -> Vec<&CameraFrame> {
        let window_us = self.aggregation_window.as_micros() as u64;

        self.camera_buffers
            .values()
            .filter_map(|buffer| {
                buffer.get_frame_near(target_time).filter(|f| {
                    let diff = (f.timestamp as i64 - target_time as i64).unsigned_abs();
                    diff <= window_us
                })
            })
            .collect()
    }

    /// Get the latest timestamp across all cameras
    pub fn latest_timestamp(&self) -> Option<u64> {
        self.camera_buffers
            .values()
            .filter_map(|buffer| buffer.frames.back().map(|f| f.timestamp))
            .max()
    }

    /// Get number of active cameras
    pub fn camera_count(&self) -> usize {
        self.camera_buffers.len()
    }

    /// Remove camera from aggregator
    pub fn remove_camera(&mut self, camera_id: CameraId) {
        self.camera_buffers.remove(&camera_id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, UVec3};
    use iluvatar_core::{
        CameraPose, GeoPosition, LocalizationStatus, PoseUncertainty, VoxelContribution,
    };

    fn make_frame(camera_id: CameraId, timestamp: u64) -> CameraFrame {
        CameraFrame {
            camera_id,
            sequence: 0,
            timestamp,
            pose: CameraPose {
                position: GeoPosition::new(0.0, 0.0, 0.0),
                orientation: Quat::IDENTITY,
                timestamp,
                uncertainty: PoseUncertainty::default(),
                status: LocalizationStatus::Nominal,
            },
            contributions: vec![VoxelContribution {
                index: UVec3::new(0, 0, 0),
                intensity: 1.0,
            }],
        }
    }

    #[test]
    fn test_aggregator() {
        let mut agg = FrameAggregator::new(Duration::from_millis(100), 10);

        agg.add_frame(make_frame(1, 1000));
        agg.add_frame(make_frame(2, 1050));
        agg.add_frame(make_frame(3, 1100));

        let frames = agg.get_frames_near(1050);
        assert_eq!(frames.len(), 3);
    }
}
