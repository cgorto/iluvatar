use glam::Vec3;
use iluvatar_core::{ObjectId, TrackedObject};
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

use crate::detector::ObjectIdGenerator;

const HISTORY_LENGTH: usize = 10;

struct TrackedState {
    object: TrackedObject,
    history: VecDeque<Vec3>,
    missing_frames: u32,
}

/// Tracks objects across frames, computing velocities and maintaining IDs
pub struct ObjectTracker {
    tracks: HashMap<ObjectId, TrackedState>,
    id_generator: Arc<ObjectIdGenerator>,
    association_threshold: f32,
    max_missing_frames: u32,
    frame_dt: f32, // seconds between frames
}

impl ObjectTracker {
    pub fn new(
        id_generator: Arc<ObjectIdGenerator>,
        association_threshold: f32,
        max_missing_frames: u32,
        frame_rate: f32,
    ) -> Self {
        Self {
            tracks: HashMap::new(),
            id_generator,
            association_threshold,
            max_missing_frames,
            frame_dt: 1.0 / frame_rate,
        }
    }

    /// Update tracks with new detections
    pub fn update(&mut self, detections: Vec<TrackedObject>) -> Vec<TrackedObject> {
        // Mark all tracks as potentially missing
        for state in self.tracks.values_mut() {
            state.missing_frames += 1;
        }

        let mut output = Vec::new();
        let mut unmatched_detections: Vec<TrackedObject> = Vec::new();

        // Match detections to existing tracks
        for detection in detections {
            if let Some(track_id) = self.find_matching_track(&detection) {
                self.do_update_track(track_id, &detection);
                if let Some(state) = self.tracks.get(&track_id) {
                    output.push(state.object.clone());
                }
            } else {
                unmatched_detections.push(detection);
            }
        }

        // Create new tracks for unmatched detections
        for detection in unmatched_detections {
            let id = self.id_generator.next();

            let mut object = detection;
            object.id = id;

            let mut history = VecDeque::with_capacity(HISTORY_LENGTH);
            history.push_back(object.centroid);

            self.tracks.insert(
                id,
                TrackedState {
                    object: object.clone(),
                    history,
                    missing_frames: 0,
                },
            );

            output.push(object);
        }

        // Remove stale tracks (immediate removal per design decision)
        self.tracks
            .retain(|_, state| state.missing_frames < self.max_missing_frames);

        output
    }

    /// Find existing track that matches a detection
    fn find_matching_track(&self, detection: &TrackedObject) -> Option<ObjectId> {
        let thresh_sq = self.association_threshold * self.association_threshold;

        self.tracks
            .iter()
            .filter(|(_, state)| {
                let predicted = self.predict_position(state);
                predicted.distance_squared(detection.centroid) <= thresh_sq
            })
            .min_by(|(_, a), (_, b)| {
                let dist_a = a.object.centroid.distance_squared(detection.centroid);
                let dist_b = b.object.centroid.distance_squared(detection.centroid);
                dist_a.partial_cmp(&dist_b).unwrap()
            })
            .map(|(id, _)| *id)
    }

    /// Predict where a track should be based on velocity
    fn predict_position(&self, state: &TrackedState) -> Vec3 {
        if let Some(velocity) = state.object.velocity {
            state.object.centroid + velocity * self.frame_dt * state.missing_frames as f32
        } else {
            state.object.centroid
        }
    }

    /// Update a track with a new detection (by track ID)
    fn do_update_track(&mut self, track_id: ObjectId, detection: &TrackedObject) {
        let frame_dt = self.frame_dt;
        if let Some(state) = self.tracks.get_mut(&track_id) {
            Self::update_track_state(state, detection, frame_dt);
        }
    }

    /// Update track state with a new detection
    fn update_track_state(state: &mut TrackedState, detection: &TrackedObject, frame_dt: f32) {
        state.history.push_back(detection.centroid);
        if state.history.len() > HISTORY_LENGTH {
            state.history.pop_front();
        }

        // Compute velocity from history
        let velocity = if state.history.len() >= 2 {
            let oldest = state.history.front().unwrap();
            let newest = state.history.back().unwrap();
            let dt = state.history.len() as f32 * frame_dt;
            Some((*newest - *oldest) / dt)
        } else {
            None
        };

        state.object = TrackedObject {
            id: state.object.id,
            centroid: detection.centroid,
            bounding_box: detection.bounding_box,
            point_count: detection.point_count,
            total_intensity: detection.total_intensity,
            velocity,
            confidence: detection.confidence,
        };

        state.missing_frames = 0;
    }

    /// Get current track count
    pub fn track_count(&self) -> usize {
        self.tracks.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use iluvatar_core::BoundingBox;

    fn make_object(id: ObjectId, pos: Vec3) -> TrackedObject {
        TrackedObject {
            id,
            centroid: pos,
            bounding_box: BoundingBox::new(pos - Vec3::ONE, pos + Vec3::ONE),
            point_count: 1,
            total_intensity: 1.0,
            velocity: None,
            confidence: 1.0,
        }
    }

    #[test]
    fn test_tracking() {
        let id_gen = Arc::new(ObjectIdGenerator::new());
        let mut tracker = ObjectTracker::new(id_gen, 5.0, 30, 60.0);

        // First frame
        let objects = tracker.update(vec![make_object(0, Vec3::new(0.0, 0.0, 0.0))]);
        assert_eq!(objects.len(), 1);
        let first_id = objects[0].id;

        // Second frame, slightly moved
        let objects = tracker.update(vec![make_object(0, Vec3::new(1.0, 0.0, 0.0))]);
        assert_eq!(objects.len(), 1);
        assert_eq!(objects[0].id, first_id); // Same track ID
    }
}
