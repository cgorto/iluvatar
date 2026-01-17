use glam::Vec3;
use iluvatar_core::{ObjectId, TrackedObject};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};

use crate::kalman::Kalman3D;

/// ID generator for tracked objects (owned by tracker)
struct ObjectIdGenerator {
    next_id: AtomicU64,
}

impl ObjectIdGenerator {
    fn new() -> Self {
        Self {
            next_id: AtomicU64::new(1), // Start at 1; 0 is reserved for anonymous detections
        }
    }

    fn next(&self) -> ObjectId {
        self.next_id.fetch_add(1, Ordering::Relaxed)
    }
}

struct TrackedState {
    object: TrackedObject,
    kalman: Kalman3D,
    missing_frames: u32,
}

/// Tracks objects across frames, computing velocities and maintaining IDs
pub struct ObjectTracker {
    tracks: HashMap<ObjectId, TrackedState>,
    id_generator: ObjectIdGenerator,
    association_threshold: f32,
    max_missing_frames: u32,
}

impl ObjectTracker {
    pub fn new(association_threshold: f32, max_missing_frames: u32, _frame_rate: f32) -> Self {
        Self {
            tracks: HashMap::new(),
            id_generator: ObjectIdGenerator::new(),
            association_threshold,
            max_missing_frames,
        }
    }

    /// Update tracks with new detections
    pub fn update(&mut self, detections: Vec<TrackedObject>, dt: f32) -> Vec<TrackedObject> {
        // Mark all tracks as potentially missing
        for state in self.tracks.values_mut() {
            state.missing_frames += 1;
        }

        let mut output = Vec::new();
        let mut unmatched_detections: Vec<TrackedObject> = Vec::new();
        let mut used_tracks: HashSet<ObjectId> = HashSet::new();

        // Match detections to existing tracks
        // Ideally we should sort matches by distance, but greedy is fine if we check "closest track for this detection"
        // AND "closest detection for this track".
        // For now, let's stick to the current flow but enforce unique assignment.
        // To do better greedy, we can collect all pairs (dist, detection_idx, track_id) and sort.

        // Let's implement a slightly better greedy association:
        // 1. Calculate all valid pairwise distances
        // 2. Sort by distance
        // 3. Assign

        let mut matches = Vec::new();
        for (det_idx, detection) in detections.iter().enumerate() {
            let thresh_sq = self.association_threshold * self.association_threshold;

            for (track_id, state) in &self.tracks {
                let predicted = self.predict_position(state, dt);
                let dist_sq = predicted.distance_squared(detection.centroid);

                if dist_sq <= thresh_sq {
                    matches.push((dist_sq, det_idx, *track_id));
                }
            }
        }

        // Sort by distance (closest first)
        matches.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let mut matched_detection_indices = HashSet::new();

        for (_, det_idx, track_id) in matches {
            if matched_detection_indices.contains(&det_idx) || used_tracks.contains(&track_id) {
                continue;
            }

            // Perform update
            self.do_update_track(track_id, &detections[det_idx], dt);
            matched_detection_indices.insert(det_idx);
            used_tracks.insert(track_id);

            if let Some(state) = self.tracks.get(&track_id) {
                output.push(state.object.clone());
            }
        }

        // Collect unmatched detections
        for (i, detection) in detections.into_iter().enumerate() {
            if !matched_detection_indices.contains(&i) {
                unmatched_detections.push(detection);
            }
        }

        // Create new tracks for unmatched detections
        for detection in unmatched_detections {
            let id = self.id_generator.next();

            let mut object = detection;
            object.id = id;
            // Initialize velocity to zero if not present
            if object.velocity.is_none() {
                object.velocity = Some(Vec3::ZERO);
            }

            // Initialize Kalman filter
            // Tuning: balance between responsive velocity estimation and stable tracking.
            // - process_noise: expected acceleration std dev (m/s²). Lower = trust motion model more
            // - measurement_noise: position uncertainty (m). Lower = trust measurements more
            //
            // For sinusoidal/accelerating targets with peak accel ~50 m/s²:
            // Very high process noise minimizes velocity lag (critical for sine tracking)
            // Low measurement noise keeps position accurate
            //
            // process_noise=20.0 -> q_vel=400 (minimal lag, some noise acceptable)
            // measurement_noise=0.1 -> r=0.01 (trust position measurements)
            let kalman = Kalman3D::new(object.centroid, 2.5, 0.4);

            self.tracks.insert(
                id,
                TrackedState {
                    object: object.clone(),
                    kalman,
                    missing_frames: 0,
                },
            );

            output.push(object);
        }

        // Remove stale tracks
        self.tracks
            .retain(|_, state| state.missing_frames < self.max_missing_frames);

        output
    }

    /// Predict where a track should be based on Kalman filter
    fn predict_position(&self, state: &TrackedState, dt: f32) -> Vec3 {
        // We predict forward by dt * missing_frames?
        // No, dt passed to update() is the time since the last update() call.
        // But state.missing_frames tracks how many *frames* (logical updates) we missed.
        // Actually, if we pass real time dt, we shouldn't rely on missing_frames count for time calculation,
        // unless dt is "time since last update call".

        // If the track was updated last frame, time since last update = dt.
        // If it was updated 2 frames ago, time since last update = dt + (time of previous frame).

        // However, the caller calls update() every frame.
        // So dt is the time since the PREVIOUS update() call.
        // The track's position is valid at time T_prev.
        // We want to predict it at time T_now.
        // So we project forward by dt.
        state.kalman.predicted_position(dt)
    }

    /// Update a track with a new detection (by track ID)
    fn do_update_track(&mut self, track_id: ObjectId, detection: &TrackedObject, dt: f32) {
        if let Some(state) = self.tracks.get_mut(&track_id) {
            Self::update_track_state(state, detection, dt);
        }
    }

    /// Update track state with a new detection
    fn update_track_state(state: &mut TrackedState, detection: &TrackedObject, dt: f32) {
        // Kalman Predict & Update
        state.kalman.predict(dt);
        state.kalman.update(detection.centroid);

        // Reset missing frames
        state.missing_frames = 0;

        // Update object properties
        state.object = TrackedObject {
            id: state.object.id,
            centroid: state.kalman.position(), // Use filtered position
            bounding_box: detection.bounding_box,
            point_count: detection.point_count,
            total_intensity: detection.total_intensity,
            velocity: Some(state.kalman.velocity()), // Use filtered velocity
            confidence: detection.confidence,
        };
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
        let mut tracker = ObjectTracker::new(5.0, 30, 60.0);
        let dt = 1.0 / 60.0;

        // First frame
        let objects = tracker.update(vec![make_object(0, Vec3::new(0.0, 0.0, 0.0))], dt);
        assert_eq!(objects.len(), 1);
        let first_id = objects[0].id;

        // Second frame, slightly moved
        let objects = tracker.update(vec![make_object(0, Vec3::new(1.0, 0.0, 0.0))], dt);
        assert_eq!(objects.len(), 1);
        assert_eq!(objects[0].id, first_id); // Same track ID
    }

    #[test]
    fn test_velocity_convergence() {
        // 10 fps -> dt = 0.1s
        let mut tracker = ObjectTracker::new(5.0, 30, 10.0);
        let dt = 0.1;

        // Constant velocity 10 m/s.
        // Pos: 0, 1, 2, 3...
        let mut objects = Vec::new();
        for i in 0..10 {
            let pos = Vec3::new(i as f32, 0.0, 0.0);
            objects = tracker.update(vec![make_object(0, pos)], dt);
        }

        // Check velocity after 10 frames
        if let Some(vel) = objects[0].velocity {
            // Should be approx 10.0
            assert!(
                (vel.x - 10.0).abs() < 1.0,
                "Velocity should be approx 10.0. Got: {:?}",
                vel
            );
        } else {
            panic!("Velocity should be present");
        }
    }

    #[test]
    fn test_association_prediction() {
        // 10 fps
        let mut tracker = ObjectTracker::new(100.0, 30, 10.0);
        let dt = 0.1;

        // Object moves 0 -> 2 -> 4 -> 6 -> 8 -> 10. (Velocity 20 m/s).
        let mut track_id = 0;

        // Establish track for 5 frames (0 to 8)
        for i in 0..5 {
            let pos = Vec3::new(i as f32 * 2.0, 0.0, 0.0);
            let res = tracker.update(vec![make_object(0, pos)], dt);
            if i == 0 {
                track_id = res[0].id;
            }
        }

        // Frame 5: Real at 10. Distractor at 8.5.
        // Last known: 8.
        // Distractor (8.5) is closer to Last (8) than Real (10).
        // Predicted (10) is closer to Real (10).

        let detections = vec![
            make_object(0, Vec3::new(10.0, 0.0, 0.0)), // Real
            make_object(0, Vec3::new(8.5, 0.0, 0.0)),  // Distractor
        ];

        let results = tracker.update(detections, dt);

        let mut found_real = false;
        let mut found_distractor = false;

        for obj in results {
            if (obj.centroid.x - 8.5).abs() < 0.5 {
                found_distractor = true;
                assert_ne!(obj.id, track_id, "Distractor should NOT steal the track");
            }
            if (obj.centroid.x - 10.0).abs() < 0.5 {
                found_real = true;
                assert_eq!(obj.id, track_id, "Real object should keep the track");
            }
        }

        assert!(found_real, "Should have found real object");
        assert!(found_distractor, "Should have found distractor");
    }
}
