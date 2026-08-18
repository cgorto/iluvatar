use glam::Vec3;
use iluvatar_core::{ObjectId, TrackedObject};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::hungarian;
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

    /// Update tracks with new detections.
    ///
    /// Uses the Hungarian algorithm to find the globally optimal assignment
    /// between detections and existing tracks, minimizing total distance.
    /// This prevents identity swaps that greedy matching causes when objects
    /// cluster or cross paths.
    pub fn update(&mut self, detections: Vec<TrackedObject>, dt: f32) -> Vec<TrackedObject> {
        assert!(dt.is_finite());

        // Mark all tracks as potentially missing.
        for state in self.tracks.values_mut() {
            state.missing_frames += 1;
        }

        // Collect track predictions for cost matrix construction. The Vec
        // gives us stable indices that map to cost matrix columns.
        let track_entries: Vec<(ObjectId, Vec3)> = self
            .tracks
            .iter()
            .map(|(id, state)| (*id, state.kalman.predicted_position(dt)))
            .collect();

        // Build cost matrix and find globally optimal assignment.
        let costs = build_cost_matrix(&detections, &track_entries);
        let assignment = hungarian::optimal_assignment(
            &costs,
            detections.len() as u32,
            track_entries.len() as u32,
            self.association_threshold,
        );

        let mut output = Vec::with_capacity(detections.len());

        // Update matched tracks with their assigned detections.
        for &(det_idx, track_idx) in &assignment.pairs {
            let track_id = track_entries[track_idx as usize].0;
            self.do_update_track(track_id, &detections[det_idx as usize], dt);
            if let Some(state) = self.tracks.get(&track_id) {
                output.push(state.object.clone());
            }
        }

        // Create new tracks for unmatched detections.
        for &det_idx in &assignment.unmatched_detections {
            let object = self.create_track(detections[det_idx as usize].clone());
            output.push(object);
        }

        // Remove tracks that have been missing too long.
        self.tracks
            .retain(|_, state| state.missing_frames < self.max_missing_frames);

        output
    }

    /// Update a track with a new detection (by track ID).
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

    /// Create a new track from an unmatched detection.
    ///
    /// Initializes a Kalman filter at the detection position with zero
    /// velocity. The high initial velocity uncertainty (p11=2500) means the
    /// filter learns the true velocity within 3-5 frames.
    fn create_track(&mut self, mut detection: TrackedObject) -> TrackedObject {
        let id = self.id_generator.next();
        detection.id = id;
        if detection.velocity.is_none() {
            detection.velocity = Some(Vec3::ZERO);
        }

        let kalman = Kalman3D::new(detection.centroid, 2.5, 0.4);
        self.tracks.insert(
            id,
            TrackedState {
                object: detection.clone(),
                kalman,
                missing_frames: 0,
            },
        );

        detection
    }

    /// Get current track count.
    pub fn track_count(&self) -> usize {
        self.tracks.len()
    }
}

/// Build the cost matrix for Hungarian assignment.
///
/// Entry `[i * track_count + j]` is the Euclidean distance from detection `i`
/// to the predicted position of track `j`. The Hungarian algorithm will find
/// the assignment that minimizes total distance across all pairs.
fn build_cost_matrix(detections: &[TrackedObject], track_entries: &[(ObjectId, Vec3)]) -> Vec<f32> {
    let capacity = detections.len() * track_entries.len();
    let mut costs = Vec::with_capacity(capacity);
    for detection in detections {
        for &(_, predicted_pos) in track_entries {
            costs.push(detection.centroid.distance(predicted_pos));
        }
    }
    assert_eq!(costs.len(), capacity);
    costs
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

    #[test]
    fn test_nan_position_handling() {
        let mut tracker = ObjectTracker::new(5.0, 30, 60.0);
        let dt = 1.0 / 60.0;

        // First establish a track.
        let objects = tracker.update(vec![make_object(0, Vec3::new(0.0, 0.0, 0.0))], dt);
        assert_eq!(objects.len(), 1);

        // NaN detection should not panic. The Hungarian algorithm treats NaN
        // distances as forbidden (is_finite check), so the NaN detection
        // becomes a new track rather than stealing an existing one.
        let nan_obj = TrackedObject {
            id: 0,
            centroid: Vec3::new(f32::NAN, 0.0, 0.0),
            bounding_box: BoundingBox::new(Vec3::ZERO, Vec3::ONE),
            point_count: 1,
            total_intensity: 1.0,
            velocity: None,
            confidence: 1.0,
        };

        let result = tracker.update(vec![nan_obj], dt);
        assert!(!result.is_empty());
    }

    #[test]
    fn test_hungarian_prevents_greedy_misassignment() {
        // Demonstrates why globally optimal assignment matters.
        //
        // Three tracks: A at x=0, B at x=5, C at x=50.
        // Three detections: D0 at x=4.8, D1 at x=5.2, D2 at x=50.5.
        //
        // Track B (x=5) is almost equidistant to D0 (0.2) and D1 (0.2).
        // Greedy picks B->D0 first (distance 0.2), stealing D0 from Track A.
        // This forces A to take D1 (distance 5.2) — the wrong detection.
        //
        // Hungarian sees the global picture: A->D0 (4.8), B->D1 (0.2),
        // C->D2 (0.5). Total cost 5.5 vs greedy's 5.9. More importantly,
        // each track keeps its correct identity.
        let mut tracker = ObjectTracker::new(100.0, 30, 60.0);
        let dt = 1.0 / 60.0;

        // Frame 1: establish three tracks.
        let frame1 = vec![
            make_object(0, Vec3::new(0.0, 0.0, 0.0)),
            make_object(0, Vec3::new(5.0, 0.0, 0.0)),
            make_object(0, Vec3::new(50.0, 0.0, 0.0)),
        ];
        let r1 = tracker.update(frame1, dt);
        assert_eq!(r1.len(), 3);

        // Identify the tracks by their initial positions.
        let id_a = r1.iter().find(|o| o.centroid.x < 2.5).unwrap().id;
        let id_b = r1
            .iter()
            .find(|o| o.centroid.x > 2.5 && o.centroid.x < 30.0)
            .unwrap()
            .id;
        let id_c = r1.iter().find(|o| o.centroid.x > 30.0).unwrap().id;

        // Frame 2: detections cluster near B. The greedy failure case.
        let frame2 = vec![
            make_object(0, Vec3::new(4.8, 0.0, 0.0)), // Near B, actually A's
            make_object(0, Vec3::new(5.2, 0.0, 0.0)), // Near B, actually B's
            make_object(0, Vec3::new(50.5, 0.0, 0.0)), // C's
        ];
        let r2 = tracker.update(frame2, dt);
        assert_eq!(r2.len(), 3);

        // Verify all three tracks preserved identity.
        let a2 = r2
            .iter()
            .find(|o| o.id == id_a)
            .expect("Track A should persist.");
        let b2 = r2
            .iter()
            .find(|o| o.id == id_b)
            .expect("Track B should persist.");
        let c2 = r2
            .iter()
            .find(|o| o.id == id_c)
            .expect("Track C should persist.");

        // Track A (predicted at x~0) should get D0 (x=4.8), not D1 (x=5.2).
        // After Kalman update, A's centroid is pulled toward 4.8.
        assert!(
            a2.centroid.x < 5.0,
            "Track A should be assigned the left detection (x=4.8), got x={:.2}",
            a2.centroid.x,
        );

        // Track B (predicted at x~5) should get D1 (x=5.2).
        assert!(
            b2.centroid.x >= 5.0,
            "Track B should be assigned the right detection (x=5.2), got x={:.2}",
            b2.centroid.x,
        );

        // Track C should match its detection without ambiguity.
        assert!(
            c2.centroid.x > 40.0,
            "Track C should remain near x=50, got x={:.2}",
            c2.centroid.x,
        );
    }
}
