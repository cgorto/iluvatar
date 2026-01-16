use glam::Vec3;
use iluvatar_core::{BoundingBox, DetectedPoint, DetectionConfig, TrackedObject};
use std::collections::HashMap;

/// Spatial hash grid for O(1) neighbor queries
struct SpatialIndex {
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
    cell_size: f32,
}

impl SpatialIndex {
    fn new(cell_size: f32) -> Self {
        Self {
            cells: HashMap::new(),
            cell_size,
        }
    }

    fn build(&mut self, points: &[DetectedPoint]) {
        self.cells.clear();

        for (i, point) in points.iter().enumerate() {
            let cell = self.position_to_cell(point.position);
            self.cells.entry(cell).or_default().push(i);
        }
    }

    fn position_to_cell(&self, pos: Vec3) -> (i32, i32, i32) {
        (
            (pos.x / self.cell_size).floor() as i32,
            (pos.y / self.cell_size).floor() as i32,
            (pos.z / self.cell_size).floor() as i32,
        )
    }

    /// Find all point indices within radius of a query position
    fn query_radius(&self, points: &[DetectedPoint], center: Vec3, radius: f32) -> Vec<usize> {
        let radius_sq = radius * radius;
        let cell_radius = (radius / self.cell_size).ceil() as i32;
        let center_cell = self.position_to_cell(center);

        let mut result = Vec::new();

        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                for dz in -cell_radius..=cell_radius {
                    let cell = (
                        center_cell.0.wrapping_add(dx),
                        center_cell.1.wrapping_add(dy),
                        center_cell.2.wrapping_add(dz),
                    );

                    if let Some(indices) = self.cells.get(&cell) {
                        for &idx in indices {
                            if points[idx].position.distance_squared(center) <= radius_sq {
                                result.push(idx);
                            }
                        }
                    }
                }
            }
        }

        result
    }
}

/// DBSCAN-style clustering for detected points
pub struct ObjectDetector {
    config: DetectionConfig,
    spatial_index: SpatialIndex,
}

impl ObjectDetector {
    pub fn new(config: DetectionConfig) -> Self {
        let cell_size = config.cluster_epsilon;
        Self {
            config,
            spatial_index: SpatialIndex::new(cell_size),
        }
    }

    /// Cluster detected points into objects using spatial-indexed DBSCAN
    pub fn detect(&mut self, points: &[DetectedPoint]) -> Vec<TrackedObject> {
        if points.is_empty() {
            return Vec::new();
        }

        // Build spatial index for efficient neighbor queries
        self.spatial_index.build(points);

        let clusters = self.dbscan_with_index(points);
        let min_points = self.config.cluster_min_points;

        clusters
            .into_iter()
            .filter(|cluster| cluster.len() >= min_points)
            .map(|cluster| self.cluster_to_object(cluster))
            .collect()
    }

    /// DBSCAN using spatial index for O(1) neighbor lookups
    fn dbscan_with_index<'a>(&self, points: &'a [DetectedPoint]) -> Vec<Vec<&'a DetectedPoint>> {
        let mut visited = vec![false; points.len()];
        let mut clusters: Vec<Vec<&DetectedPoint>> = Vec::new();
        let epsilon = self.config.cluster_epsilon;

        for i in 0..points.len() {
            if visited[i] {
                continue;
            }

            let neighbors = self
                .spatial_index
                .query_radius(points, points[i].position, epsilon);
            if neighbors.len() < self.config.cluster_min_points {
                continue;
            }

            visited[i] = true;
            let mut cluster = vec![&points[i]];
            let mut seeds: Vec<usize> = neighbors;

            while let Some(q) = seeds.pop() {
                if visited[q] {
                    continue;
                }
                visited[q] = true;
                cluster.push(&points[q]);

                let q_neighbors =
                    self.spatial_index
                        .query_radius(points, points[q].position, epsilon);
                if q_neighbors.len() >= self.config.cluster_min_points {
                    seeds.extend(q_neighbors.iter().filter(|&&idx| !visited[idx]));
                }
            }

            clusters.push(cluster);
        }

        clusters
    }

    /// Convert a cluster of points to a tracked object.
    /// Uses id=0 as a sentinel - the tracker assigns real IDs.
    fn cluster_to_object(&self, cluster: Vec<&DetectedPoint>) -> TrackedObject {
        let mut total_intensity = 0.0;
        let mut total_confidence = 0.0;
        let mut centroid = Vec3::ZERO;
        let mut min = Vec3::splat(f32::MAX);
        let mut max = Vec3::splat(f32::MIN);

        for point in &cluster {
            centroid += point.position;
            total_intensity += point.intensity;
            total_confidence += point.confidence;
            min = min.min(point.position);
            max = max.max(point.position);
        }

        let count = cluster.len();
        centroid /= count as f32;

        TrackedObject {
            id: 0, // Anonymous - tracker assigns real ID
            centroid,
            bounding_box: BoundingBox::new(min, max),
            point_count: count,
            total_intensity,
            velocity: None,
            confidence: total_confidence / count as f32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clustering() {
        let mut detector = ObjectDetector::new(DetectionConfig {
            intensity_threshold: 1.0,
            min_contributors: 1,
            cluster_epsilon: 2.0,
            cluster_min_points: 2,
        });

        // Two clusters
        let points = vec![
            DetectedPoint {
                position: Vec3::new(0.0, 0.0, 0.0),
                intensity: 1.0,
                confidence: 1.0,
            },
            DetectedPoint {
                position: Vec3::new(1.0, 0.0, 0.0),
                intensity: 1.0,
                confidence: 1.0,
            },
            DetectedPoint {
                position: Vec3::new(10.0, 0.0, 0.0),
                intensity: 1.0,
                confidence: 1.0,
            },
            DetectedPoint {
                position: Vec3::new(11.0, 0.0, 0.0),
                intensity: 1.0,
                confidence: 1.0,
            },
        ];

        let objects = detector.detect(&points);
        assert_eq!(objects.len(), 2);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn test_dbscan_no_panic(
            // Generate random positions. Limit range to avoid potential float issues if that's not the goal,
            // but let's try fairly wide range first.
            positions in prop::collection::vec(
                (any::<f32>(), any::<f32>(), any::<f32>()),
                0..50 // Limit number of points for performance
            )
        ) {
            let points: Vec<DetectedPoint> = positions
                .into_iter()
                .map(|(x, y, z)| DetectedPoint {
                    position: Vec3::new(x, y, z),
                    intensity: 1.0,
                    confidence: 1.0,
                })
                .collect();

            let mut detector = ObjectDetector::new(DetectionConfig {
                intensity_threshold: 0.5,
                min_contributors: 1,
                cluster_epsilon: 1.0,
                cluster_min_points: 2,
            });

            // Should not panic
            let _ = detector.detect(&points);
        }
    }
}
