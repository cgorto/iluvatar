//! Optimal assignment via the Hungarian algorithm.
//!
//! Given N detections and M tracks with a pairwise distance matrix, finds the
//! assignment that minimizes total distance. This replaces greedy
//! sorted-distance matching, which can misassign when objects cluster.
//!
//! The failure mode of greedy: when track B is slightly closer to detection D0
//! than track A is, greedy assigns B->D0, forcing A to a worse match. The
//! Hungarian algorithm considers the global picture and finds the assignment
//! with minimum total cost, preventing cascading misassignments and identity
//! swaps when objects cross paths.
//!
//! Time complexity: O(n^3) where n = max(detections, tracks).
//! For 20 objects this is ~8000 operations, well under a microsecond.

/// Sentinel cost for forbidden assignments (distance exceeds threshold) and
/// padding entries when the matrix is extended to square form. Large enough
/// that the solver never prefers a forbidden pair, small enough that f32
/// arithmetic stays well-behaved across n additions.
const FORBIDDEN: f32 = 1e9;

/// Result of the optimal assignment algorithm.
pub struct Assignment {
    /// Matched pairs: (detection_index, track_index).
    pub pairs: Vec<(u32, u32)>,
    /// Detection indices that were not assigned to any track.
    pub unmatched_detections: Vec<u32>,
    /// Track indices that were not assigned to any detection.
    pub unmatched_tracks: Vec<u32>,
}

/// Find the minimum-cost assignment between detections and tracks.
///
/// The cost matrix is row-major: `costs[i * track_count + j]` is the distance
/// from detection `i` to the predicted position of track `j`.
///
/// Pairs with distance above `threshold` are forbidden: they will never be
/// matched regardless of whether it would reduce total cost.
///
/// Returns matched pairs plus unmatched detections (which become new tracks)
/// and unmatched tracks (which increment their missing-frame counter).
pub fn optimal_assignment(
    costs: &[f32],
    detection_count: u32,
    track_count: u32,
    threshold: f32,
) -> Assignment {
    let nd = detection_count as usize;
    let nt = track_count as usize;
    assert_eq!(costs.len(), nd * nt);
    assert!(threshold > 0.0);
    assert!(threshold.is_finite());

    // Nothing to assign.
    if nd == 0 || nt == 0 {
        return Assignment {
            pairs: Vec::new(),
            unmatched_detections: (0..detection_count).collect(),
            unmatched_tracks: (0..track_count).collect(),
        };
    }

    // Pad to square. Padding entries are FORBIDDEN so they are only chosen
    // when no real alternative exists for that row or column.
    let size = nd.max(nt);
    let mut square = vec![FORBIDDEN; size * size];
    for i in 0..nd {
        for j in 0..nt {
            let cost = costs[i * nt + j];
            if cost <= threshold && cost.is_finite() {
                square[i * size + j] = cost;
            }
        }
    }

    let col_for_row = Solver::new(size).solve(&square);

    // Interpret results: keep only real, within-threshold assignments.
    interpret_result(&col_for_row, costs, nd, nt, threshold)
}

/// Extract matched pairs and unmatched indices from the raw assignment.
fn interpret_result(
    col_for_row: &[u32],
    costs: &[f32],
    nd: usize,
    nt: usize,
    threshold: f32,
) -> Assignment {
    assert!(nd > 0);
    assert!(nt > 0);

    let mut pairs = Vec::with_capacity(nd.min(nt));
    let mut matched_detections = vec![false; nd];
    let mut matched_tracks = vec![false; nt];

    for (row, &col) in col_for_row.iter().enumerate() {
        // Skip padding rows and padding columns.
        if row >= nd {
            continue;
        }
        let col_idx = col as usize;
        if col_idx >= nt {
            continue;
        }

        let original_cost = costs[row * nt + col_idx];
        if original_cost <= threshold && original_cost.is_finite() {
            pairs.push((row as u32, col));
            matched_detections[row] = true;
            matched_tracks[col_idx] = true;
        }
    }

    let unmatched_detections = (0..nd)
        .filter(|&i| !matched_detections[i])
        .map(|i| i as u32)
        .collect();
    let unmatched_tracks = (0..nt)
        .filter(|&j| !matched_tracks[j])
        .map(|j| j as u32)
        .collect();

    Assignment {
        pairs,
        unmatched_detections,
        unmatched_tracks,
    }
}

// ---------------------------------------------------------------------------
// Solver: Hungarian algorithm via shortest augmenting paths.
//
// Maintains dual variables (potentials) u[i] for rows and v[j] for columns
// such that cost[i][j] - u[i] - v[j] >= 0 always holds. Each row extends
// the matching by finding the shortest augmenting path in the reduced cost
// graph, then flipping it.
//
// The 1-indexed convention (row 0 and column 0 are virtual) comes from the
// classical formulation and avoids special-casing the path start.
// ---------------------------------------------------------------------------

/// Working state for the Hungarian algorithm on an n x n cost matrix.
struct Solver {
    n: usize,
    /// Row potentials (dual variables). Index 0 is unused.
    u: Vec<f32>,
    /// Column potentials (dual variables). Index 0 is unused.
    v: Vec<f32>,
    /// Column-to-row matching: match_col[j] = row matched to column j.
    /// Zero means unmatched.
    match_col: Vec<u32>,
    /// Alternating path memory: path[j] = previous column on the path to j.
    path: Vec<u32>,
    /// Shortest reduced cost from the current source row to each column.
    shortest: Vec<f32>,
    /// Whether a column has been permanently labeled in the current search.
    visited: Vec<bool>,
}

impl Solver {
    fn new(n: usize) -> Self {
        assert!(n > 0);
        Self {
            n,
            u: vec![0.0f32; n + 1],
            v: vec![0.0f32; n + 1],
            match_col: vec![0u32; n + 1],
            path: vec![0u32; n + 1],
            shortest: vec![0.0f32; n + 1],
            visited: vec![false; n + 1],
        }
    }

    /// Solve the assignment and return row-indexed result.
    /// `result[i]` is the column assigned to row `i`.
    fn solve(mut self, cost: &[f32]) -> Vec<u32> {
        assert_eq!(cost.len(), self.n * self.n);

        for row in 1..=self.n {
            let end_col = self.find_augmenting_path(cost, row);
            self.flip_path(end_col);
        }

        self.to_result()
    }

    /// Find the shortest augmenting path from `source_row` to an unmatched
    /// column, updating dual variables along the way (modified Dijkstra).
    ///
    /// Returns the unmatched column at the end of the path.
    fn find_augmenting_path(&mut self, cost: &[f32], source_row: usize) -> usize {
        let n = self.n;

        // Link the source row to the virtual column 0.
        self.match_col[0] = source_row as u32;
        for j in 0..=n {
            self.shortest[j] = f32::INFINITY;
            self.visited[j] = false;
        }

        let mut current_col: usize = 0;

        loop {
            self.visited[current_col] = true;
            let current_row = self.match_col[current_col] as usize;
            let mut delta = f32::INFINITY;
            let mut next_col: usize = 0;

            // Relax edges from current_row to all unvisited columns.
            for j in 1..=n {
                if self.visited[j] {
                    continue;
                }
                let reduced =
                    cost[(current_row - 1) * n + (j - 1)] - self.u[current_row] - self.v[j];
                if reduced < self.shortest[j] {
                    self.shortest[j] = reduced;
                    self.path[j] = current_col as u32;
                }
                if self.shortest[j] < delta {
                    delta = self.shortest[j];
                    next_col = j;
                }
            }

            // Update dual variables along visited and unvisited columns.
            for j in 0..=n {
                if self.visited[j] {
                    self.u[self.match_col[j] as usize] += delta;
                    self.v[j] -= delta;
                } else {
                    self.shortest[j] -= delta;
                }
            }

            current_col = next_col;

            // Unmatched column found: augmenting path is complete.
            if self.match_col[current_col] == 0 {
                return current_col;
            }
        }
    }

    /// Trace back along the alternating path and flip the matching.
    fn flip_path(&mut self, end_col: usize) {
        assert!(end_col > 0);
        assert!(end_col <= self.n);

        let mut col = end_col;
        loop {
            let prev = self.path[col] as usize;
            self.match_col[col] = self.match_col[prev];
            col = prev;
            if col == 0 {
                break;
            }
        }
    }

    /// Convert the column-indexed matching to a row-indexed result vector.
    fn to_result(&self) -> Vec<u32> {
        let mut result = vec![0u32; self.n];
        for j in 1..=self.n {
            if self.match_col[j] > 0 {
                result[(self.match_col[j] - 1) as usize] = (j - 1) as u32;
            }
        }

        // Postcondition: result is a valid permutation.
        debug_assert!({
            let mut seen = vec![false; self.n];
            for &col in &result {
                let c = col as usize;
                assert!(c < self.n, "Column index out of range.");
                assert!(!seen[c], "Duplicate column in assignment.");
                seen[c] = true;
            }
            true
        });

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: run assignment and verify structural invariants.
    fn check_assignment(costs: &[f32], nd: u32, nt: u32, threshold: f32) -> Assignment {
        let result = optimal_assignment(costs, nd, nt, threshold);

        // Every matched detection appears exactly once.
        let mut det_seen = vec![false; nd as usize];
        let mut trk_seen = vec![false; nt as usize];
        for &(d, t) in &result.pairs {
            assert!(!det_seen[d as usize], "Detection matched twice.");
            assert!(!trk_seen[t as usize], "Track matched twice.");
            det_seen[d as usize] = true;
            trk_seen[t as usize] = true;
        }

        // Unmatched indices are exactly the ones not in pairs.
        for &d in &result.unmatched_detections {
            assert!(!det_seen[d as usize]);
        }
        for &t in &result.unmatched_tracks {
            assert!(!trk_seen[t as usize]);
        }

        // Total counts add up.
        assert_eq!(
            result.pairs.len() + result.unmatched_detections.len(),
            nd as usize,
        );
        assert_eq!(
            result.pairs.len() + result.unmatched_tracks.len(),
            nt as usize,
        );

        result
    }

    /// Compute total cost of an assignment for verification.
    fn total_cost(assignment: &Assignment, costs: &[f32], nt: u32) -> f32 {
        assignment
            .pairs
            .iter()
            .map(|&(d, t)| costs[d as usize * nt as usize + t as usize])
            .sum()
    }

    #[test]
    fn test_empty_inputs() {
        // No detections.
        let r = check_assignment(&[], 0, 3, 10.0);
        assert!(r.pairs.is_empty());
        assert_eq!(r.unmatched_tracks.len(), 3);

        // No tracks.
        let r = check_assignment(&[], 3, 0, 10.0);
        assert!(r.pairs.is_empty());
        assert_eq!(r.unmatched_detections.len(), 3);
    }

    #[test]
    fn test_single_pair_within_threshold() {
        let costs = vec![5.0];
        let r = check_assignment(&costs, 1, 1, 10.0);
        assert_eq!(r.pairs.len(), 1);
        assert_eq!(r.pairs[0], (0, 0));
    }

    #[test]
    fn test_single_pair_exceeds_threshold() {
        let costs = vec![15.0];
        let r = check_assignment(&costs, 1, 1, 10.0);
        assert!(r.pairs.is_empty());
        assert_eq!(r.unmatched_detections.len(), 1);
        assert_eq!(r.unmatched_tracks.len(), 1);
    }

    #[test]
    fn test_two_by_two_swap() {
        // Greedy picks row0->col0 (1.0) then row1->col1 (4.0) = 5.0.
        // Optimal: row0->col1 (2.0) + row1->col0 (1.5) = 3.5.
        #[rustfmt::skip]
        let costs = vec![
            1.0, 2.0,
            1.5, 4.0,
        ];
        let r = check_assignment(&costs, 2, 2, 10.0);
        assert_eq!(r.pairs.len(), 2);
        assert!(total_cost(&r, &costs, 2) <= 3.5 + 1e-6);
    }

    #[test]
    fn test_greedy_fails_three_objects() {
        // The adversarial case: track B at x=5 is equidistant to two nearby
        // detections (D0=4.8, D1=5.2). Greedy assigns B->D0, stealing it
        // from track A at x=0. Hungarian gives A->D0, B->D1 (optimal).
        //
        //   Track A@0   Track B@5   Track C@50
        //   D0@4.8: 4.8     0.2       45.2
        //   D1@5.2: 5.2     0.2       44.8
        //   D2@50.5:50.5    45.5       0.5
        #[rustfmt::skip]
        let costs = vec![
             4.8,  0.2, 45.2,
             5.2,  0.2, 44.8,
            50.5, 45.5,  0.5,
        ];

        let r = check_assignment(&costs, 3, 3, 100.0);
        assert_eq!(r.pairs.len(), 3);

        // Optimal total: 4.8 + 0.2 + 0.5 = 5.5.
        // Greedy total:  0.2 + 5.2 + 0.5 = 5.9.
        let cost = total_cost(&r, &costs, 3);
        assert!(
            (cost - 5.5).abs() < 1e-5,
            "Expected optimal cost 5.5, got {cost}",
        );

        // Verify specific assignments: det0->track0, det1->track1, det2->track2.
        let mut assignment_map: std::collections::HashMap<u32, u32> =
            r.pairs.iter().copied().collect();
        assert_eq!(assignment_map.remove(&0), Some(0)); // D0 -> Track A
        assert_eq!(assignment_map.remove(&1), Some(1)); // D1 -> Track B
        assert_eq!(assignment_map.remove(&2), Some(2)); // D2 -> Track C
    }

    #[test]
    fn test_rectangular_more_detections() {
        // 3 detections, 2 tracks. One detection must go unmatched.
        #[rustfmt::skip]
        let costs = vec![
            1.0, 8.0,
            9.0, 2.0,
            3.0, 3.0,
        ];
        let r = check_assignment(&costs, 3, 2, 10.0);
        assert_eq!(r.pairs.len(), 2);
        assert_eq!(r.unmatched_detections.len(), 1);

        // Optimal: det0->trk0 (1.0), det1->trk1 (2.0). Total: 3.0.
        let cost = total_cost(&r, &costs, 2);
        assert!(cost <= 3.0 + 1e-6);
    }

    #[test]
    fn test_rectangular_more_tracks() {
        // 2 detections, 3 tracks. One track must go unmatched.
        #[rustfmt::skip]
        let costs = vec![
            1.0, 8.0, 3.0,
            9.0, 2.0, 3.0,
        ];
        let r = check_assignment(&costs, 2, 3, 10.0);
        assert_eq!(r.pairs.len(), 2);
        assert_eq!(r.unmatched_tracks.len(), 1);

        // Optimal: det0->trk0 (1.0), det1->trk1 (2.0). Total: 3.0.
        let cost = total_cost(&r, &costs, 3);
        assert!(cost <= 3.0 + 1e-6);
    }

    #[test]
    fn test_all_forbidden() {
        // All distances exceed threshold. Nothing should match.
        #[rustfmt::skip]
        let costs = vec![
            20.0, 30.0,
            25.0, 15.0,
        ];
        let r = check_assignment(&costs, 2, 2, 10.0);
        assert!(r.pairs.is_empty());
        assert_eq!(r.unmatched_detections.len(), 2);
        assert_eq!(r.unmatched_tracks.len(), 2);
    }

    #[test]
    fn test_nan_costs_treated_as_forbidden() {
        let costs = vec![f32::NAN, 5.0, 3.0, f32::NAN];
        let r = check_assignment(&costs, 2, 2, 10.0);
        // det0 can only match trk1 (5.0), det1 can only match trk0 (3.0).
        assert_eq!(r.pairs.len(), 2);
        assert!((total_cost(&r, &costs, 2) - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_partial_threshold_rejection() {
        // Only some pairs are within threshold.
        #[rustfmt::skip]
        let costs = vec![
            2.0, 15.0,
            15.0, 3.0,
        ];
        let r = check_assignment(&costs, 2, 2, 10.0);
        assert_eq!(r.pairs.len(), 2);
        // det0->trk0 (2.0), det1->trk1 (3.0).
        assert!((total_cost(&r, &costs, 2) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_large_identity_matrix() {
        // 10x10 identity-like: cost 0 on diagonal, 100 off-diagonal.
        // Optimal is all diagonal assignments with total cost 0.
        let n = 10;
        let mut costs = vec![100.0f32; n * n];
        for i in 0..n {
            costs[i * n + i] = 0.0;
        }
        let r = check_assignment(&costs, n as u32, n as u32, 200.0);
        assert_eq!(r.pairs.len(), n);
        assert!(total_cost(&r, &costs, n as u32) < 1e-6);
    }
}
