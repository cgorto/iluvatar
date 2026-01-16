use iluvatar_core::TrackedObject;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// A snapshot of the world state at a point in time
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorldSnapshot {
    pub timestamp: u64,
    pub objects: Vec<TrackedObject>,
    #[serde(skip, default = "Instant::now")]
    pub captured_at: Instant,
}

/// Rolling buffer persistence layer
pub struct PersistenceLayer {
    enabled: bool,
    retention: Duration,
    snapshots: VecDeque<WorldSnapshot>,
    path: Option<PathBuf>,
    last_disk_write: Instant,
    disk_write_interval: Duration,
}

impl PersistenceLayer {
    pub fn new(enabled: bool, retention: Duration, path: Option<PathBuf>) -> Self {
        Self {
            enabled,
            retention,
            snapshots: VecDeque::new(),
            path,
            last_disk_write: Instant::now(),
            disk_write_interval: Duration::from_secs(300), // 5 minutes
        }
    }

    /// Add a new snapshot
    pub fn add_snapshot(&mut self, timestamp: u64, objects: Vec<TrackedObject>) {
        if !self.enabled {
            return;
        }

        let snapshot = WorldSnapshot {
            timestamp,
            objects,
            captured_at: Instant::now(),
        };

        self.snapshots.push_back(snapshot);
        self.cleanup_old();

        // Periodically write to disk
        if self.path.is_some() && self.last_disk_write.elapsed() >= self.disk_write_interval {
            self.write_to_disk();
            self.last_disk_write = Instant::now();
        }
    }

    /// Remove snapshots older than retention period
    fn cleanup_old(&mut self) {
        let cutoff = Instant::now() - self.retention;
        while let Some(front) = self.snapshots.front() {
            if front.captured_at < cutoff {
                self.snapshots.pop_front();
            } else {
                break;
            }
        }
    }

    /// Get snapshots in a time range
    pub fn get_range(&self, start_us: u64, end_us: u64) -> Vec<&WorldSnapshot> {
        self.snapshots
            .iter()
            .filter(|s| s.timestamp >= start_us && s.timestamp <= end_us)
            .collect()
    }

    /// Get the latest snapshot
    pub fn latest(&self) -> Option<&WorldSnapshot> {
        self.snapshots.back()
    }

    /// Get snapshot count
    pub fn count(&self) -> usize {
        self.snapshots.len()
    }

    /// Write snapshots to disk (placeholder)
    fn write_to_disk(&self) {
        let Some(path) = &self.path else { return };

        if let Err(e) = std::fs::create_dir_all(path) {
            tracing::error!("Failed to create persistence directory: {}", e);
            return;
        }

        for snapshot in &self.snapshots {
            let filename = format!("snapshot_{}.bin", snapshot.timestamp);
            let file_path = path.join(filename);

            // Skip if already exists to avoid unnecessary writes
            if file_path.exists() {
                continue;
            }

            match postcard::to_allocvec(snapshot) {
                Ok(bytes) => {
                    if let Err(e) = std::fs::write(&file_path, bytes) {
                        tracing::error!("Failed to write snapshot {}: {}", snapshot.timestamp, e);
                    }
                }
                Err(e) => tracing::error!("Serialization failed: {}", e),
            }
        }
    }

    /// Load snapshots from disk (placeholder)
    pub fn load_from_disk(&mut self) {
        let Some(path) = &self.path else { return };
        if !path.exists() {
            return;
        }

        let Ok(entries) = std::fs::read_dir(path) else {
            tracing::error!("Failed to read persistence directory");
            return;
        };

        let mut loaded = Vec::new();
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().map_or(false, |e| e == "bin") {
                if let Ok(bytes) = std::fs::read(&path) {
                    if let Ok(snapshot) = postcard::from_bytes::<WorldSnapshot>(&bytes) {
                        loaded.push(snapshot);
                    }
                }
            }
        }

        // Sort by timestamp and populate buffer
        loaded.sort_by_key(|s| s.timestamp);
        self.snapshots = VecDeque::from(loaded);
        self.cleanup_old();

        if !self.snapshots.is_empty() {
            tracing::info!("Loaded {} snapshots from disk", self.snapshots.len());
        }
    }
}

impl Default for PersistenceLayer {
    fn default() -> Self {
        Self::new(false, Duration::from_secs(3600), None)
    }
}
