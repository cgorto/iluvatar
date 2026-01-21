use crate::grid::DEFAULT_MAX_VOXELS;
use glam::UVec3;
use iluvatar_core::{DecayConfig, DetectionConfig, GeoPosition, GridConfigMessage};
use serde::Deserialize;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("Failed to read config file: {0}")]
    Read(#[from] std::io::Error),
    #[error("Failed to parse config: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("Missing required field: {0}")]
    MissingField(&'static str),
}

#[derive(Debug, Clone, Deserialize)]
pub struct ServerConfig {
    pub server: NetworkSettings,
    pub grid: GridSettings,
    pub decay: DecaySettings,
    pub detection: DetectionSettings,
    pub tracking: TrackingSettings,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NetworkSettings {
    pub listen_address: String,
    #[serde(default = "default_websocket_port")]
    pub websocket_port: u16,
    #[serde(default = "default_broadcast_rate_hz")]
    pub broadcast_rate_hz: f32,
}

fn default_websocket_port() -> u16 {
    8080
}

fn default_broadcast_rate_hz() -> f32 {
    10.0
}

#[derive(Debug, Clone, Deserialize)]
pub struct GridSettings {
    pub voxel_size: f32,
    pub origin: GeoOriginConfig,
    pub dimensions: [u32; 3],
    /// Maximum number of voxels allowed in the grid (memory protection).
    /// Defaults to 1,000,000 voxels.
    #[serde(default = "default_max_voxels")]
    pub max_voxels: usize,
}

fn default_max_voxels() -> usize {
    DEFAULT_MAX_VOXELS
}

#[derive(Debug, Clone, Deserialize)]
pub struct GeoOriginConfig {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

impl From<GeoOriginConfig> for GeoPosition {
    fn from(config: GeoOriginConfig) -> Self {
        GeoPosition::new(config.latitude, config.longitude, config.altitude)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct DecaySettings {
    #[serde(default = "default_decay_rate")]
    pub rate: f32,
    #[serde(default = "default_decay_interval")]
    pub update_interval: f32,
}

fn default_decay_rate() -> f32 {
    0.5 // ~1.4 second half-life
}

fn default_decay_interval() -> f32 {
    0.1
}

#[derive(Debug, Clone, Deserialize)]
pub struct DetectionSettings {
    #[serde(default = "default_intensity_threshold")]
    pub intensity_threshold: f32,
    #[serde(default = "default_min_contributors")]
    pub min_contributors: u8,
    #[serde(default = "default_cluster_epsilon")]
    pub cluster_epsilon: f32,
    #[serde(default = "default_cluster_min_points")]
    pub cluster_min_points: usize,
}

fn default_intensity_threshold() -> f32 {
    10.0
}

fn default_min_contributors() -> u8 {
    2
}

fn default_cluster_epsilon() -> f32 {
    5.0
}

fn default_cluster_min_points() -> usize {
    3
}

#[derive(Debug, Clone, Deserialize)]
pub struct TrackingSettings {
    #[serde(default = "default_association_threshold")]
    pub association_threshold: f32,
    #[serde(default = "default_max_missing_frames")]
    pub max_missing_frames: u32,
}

fn default_association_threshold() -> f32 {
    10.0
}

fn default_max_missing_frames() -> u32 {
    30
}

impl ServerConfig {
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let content = std::fs::read_to_string(path)?;
        let config: ServerConfig = toml::from_str(&content)?;
        Ok(config)
    }

    pub fn grid_origin(&self) -> GeoPosition {
        self.grid.origin.clone().into()
    }

    pub fn grid_dimensions(&self) -> UVec3 {
        UVec3::from_array(self.grid.dimensions)
    }

    pub fn decay_interval(&self) -> Duration {
        Duration::from_secs_f32(self.decay.update_interval)
    }

    pub fn broadcast_interval(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.server.broadcast_rate_hz as f64)
    }

    pub fn to_decay_config(&self) -> DecayConfig {
        DecayConfig {
            rate: self.decay.rate,
            update_interval: self.decay.update_interval,
        }
    }

    pub fn to_detection_config(&self) -> DetectionConfig {
        DetectionConfig {
            intensity_threshold: self.detection.intensity_threshold,
            min_contributors: self.detection.min_contributors,
            cluster_epsilon: self.detection.cluster_epsilon,
            cluster_min_points: self.detection.cluster_min_points,
        }
    }

    pub fn to_grid_config_message(&self) -> GridConfigMessage {
        GridConfigMessage {
            origin_lat: self.grid.origin.latitude,
            origin_lon: self.grid.origin.longitude,
            origin_alt: self.grid.origin.altitude,
            dimensions: self.grid.dimensions,
            voxel_size: self.grid.voxel_size,
        }
    }
}
