use crate::grid::DEFAULT_MAX_VOXELS;
use glam::UVec3;
use iluvatar_core::{
    AttenuationConfig, CoordinateMode, DecayConfig, DetectionConfig, GeoPosition,
    GridConfigMessage, RaymarchConfig,
};
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
    /// Raymarching configuration for server-side raymarching.
    /// Used when cameras send MotionFrames instead of pre-computed contributions.
    #[serde(default)]
    pub raymarch: RaymarchSettings,
    /// Expected camera positions for the viewer overlay.
    /// These are displayed even before cameras connect.
    #[serde(default)]
    pub cameras: Vec<CameraEntry>,
}

/// A camera entry in the server config for viewer display.
///
/// ```toml
/// [[cameras]]
/// id = 0
/// name = "cam-0"
/// position = [0.0, 0.0, 0.6]
/// orientation = [45.0, -30.0, 0.0]
/// ```
#[derive(Debug, Clone, Deserialize)]
pub struct CameraEntry {
    pub id: u64,
    #[serde(default)]
    pub name: Option<String>,
    /// Position as [x, y, z] in the configured coordinate system.
    pub position: [f64; 3],
    /// Orientation as [yaw, pitch, roll] in degrees.
    /// Yaw: rotation around vertical axis (0 = forward/+Y, 90 = right/+X).
    /// Pitch: tilt up/down (negative = looking down).
    /// Roll: rotation around forward axis.
    #[serde(default)]
    pub orientation: Option<[f32; 3]>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NetworkSettings {
    pub listen_address: String,
    /// Optional TCP listen address for cameras that connect via plain TCP
    /// instead of QUIC (e.g. Odin camera on K230). Same protocol, different
    /// transport. Example: "0.0.0.0:5001"
    pub tcp_listen_address: Option<String>,
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
    pub origin: OriginConfig,
    pub dimensions: [u32; 3],
    /// Maximum number of voxels allowed in the grid (memory protection).
    /// Defaults to 200,000 voxels (~10 MB hash table, fits in L3 cache).
    #[serde(default = "default_max_voxels")]
    pub max_voxels: usize,
    /// Coordinate mode for position interpretation.
    /// "Gps" (default): positions are WGS84 latitude/longitude/altitude.
    /// "Local": positions are (x, y, z) in meters.
    #[serde(default)]
    pub coordinate_mode: CoordinateMode,
}

fn default_max_voxels() -> usize {
    DEFAULT_MAX_VOXELS
}

/// Origin configuration that accepts either GPS or local coordinates.
///
/// GPS mode: `origin = { latitude = 47.6, longitude = -122.3, altitude = 0.0 }`
/// Local mode: `origin = { x = 0.0, y = 0.0, z = 0.0 }`
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum OriginConfig {
    Gps(GeoOriginConfig),
    Local(LocalOriginConfig),
}

impl OriginConfig {
    pub fn to_geo_position(&self) -> GeoPosition {
        match self {
            OriginConfig::Gps(geo) => GeoPosition::new(geo.latitude, geo.longitude, geo.altitude),
            OriginConfig::Local(local) => GeoPosition::from_local_xyz(local.x, local.y, local.z),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct GeoOriginConfig {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LocalOriginConfig {
    pub x: f64,
    pub y: f64,
    pub z: f64,
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
    3.0 // ~230ms half-life
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

/// Raymarching settings for server-side raymarching of MotionFrames.
#[derive(Debug, Clone, Deserialize)]
pub struct RaymarchSettings {
    /// Maximum distance to trace rays (in meters).
    #[serde(default = "default_raymarch_max_distance")]
    pub max_distance: f32,
    /// Attenuation mode: "none", "linear", or "inverse_square".
    #[serde(default = "default_raymarch_attenuation")]
    pub attenuation: String,
    /// Reference distance for inverse-square attenuation.
    #[serde(default = "default_raymarch_reference_distance")]
    pub reference_distance: f32,
    /// Maximum unique voxel contributions per frame. Limits how many
    /// distinct voxels a single motion frame can write to the grid.
    /// Lower values reduce grid population growth and keep the hash
    /// table cache-friendly. 16384 provides adequate triangulation
    /// signal while keeping steady-state voxel count manageable.
    #[serde(default = "default_raymarch_contribution_limit")]
    pub contribution_limit: usize,
}

impl Default for RaymarchSettings {
    fn default() -> Self {
        Self {
            max_distance: default_raymarch_max_distance(),
            attenuation: default_raymarch_attenuation(),
            reference_distance: default_raymarch_reference_distance(),
            contribution_limit: default_raymarch_contribution_limit(),
        }
    }
}

fn default_raymarch_max_distance() -> f32 {
    500.0
}

fn default_raymarch_attenuation() -> String {
    "none".to_string()
}

fn default_raymarch_reference_distance() -> f32 {
    10.0
}

fn default_raymarch_contribution_limit() -> usize {
    16384
}

impl ServerConfig {
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let content = std::fs::read_to_string(path)?;
        let config: ServerConfig = toml::from_str(&content)?;
        Ok(config)
    }

    pub fn grid_origin(&self) -> GeoPosition {
        self.grid.origin.to_geo_position()
    }

    pub fn coordinate_mode(&self) -> CoordinateMode {
        self.grid.coordinate_mode
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
        let origin = self.grid_origin();
        GridConfigMessage {
            origin_lat: origin.latitude,
            origin_lon: origin.longitude,
            origin_alt: origin.altitude,
            dimensions: self.grid.dimensions,
            voxel_size: self.grid.voxel_size,
            coordinate_mode: self.grid.coordinate_mode,
        }
    }

    /// Convert server raymarch settings to the core RaymarchConfig.
    pub fn to_raymarch_config(&self) -> RaymarchConfig {
        let attenuation = match self.raymarch.attenuation.as_str() {
            "linear" => AttenuationConfig::Linear {
                max_distance: self.raymarch.max_distance,
            },
            "inverse_square" => AttenuationConfig::InverseSquare {
                reference_distance: self.raymarch.reference_distance,
            },
            _ => AttenuationConfig::None,
        };

        RaymarchConfig {
            max_distance: self.raymarch.max_distance,
            step_size: 0.0, // Unused by DDA algorithm.
            attenuation,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_example_parses() {
        let source = include_str!("../../../config/server.example.toml");
        let config: ServerConfig = toml::from_str(source).expect("server example must stay valid");

        assert_eq!(
            config.server.tcp_listen_address.as_deref(),
            Some("0.0.0.0:4434")
        );
        assert_eq!(config.coordinate_mode(), CoordinateMode::Gps);
        assert_eq!(config.grid.max_voxels, 200_000);
    }
}
