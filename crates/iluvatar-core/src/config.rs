use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridConfig {
    pub voxel_size: f32,
    pub dimensions: Option<[u32; 3]>,
    pub origin: Option<GeoOrigin>,
}

impl Default for GridConfig {
    fn default() -> Self {
        Self {
            voxel_size: 1.0,
            dimensions: None,
            origin: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeoOrigin {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecayConfig {
    pub rate: f32,
    pub update_interval: f32,
}

impl Default for DecayConfig {
    fn default() -> Self {
        Self {
            rate: 0.5,
            update_interval: 0.1,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionConfig {
    pub intensity_threshold: f32,
    pub min_contributors: u8,
    pub cluster_epsilon: f32,
    pub cluster_min_points: usize,
}

impl Default for DetectionConfig {
    fn default() -> Self {
        Self {
            intensity_threshold: 10.0,
            min_contributors: 2,
            cluster_epsilon: 5.0,
            cluster_min_points: 3,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RaymarchConfig {
    /// Maximum distance to trace rays (in meters)
    pub max_distance: f32,
    /// Step size for naive ray marching (deprecated: DDA algorithm doesn't use this)
    #[serde(default)]
    pub step_size: f32,
    /// Distance-based intensity attenuation
    pub attenuation: AttenuationConfig,
}

impl Default for RaymarchConfig {
    fn default() -> Self {
        Self {
            max_distance: 500.0,
            step_size: 0.5, // Kept for backwards compatibility, unused by DDA
            attenuation: AttenuationConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "mode")]
pub enum AttenuationConfig {
    None,
    Linear { max_distance: f32 },
    InverseSquare { reference_distance: f32 },
}

impl Default for AttenuationConfig {
    fn default() -> Self {
        Self::None
    }
}

impl AttenuationConfig {
    pub fn compute(&self, distance: f32) -> f32 {
        match self {
            AttenuationConfig::None => 1.0,
            AttenuationConfig::Linear { max_distance } => (1.0 - distance / max_distance).max(0.0),
            AttenuationConfig::InverseSquare { reference_distance } => {
                let ratio = reference_distance / distance.max(0.001);
                (ratio * ratio).min(1.0)
            }
        }
    }
}
