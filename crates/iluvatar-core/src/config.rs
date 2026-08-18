use crate::GeoPosition;
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

/// Origin configuration that accepts either GPS or local coordinates.
///
/// GPS format: `{ latitude = 47.6, longitude = -122.3, altitude = 0.0 }`
/// Local format: `{ x = 0.0, y = 0.0, z = 0.0 }`
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum GeoOrigin {
    Gps {
        latitude: f64,
        longitude: f64,
        altitude: f64,
    },
    Local {
        x: f64,
        y: f64,
        z: f64,
    },
}

impl GeoOrigin {
    pub fn to_geo_position(&self) -> GeoPosition {
        match self {
            GeoOrigin::Gps {
                latitude,
                longitude,
                altitude,
            } => GeoPosition::new(*latitude, *longitude, *altitude),
            GeoOrigin::Local { x, y, z } => GeoPosition::from_local_xyz(*x, *y, *z),
        }
    }
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

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(tag = "mode")]
pub enum AttenuationConfig {
    #[default]
    None,
    Linear {
        max_distance: f32,
    },
    InverseSquare {
        reference_distance: f32,
    },
}

impl AttenuationConfig {
    pub fn compute(&self, distance: f32) -> f32 {
        match self {
            AttenuationConfig::None => 1.0,
            AttenuationConfig::Linear { max_distance } => {
                if *max_distance <= 0.0 {
                    return 1.0; // No attenuation if max_distance is invalid
                }
                (1.0 - distance / max_distance).max(0.0)
            }
            AttenuationConfig::InverseSquare { reference_distance } => {
                let ratio = reference_distance / distance.max(0.001);
                (ratio * ratio).min(1.0)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_attenuation_linear_normal() {
        let att = AttenuationConfig::Linear {
            max_distance: 100.0,
        };
        assert!((att.compute(0.0) - 1.0).abs() < 0.001);
        assert!((att.compute(50.0) - 0.5).abs() < 0.001);
        assert!((att.compute(100.0) - 0.0).abs() < 0.001);
        assert!((att.compute(150.0) - 0.0).abs() < 0.001); // Clamped to 0
    }

    #[test]
    fn test_attenuation_linear_zero_max_distance() {
        let att = AttenuationConfig::Linear { max_distance: 0.0 };
        // Should return 1.0 (no attenuation) instead of dividing by zero
        assert!((att.compute(50.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_attenuation_linear_negative_max_distance() {
        let att = AttenuationConfig::Linear {
            max_distance: -10.0,
        };
        // Should return 1.0 (no attenuation) for invalid config
        assert!((att.compute(50.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_attenuation_none() {
        let att = AttenuationConfig::None;
        assert!((att.compute(0.0) - 1.0).abs() < 0.001);
        assert!((att.compute(1000.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_attenuation_inverse_square() {
        let att = AttenuationConfig::InverseSquare {
            reference_distance: 10.0,
        };
        assert!((att.compute(10.0) - 1.0).abs() < 0.001);
        assert!((att.compute(20.0) - 0.25).abs() < 0.001);
    }
}
