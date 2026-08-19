//! Simulator configuration loaded from TOML
//!
//! Defines the multi-camera setup, grid parameters, and server address
//! for a simulator run. Each camera entry specifies position, orientation,
//! FOV, resolution, and the TCP port used to stream frames to an external
//! camera process.

use bevy::prelude::*;
use serde::Deserialize;
use std::path::Path;

/// Top-level simulator configuration, inserted as a Bevy [`Resource`].
#[derive(Resource, Debug, Clone, Deserialize)]
pub struct SimulatorTomlConfig {
    pub grid: GridSection,
    pub server: ServerSection,
    pub cameras: Vec<CameraEntry>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct GridSection {
    pub origin: [f32; 3],
    pub dimensions: [u32; 3],
    pub voxel_size: f32,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ServerSection {
    pub address: String,
}

/// One camera in the simulator.
#[derive(Debug, Clone, Deserialize)]
pub struct CameraEntry {
    /// Unique camera ID (0-63 for bitmask tracking)
    pub id: u32,
    /// World-space position `[x, y, z]`
    pub position: [f32; 3],
    /// Point the camera looks at `[x, y, z]`
    pub look_at: [f32; 3],
    /// Horizontal field-of-view in **degrees**
    pub fov_horizontal: f32,
    /// Vertical field-of-view in **degrees**
    pub fov_vertical: f32,
    /// Render resolution `[width, height]`
    pub resolution: [u32; 2],
    /// TCP port for streaming grayscale frames to the camera process
    pub stream_port: u16,
}

impl CameraEntry {
    pub fn position_vec3(&self) -> Vec3 {
        Vec3::new(self.position[0], self.position[1], self.position[2])
    }

    pub fn look_at_vec3(&self) -> Vec3 {
        Vec3::new(self.look_at[0], self.look_at[1], self.look_at[2])
    }

    pub fn resolution_uvec2(&self) -> UVec2 {
        UVec2::new(self.resolution[0], self.resolution[1])
    }

    /// Horizontal FOV in radians
    pub fn fov_h_rad(&self) -> f32 {
        self.fov_horizontal.to_radians()
    }

    /// Vertical FOV in radians
    pub fn fov_v_rad(&self) -> f32 {
        self.fov_vertical.to_radians()
    }
}

impl SimulatorTomlConfig {
    pub fn load(path: &Path) -> Result<Self, SimConfigError> {
        let content = std::fs::read_to_string(path)?;
        let config: SimulatorTomlConfig = toml::from_str(&content)?;
        Ok(config)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum SimConfigError {
    #[error("Failed to read config file: {0}")]
    Io(#[from] std::io::Error),
    #[error("Failed to parse config: {0}")]
    Parse(#[from] toml::de::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_example_parses() {
        let source = include_str!("../../../config/simulator.example.toml");
        let config: SimulatorTomlConfig =
            toml::from_str(source).expect("simulator example must stay valid");

        assert_eq!(config.cameras.len(), 4);
        assert_eq!(config.cameras[0].resolution, [1280, 720]);

        let runtime = crate::voxels::SimulatorConfig::from(&config);
        assert_eq!(runtime.grid_origin, Vec3::new(-500.0, 0.0, -500.0));
        assert_eq!(runtime.grid_dimensions, UVec3::new(1000, 400, 1000));
        assert_eq!(runtime.voxel_size, 1.0);
    }
}
