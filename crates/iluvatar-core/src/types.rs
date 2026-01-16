use glam::{Quat, UVec2, UVec3, Vec2, Vec3};
use serde::{Deserialize, Serialize};

pub type CameraId = u64;
pub type ObjectId = u64;
pub type Timestamp = u64;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CameraPose {
    pub position: GeoPosition,
    pub orientation: Quat,
    pub timestamp: Timestamp,
    pub uncertainty: PoseUncertainty,
    pub status: LocalizationStatus,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GeoPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PoseUncertainty {
    pub position_stddev: Vec3,
    pub orientation_stddev: Vec3,
}

impl Default for PoseUncertainty {
    fn default() -> Self {
        Self {
            position_stddev: Vec3::splat(1.0),
            orientation_stddev: Vec3::splat(0.01),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub enum LocalizationStatus {
    #[default]
    Nominal,
    DeadReckoning {
        duration_ms: u64,
    },
    Unavailable,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    pub focal_length: Vec2,
    pub principal_point: Vec2,
    pub resolution: UVec2,
    pub fov: Fov,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Fov {
    pub horizontal: f32,
    pub vertical: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub intensity: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct VoxelContribution {
    pub index: UVec3,
    pub intensity: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl BoundingBox {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    pub fn contains(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    pub fn center(&self) -> Vec3 {
        (self.min + self.max) / 2.0
    }

    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackedObject {
    pub id: ObjectId,
    pub centroid: Vec3,
    pub bounding_box: BoundingBox,
    pub point_count: usize,
    pub total_intensity: f32,
    pub velocity: Option<Vec3>,
    pub confidence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedPoint {
    pub position: Vec3,
    pub intensity: f32,
    pub confidence: f32,
}
