// Odin types mirroring the Rust iluvatar-core types.
//
// Field order MUST match the Rust struct declarations exactly because
// postcard serializes fields in declaration order. Any reordering
// breaks wire compatibility.
//
// Glam types are replaced with plain arrays:
//   Vec2  → [2]f32    (x, y)
//   Vec3  → [3]f32    (x, y, z)
//   UVec2 → [2]u32    (x, y)
//   Quat  → [4]f32    (x, y, z, w)

package odin_camera

PROTOCOL_VERSION :: 2

// Frame dimensions. Compile-time constants for static allocation.
FRAME_WIDTH:  u32 : 1280
FRAME_HEIGHT: u32 : 720
PIXEL_COUNT:  u32 : FRAME_WIDTH * FRAME_HEIGHT // 921,600

// Max-pool downsampling factor. Each pool_factor×pool_factor block
// becomes one Motion_Pixel with the block's max intensity.
// 2×2 preserves tighter angular resolution for voxel ray projection
// (0.097° per cell at 720p) while still compressing the data enough
// for 100 Mbps ethernet. Noise rejection happens in the voxel grid
// via multi-camera statistical accumulation, not here.
POOL_FACTOR:  u32 : 2

// Dimensions of the pooled grid.
POOLED_WIDTH:  u32 : (FRAME_WIDTH + POOL_FACTOR - 1) / POOL_FACTOR  // 640
POOLED_HEIGHT: u32 : (FRAME_HEIGHT + POOL_FACTOR - 1) / POOL_FACTOR // 360

// Upper bound on motion pixels per frame after pooling.
MAX_MOTION_PIXELS: u32 : POOLED_WIDTH * POOLED_HEIGHT // 57,600

// --- Camera Intrinsics (types.rs:130-143) ---

Camera_Intrinsics :: struct {
	focal_length:    [2]f32, // (fx, fy) in pixels.
	principal_point: [2]f32, // (cx, cy) optical center.
	resolution:      [2]u32, // (width, height).
	fov:             Fov,
	distortion:      Distortion_Model,
}

Fov :: struct {
	horizontal: f32, // Radians.
	vertical:   f32, // Radians.
}

Distortion_Model :: enum u8 {
	None            = 0,
	Open_CV5        = 1,
	Kannala_Brandt4 = 2,
}

// --- Camera Pose (types.rs:89-96) ---

Camera_Pose :: struct {
	position:    Geo_Position,
	orientation: [4]f32, // Quaternion (x, y, z, w).
	timestamp:   u64,    // Microseconds since epoch.
	uncertainty: Pose_Uncertainty,
	status:      Localization_Status,
}

Geo_Position :: struct {
	latitude:  f64,
	longitude: f64,
	altitude:  f64,
}

Pose_Uncertainty :: struct {
	position_stddev:    [3]f32, // Vec3.
	orientation_stddev: [3]f32, // Vec3.
}

POSE_UNCERTAINTY_DEFAULT :: Pose_Uncertainty {
	position_stddev    = {1.0, 1.0, 1.0},
	orientation_stddev = {0.01, 0.01, 0.01},
}

Localization_Status :: enum u8 {
	Nominal        = 0,
	Dead_Reckoning = 1, // Has duration_ms: u64 field.
	Unavailable    = 2,
}

// --- Protocol Messages (protocol.rs) ---

Camera_Capabilities :: struct {
	motion_frames: bool,
	rle_encoding:  bool,
	flags:         u32,
}

Camera_Registration :: struct {
	version:      u8,
	camera_id:    u64,
	intrinsics:   Camera_Intrinsics,
	initial_pose: Camera_Pose,
	capabilities: Camera_Capabilities,
}

Motion_Pixel :: struct {
	x:         u16,
	y:         u16,
	intensity: u8,
}

// --- Server Messages (protocol.rs:461-489) ---

Frame_Format :: enum u8 {
	Voxel_Contributions = 0,
	Motion_Pixels       = 1,
}

Server_Preferences :: struct {
	preferred_format:  Frame_Format,
	target_fps:        f32,    // Valid only when target_fps_present is true.
	target_fps_present: bool,
	max_motion_pixels:         u32,
	max_motion_pixels_present: bool,
}

Grid_Config :: struct {
	origin_lat: f64,
	origin_lon: f64,
	origin_alt: f64,
	dimensions: [3]u32, // [x, y, z] voxel counts.
	voxel_size: f32,
}

// Decoded server response after registration.
Server_Response :: struct {
	camera_id:   u64,
	preferences: Server_Preferences,
	grid_config: Grid_Config,
}
