// Ray generation compute shader
// Reads difference mask, outputs rays for motion pixels
//
// For each pixel with motion detected, generates a ray in world space
// and appends it to a shared ray buffer using atomic operations.

struct RayBufferHeader {
    ray_count: atomic<u32>,
    max_rays: u32,
    frame: u32,
    _padding0: u32,
    // Additional padding for 32-byte alignment
    _padding1: u32,
    _padding2: u32,
    _padding3: u32,
    _padding4: u32,
}

struct GpuRay {
    camera_id: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    origin: vec3<f32>,
    _pad3: f32,
    direction: vec3<f32>,
    _pad4: f32,
}

struct CameraData {
    camera_id: u32,
    width: u32,
    height: u32,
    subsample: u32,
    position: vec3<f32>,
    _pad0: f32,
    // Rotation matrix rows (to transform from camera local to world space)
    rotation_row0: vec3<f32>,
    _pad1: f32,
    rotation_row1: vec3<f32>,
    _pad2: f32,
    rotation_row2: vec3<f32>,
    _pad3: f32,
    // Camera intrinsics
    fov_half_tan: vec2<f32>,    // tan(fov_h/2), tan(fov_v/2)
    principal_point: vec2<f32>, // cx, cy (normalized 0-1)
}

@group(0) @binding(0) var difference_mask: texture_2d<u32>;
@group(0) @binding(1) var<uniform> camera: CameraData;
@group(0) @binding(2) var<storage, read_write> ray_header: RayBufferHeader;
@group(0) @binding(3) var<storage, read_write> rays: array<GpuRay>;

@compute @workgroup_size(16, 16)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    // Apply subsampling - each thread handles one subsampled position
    let x = global_id.x * camera.subsample;
    let y = global_id.y * camera.subsample;

    // Bounds check
    if (x >= camera.width || y >= camera.height) {
        return;
    }

    // Check if this pixel has motion
    let coord = vec2<i32>(i32(x), i32(y));
    let motion = textureLoad(difference_mask, coord, 0).r;

    if (motion == 0u) {
        return;
    }

    // Generate ray for this motion pixel

    // Convert pixel to normalized coordinates [-1, 1]
    // principal_point is normalized (0-1), so scale it
    let cx = camera.principal_point.x * f32(camera.width);
    let cy = camera.principal_point.y * f32(camera.height);
    let nx = (f32(x) - cx) / (f32(camera.width) / 2.0);
    let ny = (f32(y) - cy) / (f32(camera.height) / 2.0);

    // Convert to direction using FOV half-tangent
    // This gives us the angle from the optical axis
    let angle_h = nx * camera.fov_half_tan.x;
    let angle_v = ny * camera.fov_half_tan.y;

    // Direction in camera local space (-Z is forward in Bevy)
    var local_dir = vec3<f32>(angle_h, -angle_v, -1.0);
    local_dir = normalize(local_dir);

    // Rotate to world space using camera rotation matrix
    // The rotation matrix columns are the camera's right, up, forward vectors
    let rotation = mat3x3<f32>(
        camera.rotation_row0,
        camera.rotation_row1,
        camera.rotation_row2,
    );
    let world_dir = rotation * local_dir;

    // Atomically allocate a slot in the ray buffer
    let slot = atomicAdd(&ray_header.ray_count, 1u);

    // Check if we have space
    if (slot >= ray_header.max_rays) {
        // Buffer full - roll back and drop this ray
        atomicSub(&ray_header.ray_count, 1u);
        return;
    }

    // Write the ray
    rays[slot].camera_id = camera.camera_id;
    rays[slot]._pad0 = 0u;
    rays[slot]._pad1 = 0u;
    rays[slot]._pad2 = 0u;
    rays[slot].origin = camera.position;
    rays[slot]._pad3 = 0.0;
    rays[slot].direction = world_dir;
    rays[slot]._pad4 = 0.0;
}
