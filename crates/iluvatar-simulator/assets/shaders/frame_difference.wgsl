// Frame difference compute shader
// Computes grayscale difference between current and previous frame
//
// Input: current_frame and previous_frame (RGBA textures)
// Output: difference_mask (R8Uint texture with 255 for motion, 0 for no motion)

struct DiffParams {
    width: u32,
    height: u32,
    threshold: u32,      // 0-255
    _padding: u32,
}

@group(0) @binding(0) var current_frame: texture_2d<f32>;
@group(0) @binding(1) var previous_frame: texture_2d<f32>;
@group(0) @binding(2) var<uniform> params: DiffParams;
@group(0) @binding(3) var difference_mask: texture_storage_2d<r8uint, write>;

// Standard luminance weights for RGB to grayscale
fn to_grayscale(color: vec4<f32>) -> f32 {
    return dot(color.rgb, vec3<f32>(0.299, 0.587, 0.114));
}

@compute @workgroup_size(16, 16)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let x = global_id.x;
    let y = global_id.y;

    // Bounds check
    if (x >= params.width || y >= params.height) {
        return;
    }

    let coord = vec2<i32>(i32(x), i32(y));

    // Sample both frames
    let current_color = textureLoad(current_frame, coord, 0);
    let previous_color = textureLoad(previous_frame, coord, 0);

    // Convert to grayscale (0.0 - 1.0 range)
    let current_gray = to_grayscale(current_color);
    let previous_gray = to_grayscale(previous_color);

    // Compute absolute difference, scale to 0-255
    let diff = abs(current_gray - previous_gray) * 255.0;

    // Threshold and output
    var output: u32 = 0u;
    if (diff > f32(params.threshold)) {
        output = 255u;
    }

    textureStore(difference_mask, coord, vec4<u32>(output, 0u, 0u, 0u));
}
