// Exponential moving average background model.
//
// Maintains a per-pixel f32 running average of the scene. Each new
// frame blends into the background via: bg = alpha*frame + (1-alpha)*bg.
// The absolute difference between the current frame and the rounded
// background is thresholded to produce a motion mask.
//
// Alpha ~0.05 gives a 20-frame averaging window. Lower alpha means
// slower adaptation (more robust to gradual lighting changes but
// slower to learn a new static scene). The threshold rejects sensor
// noise — pixels with difference <= threshold are zeroed.
//
// Memory: PIXEL_COUNT * 4 bytes (f32 background) + PIXEL_COUNT bytes
// (u8 diff mask) = ~4.4 MB at 1280×720. Allocated statically.

package odin_camera

Background_Model :: struct {
	background: [PIXEL_COUNT]f32,
	diff_mask:  [PIXEL_COUNT]u8,
	alpha:      f32,
	threshold:  u8,
	seeded:     bool,
}

DEFAULT_ALPHA:     f32 : 0.05 // ~20 frame averaging window.
DEFAULT_THRESHOLD: u8  : 50   // Just above OV5647 read noise in 720p binning
                               // mode. The voxel grid handles noise rejection
                               // via multi-camera statistical accumulation, so
                               // the camera preserves sub-noise-floor motion.

// Initialise the background model parameters. Does not allocate.
// The model is seeded on the first call to background_update.
background_init :: proc(model: ^Background_Model, alpha: f32, threshold: u8) {
	assert(model != nil)
	assert(alpha > 0.0)
	assert(alpha < 1.0)

	model.alpha = alpha
	model.threshold = threshold
	model.seeded = false
}

// Feed a new Y-plane frame into the model. On the first call, the
// frame seeds the background (no diff produced). On subsequent calls,
// the background is updated via EMA and the diff mask is computed.
//
// Returns the diff mask slice (PIXEL_COUNT bytes, valid until the next
// call) and whether the mask is valid (false on the seeding frame).
background_update :: proc(
	model: ^Background_Model,
	frame: []u8,
) -> (mask: []u8, valid: bool) {
	assert(model != nil)
	assert(len(frame) == int(PIXEL_COUNT))

	if !model.seeded {
		background_seed(model.background[:], frame)
		model.seeded = true
		return model.diff_mask[:], false
	}

	background_update_pixels(
		model.background[:],
		frame,
		model.diff_mask[:],
		model.alpha,
		1.0 - model.alpha,
		model.threshold,
	)

	return model.diff_mask[:], true
}

// Count active (non-zero) pixels in the diff mask.
background_motion_count :: proc(model: ^Background_Model) -> u32 {
	assert(model != nil)
	count: u32 = 0
	for i in 0 ..< PIXEL_COUNT {
		if model.diff_mask[i] > 0 do count += 1
	}
	return count
}

// --- Hot loops (standalone, primitive arguments) ----------------------------

// Seed the background with the first frame.
background_seed :: proc(background: []f32, frame: []u8) {
	assert(len(background) == int(PIXEL_COUNT))
	assert(len(frame) == int(PIXEL_COUNT))

	for i in 0 ..< PIXEL_COUNT {
		background[i] = f32(frame[i])
	}
}

// EMA update + thresholded difference. This is the per-frame hot path.
// Extracted as a standalone procedure with primitive slice arguments so
// the compiler can keep loop variables in registers.
background_update_pixels :: proc(
	background: []f32,
	frame: []u8,
	mask: []u8,
	alpha: f32,
	one_minus_alpha: f32,
	threshold: u8,
) {
	assert(len(background) == int(PIXEL_COUNT))
	assert(len(frame) == int(PIXEL_COUNT))
	assert(len(mask) == int(PIXEL_COUNT))

	for i in 0 ..< PIXEL_COUNT {
		pixel := f32(frame[i])
		background[i] = alpha * pixel + one_minus_alpha * background[i]

		// Absolute difference between frame and rounded background.
		bg_rounded := u8(background[i])
		diff: u8
		if frame[i] >= bg_rounded {
			diff = frame[i] - bg_rounded
		} else {
			diff = bg_rounded - frame[i]
		}

		// Threshold: zero out small differences (sensor noise).
		if diff > threshold {
			mask[i] = diff
		} else {
			mask[i] = 0
		}
	}
}
