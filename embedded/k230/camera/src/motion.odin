// Motion extraction: noise filtering and max-pool downsampling.
//
// Takes the diff mask from the background model, filters isolated
// noise pixels, then downsamples via max-pooling to produce a compact
// array of Motion_Pixels for transmission to the server.
//
// The pipeline is: diff_mask → erosion filter → max-pool → Motion_Pixels.
//
// Coordinates in the output are in original frame resolution (not
// pooled resolution), so the server's camera intrinsics remain valid
// for ray projection.

package odin_camera

Motion_Extractor :: struct {
	pixels: [MAX_MOTION_PIXELS]Motion_Pixel,
}

// Minimum active 8-connected neighbours to survive erosion.
// The Rust camera uses 2; fewer produces too many noise pixels
// (especially with 3DNR disabled on the K230 ISP).
NOISE_MIN_NEIGHBORS: u32 : 2

// Filter isolated noise from the diff mask in-place. A pixel survives
// only if it has at least min_neighbors active neighbours in its
// 8-connected neighbourhood.
//
// Single-pass erosion: pixels zeroed early in the scan may cause
// their neighbours to also be zeroed. This makes the filter slightly
// more aggressive than a two-pass version, which is acceptable —
// isolated noise is the target, not cluster edges.
filter_noise :: proc(mask: []u8, width: u32, height: u32, min_neighbors: u32) {
	assert(len(mask) == int(width * height))
	assert(width > 0)
	assert(height > 0)

	for y in 0 ..< height {
		for x in 0 ..< width {
			idx := y * width + x
			if mask[idx] == 0 do continue

			count := count_active_neighbors(mask, width, height, x, y)
			if count < min_neighbors do mask[idx] = 0
		}
	}
}

// Count active (non-zero) 8-connected neighbours of pixel (x, y).
count_active_neighbors :: proc(
	mask: []u8,
	width: u32,
	height: u32,
	x: u32,
	y: u32,
) -> u32 {
	assert(x < width)
	assert(y < height)

	count: u32 = 0

	y_lo := y > 0 ? y - 1 : 0
	y_hi := y + 1 < height ? y + 1 : height - 1
	x_lo := x > 0 ? x - 1 : 0
	x_hi := x + 1 < width ? x + 1 : width - 1

	for ny in y_lo ..= y_hi {
		for nx in x_lo ..= x_hi {
			if nx == x && ny == y do continue
			if mask[ny * width + nx] > 0 do count += 1
		}
	}
	return count
}

// Max-pool the diff mask and extract Motion_Pixels. Each
// pool_factor×pool_factor block is reduced to a single pixel whose
// intensity is the block's maximum. Blocks with zero max are skipped.
//
// Output coordinates are in original frame resolution (block origin
// times pool_factor), preserving the server's intrinsics for ray
// projection.
//
// Returns the number of motion pixels written to extractor.pixels.
extract_motion :: proc(
	extractor: ^Motion_Extractor,
	mask: []u8,
	width: u32,
	height: u32,
	pool_factor: u32,
) -> u32 {
	assert(extractor != nil)
	assert(len(mask) == int(width * height))
	assert(pool_factor > 0)

	pooled_w := (width + pool_factor - 1) / pool_factor
	pooled_h := (height + pool_factor - 1) / pool_factor
	assert(pooled_w * pooled_h <= MAX_MOTION_PIXELS)

	count: u32 = 0
	for by in 0 ..< pooled_h {
		for bx in 0 ..< pooled_w {
			max_val := max_pool_block(
				mask, width, height,
				bx * pool_factor, by * pool_factor,
				pool_factor,
			)
			if max_val == 0 do continue

			assert(count < MAX_MOTION_PIXELS)
			extractor.pixels[count] = Motion_Pixel {
				x         = u16(bx * pool_factor),
				y         = u16(by * pool_factor),
				intensity = max_val,
			}
			count += 1
		}
	}
	return count
}

// Find the maximum pixel value in a pool_factor×pool_factor block
// starting at (x_start, y_start). Clamps to image bounds.
max_pool_block :: proc(
	mask: []u8,
	width: u32,
	height: u32,
	x_start: u32,
	y_start: u32,
	pool_factor: u32,
) -> u8 {
	assert(x_start < width)
	assert(y_start < height)

	x_end := min(x_start + pool_factor, width)
	y_end := min(y_start + pool_factor, height)

	max_val: u8 = 0
	for y in y_start ..< y_end {
		for x in x_start ..< x_end {
			val := mask[y * width + x]
			if val > max_val do max_val = val
		}
	}
	return max_val
}
