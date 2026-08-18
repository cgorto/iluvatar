// RT-Smart camera pipeline for the Iluvatar tracking system.
//
// VICAP capture → EMA background → motion extraction → postcard encode →
// DATAFIFO. The K230's little Linux core forwards the stream over TCP.

package odin_camera

import "core:mem"

DEFAULT_CAMERA_ID: u64 : 1

// Send buffer: must fit the largest postcard-encoded motion frame
// plus the 4-byte TCP length prefix plus the 4-byte DATAFIFO header.
SEND_BUFFER_SIZE :: DATAFIFO_SLOT_SIZE

// Debug viewer: 4x downsample of the diff mask (320x180 = 57,600 bytes).
VIEWER_DOWNSAMPLE :: u32(4)
VIEWER_WIDTH      :: FRAME_WIDTH / VIEWER_DOWNSAMPLE   // 320
VIEWER_HEIGHT     :: FRAME_HEIGHT / VIEWER_DOWNSAMPLE  // 180
VIEWER_PIXELS     :: VIEWER_WIDTH * VIEWER_HEIGHT       // 57,600

// Statically allocated processing state (~7 MB BSS).
g_device:      Capture_Device
g_background:  Background_Model
g_extractor:   Motion_Extractor
g_send_buffer: [SEND_BUFFER_SIZE]u8
g_fifo_handle: K_Datafifo_Handle = K_DATAFIFO_INVALID_HANDLE
g_running:     bool = true

// Signal handler: just set the flag. The main loop checks g_running
// and does proper cleanup when it exits. Signal handlers can't call
// Odin procs (no context), so we keep this minimal.
shutdown_handler :: proc "c" (sig: i32) {
	g_running = false
}

CAMERA_REGISTRATION :: Camera_Registration {
	version   = PROTOCOL_VERSION,
	camera_id = DEFAULT_CAMERA_ID,
	intrinsics = Camera_Intrinsics {
		focal_length    = {600.0, 600.0},
		principal_point = {640.0, 360.0},
		resolution      = {1280, 720},
		fov             = {horizontal = 1.2, vertical = 0.7},
		distortion      = .None,
	},
	initial_pose = Camera_Pose {
		position    = {latitude = 47.6062, longitude = -122.3321, altitude = 10.0},
		orientation = {0.0, 0.0, 0.0, 1.0},
		timestamp   = 0,
		uncertainty = POSE_UNCERTAINTY_DEFAULT,
		status      = .Nominal,
	},
	capabilities = Camera_Capabilities {
		motion_frames = true,
		rle_encoding  = false,
		flags         = 0,
	},
}

main :: proc() {
	printf("=== Iluvatar RT-Smart Camera ===\n")

	// Register signal handlers for clean shutdown.
	signal(SIGINT, rawptr(shutdown_handler))
	signal(SIGTERM, rawptr(shutdown_handler))

	// Step 1: Open DATAFIFO as writer.
	fifo_params := K_Datafifo_Params {
		entries_num     = DATAFIFO_SLOT_COUNT,
		cache_line_size = DATAFIFO_SLOT_SIZE,
		release_by_writer = K_TRUE,
		open_mode       = DATAFIFO_WRITER,
	}

	ret := kd_datafifo_open(&g_fifo_handle, &fifo_params)
	if ret != 0 {
		printf("ERROR: datafifo_open failed: 0x%x\n", ret)
		_exit(1)
	}

	// Get physical address of the ring buffer.
	fifo_phys_addr: u64 = 0
	ret = kd_datafifo_cmd(
		g_fifo_handle,
		DATAFIFO_CMD_GET_PHY_ADDR,
		&fifo_phys_addr,
	)
	if ret != 0 {
		printf("ERROR: datafifo get_phy_addr failed: 0x%x\n", ret)
		_exit(1)
	}
	printf("DATAFIFO phys_addr: 0x%lx\n", fifo_phys_addr)

	// ShareFS is unreliable on some dual-system images. Emit the startup
	// metadata on the RT-Smart console so deployment tooling can provide it
	// to the Linux reader without blocking camera startup.
	print_registration_hex()
	printf("Provide DATAFIFO_PHYS_ADDR and REGISTRATION_HEX to the Linux reader.\n")

	// Step 2: Open VICAP and warm up the ISP. Use the firmware's proven
	// 1080p sensor mode and let the ISP downscale to 720p. The native
	// 720p mode reaches 60 fps but produces unstable luma on this image.
	if !capture_open(&g_device, sensor_type = OV5647_CSI0_1080P_30FPS) {
		printf("Failed to open VICAP\n")
		kd_datafifo_close(g_fifo_handle)
		_exit(1)
	}
	capture_warmup(&g_device)

	// Initialise background model.
	// Tuned for the v2.0 ISP (cleaner than v0.6.9, lower noise floor).
	// Alpha 0.02 = ~50-frame window, more sustained motion response.
	// Threshold 15 = catches subtle movement the v2.0 ISP preserves.
	background_init(&g_background, /*alpha=*/0.02, /*threshold=*/15)

	printf("Streaming motion frames to DATAFIFO...\n")

	HEARTBEAT_INTERVAL_US :: i64(15_000_000) // 15 seconds.

	// Step 3: Main capture-process-send loop.
	// Uses begin/end frame for zero-copy: background model reads
	// directly from the cached mmap'd ISP buffer.
	sequence: u64 = 0
	frames_sent: u64 = 0
	frames_dropped: u64 = 0
	last_send_us: i64 = 0
	{
		ts: Timespec
		clock_gettime(CLOCK_MONOTONIC, &ts)
		last_send_us = ts.tv_sec * 1_000_000 + ts.tv_nsec / 1_000
	}

	// Timing accumulators (microseconds, reset every stats interval).
	t_capture_us:    i64 = 0
	t_background_us: i64 = 0
	t_motion_us:     i64 = 0
	t_encode_us:     i64 = 0
	t_fifo_us:       i64 = 0
	t_total_us:      i64 = 0
	stats_frames:    u64 = 0

	for g_running {
		t0, t1, t2, t3, t4, t5: Timespec
		clock_gettime(CLOCK_MONOTONIC, &t0)

		// Acquire frame (zero-copy cached mmap).
		y_plane, cap_ok := capture_begin_frame(&g_device)
		if !cap_ok {
			printf("Capture failed at sequence %lu\n", sequence)
			break
		}
		clock_gettime(CLOCK_MONOTONIC, &t1)

		// Background model reads directly from the mmap'd buffer.
		mask, valid := background_update(&g_background, y_plane)

		// Release the ISP frame buffer. The background model has
		// consumed the Y plane; it no longer needs the source data.
		capture_end_frame(&g_device)
		clock_gettime(CLOCK_MONOTONIC, &t2)

		if !valid {
			sequence += 1
			continue // Seeding frame.
		}

		pixel_count := extract_motion(
			&g_extractor, mask,
			FRAME_WIDTH, FRAME_HEIGHT, POOL_FACTOR,
		)
		clock_gettime(CLOCK_MONOTONIC, &t3)

		if pixel_count == 0 {
			// Still accumulate timing for the capture+bg+motion path.
			t_capture_us += timespec_diff_us(t0, t1)
			t_background_us += timespec_diff_us(t1, t2)
			t_motion_us += timespec_diff_us(t2, t3)
			t_total_us += timespec_diff_us(t0, t3)
			stats_frames += 1

			// Keep the downstream DATAFIFO and TCP connection alive even when
			// the scene is completely static.
			now_mono: Timespec
			clock_gettime(CLOCK_MONOTONIC, &now_mono)
			now_us := now_mono.tv_sec * 1_000_000 + now_mono.tv_nsec / 1_000
			if now_us - last_send_us > HEARTBEAT_INTERVAL_US {
				fifo_write_heartbeat(sequence)
				last_send_us = now_us
			}

			sequence += 1
			continue
		}

		send_count := min(pixel_count, MAX_DATAFIFO_PIXELS)

		encoded_size := encode_datafifo_frame(
			&g_send_buffer,
			sequence, send_count, mask,
		)
		clock_gettime(CLOCK_MONOTONIC, &t4)

		sent_this_frame := false
		if encoded_size > 0 {
			// Flush pending releases, check space, write.
			kd_datafifo_write(g_fifo_handle, nil)
			avail_write: u32 = 0
			kd_datafifo_cmd(
				g_fifo_handle,
				DATAFIFO_CMD_GET_AVAIL_WRITE_LEN,
				&avail_write,
			)

			if avail_write >= DATAFIFO_SLOT_SIZE {
				ret = kd_datafifo_write(g_fifo_handle, &g_send_buffer)
				if ret == 0 {
					kd_datafifo_cmd(g_fifo_handle, DATAFIFO_CMD_WRITE_DONE, nil)
					frames_sent += 1
					sent_this_frame = true
				} else {
					frames_dropped += 1
				}
			} else {
				frames_dropped += 1
			}
		}

		// Track last send time for heartbeat.
		{
			now_mono: Timespec
			clock_gettime(CLOCK_MONOTONIC, &now_mono)
			now_us := now_mono.tv_sec * 1_000_000 + now_mono.tv_nsec / 1_000

			if sent_this_frame {
				last_send_us = now_us
			} else if now_us - last_send_us > HEARTBEAT_INTERVAL_US {
				// No motion sent recently — send a heartbeat to keep
				// the server connection alive.
				fifo_write_heartbeat(sequence)
				last_send_us = now_us
			}
		}
		clock_gettime(CLOCK_MONOTONIC, &t5)

		// Accumulate per-phase timing.
		t_capture_us += timespec_diff_us(t0, t1)
		t_background_us += timespec_diff_us(t1, t2)
		t_motion_us += timespec_diff_us(t2, t3)
		t_encode_us += timespec_diff_us(t3, t4)
		t_fifo_us += timespec_diff_us(t4, t5)
		t_total_us += timespec_diff_us(t0, t5)
		stats_frames += 1

		// Print timing breakdown every 200 frames.
		if stats_frames == 200 {
			n := i64(stats_frames)
			fps := n * 1_000_000 / max(t_total_us, 1)
			printf(
				"fps=%ld  cap=%ldus  bg=%ldus  mot=%ldus  enc=%ldus  fifo=%ldus  total=%ldus  sent=%lu\n",
				fps,
				t_capture_us / n,
				t_background_us / n,
				t_motion_us / n,
				t_encode_us / n,
				t_fifo_us / n,
				t_total_us / n,
				frames_sent,
			)
			t_capture_us = 0
			t_background_us = 0
			t_motion_us = 0
			t_encode_us = 0
			t_fifo_us = 0
			t_total_us = 0
			stats_frames = 0
		}

		sequence += 1
	}

	// Cleanup.
	// Flush: write NULL to trigger release callbacks.
	kd_datafifo_write(g_fifo_handle, nil)
	kd_datafifo_close(g_fifo_handle)
	capture_close(&g_device)

	printf("=== EXIT: sent=%lu dropped=%lu ===\n", frames_sent, frames_dropped)
	_exit(0)
}

// Encode a motion frame + viewer diff mask into the send buffer.
//
// DATAFIFO slot layout:
//   [u32 motion_size LE]  — bytes of TCP frame that follows.
//   [TCP frame]           — 4-byte BE length + postcard body.
//   [u32 viewer_size LE]  — bytes of viewer frame that follows (0 if none).
//   [viewer frame]        — 4-byte BE total_len + 2-byte BE width +
//                           2-byte BE height + 4-byte BE sequence +
//                           pixel_data (VIEWER_WIDTH * VIEWER_HEIGHT bytes).
//
// Returns total bytes written, or 0 on error.
encode_datafifo_frame :: proc(
	buffer: ^[SEND_BUFFER_SIZE]u8,
	sequence: u64,
	pixel_count: u32,
	mask: []u8,
) -> u32 {
	assert(buffer != nil)
	assert(pixel_count > 0)

	// --- Motion frame (for server) ---

	// Encode postcard payload starting at offset 8 (4 for our header +
	// 4 for TCP frame length prefix).
	encoder := Encoder{buffer = buffer[8:], offset = 0}

	now_us := get_timestamp_us()
	pose := CAMERA_REGISTRATION.initial_pose
	pose.timestamp = now_us

	encode_motion_frame(
		&encoder,
		DEFAULT_CAMERA_ID,
		sequence,
		now_us,
		&pose,
		g_extractor.pixels[:pixel_count],
	)

	postcard_size := encoder.offset
	if postcard_size == 0 do return 0

	// Write TCP frame header (4-byte big-endian length).
	buffer[4] = u8(postcard_size >> 24)
	buffer[5] = u8(postcard_size >> 16)
	buffer[6] = u8(postcard_size >> 8)
	buffer[7] = u8(postcard_size)

	// Write motion_size: total TCP frame bytes (4 + postcard).
	motion_tcp_size := 4 + postcard_size
	buffer[0] = u8(motion_tcp_size)
	buffer[1] = u8(motion_tcp_size >> 8)
	buffer[2] = u8(motion_tcp_size >> 16)
	buffer[3] = u8(motion_tcp_size >> 24)

	cursor := 8 + postcard_size

	// --- Viewer diff mask (for debug viewer) ---

	// Viewer frame: [total_len:u32 BE][width:u16 BE][height:u16 BE]
	//               [sequence:u32 BE][pixel_data]
	viewer_header_size :: u32(4 + 2 + 2 + 4) // 12 bytes.
	viewer_frame_size := viewer_header_size + VIEWER_PIXELS

	// Check there's room in the slot.
	if cursor + 4 + viewer_frame_size > SEND_BUFFER_SIZE {
		// No room for viewer data; write viewer_size = 0.
		buffer[cursor]     = 0
		buffer[cursor + 1] = 0
		buffer[cursor + 2] = 0
		buffer[cursor + 3] = 0
		return cursor + 4
	}

	// Write viewer_size.
	buffer[cursor]     = u8(viewer_frame_size)
	buffer[cursor + 1] = u8(viewer_frame_size >> 8)
	buffer[cursor + 2] = u8(viewer_frame_size >> 16)
	buffer[cursor + 3] = u8(viewer_frame_size >> 24)
	cursor += 4

	// Viewer protocol: total_len (BE u32) = 8 + pixel_count.
	total_len := u32(8) + VIEWER_PIXELS
	buffer[cursor]     = u8(total_len >> 24)
	buffer[cursor + 1] = u8(total_len >> 16)
	buffer[cursor + 2] = u8(total_len >> 8)
	buffer[cursor + 3] = u8(total_len)
	cursor += 4

	// Width (BE u16).
	vw := u16(VIEWER_WIDTH)
	buffer[cursor]     = u8(vw >> 8)
	buffer[cursor + 1] = u8(vw & 0xFF)
	cursor += 2

	// Height (BE u16).
	vh := u16(VIEWER_HEIGHT)
	buffer[cursor]     = u8(vh >> 8)
	buffer[cursor + 1] = u8(vh & 0xFF)
	cursor += 2

	// Sequence (BE u32).
	seq32 := u32(sequence)
	buffer[cursor]     = u8(seq32 >> 24)
	buffer[cursor + 1] = u8(seq32 >> 16)
	buffer[cursor + 2] = u8(seq32 >> 8)
	buffer[cursor + 3] = u8(seq32)
	cursor += 4

	// Downsample diff mask: max-pool each VIEWER_DOWNSAMPLE x
	// VIEWER_DOWNSAMPLE block into one pixel.
	downsample_max_pool(
		mask, FRAME_WIDTH, FRAME_HEIGHT,
		buffer[cursor:][:VIEWER_PIXELS],
		VIEWER_WIDTH, VIEWER_HEIGHT,
		VIEWER_DOWNSAMPLE,
	)
	cursor += VIEWER_PIXELS

	return cursor
}

// Max-pool downsample a grayscale image.
downsample_max_pool :: proc(
	source: []u8,
	source_width:  u32,
	source_height: u32,
	target: []u8,
	target_width:  u32,
	target_height: u32,
	factor: u32,
) {
	assert(len(source) >= int(source_width * source_height))
	assert(len(target) >= int(target_width * target_height))

	for ty in 0 ..< target_height {
		for tx in 0 ..< target_width {
			max_val: u8 = 0
			sy_end := min(ty * factor + factor, source_height)
			sx_end := min(tx * factor + factor, source_width)
			for sy in ty * factor ..< sy_end {
				for sx in tx * factor ..< sx_end {
					val := source[sy * source_width + sx]
					if val > max_val do max_val = val
				}
			}
			target[ty * target_width + tx] = max_val
		}
	}
}

// Encode and write a heartbeat message to the DATAFIFO.
// Same slot layout as motion frames: [u32 motion_size LE][TCP frame].
// Heartbeats are tiny (~10 bytes) so they always fit.
fifo_write_heartbeat :: proc(sequence: u64) {
	// Encode into the first 64 bytes of g_send_buffer.
	encoder := Encoder{buffer = g_send_buffer[8:64], offset = 0}
	now_us := get_timestamp_us()
	encode_heartbeat(&encoder, DEFAULT_CAMERA_ID, now_us)

	postcard_size := encoder.offset

	// TCP frame header (4-byte BE length).
	g_send_buffer[4] = u8(postcard_size >> 24)
	g_send_buffer[5] = u8(postcard_size >> 16)
	g_send_buffer[6] = u8(postcard_size >> 8)
	g_send_buffer[7] = u8(postcard_size)

	// DATAFIFO header: total TCP frame size.
	tcp_size := 4 + postcard_size
	g_send_buffer[0] = u8(tcp_size)
	g_send_buffer[1] = u8(tcp_size >> 8)
	g_send_buffer[2] = u8(tcp_size >> 16)
	g_send_buffer[3] = u8(tcp_size >> 24)

	// Viewer size = 0 (no viewer data for heartbeats).
	cursor := 8 + postcard_size
	g_send_buffer[cursor] = 0
	g_send_buffer[cursor + 1] = 0
	g_send_buffer[cursor + 2] = 0
	g_send_buffer[cursor + 3] = 0

	// Write to DATAFIFO (best-effort, don't block on full).
	kd_datafifo_write(g_fifo_handle, nil) // Flush releases.
	avail: u32 = 0
	kd_datafifo_cmd(g_fifo_handle, DATAFIFO_CMD_GET_AVAIL_WRITE_LEN, &avail)
	if avail >= DATAFIFO_SLOT_SIZE {
		ret := kd_datafifo_write(g_fifo_handle, &g_send_buffer)
		if ret == 0 {
			kd_datafifo_cmd(g_fifo_handle, DATAFIFO_CMD_WRITE_DONE, nil)
		}
	}
}

// Print the pre-encoded TCP-framed registration message for deployment
// tooling. The Linux reader sends these exact bytes to the server.
print_registration_hex :: proc() {
	reg_buf: [4096]u8

	// Encode postcard payload at offset 4 (leave room for length prefix).
	encoder := Encoder{buffer = reg_buf[4:], offset = 0}
	reg := CAMERA_REGISTRATION
	encode_camera_registration(&encoder, &reg)

	// Write 4-byte big-endian length prefix.
	payload_len := encoder.offset
	reg_buf[0] = u8(payload_len >> 24)
	reg_buf[1] = u8(payload_len >> 16)
	reg_buf[2] = u8(payload_len >> 8)
	reg_buf[3] = u8(payload_len)

	total := 4 + payload_len

	printf("REGISTRATION_HEX:")
	for byte in reg_buf[:total] {
		printf("%02x", byte)
	}
	printf("\n")
}
