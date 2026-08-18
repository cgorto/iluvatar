// VICAP frame capture for the K230 OV5647 sensor on RT-Smart.
//
// Replaces the V4L2 capture module (odin-camera/src/capture.odin) with
// MPP VICAP API calls. Uses a begin/end frame interface to allow
// zero-copy processing of the Y plane directly from the ISP buffer.
//
// Performance: uses kd_mpi_sys_mmap_cached + mmz_flush_cache so the
// CPU reads Y-plane data through L1/L2 cache instead of uncached DRAM.
// This is critical — uncached reads of 921KB are ~10x slower.

package odin_camera

import "core:mem"

CAPTURE_BUFFER_COUNT :: 5   // VB pool buffer count.
WARMUP_FRAME_COUNT  :: 30  // ISP auto-exposure convergence frames.

// NV12 frame size aligned to 1K boundary.
FRAME_SIZE_NV12 :: u64(
	((u64(FRAME_WIDTH) * u64(FRAME_HEIGHT) * 3 / 2) + VICAP_ALIGN_1K - 1) &
	~(VICAP_ALIGN_1K - 1),
)

Capture_Device :: struct {
	sensor_type:    i32,
	dev:            i32,
	chn:            i32,
	width:          u32,
	height:         u32,
	streaming:      bool,

	// Current frame state (valid between begin_frame and end_frame).
	frame_active:   bool,
	frame_info:     K_Video_Frame_Info,
	frame_virt:     rawptr,   // Cached mmap'd pointer to Y plane.
	frame_y_size:   u32,      // Y plane size in bytes.
}

// Open the VICAP device and configure for NV12 capture at 1280x720.
capture_open :: proc(
	device: ^Capture_Device,
	sensor_type: i32 = OV5647_CSI0_1080P_30FPS,
) -> bool {
	assert(device != nil)

	device.sensor_type = sensor_type
	device.dev = VICAP_DEV_ID_0
	device.chn = VICAP_CHN_ID_0
	device.width = FRAME_WIDTH
	device.height = FRAME_HEIGHT

	// Reset any leftover state from a previous run or failed attempt.
	// Must deinit VICAP first — it holds VB blocks that prevent vb_exit.
	kd_mpi_vicap_stop_stream(VICAP_DEV_ID_0)
	kd_mpi_vicap_deinit(VICAP_DEV_ID_0)
	kd_mpi_vb_exit()

	// Step 1: Query sensor info.
	sensor_info: K_VICAP_Sensor_Info
	mem.zero(&sensor_info, size_of(K_VICAP_Sensor_Info))

	ret := kd_mpi_vicap_get_sensor_info(sensor_type, &sensor_info)
	if ret != 0 {
		printf("ERROR: get_sensor_info(%d) failed: 0x%x\n",
			sensor_type, ret)
		return false
	}
	printf("sensor: %s, input: %dx%d\n",
		sensor_info.sensor_name, sensor_info.width, sensor_info.height)

	// Step 2: Configure VB pool.
	vb_config: K_VB_Config
	mem.zero(&vb_config, size_of(K_VB_Config))
	vb_config.max_pool_cnt = 64
	vb_config.comm_pool[0].blk_cnt = CAPTURE_BUFFER_COUNT
	vb_config.comm_pool[0].blk_size = FRAME_SIZE_NV12
	vb_config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE

	ret = kd_mpi_vb_set_config(&vb_config)
	if ret != 0 {
		printf("ERROR: vb_set_config failed: 0x%x\n", ret)
		return false
	}

	ret = kd_mpi_vb_init()
	if ret != 0 {
		printf("ERROR: vb_init failed: 0x%x\n", ret)
		return false
	}

	// Step 3: Configure VICAP device.
	dev_attr: K_VICAP_Dev_Attr
	mem.zero(&dev_attr, size_of(K_VICAP_Dev_Attr))
	dev_attr.acq_win.width = sensor_info.width
	dev_attr.acq_win.height = sensor_info.height
	dev_attr.mode = VICAP_WORK_ONLINE_MODE
	dev_attr.input_type = VICAP_INPUT_TYPE_SENSOR
	dev_attr.sensor_info = sensor_info
	dev_attr.dev_enable = K_TRUE

	// Enable auto-exposure and auto-white-balance. Reliable motion
	// extraction is more important than holding 60 fps in a dark scene;
	// the measured capture rate is reported during warmup.
	dev_attr.pipe_ctrl = ISP_PIPE_AE | ISP_PIPE_AWB

	ret = kd_mpi_vicap_set_dev_attr(device.dev, dev_attr)
	if ret != 0 {
		printf("ERROR: set_dev_attr failed: 0x%x\n", ret)
		kd_mpi_vb_exit()
		return false
	}

	// Step 4: Configure output channel (1280x720 NV12).
	chn_attr: K_VICAP_Chn_Attr
	mem.zero(&chn_attr, size_of(K_VICAP_Chn_Attr))
	chn_attr.out_win.width = u16(device.width)
	chn_attr.out_win.height = u16(device.height)
	chn_attr.crop_win.width = sensor_info.width
	chn_attr.crop_win.height = sensor_info.height
	chn_attr.scale_win = chn_attr.out_win
	chn_attr.crop_enable = K_FALSE
	chn_attr.scale_enable = K_FALSE
	chn_attr.chn_enable = K_TRUE
	chn_attr.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420
	chn_attr.buffer_num = CAPTURE_BUFFER_COUNT
	chn_attr.buffer_size = u32(FRAME_SIZE_NV12)

	ret = kd_mpi_vicap_set_chn_attr(device.dev, device.chn, chn_attr)
	if ret != 0 {
		printf("ERROR: set_chn_attr failed: 0x%x\n", ret)
		kd_mpi_vb_exit()
		return false
	}

	// Step 5: Initialize and start streaming.
	ret = kd_mpi_vicap_init(device.dev)
	if ret != 0 {
		printf("ERROR: vicap_init failed: 0x%x\n", ret)
		kd_mpi_vb_exit()
		return false
	}

	ret = kd_mpi_vicap_start_stream(device.dev)
	if ret != 0 {
		printf("ERROR: start_stream failed: 0x%x\n", ret)
		kd_mpi_vicap_deinit(device.dev)
		kd_mpi_vb_exit()
		return false
	}
	device.streaming = true

	printf("VICAP capture: NV12 %dx%d, %d buffers, streaming\n",
		device.width, device.height, CAPTURE_BUFFER_COUNT)
	return true
}

// Close the VICAP device and release all resources.
capture_close :: proc(device: ^Capture_Device) {
	assert(device != nil)

	if device.frame_active do capture_end_frame(device)

	if device.streaming {
		kd_mpi_vicap_stop_stream(device.dev)
		device.streaming = false
	}

	kd_mpi_vicap_deinit(device.dev)
	kd_mpi_vb_exit()
}

// Acquire the next frame from the ISP. Returns a Y-plane slice
// backed by a cached mmap of the ISP frame buffer. The slice is
// valid until capture_end_frame is called.
//
// Uses cached mmap + cache invalidate so the CPU reads through
// L1/L2 cache instead of uncached DRAM. The ISP writes to physical
// memory via DMA, so we invalidate the cache region before reading
// to ensure coherency.
capture_begin_frame :: proc(
	device: ^Capture_Device,
) -> (y_plane: []u8, ok: bool) {
	assert(device != nil)
	assert(device.streaming)
	assert(!device.frame_active)

	mem.zero(&device.frame_info, size_of(K_Video_Frame_Info))

	ret := kd_mpi_vicap_dump_frame(
		device.dev, device.chn,
		VICAP_DUMP_YUV,
		&device.frame_info,
		/*timeout_ms=*/1000,
	)
	if ret != 0 {
		printf("ERROR: dump_frame failed: 0x%x\n", ret)
		return nil, false
	}

	y_size := device.frame_info.v_frame.stride[0] *
	          device.frame_info.v_frame.height
	assert(y_size > 0)
	assert(y_size <= PIXEL_COUNT)

	// Cached mmap: CPU reads go through L1/L2 cache.
	virt := kd_mpi_sys_mmap_cached(
		device.frame_info.v_frame.phys_addr[0],
		y_size,
	)
	if virt == nil {
		printf("ERROR: sys_mmap_cached failed for 0x%lx\n",
			device.frame_info.v_frame.phys_addr[0])
		kd_mpi_vicap_dump_release(device.dev, device.chn, &device.frame_info)
		return nil, false
	}

	// Invalidate cache: the ISP wrote via DMA, so stale cache lines
	// from a previous mmap of the same physical address must be flushed.
	kd_mpi_sys_mmz_flush_cache(
		device.frame_info.v_frame.phys_addr[0],
		virt,
		y_size,
	)

	device.frame_virt = virt
	device.frame_y_size = y_size
	device.frame_active = true

	return ([^]u8)(virt)[:y_size], true
}

// Release the current frame. The Y-plane slice from capture_begin_frame
// becomes invalid after this call.
capture_end_frame :: proc(device: ^Capture_Device) {
	assert(device != nil)
	assert(device.frame_active)

	kd_mpi_sys_munmap(device.frame_virt, device.frame_y_size)
	kd_mpi_vicap_dump_release(device.dev, device.chn, &device.frame_info)

	device.frame_virt = nil
	device.frame_active = false
}

// Convenience: capture one frame with copy (for warmup where we
// don't need the data). Equivalent to begin + end with no processing.
capture_frame :: proc(
	device: ^Capture_Device,
) -> (y_plane: []u8, ok: bool) {
	// For backward compat with the warmup loop, just begin+end.
	_, begin_ok := capture_begin_frame(device)
	if !begin_ok do return nil, false
	capture_end_frame(device)
	return nil, true
}

// Discard frames while the ISP auto-exposure converges, then
// benchmark the raw dump_frame rate (no mmap, no processing).
capture_warmup :: proc(device: ^Capture_Device) {
	assert(device != nil)
	assert(device.streaming)

	printf("Warming up ISP (%d frames)...\n", WARMUP_FRAME_COUNT)
	for i in 0 ..< WARMUP_FRAME_COUNT {
		_, ok := capture_frame(device)
		if !ok {
			printf("Warmup failed at frame %d\n", i)
			return
		}
	}

	// Benchmark: raw dump+release rate (no mmap, no processing).
	BENCH_FRAMES :: 60
	t_start, t_end: Timespec
	clock_gettime(CLOCK_MONOTONIC, &t_start)
	for i in 0 ..< BENCH_FRAMES {
		frame_info: K_Video_Frame_Info
		mem.zero(&frame_info, size_of(K_Video_Frame_Info))
		ret := kd_mpi_vicap_dump_frame(
			device.dev, device.chn,
			VICAP_DUMP_YUV, &frame_info,
			/*timeout_ms=*/1000,
		)
		if ret != 0 {
			printf("Bench dump failed at %d: 0x%x\n", i, ret)
			break
		}
		kd_mpi_vicap_dump_release(device.dev, device.chn, &frame_info)
	}
	clock_gettime(CLOCK_MONOTONIC, &t_end)
	bench_us := timespec_diff_us(t_start, t_end)
	bench_fps := i64(BENCH_FRAMES) * 1_000_000 / max(bench_us, 1)
	printf("Warmup complete. Raw dump rate: %ld fps (%ld us/frame)\n",
		bench_fps, bench_us / BENCH_FRAMES)
}
