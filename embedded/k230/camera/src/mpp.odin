// K230 MPP (Multimedia Processing Platform) bindings for Odin.
//
// Mirrors the C struct layouts from the K230 SDK headers:
//   k_type.h, k_vb_comm.h, k_vicap_comm.h, k_video_comm.h,
//   k_sensor_comm.h, k_module.h.
//
// Only includes the types and functions needed for VICAP frame
// capture. The MPP libraries are linked via the build script
// (--start-group -lvicap -lsensor -lvb -lsys ... --end-group).
//
// C enum types are represented as i32 (C int) with named constants.
// C k_bool is i32 (enum { K_FALSE=0, K_TRUE=1 }).
// C k_u64 is u64 (unsigned long on RISC-V LP64).
// Pointer fields are rawptr or cstring as appropriate.
//
// Compile-time assertions at the bottom verify struct sizes match
// the C layout on RISC-V 64-bit LP64D.

package odin_camera

import "core:c"

// =========================================================================
// Primitive type aliases.
// =========================================================================

K_Bool :: distinct i32
K_FALSE :: K_Bool(0)
K_TRUE  :: K_Bool(1)

// =========================================================================
// VB (Video Buffer) types — k_vb_comm.h
// =========================================================================

VB_MAX_COMM_POOLS :: 16
MAX_MMZ_NAME_LEN  :: 16

VB_REMAP_MODE_NONE    :: i32(0)
VB_REMAP_MODE_NOCACHE :: i32(1)
VB_REMAP_MODE_CACHED  :: i32(2)

K_VB_Pool_Config :: struct {
	blk_size: u64,
	blk_cnt:  u32,
	mode:     i32,                     // k_vb_remap_mode enum.
	mmz_name: [MAX_MMZ_NAME_LEN]u8,
}

K_VB_Config :: struct {
	max_pool_cnt: u32,
	_pad0:        u32,                 // Align comm_pool to 8 bytes.
	comm_pool:    [VB_MAX_COMM_POOLS]K_VB_Pool_Config,
}

// =========================================================================
// VICAP types — k_vicap_comm.h
// =========================================================================

// Device IDs.
VICAP_DEV_ID_0 :: i32(0)
VICAP_DEV_ID_1 :: i32(1)
VICAP_DEV_ID_2 :: i32(2)

// Channel IDs.
VICAP_CHN_ID_0 :: i32(0)
VICAP_CHN_ID_1 :: i32(1)
VICAP_CHN_ID_2 :: i32(2)

// Dump format.
VICAP_DUMP_YUV :: i32(0)

// Work mode.
VICAP_WORK_ONLINE_MODE  :: i32(0)
VICAP_WORK_OFFLINE_MODE :: i32(1)

// Input type.
VICAP_INPUT_TYPE_SENSOR :: i32(0)

// Mirror.
VICAP_MIRROR_NONE :: i32(0)

// ISP pipe control bits (k_vicap_isp_pipe_ctrl union). The C type
// is a union of a bitfield struct and a u32. We use the u32 form
// and set bits explicitly. Bit positions match the C header.
ISP_PIPE_AE   :: u32(1 << 0)
ISP_PIPE_AF   :: u32(1 << 1)
ISP_PIPE_AWB  :: u32(1 << 3)
ISP_PIPE_DNR3 :: u32(1 << 25)

// OV5647 sensor type constants (k_vicap_sensor_type enum).
OV5647_CSI0_1080P_30FPS :: i32(24)  // Known working on v2.0 image.
OV5647_CSI0_720P_60FPS  :: i32(44)  // Native 720p, may not exist.

// 1K alignment for VB frame size calculation.
VICAP_ALIGN_1K :: u64(0x400)

K_VICAP_Window :: struct {
	h_start: u16,
	v_start: u16,
	width:   u16,
	height:  u16,
}

K_VICAP_Sensor_Info :: struct {
	sensor_name:   cstring,   // const char * (8 bytes).
	database_name: cstring,   // const char * (8 bytes).
	width:         u16,
	height:        u16,
	csi_num:       i32,       // k_vicap_csi_num enum.
	mipi_lanes:    i32,       // k_vicap_mipi_lanes enum.
	source_id:     i32,       // k_vicap_data_source enum.
	is_3d_sensor:  K_Bool,
	phy_freq:      i32,       // k_vicap_mipi_phy_freq enum.
	data_type:     i32,       // k_vicap_csi_data_type enum.
	hdr_mode:      i32,       // k_vicap_hdr_mode enum.
	flash_mode:    i32,       // k_vicap_vi_flash_mode enum.
	first_frame:   i32,       // k_vicap_vi_first_frame_sel enum.
	glitch_filter: u16,
	fps:           u16,
	sensor_type:   i32,       // k_vicap_sensor_type enum.
}

K_VICAP_Dev_Attr :: struct {
	acq_win:         K_VICAP_Window,
	mode:            i32,          // k_vicap_work_mode enum.
	input_type:      i32,          // k_vicap_input_type enum.
	image_pat:       i32,          // k_sensor_bayer_pattern enum.
	pipe_ctrl:       u32,          // k_vicap_isp_pipe_ctrl (as u32).
	cpature_frame:   u32,          // Typo in SDK header, preserved.
	// 4 bytes implicit padding: sensor_info has 8-byte alignment
	// because it starts with pointer fields.
	sensor_info:     K_VICAP_Sensor_Info,
	dw_enable:       K_Bool,
	dev_enable:      K_Bool,
	buffer_num:      u32,
	buffer_size:     u32,
	mirror:          i32,          // k_vicap_mirror enum.
	fastboot_enable: K_Bool,
}

K_VICAP_Chn_Attr :: struct {
	out_win:      K_VICAP_Window,
	crop_win:     K_VICAP_Window,
	scale_win:    K_VICAP_Window,
	crop_enable:  K_Bool,
	scale_enable: K_Bool,
	chn_enable:   K_Bool,
	pix_format:   i32,            // k_pixel_format enum.
	buffer_num:   u32,
	buffer_size:  u32,
	alignment:    u8,
	fps:          u8,
	// 2 bytes implicit padding to reach struct alignment (4).
}

// =========================================================================
// Video frame types — k_video_comm.h
// =========================================================================

// Pixel format constants (k_pixel_format enum). Only the one we use.
PIXEL_FORMAT_YUV_SEMIPLANAR_420 :: i32(31)  // NV12.

K_Video_Supplement :: struct {
	jpeg_dcf_phy_addr:    u64,
	isp_info_phy_addr:    u64,
	jpeg_dcf_kvirt_addr:  rawptr,
	isp_info_kvirt_addr:  rawptr,
}

K_Video_Frame :: struct {
	width:            u32,
	height:           u32,
	field:            i32,     // k_video_field enum.
	pixel_format:     i32,     // k_pixel_format enum.
	video_format:     i32,     // k_video_format enum.
	dynamic_range:    i32,     // k_dynamic_range enum.
	compress_mode:    i32,     // k_compress_mode enum.
	color_gamut:      i32,     // k_color_gamut enum.

	header_stride:    [3]u32,
	stride:           [3]u32,

	header_phys_addr: [3]u64,
	header_virt_addr: [3]u64,

	phys_addr:        [3]u64,
	virt_addr:        [3]u64,

	offset_top:       i16,
	offset_bottom:    i16,
	offset_left:      i16,
	offset_right:     i16,

	time_ref:         u32,
	// 4 bytes implicit padding: pts has 8-byte alignment.
	pts:              u64,

	priv_data:        u64,
	supplement:       K_Video_Supplement,
}

K_Video_Frame_Info :: struct {
	v_frame: K_Video_Frame,
	pool_id: u32,
	mod_id:  i32,              // k_mod_id enum.
}

// =========================================================================
// DATAFIFO types — k_datafifo.h
//
// Inter-processor FIFO for streaming data from big core (RT-Smart)
// to little core (Linux). The ring buffer lives in shared MMZ memory.
// =========================================================================

K_Datafifo_Handle :: distinct u64
K_DATAFIFO_INVALID_HANDLE :: K_Datafifo_Handle(max(u64))

// Open mode: writer allocates, reader maps by address.
DATAFIFO_READER :: i32(0)
DATAFIFO_WRITER :: i32(1)

// Commands for kd_datafifo_cmd.
DATAFIFO_CMD_GET_PHY_ADDR             :: i32(0)
DATAFIFO_CMD_READ_DONE                :: i32(1)
DATAFIFO_CMD_WRITE_DONE               :: i32(2)
DATAFIFO_CMD_SET_DATA_RELEASE_CALLBACK :: i32(3)
DATAFIFO_CMD_GET_AVAIL_WRITE_LEN      :: i32(4)
DATAFIFO_CMD_GET_AVAIL_READ_LEN       :: i32(5)

K_Datafifo_Params :: struct {
	entries_num:     u32,     // Number of slots in the ring buffer.
	cache_line_size: u32,     // Size of each slot in bytes.
	release_by_writer: K_Bool, // Whether writer controls buffer release.
	open_mode:       i32,     // DATAFIFO_READER or DATAFIFO_WRITER.
}

// DATAFIFO slot dimensions for motion frame streaming.
// Each slot carries: [u32 payload_size][TCP frame bytes][padding].
// 256KB per slot covers up to ~50K motion pixels encoded.
DATAFIFO_SLOT_SIZE  :: u32(256 * 1024)
DATAFIFO_SLOT_COUNT :: u32(4)

// Cap motion pixels to fit within a DATAFIFO slot. At ~5 bytes per
// encoded pixel plus ~108 bytes header plus 8 bytes framing, 50K
// pixels = ~250KB which fits in a 256KB slot.
MAX_DATAFIFO_PIXELS :: u32(50000)

// =========================================================================
// Foreign function declarations — mpi_vb_api.h, mpi_vicap_api.h,
// mpi_sys_api.h, k_datafifo.h.
//
// Linked via the build script against the MPP static libraries.
// The "system:c" import is a placeholder; actual resolution happens
// at link time with -lvicap -lsensor -lvb -lsys -ldatafifo etc.
// =========================================================================

foreign import mpp_lib "system:c"

@(default_calling_convention = "c")
foreign mpp_lib {
	// VB (Video Buffer).
	kd_mpi_vb_set_config :: proc(config: ^K_VB_Config) -> i32 ---
	kd_mpi_vb_init       :: proc() -> i32 ---
	kd_mpi_vb_exit       :: proc() -> i32 ---

	// VICAP.
	kd_mpi_vicap_get_sensor_info :: proc(
		sensor_type: i32,
		sensor_info: ^K_VICAP_Sensor_Info,
	) -> i32 ---

	kd_mpi_vicap_set_dev_attr :: proc(
		dev:      i32,
		dev_attr: K_VICAP_Dev_Attr,
	) -> i32 ---

	kd_mpi_vicap_set_chn_attr :: proc(
		dev:      i32,
		chn:      i32,
		chn_attr: K_VICAP_Chn_Attr,
	) -> i32 ---

	kd_mpi_vicap_init         :: proc(dev: i32) -> i32 ---
	kd_mpi_vicap_deinit       :: proc(dev: i32) -> i32 ---
	kd_mpi_vicap_start_stream :: proc(dev: i32) -> i32 ---
	kd_mpi_vicap_stop_stream  :: proc(dev: i32) -> i32 ---

	kd_mpi_vicap_dump_frame :: proc(
		dev:        i32,
		chn:        i32,
		format:     i32,
		frame_info: ^K_Video_Frame_Info,
		timeout_ms: u32,
	) -> i32 ---

	kd_mpi_vicap_dump_release :: proc(
		dev:        i32,
		chn:        i32,
		frame_info: ^K_Video_Frame_Info,
	) -> i32 ---

	// System (mmap). Use mmap_cached + flush_cache for performance:
	// uncached reads from DRAM are ~10x slower than cached.
	kd_mpi_sys_mmap        :: proc(phys_addr: u64, size: u32) -> rawptr ---
	kd_mpi_sys_mmap_cached :: proc(phys_addr: u64, size: u32) -> rawptr ---
	kd_mpi_sys_munmap      :: proc(virt_addr: rawptr, size: u32) -> i32 ---
	kd_mpi_sys_mmz_flush_cache :: proc(
		phys_addr: u64, virt_addr: rawptr, size: u32,
	) -> i32 ---

	// DATAFIFO.
	kd_datafifo_open :: proc(
		handle: ^K_Datafifo_Handle,
		params: ^K_Datafifo_Params,
	) -> i32 ---

	kd_datafifo_open_by_addr :: proc(
		handle:    ^K_Datafifo_Handle,
		params:    ^K_Datafifo_Params,
		phys_addr: u64,
	) -> i32 ---

	kd_datafifo_close :: proc(handle: K_Datafifo_Handle) -> i32 ---

	kd_datafifo_read :: proc(
		handle: K_Datafifo_Handle,
		data:   ^rawptr,
	) -> i32 ---

	kd_datafifo_write :: proc(
		handle: K_Datafifo_Handle,
		data:   rawptr,
	) -> i32 ---

	kd_datafifo_cmd :: proc(
		handle: K_Datafifo_Handle,
		cmd:    i32,
		arg:    rawptr,
	) -> i32 ---

	// Timing (for performance measurement).
	clock_gettime :: proc(clockid: i32, tp: ^Timespec) -> i32 ---

	// Signal handling (for clean shutdown on Ctrl+C).
	signal :: proc(signum: i32, handler: rawptr) -> rawptr ---

	// libc (for output and file I/O on RT-Smart).
	printf  :: proc(fmt_str: cstring, #c_vararg args: ..any) -> i32 ---
	snprintf :: proc(buf: [^]u8, size: uint, fmt_str: cstring,
		#c_vararg args: ..any) -> i32 ---
	_exit   :: proc(status: i32) -> ! ---
	@(link_name = "open")
	posix_open :: proc(path: cstring, flags: i32, #c_vararg args: ..any) -> i32 ---
	@(link_name = "write")
	posix_write :: proc(fd: i32, buf: rawptr, count: uint) -> int ---
	@(link_name = "close")
	posix_close :: proc(fd: i32) -> i32 ---
}

// POSIX open() flags (C preprocessor macros, not symbols).
POSIX_O_WRONLY :: i32(1)
POSIX_O_CREAT  :: i32(64)
POSIX_O_TRUNC  :: i32(512)

// clock_gettime clock IDs.
CLOCK_REALTIME  :: i32(0)
CLOCK_MONOTONIC :: i32(1)
SIGINT  :: i32(2)
SIGTERM :: i32(15)

Timespec :: struct {
	tv_sec:  i64,
	tv_nsec: i64,
}

// Get elapsed microseconds between two timespecs.
timespec_diff_us :: proc(start: Timespec, end_: Timespec) -> i64 {
	sec_diff := end_.tv_sec - start.tv_sec
	nsec_diff := end_.tv_nsec - start.tv_nsec
	return sec_diff * 1_000_000 + nsec_diff / 1_000
}

// Get current wall-clock time as microseconds since Unix epoch.
// Uses CLOCK_REALTIME for absolute timestamps that the server can
// correlate across multiple cameras.
get_timestamp_us :: proc() -> u64 {
	ts: Timespec
	clock_gettime(CLOCK_REALTIME, &ts)
	return u64(ts.tv_sec) * 1_000_000 + u64(ts.tv_nsec / 1000)
}

// =========================================================================
// Compile-time layout assertions.
//
// These verify that Odin's struct layout matches the C layout on
// RISC-V 64-bit LP64D. If any assertion fires, the struct padding
// is wrong and VICAP calls will corrupt memory.
// =========================================================================

#assert(size_of(K_VB_Pool_Config)     == 32)
#assert(size_of(K_VB_Config)          == 520)
#assert(size_of(K_VICAP_Window)       == 8)
#assert(size_of(K_VICAP_Sensor_Info)  == 64)
#assert(size_of(K_VICAP_Dev_Attr)     == 120)
#assert(size_of(K_VICAP_Chn_Attr)     == 52)
#assert(size_of(K_Video_Supplement)   == 32)
#assert(size_of(K_Video_Frame)        == 216)
#assert(size_of(K_Video_Frame_Info)   == 224)
