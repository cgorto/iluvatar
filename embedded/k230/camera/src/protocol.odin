// Iluvatar protocol: message encoding and decoding.
//
// Encoding: Odin camera → Rust server (CameraMessage variants).
// Decoding: Rust server → Odin camera (ServerMessage variants).
//
// Wire format: 4-byte big-endian length prefix + postcard payload.
// Enum discriminants must match Rust declaration order exactly.

package odin_camera

// CameraMessage discriminants (must match Rust enum order).
CAMERA_MSG_REGISTER  :: 0
CAMERA_MSG_FRAME     :: 1
CAMERA_MSG_HEARTBEAT :: 2
CAMERA_MSG_TIMESYNC  :: 3
CAMERA_MSG_MOTION    :: 4

// ServerMessage discriminants.
SERVER_MSG_REGISTERED       :: 0
SERVER_MSG_GRID_CONFIG      :: 1
SERVER_MSG_ERROR            :: 2
SERVER_MSG_REGISTERED_PREFS :: 3

// MotionData discriminants.
MOTION_DATA_SPARSE     :: 0
MOTION_DATA_RUN_LENGTH :: 1

// --- Encode: Camera → Server ------------------------------------------------

encode_camera_registration :: proc(
	encoder: ^Encoder,
	registration: ^Camera_Registration,
) {
	assert(encoder != nil)
	assert(registration != nil)
	assert(registration.version == PROTOCOL_VERSION)

	encode_varint_u64(encoder, CAMERA_MSG_REGISTER)

	// CameraRegistration fields in Rust declaration order.
	encode_u8(encoder, registration.version)
	encode_varint_u64(encoder, registration.camera_id)
	encode_intrinsics(encoder, &registration.intrinsics)
	encode_pose(encoder, &registration.initial_pose)
	encode_capabilities(encoder, &registration.capabilities)
}

encode_motion_frame :: proc(
	encoder: ^Encoder,
	camera_id: u64,
	sequence: u64,
	timestamp: u64,
	pose: ^Camera_Pose,
	pixels: []Motion_Pixel,
) {
	assert(encoder != nil)
	assert(pose != nil)

	encode_varint_u64(encoder, CAMERA_MSG_MOTION)

	// MotionFrame fields.
	encode_varint_u64(encoder, camera_id)
	encode_varint_u64(encoder, sequence)
	encode_varint_u64(encoder, timestamp)
	encode_pose(encoder, pose)

	// MotionData::Sparse(Vec<MotionPixel>).
	encode_varint_u64(encoder, MOTION_DATA_SPARSE)
	encode_varint_u64(encoder, u64(len(pixels)))
	for i in 0 ..< len(pixels) {
		encode_varint_u16(encoder, pixels[i].x)
		encode_varint_u16(encoder, pixels[i].y)
		encode_u8(encoder, pixels[i].intensity)
	}
}

encode_heartbeat :: proc(
	encoder: ^Encoder,
	camera_id: u64,
	timestamp: u64,
) {
	assert(encoder != nil)

	encode_varint_u64(encoder, CAMERA_MSG_HEARTBEAT)
	encode_varint_u64(encoder, camera_id)
	encode_varint_u64(encoder, timestamp)
}

// --- Encode helpers ---------------------------------------------------------

encode_intrinsics :: proc(encoder: ^Encoder, intrinsics: ^Camera_Intrinsics) {
	assert(intrinsics != nil)

	// Vec2 focal_length: (x, y) as sequential f32.
	encode_f32(encoder, intrinsics.focal_length[0])
	encode_f32(encoder, intrinsics.focal_length[1])

	// Vec2 principal_point.
	encode_f32(encoder, intrinsics.principal_point[0])
	encode_f32(encoder, intrinsics.principal_point[1])

	// UVec2 resolution: (width, height) as varint u32.
	encode_varint_u32(encoder, intrinsics.resolution[0])
	encode_varint_u32(encoder, intrinsics.resolution[1])

	// Fov.
	encode_f32(encoder, intrinsics.fov.horizontal)
	encode_f32(encoder, intrinsics.fov.vertical)

	// DistortionModel enum.
	encode_varint_u64(encoder, u64(intrinsics.distortion))
}

encode_pose :: proc(encoder: ^Encoder, pose: ^Camera_Pose) {
	assert(pose != nil)

	// GeoPosition: lat, lon, alt as f64.
	encode_f64(encoder, pose.position.latitude)
	encode_f64(encoder, pose.position.longitude)
	encode_f64(encoder, pose.position.altitude)

	// Quat: x, y, z, w as sequential f32.
	encode_f32(encoder, pose.orientation[0])
	encode_f32(encoder, pose.orientation[1])
	encode_f32(encoder, pose.orientation[2])
	encode_f32(encoder, pose.orientation[3])

	// Timestamp: varint u64.
	encode_varint_u64(encoder, pose.timestamp)

	// PoseUncertainty: position_stddev (Vec3), orientation_stddev (Vec3).
	encode_f32(encoder, pose.uncertainty.position_stddev[0])
	encode_f32(encoder, pose.uncertainty.position_stddev[1])
	encode_f32(encoder, pose.uncertainty.position_stddev[2])
	encode_f32(encoder, pose.uncertainty.orientation_stddev[0])
	encode_f32(encoder, pose.uncertainty.orientation_stddev[1])
	encode_f32(encoder, pose.uncertainty.orientation_stddev[2])

	// LocalizationStatus enum.
	encode_varint_u64(encoder, u64(pose.status))
	// Note: DeadReckoning has a duration_ms field, but we only
	// send Nominal from the camera, so we skip encoding it.
}

encode_capabilities :: proc(
	encoder: ^Encoder,
	capabilities: ^Camera_Capabilities,
) {
	assert(capabilities != nil)

	encode_bool(encoder, capabilities.motion_frames)
	encode_bool(encoder, capabilities.rle_encoding)
	encode_varint_u32(encoder, capabilities.flags)
}

// --- Decode: Server → Camera ------------------------------------------------

// Decoded server message tag.
Server_Message_Kind :: enum u8 {
	Registered_With_Prefs = 0,
	Grid_Config           = 1,
	Unknown               = 2,
}

// Decode a server message. Returns the kind and populates the
// appropriate field in the response struct.
decode_server_message :: proc(
	data: []u8,
	response: ^Server_Response,
) -> (kind: Server_Message_Kind, ok: bool) {
	assert(response != nil)
	assert(data != nil)

	decoder: Decoder
	decoder_init(&decoder, data)

	discriminant := decode_varint_u64(&decoder)
	if !decoder.ok do return .Unknown, false

	switch discriminant {
	case SERVER_MSG_REGISTERED_PREFS:
		response.camera_id = decode_varint_u64(&decoder)
		decode_preferences(&decoder, &response.preferences)
		return .Registered_With_Prefs, decoder.ok

	case SERVER_MSG_GRID_CONFIG:
		decode_grid_config(&decoder, &response.grid_config)
		return .Grid_Config, decoder.ok

	case:
		return .Unknown, false
	}
}

decode_preferences :: proc(decoder: ^Decoder, prefs: ^Server_Preferences) {
	assert(prefs != nil)

	// FrameFormat enum discriminant.
	format := decode_varint_u64(decoder)
	if format <= u64(Frame_Format.Motion_Pixels) {
		prefs.preferred_format = Frame_Format(format)
	} else {
		decoder.ok = false
		return
	}

	// Option<f32> target_fps.
	prefs.target_fps, prefs.target_fps_present = decode_option_f32(decoder)

	// Option<u32> max_motion_pixels.
	prefs.max_motion_pixels, prefs.max_motion_pixels_present =
		decode_option_u32(decoder)
}

decode_grid_config :: proc(decoder: ^Decoder, config: ^Grid_Config) {
	assert(config != nil)

	config.origin_lat = decode_f64(decoder)
	config.origin_lon = decode_f64(decoder)
	config.origin_alt = decode_f64(decoder)

	// [u32; 3]: fixed-size array, no length prefix.
	config.dimensions[0] = decode_varint_u32(decoder)
	config.dimensions[1] = decode_varint_u32(decoder)
	config.dimensions[2] = decode_varint_u32(decoder)

	config.voxel_size = decode_f32(decoder)
}

// --- Framing ----------------------------------------------------------------

// Write a 4-byte big-endian length prefix before the payload.
write_frame_header :: proc(buffer: []u8, payload_length: u32) {
	assert(len(buffer) >= 4)
	buffer[0] = u8(payload_length >> 24)
	buffer[1] = u8(payload_length >> 16)
	buffer[2] = u8(payload_length >> 8)
	buffer[3] = u8(payload_length)
}

// Parse a 4-byte big-endian length prefix.
read_frame_length :: proc(header: [4]u8) -> u32 {
	return u32(header[0]) << 24 |
	       u32(header[1]) << 16 |
	       u32(header[2]) << 8 |
	       u32(header[3])
}
