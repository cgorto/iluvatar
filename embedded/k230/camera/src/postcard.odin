// Minimal postcard binary encoder/decoder.
//
// Postcard is a compact serde-based format used by the Rust server.
// Integers (except u8/i8) use unsigned LEB128 varint encoding.
// Floats are raw little-endian bytes. Structs are sequential fields
// in declaration order. Enums are a varint discriminant followed by
// variant fields. Vecs have a varint length prefix. Options are a
// u8 tag (0=None, 1=Some) followed by the value.
//
// We only implement the subset needed for the Iluvatar protocol:
// encode u8, bool, varint u16/u32/u64, f32 LE, f64 LE, and their
// decode counterparts plus Option<f32> and Option<u32>.

package odin_camera

// Both x86_64 (desktop) and riscv64 (K230) are little-endian.
// Postcard stores floats as raw LE bytes, so transmute is correct.
#assert(ODIN_ENDIAN == .Little)

// --- Encoder ----------------------------------------------------------------

Encoder :: struct {
	buffer: []u8,
	offset: u32,
}

encoder_reset :: proc(encoder: ^Encoder) {
	assert(encoder != nil)
	assert(encoder.buffer != nil)
	encoder.offset = 0
}

// Returns the encoded bytes as a slice.
encoder_bytes :: proc(encoder: ^Encoder) -> []u8 {
	assert(encoder != nil)
	assert(encoder.offset <= u32(len(encoder.buffer)))
	return encoder.buffer[:encoder.offset]
}

encode_u8 :: proc(encoder: ^Encoder, value: u8) {
	assert(encoder.offset < u32(len(encoder.buffer)))
	encoder.buffer[encoder.offset] = value
	encoder.offset += 1
}

encode_bool :: proc(encoder: ^Encoder, value: bool) {
	encode_u8(encoder, value ? 1 : 0)
}

// Unsigned LEB128 varint. Each byte carries 7 data bits; MSB is
// the continuation flag. Worst case: 10 bytes for u64.
encode_varint_u16 :: proc(encoder: ^Encoder, value: u16) {
	encode_varint_u64(encoder, u64(value))
}

encode_varint_u32 :: proc(encoder: ^Encoder, value: u32) {
	encode_varint_u64(encoder, u64(value))
}

encode_varint_u64 :: proc(encoder: ^Encoder, value: u64) {
	assert(encoder.offset + 10 <= u32(len(encoder.buffer)))
	v := value
	for {
		byte := u8(v & 0x7F)
		v >>= 7
		if v != 0 do byte |= 0x80
		encoder.buffer[encoder.offset] = byte
		encoder.offset += 1
		if v == 0 do break
	}
}

encode_f32 :: proc(encoder: ^Encoder, value: f32) {
	assert(encoder.offset + 4 <= u32(len(encoder.buffer)))
	bytes := transmute([4]u8)value
	copy(encoder.buffer[encoder.offset:][:4], bytes[:])
	encoder.offset += 4
}

encode_f64 :: proc(encoder: ^Encoder, value: f64) {
	assert(encoder.offset + 8 <= u32(len(encoder.buffer)))
	bytes := transmute([8]u8)value
	copy(encoder.buffer[encoder.offset:][:8], bytes[:])
	encoder.offset += 8
}

// --- Decoder ----------------------------------------------------------------

Decoder :: struct {
	data:   []u8,
	offset: u32,
	ok:     bool, // Cleared on any decode error (malformed network data).
}

decoder_init :: proc(decoder: ^Decoder, data: []u8) {
	assert(decoder != nil)
	assert(data != nil)
	decoder.data = data
	decoder.offset = 0
	decoder.ok = true
}

decode_u8 :: proc(decoder: ^Decoder) -> u8 {
	if !decoder.ok do return 0
	if decoder.offset >= u32(len(decoder.data)) {
		decoder.ok = false
		return 0
	}
	value := decoder.data[decoder.offset]
	decoder.offset += 1
	return value
}

decode_bool :: proc(decoder: ^Decoder) -> bool {
	return decode_u8(decoder) != 0
}

decode_varint_u32 :: proc(decoder: ^Decoder) -> u32 {
	return u32(decode_varint_u64(decoder))
}

decode_varint_u64 :: proc(decoder: ^Decoder) -> u64 {
	if !decoder.ok do return 0
	result: u64 = 0
	shift: u32 = 0
	for _ in 0 ..< 10 {
		if decoder.offset >= u32(len(decoder.data)) {
			decoder.ok = false
			return 0
		}
		byte := decoder.data[decoder.offset]
		decoder.offset += 1
		result |= u64(byte & 0x7F) << shift
		if byte & 0x80 == 0 do return result
		shift += 7
	}
	// More than 10 continuation bytes: malformed.
	decoder.ok = false
	return 0
}

decode_f32 :: proc(decoder: ^Decoder) -> f32 {
	if !decoder.ok do return 0
	if decoder.offset + 4 > u32(len(decoder.data)) {
		decoder.ok = false
		return 0
	}
	bytes: [4]u8
	copy(bytes[:], decoder.data[decoder.offset:][:4])
	decoder.offset += 4
	return transmute(f32)bytes
}

decode_f64 :: proc(decoder: ^Decoder) -> f64 {
	if !decoder.ok do return 0
	if decoder.offset + 8 > u32(len(decoder.data)) {
		decoder.ok = false
		return 0
	}
	bytes: [8]u8
	copy(bytes[:], decoder.data[decoder.offset:][:8])
	decoder.offset += 8
	return transmute(f64)bytes
}

decode_option_f32 :: proc(decoder: ^Decoder) -> (value: f32, present: bool) {
	tag := decode_u8(decoder)
	if !decoder.ok do return 0, false
	if tag == 0 do return 0, false
	if tag != 1 {
		decoder.ok = false
		return 0, false
	}
	return decode_f32(decoder), true
}

decode_option_u32 :: proc(decoder: ^Decoder) -> (value: u32, present: bool) {
	tag := decode_u8(decoder)
	if !decoder.ok do return 0, false
	if tag == 0 do return 0, false
	if tag != 1 {
		decoder.ok = false
		return 0, false
	}
	return decode_varint_u32(decoder), true
}
