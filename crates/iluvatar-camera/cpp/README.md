# K230 VICAP Camera Shim

This directory contains a C++ shim that wraps the K230 SDK's VICAP API for
hardware-accelerated camera capture. It provides a simple C interface for
FFI with Rust.

## Why not V4L2?

While V4L2 works on the K230, it involves userspace buffer copies that limit
performance to ~8 FPS at 1280x720. The VICAP API provides direct access to
ISP output buffers, potentially achieving 30+ FPS.

## Files

- `k230_capture.h` - C FFI header
- `k230_capture.cpp` - VICAP wrapper implementation
- `k230_sdk/` - Stub SDK headers (replace with real headers from board)

## Important: K230 Linux SDK vs RT-Smart SDK

**Finding:** The K230 Linux SDK v0.6.9 image does NOT include the VICAP MPI
libraries. It uses V4L2 through the ISP kernel driver instead. The `camera_rtsp_demo`
and other camera applications on this image link against `libv4l2.so` and FFmpeg.

The VICAP MPI libraries (`libmpi_vb`, `libmpi_vicap`, `libmpi_sys`) are part of the
**RT-Smart SDK** (the dual-system image where Linux runs on the small core and
RT-Smart runs on the big core).

### Options for K230 Performance Improvement

1. **Stick with V4L2 (Current)**: The V4L2 backend works at ~8 FPS. Consider
   optimizing at the Rust level (zero-copy, async, parallel processing).

2. **Use RT-Smart SDK Image**: Flash the RT-Smart dual-system image which has
   the VICAP MPI libraries. However, this gives Linux only ~103MB RAM.

3. **Build Custom Linux SDK**: Build the K230 Linux SDK from source with MPP
   support enabled. This requires setting up the Buildroot environment.

4. **libv4l2-drm Approach**: The `face_detection` demo uses `libv4l2-drm.so`
   which may provide better performance than standard V4L2 by using DRM buffers.

### If You Have VICAP MPI Libraries

If you're using an image that has the VICAP MPI libraries (e.g., RT-Smart image
or custom build), you can extract the headers and libraries:

**From the Board:**
```bash
# SSH to the K230 board
ssh root@192.168.0.190

# Find the MPP headers and libraries
find /usr -name "*vicap*" -o -name "*mpi*" 2>/dev/null

# Copy headers (if present):
scp -r root@192.168.0.190:/usr/include/mpp ./k230_sdk/

# Copy libraries:
scp root@192.168.0.190:/usr/lib/libmpi_*.a ./libs/
```

**From K230 SDK Source:**
```bash
# Headers are typically in:
# buildroot-overlay/package/mpp_sdk/include/

cp -r /path/to/k230_sdk/buildroot-overlay/package/mpp_sdk/include/* ./k230_sdk/
```

### Using Stub Headers

The stub headers in `k230_sdk/` provide minimal definitions for compilation.
They allow building on the host and will be replaced by real SDK implementations
when linking against the actual K230 libraries.

## Building

### Development (Host, Stub Mode)

```bash
# Build with k230 feature (uses stubs, won't actually work)
cargo build -p iluvatar-camera --features k230 --no-default-features
```

### Cross-Compilation for K230

```bash
# Basic build
RUSTFLAGS="-C target-feature=+crt-static" \
cargo build -p iluvatar-camera \
    --target riscv64gc-unknown-linux-gnu \
    --release --features k230 --no-default-features

# With K230 SDK libraries (if you have them)
K230_SDK_LIB_PATH=/path/to/k230/libs \
RUSTFLAGS="-C target-feature=+crt-static" \
cargo build -p iluvatar-camera \
    --target riscv64gc-unknown-linux-gnu \
    --release --features k230 --no-default-features
```

### With V4L2 Fallback

```bash
# Enable both k230 and real features for V4L2 fallback
RUSTFLAGS="-C target-feature=+crt-static" \
BINDGEN_EXTRA_CLANG_ARGS="--sysroot=/usr/riscv64-linux-gnu --target=riscv64-linux-gnu" \
cargo build -p iluvatar-camera \
    --target riscv64gc-unknown-linux-gnu \
    --release --features k230,real --no-default-features
```

## Deployment

1. Copy the binary to the K230:
   ```bash
   scp target/riscv64gc-unknown-linux-gnu/release/iluvatar-camera root@192.168.0.190:/root/
   ```

2. Copy the K230 config:
   ```bash
   scp k230/camera-k230.toml root@192.168.0.190:/root/camera.toml
   ```

3. Run on the board:
   ```bash
   ssh root@192.168.0.190
   ./iluvatar-camera camera.toml
   ```

## Troubleshooting

### "undefined reference to kd_mpi_*"

The K230 SDK libraries are not linked. Either:
- Set `K230_SDK_LIB_PATH` to the directory containing the SDK libs
- Ensure the SDK libraries are in the board's `/usr/lib/`

### "VB init failed" or "VICAP init failed"

The VICAP API may conflict with `isp_media_server`. Try:
```bash
# Stop isp_media_server (may break V4L2)
killall isp_media_server

# Or check if it's already using the camera
ps aux | grep isp
```

### Performance Issues

Check that you're using the VICAP backend:
```
K230 VICAP camera initialized
```

If you see "V4L2 camera" or "dummy camera", the VICAP backend isn't active.
Verify `device = "k230"` in your config.

## API Reference

See `k230_capture.h` for the complete API documentation.
