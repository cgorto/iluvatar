# K230 Bringup Notes

## Hardware

- **Board**: CanMV-K230 (youyeetoo variant, K230 V1.0/1.1)
- **SoC**: Kendryte K230 — dual RISC-V cores (XuanTie C908), KPU neural accelerator
- **ISA**: `rv64imafdcvxthead` (standard rv64gc + T-Head vendor extensions)
- **Camera**: OV5647 (on-board, MIPI CSI-2, 2 lanes, 800MHz PHY)
- **SD Card**: 64GB microSD (only ~500MB partitioned — needs expansion)

## Host Setup

- **Host OS**: Bazzite 43 (Fedora Atomic / Kinoite)
- **Dev environment**: Arch Linux distrobox
- **USB serial**: vendor `1a86`, product `55d2`
- **Serial devices**: `/dev/ttyACM1` and `/dev/ttyACM2` (numbering shifts; `ttyACM0` sometimes taken)
- **Serial config**: 115200 8N1, connect with `screen /dev/ttyACM1 115200`
- **udev rule** (on host, not distrobox): `/etc/udev/rules.d/99-k230.rules`
  ```
  SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d2", MODE="0666"
  ```
- **Board IP** (DHCP): `192.168.0.190`
- **Host IP**: `192.168.0.185`
- **Login**: `root`, no password

## Firmware

### Original: K230 SDK v2.0 (Linux + RT-Smart dual system)
- Image: `CanMV-K230_sdcard_v2.0_nncase_v2.10.0.img`
- Downloaded from: `https://kendryte-download.canaan-creative.com/k230/release/sdk_images/v2.0/k230_canmv_defconfig/`
- Kernel: Linux 5.10.4, riscv64
- RAM available to Linux: **103MB** (rest allocated to RT-Smart big core)
- Root fs: 120MB (42MB free), `/sharefs`: 256MB FAT32 (164MB free)
- **Camera NOT accessible from Linux** — `/dev/video*` does not exist; camera hardware owned by RT-Smart core
- Rust hello-world and iluvatar-camera both ran successfully on this image

### Current: K230 Linux SDK v0.6.9 (Linux on both cores)
- Image: `CanMV-K230_V1P0_P1_linux_v0.6.9_nncase_v2.10.0.img`
- Downloaded from: `https://kendryte-download.canaan-creative.com/k230/release/linux_sdk_images/daily_build/` (daily build page, JS-rendered)
- MD5: `721eba85491ef35c73c41b5678a7eee0`
- Kernel: **Linux 6.6.36**, riscv64
- RAM available: **468MB** (4.5x more than dual-system image)
- Camera accessible: `/dev/video0` through `/dev/video4` present
- Has `isp_media_server`, `camera_rtsp_demo`, face detection demos

## Flashing

From host (Bazzite terminal, NOT distrobox):
```bash
sudo dd if=<image.img> of=/dev/sdb bs=1M oflag=sync
```
- `/dev/sdb` is the SD card (verify with `lsblk` — `/dev/sda` is the internal drive)
- After flashing, partition table only uses ~500MB of the 64GB card (GPT needs expanding)

## Boot Sequence

1. Insert SD card, plug USB-C power
2. Red LED = powered on
3. Connect serial: `screen /dev/ttyACM1 115200`
4. **DO NOT press Ctrl+C during boot** — drops into U-Boot shell (`K230#` prompt). Type `boot` to continue if this happens.
5. Boot takes ~30 seconds, watch for `udhcpc` lease message
6. Login prompt: `canaan login:` → enter `root`
7. DHCP only works if ethernet cable is plugged in before/during boot

## Cross-Compilation

### Toolchain
- Rust target: `riscv64gc-unknown-linux-gnu`
- Cross-linker: `riscv64-linux-gnu-gcc` (Arch package: `riscv64-linux-gnu-gcc`)
- Cross sysroot: `/usr/riscv64-linux-gnu` (has V4L2 headers at `include/linux/videodev2.h`)

### Cargo config (`.cargo/config.toml`)
```toml
[target.riscv64gc-unknown-linux-gnu]
linker = "riscv64-linux-gnu-gcc"
```

### CRITICAL: Static linking required
The Buildroot rootfs has libraries in `lib64/lp64d/` and `lib64xthead/`, NOT in `/lib`. The dynamic linker path (`/lib/ld-linux-riscv64-lp64d.so.1`) in Rust binaries won't resolve. **Always use static linking**:
```bash
RUSTFLAGS="-C target-feature=+crt-static"
```
- Dynamically linked binary: "not found" error (misleading — it's the dynamic linker that's not found)
- Statically linked hello-world: ~1MB
- Statically linked iluvatar-camera: ~5.6MB

### Build commands
```bash
# Simple build (sim mode, no V4L2)
RUSTFLAGS="-C target-feature=+crt-static" cargo build -p iluvatar-camera \
  --target riscv64gc-unknown-linux-gnu --release

# With V4L2 support (needs cross sysroot for bindgen)
RUSTFLAGS="-C target-feature=+crt-static" \
BINDGEN_EXTRA_CLANG_ARGS="--sysroot=/usr/riscv64-linux-gnu --target=riscv64-linux-gnu" \
cargo build -p iluvatar-camera --target riscv64gc-unknown-linux-gnu \
  --release --features real --no-default-features
```

### Deploying to board
```bash
scp target/riscv64gc-unknown-linux-gnu/release/iluvatar-camera root@192.168.0.190:/root/
```
If host key changes after reflash: `ssh-keygen -R 192.168.0.190`

## What Works

1. **Rust cross-compilation** — hello-world and full iluvatar-camera binary run on K230
2. **QUIC networking** — camera connects to server over local network, registers, receives grid config
3. **Dummy camera pipeline** — full main loop at 30 FPS with dummy frames (no motion detected, so `frames_sent=0`)
4. **Server sees the K230** — `Camera registered camera_id=1 remote=192.168.0.190:54310`
5. **`camera_rtsp_demo`** — successfully captures 1280x720 NV12 from the OV5647 and streams H.264 over RTSP at `rtsp://192.168.0.190:8554/test`
6. **Real V4L2 camera pipeline** — iluvatar-camera captures NV12 @ 1280x720 from `/dev/video1`, runs frame differencing, raymarching, and sends voxel contributions to server over QUIC. Runs at ~8.4 fps at 10fps target. Uses Rust `v4l` crate (no raw ioctls needed).

## Camera / V4L2 Status (WORKING)

### Media Pipeline Topology
```
OV5647 sensor (MIPI CSI-2, 2 lanes, 800MHz)
  → vvcam_mipi.ko (MIPI CSI receiver)
    → vvcam_isp.ko (Verisilicon ISP at MMIO 0x90000000)
      → vvcam_video.ko (V4L2 video device nodes)

/dev/video0 = Linlon Video device (hardware H.264/MJPEG codec, M2M)
/dev/video1 = vvcam-video.0.0 (ISP output pad 1) ← main capture
/dev/video2 = vvcam-video.0.1 (ISP output pad 2)
/dev/video3 = vvcam-video.0.2 (ISP output pad 3)
/dev/video4 = vvcam-video.0.3 (ISP output pad 4)
/dev/v4l-subdev0 = vvcam-isp-subdev.0 (ISP subdevice, 20 pads)
/dev/media0 = verisilicon_media (media controller)
```

### Critical requirement: `isp_media_server` daemon
The V4L2 devices **require** a userspace daemon (`/usr/bin/isp_media_server`) to function. Without it, V4L2 ioctls return EINVAL. It is started by `/etc/init.d/S99canaanboot`:
```bash
modprobe vvcam_isp
modprobe vvcam_mipi
modprobe vvcam_vb
modprobe vvcam_isp_subdev
modprobe vvcam_video
ISP_MEDIA_SENSOR_DRIVER=/usr/lib/libvvcam.so /usr/bin/isp_media_server > /dev/null 2> /tmp/isp.err.log &
```
`isp_media_server` is a **closed-source 2.3MB prebuilt binary**. It:
- Detects the sensor (OV5647) and programs registers over I2C (`/dev/i2c-0`, address `0x36`)
- Configures MIPI CSI-2 (2 lanes, 800MHz)
- Initializes ISP with calibration from `/etc/vvcam/`
- Runs 3A control loop (AE/AWB/AF) continuously
- Makes V4L2 video nodes functional
- Loads sensor drivers from `libvvcam.so` (open-source, contains OV5647/IMX335/GC2093 drivers)

### ISP-configured default format
When `isp_media_server` is running, `/dev/video1` reports:
- Resolution: 1920x1080
- Format: NV16 (YUV 4:2:2 semi-planar)

### Historical: what we tried before the fix

| Attempt | Format | Resolution | Result |
|---------|--------|------------|--------|
| v4l2-ctl capture | YUYV | 640x480 | Silent failure, no frames captured |
| Rust v4l crate (YUYV) | YUYV | 640x480 | V4L2 init succeeded, but `stream.next()` hung indefinitely (no frames delivered) |
| Rust v4l crate (NV12) | NV12 | 640x480 | **ISP CRASHED** — kernel disabled IRQs 102/103, store page fault, required reboot |
| Rust v4l crate (ISP default) | NV16 | 1920x1080 | EINVAL during stream setup (REQBUFS or STREAMON), fell back to dummy |
| `camera_rtsp_demo` (FFmpeg) | NV12 | 1280x720 | **SUCCESS** — captures and streams H.264 over RTSP |
| Face detection demo | BGR planar | 1280x720 | **SUCCESS** — uses `libv4l2-drm.so` |

### Root cause and fix

The bug was in `capture.rs`: calling `stream.start()` (STREAMON) explicitly before `stream.next()`.
The v4l crate's `next()` method handles initialization internally — it queues all buffers
first, then calls STREAMON. Calling `start()` manually set `active = true`, so `next()`
skipped the buffer-queue step. The ISP got STREAMON with zero buffers queued, captured
one frame into the single post-STREAMON QBUF, then hung because it had no more buffers.

We confirmed via C test programs on the board that:
- `O_NONBLOCK` vs blocking: **doesn't matter** (both work)
- `VIDIOC_EXPBUF`: **not required**
- Buffer count 4 vs 5: **doesn't matter**
- `VIDIOC_S_FMT` to set NV12: **required** (ISP default NV16 works but we need NV12 for our conversion code)

The fix:
1. **Removed explicit `stream.start()` call** — let `next()` handle buffer queuing + STREAMON
2. **Always call `S_FMT`** — explicitly set NV12 at the configured resolution instead of trusting ISP defaults
3. **Added 30-frame warmup** — discard initial frames while ISP auto-exposure stabilizes, prevents 100% motion diff on first real frame pair

### Key source references (k230_linux_sdk repo)
- Boot script: `buildroot-overlay/board/canaan/k230-soc/rootfs_overlay/etc/init.d/S99canaanboot`
- v4l2-drm library: `buildroot-overlay/package/vvcam/v4l2-drm/src/lib.c`
- Sensor drivers: `buildroot-overlay/package/vvcam/src/ov5647.c`
- ISP V4L2 driver: `buildroot-overlay/package/vvcam/v4l2/video/`
- AI demo using camera: `buildroot-overlay/package/ai_demo/face_detection/main.cc`

## TODO

### Later
- **Expand SD card partitions** to use full 64GB (GPT backup header at ~512MB, needs `gdisk` fix)
- **Set system clock** — board has no RTC battery, timestamps show 1970
- **Camera calibration** — OV5647 intrinsics for accurate ray projection
- **V4L2 feature flag for K230** — may want a `k230` feature that uses raw ioctls instead of the `v4l` crate
- **Performance profiling** — 468MB RAM is workable but tight for 1920x1080 processing at 30fps
- **NTP or GPS time sync** — need accurate timestamps for multi-camera frame synchronization
- **Try different video device** — `/dev/video2` through `/dev/video4` might behave differently
- **Check `strace`** on `camera_rtsp_demo` to see exact V4L2 ioctl sequence
