# K230 dual-core camera unit

This directory contains the validated embedded half of Iluvatar for the CanMV-K230
dual-system image:

```text
OV5647 → VICAP/ISP → RT-Smart Odin process → DATAFIFO
       → little-core Linux C process → TCP → Iluvatar server
```

The implementation was tested against K230 SDK image
`v2.0-20250912-105454-gitlab-runner-75beab9`. The SDK ABI and firmware container are
vendor-specific; other images may require changes.

## Layout

```text
camera/   Odin capture, motion processing, protocol encoding, DATAFIFO writer
reader/   Linux DATAFIFO reader and latest-frame TCP forwarder
runtime/  RT-Smart entry point, C runtime bridge, and linker script
tools/    RT firmware image patcher used to bypass broken ShareFS startup
```

## External inputs

The repository does not redistribute the K230 toolchain, MPP static libraries, or
firmware. Obtain them from the matching
[K230 SDK](https://github.com/kendryte/k230_sdk) release.

The build expects:

- Odin on `PATH`.
- Kendryte's RT-Smart RISC-V musl toolchain.
- A RISC-V glibc cross-compiler for the little Linux core.
- RT-Smart MPP static libraries, including VICAP, ISP, and DATAFIFO.
- The Linux variant `libdatafifo_linux.a`.

## Build

```sh
export K230_MUSL_TOOLCHAIN=/path/to/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu_rtt
export K230_MPP_LIB_DIR=/path/to/k230-mpp-libraries

./camera/build.sh
./reader/build_reader.sh
```

`camera/build.sh` enables RVV 1.0 (`v,zvl128b`), supplies the RT-Smart-specific CRT,
and links a static ELF at the address required by the RT-Smart loader.

## Firmware embedding workaround

On the validated image, RT-Smart's ShareFS client hung even though IPCM reported a
connection. The camera therefore could not be launched reliably from `/sharefs`.
`tools/build_firmware.py` replaces the image's embedded `fastboot_app.elf` and changes
the embedded init command to launch that slot directly:

```sh
python3 tools/build_firmware.py \
  original-rtt-partition.img \
  camera/rtsmart-camera \
  iluvatar-rtt-partition.img
```

The tool:

1. Verifies the outer K230 SHA-256 header.
2. Verifies the nested U-Boot payload CRC.
3. Locates and bounds-checks the existing ELF slot.
4. Inserts the replacement without changing partition size.
5. Rebuilds every checksum and validates the result.

It is intentionally strict and version-specific. Keep a complete SD-card backup and
verify the target block device before writing any image. The repository deliberately
does not provide an automated flashing command.

The validated U-Boot sequence starts RT-Smart before Linux:

```text
k230_boot auto rtt; k230_boot auto linux;
```

Changing persistent U-Boot state can make a board unbootable; inspect the current
environment and preserve a recovery image first.

## Start the Linux forwarder

At startup the RT-Smart serial console prints:

```text
DATAFIFO phys_addr: 0x...
REGISTRATION_HEX:...
```

Pass those values to the little-core Linux process without ShareFS:

```sh
export ILUVATAR_DATAFIFO_PHYS_ADDR=0x10000000
export ILUVATAR_REGISTRATION_HEX='<hex printed by RT-Smart>'
./reader/reader 192.168.50.1:4434
```

An optional second `host:port` argument sends downsampled diagnostic masks to a debug
sink. Each destination owns a single latest-frame slot. If a sender falls behind, the
producer overwrites stale data and increments its drop counter instead of growing a
queue.

Camera identity, intrinsics, and pose are compile-time values in
`camera/src/main.odin`; update them before building another physical camera.

## Static memory and bounds

The edge pipeline allocates frame state statically. DATAFIFO has four fixed 256 KiB
slots, and each transmitted frame is capped at 50,000 pooled motion pixels. A full
consumer cannot block capture: the camera records a drop and continues. These bounds
are part of the latency model, not merely implementation details.
