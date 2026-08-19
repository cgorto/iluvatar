# Architecture

## Technique

Each camera computes a motion mask against an exponential moving background. For
every changed image location, the server reconstructs a world-space ray from the
camera intrinsics and pose, then traverses the configured voxel grid with 3D-DDA.
Voxels accumulate motion magnitude and a per-camera contribution mask.

A changed pixel from one camera produces an ambiguous line through space. Intersections
between independent camera rays produce compact regions with multiple contributors.
The server periodically decays the grid, extracts sufficiently strong multi-camera
voxels, clusters them with DBSCAN, and feeds cluster centroids to the tracker.

The tracker uses global minimum-cost assignment (Hungarian algorithm) rather than
greedy nearest-neighbour matching. Each track carries a constant-velocity Kalman
filter for prediction and velocity estimation.

## K230 data plane

The K230 dual-system image divides the SoC asymmetrically:

| Core | Runtime | Responsibility |
|---|---|---|
| 1.6 GHz C908 | RT-Smart | MIPI capture, ISP, luma processing, motion extraction |
| 800 MHz C908 | Linux | DATAFIFO consumption and TCP forwarding |

The RT-Smart process is written in Odin and uses statically allocated frame state.
It configures the OV5647 through the vendor VICAP API, consumes the NV12 luma plane,
and performs:

1. Exponential moving-average background update.
2. Absolute difference and thresholding.
3. 2×2 max-pooling.
4. Bounded sparse-pixel extraction.
5. Postcard-compatible encoding into a fixed DATAFIFO slot.

DATAFIFO is the vendor's shared-memory ring between cores. The Linux process copies
each completed slot into one single-slot buffer per network destination. The sender
thread always consumes the newest available frame; overwriting an older unsent frame
increments a drop counter instead of extending queue latency.

The TCP stream uses a four-byte big-endian length followed by a postcard message. The
wire frame is capped at 1 MiB. Before enqueueing, both TCP and QUIC validate protocol
version, connection-bound camera identity, negotiated capabilities, image and RLE
bounds, pose/intrinsics finiteness, contribution limits, and voxel coordinates. Invalid
derived rays are fallible and skipped rather than allowed to panic the processing loop.
The Odin registration and motion encoders are checked against Rust reference bytes in
`iluvatar-core` tests.

## Server data plane

The TCP listener validates registration, negotiates motion-frame delivery, and places
messages on a bounded channel. The main loop owns the mutable spatial state:

1. Convert each motion pixel to a world-space ray.
2. Traverse the grid and merge duplicate voxel writes within the frame.
3. Add bounded contributions to a cache-conscious open-addressed voxel table.
4. Apply exponential decay and compact dead entries.
5. Extract voxels above intensity and contributor thresholds.
6. Cluster, assign, and update tracks.
7. Publish sampled voxels, cameras, and tracks over WebSocket.

The open-addressed table is capped by configuration. Message draining is bounded by
both count and wall-clock budget so a noisy camera cannot indefinitely starve decay
and tracking.

## Simulation

The Bevy simulator supplies two validation modes:

- **Geometric mode** projects known target positions directly into idealized cameras.
  It is deterministic and useful for tracking and coordinate-system tests.
- **Render mode** renders each camera to a texture, computes frame differences on the
  GPU, reads sparse rays back, and exercises a more realistic visual path.

Integration tests cover multi-camera triangulation, crossing tracks, velocity
convergence, target lifecycle, high-speed movement, grid boundaries, and deterministic
execution.

## Control plane and known omissions

Configuration supplies camera geometry, voxel dimensions, thresholds, bounds, and
network addresses. The prototype does not include camera provisioning, authenticated
identity, remote configuration, OTA updates, or production telemetry. Those are
explicitly outside the preserved experiment.
