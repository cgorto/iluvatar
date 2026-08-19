# Performance evidence

Measurements below were reproduced on 18 August 2026 before the repository was
curated. Raw logs and checksums are retained in the private project archive. Numbers
are observations from development instrumentation, not a formal comparative benchmark.

## Hardware

### Camera unit

- CanMV-K230
- 1.6 GHz C908 big core running RT-Smart
- 800 MHz C908 little core running Linux
- RVV 1.0, VLEN 128
- OV5647, two-lane MIPI CSI-2
- 512 MB physical DRAM split between the two runtimes

### Server host

- Intel Core i7-11700K, 8 cores / 16 threads, 3.60 GHz nominal
- Release Rust build
- Direct Ethernet to the K230 (`192.168.50.1` ↔ `192.168.50.2`)

## Results

| Test | Observed result | What it establishes |
|---|---:|---|
| Native OV5647 mode 44, raw VICAP, no AE | 59–60 FPS | Sensor and capture path can reach 60 FPS |
| Mode 24, AE/AWB, 1280×720 VICAP output, complete edge processing | 30 FPS | Stable full camera loop |
| DATAFIFO plus Linux TCP forwarder | 30 FPS, no reported drops | Cross-core and network path keeps up with camera |
| Release server with real camera stream | 30 FPS | Full real stream is raymarched without backlog |
| Release server with each real input duplicated once | 60.0–60.1 FPS | Server throughput at controlled 60 FPS input |

The 60 FPS server probe consumed 69.4% of one logical CPU according to process CPU
measurement. The grid held roughly 134,000–149,000 active voxels in a 12 MB table.
The reader was returned to one output frame per input immediately after the probe.

## Camera timing

The stable sensor-mode-24 run reported a frame period near 33.3 ms:

| Stage | Approximate time |
|---|---:|
| VICAP capture wait | 25.6 ms |
| EMA background update | 4.5 ms |
| Motion extraction | 2.36 ms |
| Encoding | 0.67–0.70 ms |
| DATAFIFO write | 0.12 ms |
| Total | 33.26–33.31 ms |

The capture wait is sensor-paced; processing occupies approximately 7.6 ms of the
frame. The native 720p mode reached 60 FPS without AE, but enabling automatic exposure
on the validated firmware reduced and destabilized its rate. The full path therefore
uses the proven 1080p sensor mode, a 1280×720 VICAP channel, and AE/AWB at 30 FPS.

## Motion density

With the threshold used during validation, a quiet scene produced roughly 1,900–2,500
nonzero pooled blocks per frame. Hand movement produced approximately 31,000–165,000
before the fixed transmission cap. This clearly separated motion from baseline, but
the quiet-scene floor is still too environment-dependent to call tuned.

## Corrections to earlier results

An initial server run processed only 7.2–7.5 frames/s. That binary was an unoptimized
`target/debug` build. Repeating the same path with `target/release` sustained the full
30 FPS camera stream. The debug result is not representative and is not used as a
performance claim.

## Limits of the evidence

- The full physical path used one camera, so no real multi-camera tracks were expected
  with `min_contributors = 2`.
- The 60 FPS server measurement duplicated real frames; it is a throughput test, not
  evidence of a 60 FPS end-to-end camera path.
- CPU, grid occupancy, and motion density depend on scene content and configuration.
- No long-duration thermal or reliability test was run.
