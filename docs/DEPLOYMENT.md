# Iluvatar Deployment Guide

This guide covers the practical steps for deploying Iluvatar from hardware procurement through operational monitoring.

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Hardware Assembly](#hardware-assembly)
3. [Software Installation](#software-installation)
4. [Site Installation](#site-installation)
5. [Calibration](#calibration)
6. [Configuration](#configuration)
7. [Testing](#testing)
8. [Monitoring](#monitoring)
9. [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

### Camera Unit (per camera)

**Compute Platform** (choose one):
- Raspberry Pi 4/5 (4GB+ RAM) - Budget option, adequate for 1080p30
- Jetson Nano/Orin Nano - Better for 1080p60, has GPU for future ML
- Intel NUC (used) - Best performance, higher power consumption

**Camera** (choose one):
- Raspberry Pi Camera Module 3 Wide - 102° FOV, 1080p60, ~$35
- Arducam IMX477 - 79° FOV, interchangeable lens, ~$80
- USB webcam (Logitech C920/C922) - 78° FOV, 1080p30, ~$70
- Industrial USB camera (see-3cam, e-con) - Various FOV, global shutter, ~$150-400

**GPS Receiver**:
- u-blox NEO-6M/7M/8M module - ~$15-30, USB or UART
- u-blox ZED-F9P (RTK capable) - ~$200, centimeter accuracy
- For fixed installations, can skip GPS and survey camera position

**Enclosure**:
- Weatherproof junction box (IP65+) - ~$20-50
- Or 3D-printed enclosure with waterproof seals
- Include: ventilation, cable glands, mounting brackets

**Power**:
- If wired: 5V/3A USB-C power supply for Pi, 12V/2A for Jetson
- If solar: 20W panel, 12V 7Ah+ battery, charge controller, DC-DC converter

**Networking**:
- Ethernet: PoE splitter if using PoE switch
- WiFi: Built-in (Pi) or USB adapter
- Cellular: USB 4G/LTE modem (Huawei E3372, Sierra Wireless, etc.)

### Server

**Minimum** (1-3 cameras):
- Any modern PC/laptop
- 4GB RAM
- 1Gbps Ethernet

**Recommended** (5-10 cameras):
- 4+ core CPU
- 8GB RAM
- Gigabit Ethernet
- SSD for persistence

**Cloud Option**:
- Small VPS (2 vCPU, 4GB RAM) can handle moderate deployments
- Use closest region to cameras for lowest latency
- Ensure adequate bandwidth for camera uploads

### Bill of Materials: 3-Camera Starter Kit

| Item | Qty | Unit Price | Total |
|------|-----|------------|-------|
| Raspberry Pi 4 (4GB) | 3 | $55 | $165 |
| Pi Camera Module 3 Wide | 3 | $35 | $105 |
| u-blox GPS module | 3 | $20 | $60 |
| SD Card 32GB | 3 | $10 | $30 |
| Weatherproof enclosure | 3 | $25 | $75 |
| Power supply + cable | 3 | $15 | $45 |
| 4G USB modem | 3 | $40 | $120 |
| SIM cards (data plan) | 3 | varies | varies |
| Mounting hardware | 1 lot | $50 | $50 |
| Server (mini PC or VPS) | 1 | $100-200 | $150 |
| **Total (excluding SIM)** | | | **~$800** |

---

## Hardware Assembly

### Camera Unit Assembly

1. **Prepare compute board**
   - Flash OS (Raspberry Pi OS Lite recommended)
   - Enable camera interface, SSH
   - Set hostname (e.g., `iluvatar-cam-01`)

2. **Connect camera**
   - Pi Camera: ribbon cable to CSI port
   - USB camera: connect to USB 3.0 port if available

3. **Connect GPS**
   - USB GPS: plug into USB port
   - UART GPS: connect to GPIO pins (TX→RX, RX→TX, 3.3V, GND)

4. **Test components**
   ```bash
   # Test camera
   libcamera-still -o test.jpg
   # or for USB:
   fswebcam test.jpg
   
   # Test GPS
   cat /dev/ttyUSB0  # or /dev/ttyAMA0 for UART
   # Should see NMEA sentences
   ```

5. **Mount in enclosure**
   - Position camera at front with clear view through lens port
   - Secure GPS antenna with view of sky (can be external)
   - Route power and network cables through glands
   - Ensure ventilation (passive or fan)

### GPS Antenna Placement

For accurate positioning:
- GPS antenna needs clear sky view (>90° above horizon)
- Mount antenna on top of enclosure or separately above camera
- Avoid placement near metal surfaces or under overhangs
- For fixed installations, RTK GPS or surveyed positions are more accurate

---

## Software Installation

### Camera Software

1. **Install Rust** (if building from source)
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source ~/.cargo/env
   ```

2. **Build camera binary**
   ```bash
   git clone https://github.com/your-org/iluvatar.git
   cd iluvatar
   cargo build --release -p iluvatar-camera
   ```

3. **Install binary**
   ```bash
   sudo cp target/release/iluvatar-camera /usr/local/bin/
   ```

4. **Create configuration**
   ```bash
   sudo mkdir -p /etc/iluvatar
   sudo cp config/camera.example.toml /etc/iluvatar/camera.toml
   sudo nano /etc/iluvatar/camera.toml  # Edit for this camera
   ```

5. **Create systemd service**
   ```bash
   sudo nano /etc/systemd/system/iluvatar-camera.service
   ```
   
   Contents:
   ```ini
   [Unit]
   Description=Iluvatar Camera Service
   After=network-online.target
   Wants=network-online.target
   
   [Service]
   Type=simple
   ExecStart=/usr/local/bin/iluvatar-camera --config /etc/iluvatar/camera.toml
   Restart=always
   RestartSec=5
   User=pi
   
   [Install]
   WantedBy=multi-user.target
   ```

6. **Enable and start**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable iluvatar-camera
   sudo systemctl start iluvatar-camera
   ```

### Server Software

1. **Build server binary**
   ```bash
   cargo build --release -p iluvatar-server
   ```

2. **Install binary**
   ```bash
   sudo cp target/release/iluvatar-server /usr/local/bin/
   ```

3. **Create configuration**
   ```bash
   sudo mkdir -p /etc/iluvatar
   sudo cp config/server.example.toml /etc/iluvatar/server.toml
   sudo nano /etc/iluvatar/server.toml  # Edit for your deployment
   ```

4. **Create systemd service** (similar to camera, adjust User and paths)

5. **Configure firewall**
   ```bash
   # Allow QUIC from cameras
   sudo ufw allow 4433/udp
   # Allow WebSocket for clients
   sudo ufw allow 8080/tcp
   ```

### TLS Certificates (Required for QUIC)

QUIC requires TLS. For testing, generate self-signed certificates:

```bash
# Generate CA
openssl genrsa -out ca.key 4096
openssl req -new -x509 -days 3650 -key ca.key -out ca.crt -subj "/CN=Iluvatar CA"

# Generate server certificate
openssl genrsa -out server.key 2048
openssl req -new -key server.key -out server.csr -subj "/CN=iluvatar-server"
openssl x509 -req -days 365 -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt

# Distribute ca.crt to all cameras for verification
```

For production, use proper PKI or Let's Encrypt with a domain name.

---

## Site Installation

### Pre-Installation Checklist

- [ ] Site survey completed (see SITE_PLANNING.md)
- [ ] All permissions obtained
- [ ] Hardware assembled and tested on bench
- [ ] Server deployed and accessible from installation sites
- [ ] Network connectivity verified at each camera location

### Physical Installation

1. **Mount camera**
   - Secure enclosure to mounting surface (pole clamp, wall bracket, etc.)
   - Level camera (bubble level on enclosure)
   - Aim camera at target coverage area (approximate, will refine during calibration)

2. **Connect power**
   - Verify voltage before connecting
   - For solar: orient panel, connect battery, verify charging

3. **Connect network**
   - Ethernet: run cable, verify link
   - WiFi: verify signal strength, connect to network
   - Cellular: insert SIM, verify connectivity

4. **Verify GPS lock**
   - Wait for GPS to acquire satellites (can take 1-10 minutes cold start)
   - Verify reported position is reasonable
   - For fixed installations, record final surveyed position

5. **Verify camera view**
   - SSH into camera unit
   - Capture test image: `libcamera-still -o test.jpg`
   - Download and verify framing covers target area

6. **Verify server connectivity**
   ```bash
   # From camera unit
   ping <server-address>
   # Check logs
   journalctl -u iluvatar-camera -f
   ```

---

## Calibration

### Position Calibration

**Option A: GPS (easiest)**
- Let GPS acquire fix
- Position reported in config/logs
- Accuracy: typically 2-5m horizontal

**Option B: RTK GPS (most accurate)**
- Use RTK-capable receiver with base station or NTRIP service
- Accuracy: centimeter-level
- Recommended for permanent installations

**Option C: Survey (for fixed installations)**
- Use surveying equipment or measure from known reference points
- Enter fixed position in config, disable GPS updates

### Orientation Calibration

This is the critical step. Camera orientation must be known accurately for ray projection.

#### Method 1: Known Landmarks

1. Identify 3+ landmarks visible in camera frame with known GPS positions
   - Antenna towers
   - Building corners
   - Runway markers (airports)
   - Surveyed points

2. For each landmark:
   - Note its GPS position (lat, lon, alt)
   - Note its pixel coordinates in camera frame (u, v)

3. Run calibration tool:
   ```bash
   iluvatar-camera --calibrate \
     --landmark "47.123,-122.456,50,960,540" \
     --landmark "47.124,-122.457,52,1200,400" \
     --landmark "47.122,-122.455,48,700,600"
   ```

4. Tool outputs orientation (azimuth, elevation, roll) that minimizes reprojection error

#### Method 2: Compass + Inclinometer (approximate)

1. Mount compass on camera (away from metal/magnets)
2. Read magnetic azimuth, apply local declination for true azimuth
3. Mount inclinometer (smartphone app works)
4. Read elevation angle

Accuracy: typically ±2-5° — acceptable for initial setup, refine with Method 1

#### Method 3: Celestial (high accuracy, requires clear sky)

1. Capture image of stars/sun with precise timestamp
2. Use astrometry.net or similar to determine exact pointing direction
3. Most accurate method but requires specific conditions

### Intrinsics Calibration

For high accuracy, calibrate camera intrinsics:

1. Print checkerboard pattern (8x6 or larger)
2. Capture 20+ images of checkerboard at various angles/distances
3. Run OpenCV calibration:
   ```python
   import cv2
   import numpy as np
   # ... standard OpenCV calibration code
   ```
4. Extract focal length, principal point, distortion coefficients
5. Enter in camera config

For typical deployments with quality cameras, manufacturer specs are often sufficient.

---

## Configuration

### Server Configuration (`/etc/iluvatar/server.toml`)

See `config/server.example.toml` for full documentation.

Key settings for aircraft tracking:

```toml
[server]
listen_address = "0.0.0.0:4433"
websocket_port = 8080
broadcast_rate_hz = 10.0

[grid]
# Set origin to center of coverage area
origin = { latitude = 47.5000, longitude = -122.3000, altitude = 30.0 }
# Size for 4km x 1km x 500m coverage at 2m resolution
dimensions = [2000, 500, 250]
voxel_size = 2.0

[decay]
rate = 2.0           # Fast decay for fast targets
update_interval = 0.05

[detection]
intensity_threshold = 5.0
min_contributors = 2
cluster_epsilon = 15.0   # Aircraft are large
cluster_min_points = 5

[tracking]
association_threshold = 100.0  # Aircraft move fast between frames
max_missing_frames = 120       # 2 seconds at 60fps
```

### Camera Configuration (`/etc/iluvatar/camera.toml`)

See `config/camera.example.toml` for full documentation.

Key settings:

```toml
[identity]
camera_id = 1  # Unique per camera

[hardware]
device = "/dev/video0"  # Adjust for your camera
width = 1920
height = 1080
fps = 60

[localization]
gps_device = "/dev/ttyUSB0"
gps_timeout_secs = 120

[processing]
difference_threshold = 20  # Lower for distant targets
motion_threshold_fraction = 0.0005

[processing.grid_origin]
# MUST MATCH SERVER ORIGIN
latitude = 47.5000
longitude = -122.3000
altitude = 30.0

[processing.raymarch]
max_distance = 3000.0  # Cover full deployment range

[network]
server_address = "server.example.com:4433"
connection_timeout_secs = 60
reconnect_timeout_secs = 3600  # 1 hour
```

---

## Testing

### Initial Verification

1. **Server receiving data**
   ```bash
   # Server logs should show camera connections
   journalctl -u iluvatar-server -f
   # Look for: "Camera 1 connected" etc.
   ```

2. **Camera sending data**
   ```bash
   # Camera logs should show successful sends
   journalctl -u iluvatar-camera -f
   # Look for: "Frame sent, N contributions"
   ```

3. **WebSocket output**
   ```bash
   # Connect to websocket and observe data
   websocat ws://server:8080
   # Should see JSON messages with tracked objects
   ```

### Detection Verification

1. **Induce motion** in camera view
   - Walk through field of view
   - Wave a flag or move a vehicle
   
2. **Verify voxel contributions** in server logs/metrics
   - Each camera should report contributions when motion detected
   
3. **Verify clustering** produces detected objects
   - Objects should appear in WebSocket output
   - Position should correspond to actual location

### Multi-Camera Triangulation Test

1. Move a target (person, vehicle) through area covered by multiple cameras
2. Verify:
   - Detection occurs (object in WebSocket feed)
   - Position is accurate (compare to GPS ground truth if available)
   - Position is stable (not jumping around)
   - Tracking maintains consistent ID as target moves

### Performance Baseline

Record these metrics during testing:
- Camera frame rate (should be ~60fps)
- Contributions per frame (depends on motion)
- Server processing time per frame
- End-to-end latency (timestamp in message vs current time)
- Detection rate (fraction of time target is detected when in coverage)

---

## Monitoring

### Health Checks

**Camera Unit**
```bash
# Is service running?
systemctl status iluvatar-camera

# Check logs for errors
journalctl -u iluvatar-camera --since "1 hour ago" | grep -i error

# Check GPS status
gpspipe -r | head -10  # Should show NMEA sentences with fix

# Check network connectivity
ping -c 3 <server-address>
```

**Server**
```bash
# Is service running?
systemctl status iluvatar-server

# Check connected cameras
curl http://localhost:8080/api/cameras  # If API implemented

# Check logs for errors
journalctl -u iluvatar-server --since "1 hour ago" | grep -i error
```

### Metrics to Monitor

**Camera Metrics**:
- Frame capture rate (fps)
- Motion detection rate (fraction of frames with motion)
- Contributions per frame
- Network send success rate
- GPS fix status
- CPU temperature (Raspberry Pi throttles at 80°C+)

**Server Metrics**:
- Connected camera count
- Frames received per second (per camera)
- Active voxel count
- Detected object count
- Tracking update rate
- WebSocket client count
- Processing latency

### Alerting

Set up alerts for:
- Camera disconnected > 5 minutes
- Camera GPS lost > 10 minutes
- Server processing latency > 200ms
- No detections for > 30 minutes (if targets expected)
- Disk space low (if persisting data)

### Logging

Retain logs for debugging:
```bash
# Configure journald retention
sudo nano /etc/systemd/journald.conf
# Set: MaxRetentionSec=7d
# Set: SystemMaxUse=1G
```

---

## Troubleshooting

### Camera Not Connecting to Server

**Symptoms**: Camera logs show connection failures

**Check**:
1. Network connectivity: `ping <server>`
2. Port open: `nc -zv <server> 4433`
3. TLS certificates: verify CA cert is installed on camera
4. Firewall: ensure UDP 4433 is open on server
5. Server running: `systemctl status iluvatar-server`

### Camera Connected But No Contributions

**Symptoms**: Server shows camera connected, but no voxel data

**Check**:
1. Camera capturing frames: check frame rate in logs
2. Motion detection working: lower `difference_threshold`
3. Grid origin matches: verify `processing.grid_origin` in camera config matches server
4. Raymarch range: ensure `max_distance` covers target area

### Detections Are Noisy/Unstable

**Symptoms**: Objects appearing/disappearing rapidly, positions jumping

**Check**:
1. Camera orientation calibration: recalibrate if positions are wrong
2. Too sensitive: raise `difference_threshold`
3. Single-camera detections: ensure `min_contributors >= 2`
4. Clustering too tight: increase `cluster_epsilon`
5. Decay too fast: lower `decay.rate`

### Detections Are Missing

**Symptoms**: Known targets not being detected

**Check**:
1. Target in coverage: verify target is in field of view of 2+ cameras
2. Target visible: check for obstructions, weather, glare
3. Target too small: may need higher resolution or closer cameras
4. Threshold too high: lower `intensity_threshold`
5. Contributor requirement: temporarily set `min_contributors = 1` to debug

### High Latency

**Symptoms**: Detections lag real-time by >200ms

**Check**:
1. Network latency: `ping <server>` from camera
2. Camera processing: check CPU usage, may need to reduce resolution
3. Server processing: check CPU usage, may need more powerful hardware
4. Frame buffer full: camera may be dropping frames

### Position Accuracy Is Poor

**Symptoms**: Detected positions don't match actual positions

**Check**:
1. Camera positions: verify GPS/surveyed positions are accurate
2. Camera orientations: recalibrate using known landmarks
3. Grid origin: verify origin is set correctly
4. Baseline: cameras may be too close together for the range

### Specific Issues

**"GPS timeout" on camera**
- Check GPS antenna has sky view
- Check GPS device path in config
- Wait longer for cold start (up to 15 minutes in some cases)
- Check GPS module is powered correctly

**"QUIC handshake failed"**
- TLS certificate issue
- Verify `ca.crt` on camera matches `server.crt` issuer
- Check certificate expiration
- Verify server hostname matches certificate CN/SAN

**"Voxel grid memory exceeded"**
- Noisy camera sending too many contributions
- Raise `difference_threshold` on offending camera
- Set `max_voxels` limit in server config

**Server CPU at 100%**
- Too many contributions to process
- Raise `difference_threshold` on cameras
- Reduce `fps` on cameras
- Increase `voxel_size` to reduce grid resolution

---

## Maintenance

### Regular Tasks

**Daily**:
- Review alerts/logs for issues
- Verify all cameras connected

**Weekly**:
- Check camera enclosure seals (especially after rain)
- Clean camera lenses if accessible
- Review detection metrics for degradation

**Monthly**:
- Verify GPS positions haven't drifted (for non-fixed installations)
- Review and rotate logs
- Check for software updates

**Annually**:
- Replace desiccant packets in enclosures
- Inspect mounting hardware for corrosion
- Recalibrate camera orientations
- Replace batteries (if solar powered)

### Firmware/Software Updates

1. Test updates on non-production system first
2. Update one camera at a time
3. Verify connectivity after each update
4. Keep rollback plan ready

```bash
# Update camera software
cd /path/to/iluvatar
git pull
cargo build --release -p iluvatar-camera
sudo systemctl stop iluvatar-camera
sudo cp target/release/iluvatar-camera /usr/local/bin/
sudo systemctl start iluvatar-camera
```
