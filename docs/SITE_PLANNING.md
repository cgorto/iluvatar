# Site Planning Guide

This document covers the geometry and practical considerations for deploying Iluvatar at a new site. The primary example is airport approach monitoring, but the principles apply to any deployment.

## Core Concepts

### Triangulation Geometry

Iluvatar works by intersecting rays from multiple cameras. For a target at distance D from a camera, the position uncertainty depends on:

```
position_error ~ D^2 / (B * f * pixel_precision)
```

Where:
- **D** = distance to target (meters)
- **B** = baseline between cameras (meters)  
- **f** = effective focal length (pixels per radian)
- **pixel_precision** = motion detection precision (~1-2 pixels)

**Key insight**: Position error grows with the *square* of distance but only linearly with baseline. This means:
- Doubling the baseline halves the error
- Doubling the distance quadruples the error

For aircraft at 1-2km range, you need wide baselines (100-500m) to achieve meter-level accuracy.

### Field of View Coverage

A camera with horizontal FOV of theta at distance D covers a width of:

```
coverage_width = 2 * D * tan(theta/2)
```

| FOV   | Coverage at 500m | Coverage at 1km | Coverage at 2km |
|-------|------------------|-----------------|-----------------|
| 60°   | 577m             | 1155m           | 2309m           |
| 90°   | 1000m            | 2000m           | 4000m           |
| 120°  | 1732m            | 3464m           | 6928m           |

For approach path monitoring, 90° FOV cameras are a good balance between coverage and resolution.

### Minimum Camera Count

- **2 cameras**: Minimum for triangulation. Provides a single line of intersection. Works if cameras are roughly perpendicular to the target's path.
- **3 cameras**: Recommended minimum. Provides redundancy and better geometry for targets moving in any direction.
- **4+ cameras**: Ideal for production. Handles occlusion, camera failures, and provides coverage from multiple angles.

## Airport Deployment Geometry

### Approach Path Characteristics

Aircraft on approach typically follow a 3° glide slope:
- At 3km from threshold: ~150m altitude
- At 2km from threshold: ~100m altitude  
- At 1km from threshold: ~50m altitude
- At threshold: ~15m (over threshold crossing height)

Approach speeds vary by aircraft type:
- Small GA (Cessna): 60-80 knots (30-40 m/s)
- Regional turboprop: 100-130 knots (50-65 m/s)
- Business jet: 120-150 knots (60-75 m/s)
- Commercial jet: 130-160 knots (65-80 m/s)

### Recommended Camera Placement

For monitoring an approach path, place cameras in a **triangular formation**:

```
                    Approach Path
                         |
                         |
                         v
    [CAM-A] --------x--------x--------x-------- [Runway]
               \         |         /
                \        |        /
                 \       |       /
                  \      |      /
                   \     |     /
                    \    |    /
                     \   |   /
                      [CAM-B]
```

But better is a **trapezoidal** or **L-shaped** arrangement:

```
    [CAM-A] ------------------------------------ [Runway]
         \                                    
          \        Approach Path              [CAM-C]
           \            |                        |
            \           v                        |
             \    x----x----x----x               |
              \                                  |
               \                                 |
                [CAM-B] -------------------------+
```

### Camera Placement Guidelines

1. **Baseline distance**: 200-500m between cameras for 1-2km range targets
2. **Angle separation**: At least 30° angular separation between camera viewing directions
3. **No colinearity**: Don't place all cameras in a line parallel to the approach path
4. **Clear sight lines**: Each camera needs unobstructed view of the entire approach corridor
5. **Ground position**: Cameras should be 2-10m above ground level (rooftops, poles, towers)

### Example: 3-Camera Approach Monitor

```
Grid coordinates (meters from runway threshold):
- CAM-A: (-300, +200, +5)   # West side, 5m elevation
- CAM-B: (-300, -200, +5)   # East side, 5m elevation  
- CAM-C: (-800, 0, +8)      # Far end, center, 8m elevation

Camera orientations (azimuth from north, elevation):
- CAM-A: 315° azimuth, +5° elevation (looking NW and slightly up)
- CAM-B: 225° azimuth, +5° elevation (looking SW and slightly up)
- CAM-C: 180° azimuth, +8° elevation (looking S and up toward approach)

This covers the approach from 0-1500m before threshold, 0-200m altitude.
```

### Practical Site Constraints

**Where can you actually mount cameras?**
- Airport perimeter fencing (with permission)
- Nearby buildings (hangars, FBOs, control towers)
- Dedicated poles/masts (requires installation)
- Existing infrastructure (light poles, utility poles)

**Power options:**
- Building-mounted: Use building power (most reliable)
- Pole-mounted: Run power cable or use PoE if within 100m of switch
- Remote: Solar panel + battery (requires sizing for weather and duty cycle)

**Network options:**
- Wired Ethernet: Best latency and reliability, limited to ~100m runs
- WiFi: Good for <500m to access point, weather-dependent
- Cellular (4G/5G): Works anywhere with coverage, variable latency (20-100ms)

## Grid Configuration for Aircraft

### Dimensions

For approach monitoring covering 2km approach x 500m lateral x 500m altitude:

```toml
[grid]
voxel_size = 2.0  # 2m resolution (good balance for aircraft)
dimensions = [1000, 250, 250]  # 2000m x 500m x 500m
origin = { latitude = 47.XXXX, longitude = -122.XXXX, altitude = 0.0 }
```

Why 2m voxels instead of 1m?
- Aircraft are large (10-50m wingspan) - 2m resolution is sufficient
- Reduces computation by 8x vs 1m voxels
- Reduces memory by 8x
- Motion blur at 60m/s spans 1m per frame anyway

### Raymarch Distance

For cameras 500m from the approach path monitoring targets up to 2km away:

```toml
[processing.raymarch]
max_distance = 2500.0  # meters - covers full approach corridor
step_size = 1.0        # not used by DDA but kept for compatibility
```

### Decay Tuning

Aircraft move fast. A Cessna at 40m/s crosses a 2m voxel in 50ms. Settings:

```toml
[decay]
rate = 2.0            # Fast decay - half-life ~350ms
update_interval = 0.05  # 20Hz decay updates
```

This prevents "ghost trails" from lingering after aircraft pass.

### Detection Tuning

```toml
[detection]
intensity_threshold = 5.0     # Lower than default - aircraft are far away
min_contributors = 2          # Require 2 cameras to confirm
cluster_epsilon = 10.0        # 10m clustering - aircraft are large objects
cluster_min_points = 5        # Need multiple voxels to form cluster
```

### Tracking Tuning

```toml
[tracking]
association_threshold = 50.0  # 50m - aircraft can move 60m between frames
max_missing_frames = 60       # 1 second at 60fps before dropping track
```

## Camera Intrinsics and Calibration

### Resolution and FOV Tradeoffs

| Resolution | Horizontal FOV | Pixels per degree | Range for 10-pixel target |
|------------|---------------|-------------------|---------------------------|
| 1920x1080  | 60°           | 32                | ~1.7km for 30m wingspan   |
| 1920x1080  | 90°           | 21                | ~1.1km for 30m wingspan   |
| 3840x2160  | 60°           | 64                | ~3.4km for 30m wingspan   |
| 3840x2160  | 90°           | 43                | ~2.2km for 30m wingspan   |

For aircraft at 1-2km, 1080p with 60-90° FOV provides adequate resolution.

### Calibration Procedure

Camera calibration determines:
1. **Intrinsic parameters**: Focal length, principal point, distortion
2. **Extrinsic parameters**: Position (from GPS) and orientation (manual alignment)

#### Field Calibration Steps

1. **Position**: Record GPS coordinates (or survey with RTK GPS for precision)
2. **Orientation - Rough**: Use compass for initial azimuth, inclinometer for elevation
3. **Orientation - Fine**: Point camera at 2-3 known landmarks at known GPS positions
   - Measure pixel coordinates of each landmark in frame
   - Compute orientation that minimizes reprojection error
4. **Intrinsics**: Use manufacturer specs or perform checkerboard calibration

#### Known-Landmark Calibration

If you have landmarks with known GPS positions (antenna towers, building corners, runway markers):

```
For each landmark L with known WGS84 position:
  1. Convert L to local ENU relative to camera position
  2. Find pixel (u, v) where landmark appears in camera frame
  3. Compute expected ray direction from pixel using nominal intrinsics
  4. Adjust camera orientation until all landmarks align

This can be automated with bundle adjustment.
```

### Handling Camera Movement

If a camera gets bumped or shifts:
- Tracking accuracy will degrade (detections will "jump" or become inconsistent)
- Run recalibration using known landmarks
- For temporary installations, consider rigid mounting with tamper indicators

## Environmental Considerations

### Weather

**Rain**: Water droplets on lens scatter light, reducing contrast. Solutions:
- Lens hood to reduce direct exposure
- Hydrophobic lens coating
- Heating element to prevent condensation (for cold climates)
- Accept degraded performance in heavy rain

**Fog**: Severely limits visibility. System may not function in thick fog.

**Sun glare**: Direct sun in frame blinds camera. Solutions:
- Orient cameras away from sunrise/sunset directions
- Use lens hood
- Accept blind spots during glare periods

**Snow/Ice**: Can accumulate on lens. Solutions:
- Heating element
- Downward-angled lens to shed precipitation
- Regular manual clearing for critical deployments

### Lighting Conditions

**Day**: Best performance. High contrast, good motion detection.

**Dusk/Dawn**: Acceptable. May need to lower difference threshold.

**Night**: Requires aircraft lighting or IR illumination. Standard visible cameras will not detect unlit objects at night.

**Shadows**: Moving shadows (from clouds, trees) can trigger false motion. Solutions:
- Raise difference threshold
- Filter detections by size (shadows don't triangulate to consistent 3D points)
- Accept some false positives in variable lighting

### Wildlife and Clutter

Birds, insects, and blowing debris can trigger motion detection. Mitigations:
- Require min_contributors >= 2 (random motion won't triangulate)
- Size filtering (birds are smaller than aircraft)
- Speed filtering (birds typically slower than aircraft on approach)
- Elevation filtering (reject detections below certain altitude)

## Legal and Safety Considerations

**This section does not constitute legal advice. Consult local authorities and legal counsel.**

### General Considerations

- Installing cameras may require property owner permission
- Pointing cameras at airports may require coordination with airport authority
- Some jurisdictions regulate surveillance equipment
- Data retention may be subject to privacy laws

### Aviation-Specific

- Do not install equipment that could interfere with navigation aids
- Do not install equipment that could distract pilots (bright lights, reflective surfaces)
- Stay clear of runway safety areas and obstacle limitation surfaces
- Laser and RF transmitters require special authorization near airports

### Data Handling

- Consider who has access to detection data
- Consider retention period
- Consider notification/disclosure requirements

## Site Survey Checklist

Before deploying at a new site, gather this information:

### 1. Target Characteristics
- [ ] What are you tracking? (aircraft, vehicles, people, etc.)
- [ ] Expected target size (meters)
- [ ] Expected target speed (m/s)
- [ ] Expected target altitude range
- [ ] Expected target paths/corridors

### 2. Coverage Requirements
- [ ] What area needs coverage? (draw on map)
- [ ] What is the maximum required range?
- [ ] What position accuracy is needed?
- [ ] What detection probability is acceptable?

### 3. Camera Positions
- [ ] Identify potential mounting locations
- [ ] Photograph each location
- [ ] Record GPS coordinates
- [ ] Measure mounting height above ground
- [ ] Check for obstructions to target area
- [ ] Check for obstructions between camera positions (inter-camera LOS)
- [ ] Identify power source for each location
- [ ] Identify network connectivity for each location

### 4. Environmental
- [ ] Typical weather conditions
- [ ] Sun angles throughout day (use sun path calculator)
- [ ] Potential sources of false motion (trees, flags, traffic)
- [ ] Security of camera locations (theft, vandalism risk)

### 5. Infrastructure
- [ ] Server location (local or cloud?)
- [ ] Network infrastructure (existing or to be installed?)
- [ ] Power infrastructure (grid, solar, battery backup?)
- [ ] Physical access for installation and maintenance

### 6. Legal/Administrative
- [ ] Who owns/controls each mounting location?
- [ ] What permissions are required?
- [ ] What notifications are required?
- [ ] What regulations apply?

## Example Site Plan: Regional Airport Approach Monitor

### Site Overview

- **Location**: Example Regional Airport (KXXX)
- **Objective**: Track aircraft on final approach from 2nm to touchdown
- **Coverage**: 3.7km approach corridor, 500m lateral, 200m altitude

### Camera Positions

| ID    | Location           | Lat/Lon            | Alt (m) | Mount Height | Power   | Network |
|-------|--------------------|--------------------|---------|--------------|---------|---------|
| CAM-1 | FBO Rooftop        | 47.XXXX/-122.XXXX  | 45      | 12m          | AC      | Fiber   |
| CAM-2 | Fuel Farm          | 47.XXXX/-122.XXXX  | 42      | 8m           | AC      | WiFi    |
| CAM-3 | Perimeter Pole     | 47.XXXX/-122.XXXX  | 38      | 10m          | Solar   | 4G      |

### Camera Configuration

All cameras: 1920x1080, 90° FOV, 60fps

| ID    | Azimuth | Elevation | Target Coverage |
|-------|---------|-----------|-----------------|
| CAM-1 | 270°    | +5°       | Approach 0-2km  |
| CAM-2 | 290°    | +8°       | Approach 0.5-2.5km |
| CAM-3 | 120°    | +10°      | Approach 1-3.7km |

### Server Configuration

Server hosted on-premises at FBO building with fiber internet.

```toml
[grid]
voxel_size = 2.0
origin = { latitude = 47.XXXX, longitude = -122.XXXX, altitude = 30.0 }
dimensions = [2000, 300, 150]  # 4km x 600m x 300m
```

### Expected Performance

- **Position accuracy**: ~5-10m at 2km range
- **Latency**: ~80ms (limited by 4G uplink from CAM-3)
- **Detection range**: 100m - 3.5km (limited by pixel resolution at far end)
- **Blind spots**: None in primary coverage, gaps beyond 3.5km
