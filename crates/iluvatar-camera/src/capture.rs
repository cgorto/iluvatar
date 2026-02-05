use crate::arena::FrameArena;
#[allow(unused_imports)]
use crate::profile_scope;
use glam::Vec3;
use iluvatar_core::{CameraIntrinsics, CameraPose};
use thiserror::Error;

#[cfg(all(target_os = "linux", feature = "real"))]
use v4l::io::mmap::Stream;
#[cfg(all(target_os = "linux", feature = "real"))]
use v4l::{
    Device,
    buffer::Type,
    format::FourCC,
    io::traits::CaptureStream,
    video::Capture,
};

#[derive(Debug, Error)]
pub enum CaptureError {
    #[error("Failed to open camera device: {0}")]
    DeviceOpen(String),
    #[error("Failed to configure format: {0}")]
    Format(String),
    #[error("Failed to create stream: {0}")]
    StreamCreation(String),
    #[error("Failed to capture frame: {0}")]
    Capture(String),
    #[error("Unsupported pixel format: {0}")]
    UnsupportedFormat(String),
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

/// A grayscale frame for motion detection
pub struct GrayscaleFrame<S> {
    pub width: u32,
    pub height: u32,
    pub data: S,
}

impl<S> GrayscaleFrame<S>
where
    S: AsRef<[u8]>,
{
    pub fn pixels(&self) -> &[u8] {
        self.data.as_ref()
    }

    pub fn get(&self, x: u32, y: u32) -> u8 {
        self.data.as_ref()[(y * self.width + x) as usize]
    }
}

impl<S> GrayscaleFrame<S>
where
    S: AsMut<[u8]>,
{
    pub fn pixels_mut(&mut self) -> &mut [u8] {
        self.data.as_mut()
    }

    pub fn set(&mut self, x: u32, y: u32, value: u8) {
        self.data.as_mut()[(y * self.width + x) as usize] = value;
    }
}

impl GrayscaleFrame<Vec<u8>> {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            data: vec![0; (width * height) as usize],
        }
    }
}

impl<'a> GrayscaleFrame<&'a mut [u8]> {
    pub fn new_in(arena: &'a FrameArena, width: u32, height: u32) -> Self {
        let data = arena.alloc_slice((width * height) as usize);
        Self {
            width,
            height,
            data,
        }
    }
}

/// Abstract camera capture interface
pub trait CameraCapture: Send {
    fn resolution(&self) -> (u32, u32);

    /// Capture a grayscale frame into the arena.
    ///
    /// # Safety Note
    /// This returns a mutable slice from an immutable arena reference.
    /// This is safe because `FrameArena` uses interior mutability (`bumpalo::Bump`).
    #[allow(clippy::mut_from_ref)]
    fn capture_grayscale<'a>(
        &mut self,
        arena: &'a FrameArena,
        pose: &CameraPose,
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError>;
}

/// Dummy camera for testing that simulates a 3D environment
pub struct DummyCamera {
    width: u32,
    height: u32,
    intrinsics: Option<CameraIntrinsics>,
    frame_count: u64,
}

impl DummyCamera {
    pub fn new(width: u32, height: u32, intrinsics: Option<CameraIntrinsics>) -> Self {
        Self {
            width,
            height,
            intrinsics,
            frame_count: 0,
        }
    }
}

impl CameraCapture for DummyCamera {
    fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn capture_grayscale<'a>(
        &mut self,
        arena: &'a FrameArena,
        pose: &CameraPose,
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError> {
        self.frame_count += 1;
        let mut frame = GrayscaleFrame::new_in(arena, self.width, self.height);
        let width = self.width;
        let height = self.height;
        let data = frame.pixels_mut();

        // Clear frame
        data.fill(0);

        if let Some(intrinsics) = &self.intrinsics {
            // Simulate a bouncing ball at (0, 0, Z)
            // Bouncing height 0 to 2 meters. Period 2 seconds (120 frames)
            let time = self.frame_count as f32 / 60.0;
            let ball_height = (time * 5.0).sin().abs() * 2.0;

            // Ball moves in a circle around origin
            let ball_pos = Vec3::new(
                (time * 0.5).cos() * 5.0,
                (time * 0.5).sin() * 5.0,
                ball_height,
            );

            // HACK: We assume pose.position stores (x, y, z) directly in (lat, lon, alt) for the DummyLocalizer
            let cam_pos = Vec3::new(
                pose.position.latitude as f32,
                pose.position.longitude as f32,
                pose.position.altitude as f32,
            );

            let p_local = pose.orientation.conjugate() * (ball_pos - cam_pos);

            if p_local.z < -0.1 {
                // In front of camera (Bevy camera looks down -Z)
                let z = -p_local.z;
                let x = p_local.x / z;
                let y = p_local.y / z;

                let u = intrinsics.focal_length.x * x + intrinsics.principal_point.x;
                // Flip Y for image coordinates (assuming standard computer vision vs Bevy Y-up)
                let v = intrinsics.principal_point.y - intrinsics.focal_length.y * y;

                let radius = (50.0 / z) as i32; // Perspective scaling for radius

                // Draw circle
                let center_x = u as i32;
                let center_y = v as i32;

                let min_y = (center_y - radius).max(0);
                let max_y = (center_y + radius).min(height as i32 - 1);
                let min_x = (center_x - radius).max(0);
                let max_x = (center_x + radius).min(width as i32 - 1);

                for y in min_y..=max_y {
                    for x in min_x..=max_x {
                        let dx = x - center_x;
                        let dy = y - center_y;
                        if dx * dx + dy * dy <= radius * radius {
                            data[(y as u32 * width + x as u32) as usize] = 255;
                        }
                    }
                }
            }
        } else {
            // Fallback to simple pattern
            let offset = (self.frame_count % 100) as u32;
            for y in 0..height {
                for x in 0..width {
                    if (x + offset) % 50 < 25 && (y + offset) % 50 < 25 {
                        data[(y * width + x) as usize] = 200;
                    }
                }
            }
        }

        Ok(frame)
    }
}

/// Internal struct that holds the self-referential Device -> Stream relationship.
/// Uses ouroboros to safely handle the lifetime dependency.
#[cfg(all(target_os = "linux", feature = "real"))]
#[ouroboros::self_referencing]
struct V4L2CameraInner {
    /// The v4l2 device - must be boxed for address stability
    device: Box<Device>,

    /// The mmap stream, borrowing from the device
    #[borrows(device)]
    #[covariant]
    stream: Stream<'this>,
}

#[cfg(all(target_os = "linux", feature = "real"))]
pub struct V4L2Camera {
    inner: V4L2CameraInner,
    width: u32,
    height: u32,
    format: FourCC,
}

#[cfg(all(target_os = "linux", feature = "real"))]
impl V4L2Camera {
    pub fn new(path: &str, width: u32, height: u32) -> Result<Self, CaptureError> {
        let device =
            Device::with_path(path).map_err(|e| CaptureError::DeviceOpen(e.to_string()))?;

        let device = Box::new(device);

        // Always set the capture format explicitly. The ISP may report a default
        // (e.g. NV16 @ 1920x1080) that doesn't match what we need, and some ISP
        // drivers require S_FMT to finalize the pipeline configuration.
        //
        // Some ISP drivers (e.g. K230 vvcam) do not enumerate formats via
        // VIDIOC_ENUM_FMT even though S_FMT succeeds. When enumeration returns
        // an empty list, fall through to setting NV12 directly.
        let formats = device
            .enum_formats()
            .map_err(|e| CaptureError::DeviceOpen(e.to_string()))?;
        let preferred = [
            FourCC::new(b"NV12"),
            FourCC::new(b"YUYV"),
            FourCC::new(b"MJPG"),
        ];
        let chosen_fourcc = match preferred
            .iter()
            .find_map(|pref| formats.iter().find(|f| &f.fourcc == pref))
        {
            Some(desc) => desc.fourcc,
            None => {
                // Driver enumerated no supported formats. Try NV12 via S_FMT
                // directly — the driver may still accept it.
                tracing::warn!(
                    "VIDIOC_ENUM_FMT returned no supported formats. \
                     Attempting NV12 via S_FMT directly."
                );
                FourCC::new(b"NV12")
            }
        };

        let mut fmt = device
            .format()
            .map_err(|e| CaptureError::DeviceOpen(e.to_string()))?;
        fmt.width = width;
        fmt.height = height;
        fmt.fourcc = chosen_fourcc;

        let fmt = device
            .set_format(&fmt)
            .map_err(|e| CaptureError::Format(e.to_string()))?;

        tracing::info!(
            width = fmt.width,
            height = fmt.height,
            fourcc = ?fmt.fourcc,
            "V4L2 format configured"
        );

        // Build the self-referential struct safely using ouroboros
        let inner = V4L2CameraInnerTryBuilder {
            device,
            stream_builder: |device: &Box<Device>| -> Result<Stream<'_>, CaptureError> {
                Stream::new(device, Type::VideoCapture)
                    .map_err(|e| CaptureError::StreamCreation(e.to_string()))
            },
        }
        .try_build()?;

        // Don't call stream.start() here — the v4l crate's next() method
        // handles initialization internally: it queues all buffers first, then
        // calls STREAMON. Calling start() manually bypasses buffer queuing,
        // which causes the ISP driver to hang after one frame.
        Ok(Self {
            inner,
            width: fmt.width,
            height: fmt.height,
            format: fmt.fourcc,
        })
    }
}

#[cfg(all(target_os = "linux", feature = "real"))]
impl CameraCapture for V4L2Camera {
    fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn capture_grayscale<'a>(
        &mut self,
        arena: &'a FrameArena,
        _pose: &CameraPose,
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError> {
        let (buf, _meta) = {
            profile_scope!("v4l2_next");
            self.inner.with_stream_mut(|stream| {
                stream
                    .next()
                    .map_err(|e| CaptureError::Capture(e.to_string()))
            })?
        };

        profile_scope!("format_convert");
        let mut frame = GrayscaleFrame::new_in(arena, self.width, self.height);
        let target = frame.pixels_mut();

        match self.format {
            // NV12: Y plane (width*height) followed by interleaved UV plane
            // Grayscale = just the Y plane, which is the first width*height bytes
            f if f == FourCC::new(b"NV12") => {
                let count = (self.width as usize) * (self.height as usize);
                if buf.len() < count {
                    return Err(CaptureError::Capture(
                        "Buffer too small for NV12 Y plane".to_string(),
                    ));
                }
                target.copy_from_slice(&buf[..count]);
            }
            // YUYV: Y0 U0 Y1 V1
            // We want Y0, Y1...
            f if f == FourCC::new(b"YUYV") => {
                // Optimized YUYV to Grayscale conversion
                // 4 bytes = 2 pixels. We need bytes 0 and 2.
                let count = (self.width as usize)
                    .checked_mul(self.height as usize)
                    .ok_or_else(|| {
                        CaptureError::Capture("Frame dimensions too large".to_string())
                    })?;

                let required_size = count.checked_mul(2).ok_or_else(|| {
                    CaptureError::Capture("Required buffer size too large".to_string())
                })?;

                if buf.len() < required_size {
                    return Err(CaptureError::Capture(
                        "Buffer too small for YUYV".to_string(),
                    ));
                }

                // Optimized copy using iterators
                // Each pixel corresponds to 2 bytes in YUYV stream (effective 16bpp, but shared chroma)
                // We just want the Luma (Y) component which is the first byte of every 2-byte pair.
                // chunks_exact(2) gives us [Y, C] pairs. We take Y.
                for (chunk, pixel) in buf.chunks_exact(2).zip(target.iter_mut()) {
                    *pixel = chunk[0];
                }
            }
            // MJPEG
            f if f == FourCC::new(b"MJPG") => {
                let img = image::load_from_memory_with_format(buf, image::ImageFormat::Jpeg)
                    .map_err(|e| CaptureError::Capture(format!("JPEG decode failed: {}", e)))?;
                let gray = img.to_luma8();

                if gray.width() != self.width || gray.height() != self.height {
                    // Resize if needed (camera might have given different res)
                    // But for performance we hope it matches
                    let resized = image::imageops::resize(
                        &gray,
                        self.width,
                        self.height,
                        image::imageops::FilterType::Nearest,
                    );
                    target.copy_from_slice(&resized);
                } else {
                    target.copy_from_slice(gray.as_raw());
                }
            }
            _ => {
                return Err(CaptureError::UnsupportedFormat(format!(
                    "Unknown format {:?}",
                    self.format
                )));
            }
        }

        Ok(frame)
    }
}
