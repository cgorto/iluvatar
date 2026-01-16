use crate::arena::FrameArena;
use thiserror::Error;

#[cfg(target_os = "linux")]
use v4l::io::mmap::Stream;
#[cfg(target_os = "linux")]
use v4l::{
    Device,
    buffer::Type,
    format::FourCC,
    io::traits::{CaptureStream, Stream as StreamTrait},
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
    fn capture_grayscale<'a>(
        &mut self,
        arena: &'a FrameArena,
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError>;
}

/// Dummy camera for testing
pub struct DummyCamera {
    width: u32,
    height: u32,
    frame_count: u64,
}

impl DummyCamera {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
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
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError> {
        self.frame_count += 1;
        let mut frame = GrayscaleFrame::new_in(arena, self.width, self.height);

        // Generate a simple moving pattern for testing
        let offset = (self.frame_count % 100) as u32;
        let width = self.width;
        let height = self.height;

        // Optimize loop to avoid boundary checks in set()
        let data = frame.pixels_mut();
        for y in 0..height {
            for x in 0..width {
                let value = if (x + offset) % 50 < 25 && (y + offset) % 50 < 25 {
                    200
                } else {
                    50
                };
                data[(y * width + x) as usize] = value;
            }
        }

        Ok(frame)
    }
}

/// Internal struct that holds the self-referential Device -> Stream relationship.
/// Uses ouroboros to safely handle the lifetime dependency.
#[cfg(target_os = "linux")]
#[ouroboros::self_referencing]
struct V4L2CameraInner {
    /// The v4l2 device - must be boxed for address stability
    device: Box<Device>,

    /// The mmap stream, borrowing from the device
    #[borrows(device)]
    #[covariant]
    stream: Stream<'this>,
}

#[cfg(target_os = "linux")]
pub struct V4L2Camera {
    inner: V4L2CameraInner,
    width: u32,
    height: u32,
    format: FourCC,
}

#[cfg(target_os = "linux")]
impl V4L2Camera {
    pub fn new(path: &str, width: u32, height: u32) -> Result<Self, CaptureError> {
        let device =
            Device::with_path(path).map_err(|e| CaptureError::DeviceOpen(e.to_string()))?;

        let device = Box::new(device);

        // Request specific format
        let format = device
            .enum_formats()
            .map_err(|e| CaptureError::DeviceOpen(e.to_string()))?
            .into_iter()
            .find(|f| f.fourcc == FourCC::new(b"YUYV") || f.fourcc == FourCC::new(b"MJPG"))
            .ok_or_else(|| {
                CaptureError::UnsupportedFormat(
                    "No supported format (YUYV or MJPG) found".to_string(),
                )
            })?;

        let mut fmt = device
            .format()
            .map_err(|e| CaptureError::DeviceOpen(e.to_string()))?;
        fmt.width = width;
        fmt.height = height;
        fmt.fourcc = format.fourcc;

        let fmt = device
            .set_format(&fmt)
            .map_err(|e| CaptureError::Format(e.to_string()))?;

        // Build the self-referential struct safely using ouroboros
        let inner = V4L2CameraInnerTryBuilder {
            device,
            stream_builder: |device: &Box<Device>| -> Result<Stream<'_>, CaptureError> {
                Stream::new(device, Type::VideoCapture)
                    .map_err(|e| CaptureError::StreamCreation(e.to_string()))
            },
        }
        .try_build()?;

        let mut camera = Self {
            inner,
            width: fmt.width,
            height: fmt.height,
            format: fmt.fourcc,
        };

        // Start the stream
        camera.inner.with_stream_mut(|stream| {
            stream
                .start()
                .map_err(|e| CaptureError::Capture(e.to_string()))
        })?;

        Ok(camera)
    }
}

#[cfg(target_os = "linux")]
impl CameraCapture for V4L2Camera {
    fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn capture_grayscale<'a>(
        &mut self,
        arena: &'a FrameArena,
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError> {
        let (buf, _meta) = self.inner.with_stream_mut(|stream| {
            stream
                .next()
                .map_err(|e| CaptureError::Capture(e.to_string()))
        })?;

        let mut frame = GrayscaleFrame::new_in(arena, self.width, self.height);
        let target = frame.pixels_mut();

        match self.format {
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

                // Vectorized copy could be better, but loop is simple for now
                for (i, p) in target.iter_mut().enumerate() {
                    *p = buf[i * 2];
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
