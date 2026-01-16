use crate::arena::FrameArena;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum CaptureError {
    #[error("Failed to open camera device: {0}")]
    DeviceOpen(String),
    #[error("Failed to capture frame: {0}")]
    Capture(String),
    #[error("Unsupported pixel format")]
    UnsupportedFormat,
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
