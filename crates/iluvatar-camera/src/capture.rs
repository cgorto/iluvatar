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
pub struct GrayscaleFrame {
    pub width: u32,
    pub height: u32,
    pub data: Vec<u8>,
}

impl GrayscaleFrame {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            data: vec![0; (width * height) as usize],
        }
    }

    pub fn pixels(&self) -> &[u8] {
        &self.data
    }

    pub fn pixels_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }

    pub fn get(&self, x: u32, y: u32) -> u8 {
        self.data[(y * self.width + x) as usize]
    }

    pub fn set(&mut self, x: u32, y: u32, value: u8) {
        self.data[(y * self.width + x) as usize] = value;
    }
}

/// Abstract camera capture interface
pub trait CameraCapture: Send {
    fn resolution(&self) -> (u32, u32);
    fn capture_grayscale(&mut self) -> Result<GrayscaleFrame, CaptureError>;
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

    fn capture_grayscale(&mut self) -> Result<GrayscaleFrame, CaptureError> {
        self.frame_count += 1;
        let mut frame = GrayscaleFrame::new(self.width, self.height);

        // Generate a simple moving pattern for testing
        let offset = (self.frame_count % 100) as u32;
        for y in 0..self.height {
            for x in 0..self.width {
                let value = if (x + offset) % 50 < 25 && (y + offset) % 50 < 25 {
                    200
                } else {
                    50
                };
                frame.set(x, y, value);
            }
        }

        Ok(frame)
    }
}
