//! TCP-based camera capture for receiving frames from the simulator.
//!
//! The simulator renders each camera's view and streams grayscale frames
//! over TCP. This module implements `CameraCapture` by connecting to the
//! simulator's per-camera TCP server and reading frames using a simple
//! binary protocol:
//!
//! ```text
//! [4 bytes: frame_number  (u32 BE)]
//! [4 bytes: width         (u32 BE)]
//! [4 bytes: height        (u32 BE)]
//! [8 bytes: timestamp_us  (u64 BE)]
//! [width * height bytes: grayscale pixel data]
//! ```

use std::io::{self, Read};
use std::net::TcpStream;
use std::time::Duration;

use crate::arena::FrameArena;
use crate::capture::{CameraCapture, CaptureError, GrayscaleFrame};
use iluvatar_core::CameraPose;
use tracing::{debug, info};

/// Header size in bytes: frame_number(4) + width(4) + height(4) + timestamp_us(8)
const HEADER_SIZE: usize = 20;

pub struct TcpCamera {
    stream: TcpStream,
    width: u32,
    height: u32,
    header_buf: [u8; HEADER_SIZE],
}

impl TcpCamera {
    /// Connect to the simulator's frame server.
    ///
    /// `host` and `port` correspond to a single camera's `stream_port` in
    /// `simulator.toml`. Retries the connection a few times so the camera
    /// process can start before the simulator is fully ready.
    pub fn new(host: &str, port: u16, width: u32, height: u32) -> Result<Self, CaptureError> {
        let addr = format!("{}:{}", host, port);
        info!("Connecting to simulator frame server at {}...", addr);

        let stream = retry_connect(&addr, 10, Duration::from_secs(2))?;
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .map_err(CaptureError::Io)?;

        info!(
            "Connected to simulator frame server at {} (expecting {}x{} frames)",
            addr, width, height
        );

        Ok(Self {
            stream,
            width,
            height,
            header_buf: [0u8; HEADER_SIZE],
        })
    }

    /// Parse a `tcp:host:port` device string.
    ///
    /// Returns `(host, port)` or `None` if the string doesn't match the format.
    pub fn parse_device(device: &str) -> Option<(&str, u16)> {
        let rest = device.strip_prefix("tcp:")?;
        let (host, port_str) = rest.rsplit_once(':')?;
        let port = port_str.parse().ok()?;
        Some((host, port))
    }
}

impl CameraCapture for TcpCamera {
    fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn capture_grayscale<'a>(
        &mut self,
        arena: &'a FrameArena,
        _pose: &CameraPose,
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError> {
        // Read header
        self.stream
            .read_exact(&mut self.header_buf)
            .map_err(|e| CaptureError::Capture(format!("Failed to read frame header: {}", e)))?;

        let _frame_number = u32::from_be_bytes(self.header_buf[0..4].try_into().unwrap());
        let width = u32::from_be_bytes(self.header_buf[4..8].try_into().unwrap());
        let height = u32::from_be_bytes(self.header_buf[8..12].try_into().unwrap());
        let _timestamp_us = u64::from_be_bytes(self.header_buf[12..20].try_into().unwrap());

        if width != self.width || height != self.height {
            return Err(CaptureError::Capture(format!(
                "Frame resolution mismatch: expected {}x{}, got {}x{}",
                self.width, self.height, width, height
            )));
        }

        let pixel_count = (width * height) as usize;
        let mut frame = GrayscaleFrame::new_in(arena, width, height);
        self.stream.read_exact(frame.pixels_mut()).map_err(|e| {
            CaptureError::Capture(format!(
                "Failed to read frame pixels ({} bytes): {}",
                pixel_count, e
            ))
        })?;

        debug!(
            "Received frame {} ({}x{}) from simulator",
            _frame_number, width, height
        );

        Ok(frame)
    }
}

/// Retry TCP connection with exponential backoff.
fn retry_connect(
    addr: &str,
    max_attempts: u32,
    initial_delay: Duration,
) -> Result<TcpStream, CaptureError> {
    let mut delay = initial_delay;

    for attempt in 1..=max_attempts {
        match TcpStream::connect(addr) {
            Ok(stream) => return Ok(stream),
            Err(e) if attempt < max_attempts => {
                debug!(
                    "Connection attempt {}/{} to {} failed: {}. Retrying in {:?}",
                    attempt, max_attempts, addr, e, delay
                );
                std::thread::sleep(delay);
                delay = (delay * 2).min(Duration::from_secs(10));
            }
            Err(e) => {
                return Err(CaptureError::Io(io::Error::new(
                    io::ErrorKind::ConnectionRefused,
                    format!(
                        "Failed to connect to simulator at {} after {} attempts: {}",
                        addr, max_attempts, e
                    ),
                )));
            }
        }
    }

    unreachable!()
}
