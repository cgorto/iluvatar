// Debug viewer: streams downsampled difference masks over TCP for real-time
// visual inspection on a host machine. Fire-and-forget — if the viewer is
// unreachable or a write blocks, frames are silently skipped. Zero impact
// on the main pipeline when no viewer is configured.
//
// Wire protocol (camera → viewer):
//
//   [total_len: u32 BE][width: u16 BE][height: u16 BE][sequence: u32 BE][data: width*height bytes]
//    └── 4 bytes ──┘   └────────────────── total_len bytes ──────────────────┘
//
// total_len = 8 + (width * height).

use crate::config::DebugConfig;
use crate::difference::DifferenceMask;
use futures_lite::io::AsyncWriteExt;
use smol::net::TcpStream;
use std::time::{Duration, Instant};
use tracing::{debug, info};

/// Interval between reconnection attempts when the viewer is disconnected.
const RECONNECT_INTERVAL: Duration = Duration::from_secs(5);

/// Timeout for the initial TCP connect attempt.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);

/// Timeout for writing a single frame to the TCP socket.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

/// Header size: width(2) + height(2) + sequence(4) = 8 bytes.
const HEADER_SIZE: u32 = 8;

/// Streams downsampled difference masks to a debug viewer over TCP.
///
/// Holds an optional TCP connection, pre-allocated buffers for downsampling
/// and wire encoding, and a sequence counter. The caller drives reconnection
/// by calling `try_reconnect()` periodically.
pub struct DebugSender {
    stream:            Option<TcpStream>,
    address:           String,
    factor:            u32,
    target_width:      u32,
    target_height:     u32,
    downsample_buffer: Vec<u8>,
    send_buffer:       Vec<u8>,
    sequence:          u32,
    last_reconnect:    Instant,
}

impl DebugSender {
    /// Create a new debug sender.
    ///
    /// Attempts an initial TCP connection. If it fails, the sender starts
    /// in disconnected mode and will retry on `try_reconnect()`.
    /// Buffers are pre-allocated for the target (downsampled) resolution.
    pub async fn new(
        config: &DebugConfig,
        source_width:  u32,
        source_height: u32,
    ) -> Self {
        assert!(config.downsample_factor >= 1);
        assert!(source_width > 0);
        assert!(source_height > 0);

        let factor        = config.downsample_factor;
        let target_width  = source_width  / factor;
        let target_height = source_height / factor;
        assert!(target_width > 0);
        assert!(target_height > 0);

        let pixel_count = (target_width * target_height) as usize;

        // Pre-allocate buffers. send_buffer holds the 4-byte length prefix
        // plus the header and pixel data.
        let downsample_buffer = vec![0u8; pixel_count];
        let send_buffer       = vec![0u8; 4 + HEADER_SIZE as usize + pixel_count];

        let stream = match try_connect(&config.viewer_address).await {
            Ok(s) => {
                info!(
                    address = config.viewer_address,
                    "Connected to debug viewer"
                );
                Some(s)
            }
            Err(e) => {
                info!(
                    address = config.viewer_address,
                    error = %e,
                    "Debug viewer not available, will retry"
                );
                None
            }
        };

        Self {
            stream,
            address: config.viewer_address.clone(),
            factor,
            target_width,
            target_height,
            downsample_buffer,
            send_buffer,
            sequence: 0,
            last_reconnect: Instant::now(),
        }
    }

    /// Send a difference mask to the viewer.
    ///
    /// The mask is downsampled via max-pooling, encoded into the wire format,
    /// and written to the TCP socket. On any error the connection is dropped
    /// silently — the caller should call `try_reconnect()` periodically to
    /// re-establish it.
    pub async fn send_mask<S: AsRef<[u8]>>(&mut self, mask: &DifferenceMask<S>) {
        let stream = match self.stream {
            Some(ref mut s) => s,
            None => return,
        };

        // Downsample.
        downsample_mask_max(
            mask.data.as_ref(),
            mask.width,
            mask.height,
            self.factor,
            &mut self.downsample_buffer,
        );

        // Encode wire frame into pre-allocated send buffer.
        let pixel_count = (self.target_width * self.target_height) as usize;
        let total_len   = HEADER_SIZE + pixel_count as u32;

        self.send_buffer[0..4].copy_from_slice(&total_len.to_be_bytes());
        self.send_buffer[4..6].copy_from_slice(&(self.target_width as u16).to_be_bytes());
        self.send_buffer[6..8].copy_from_slice(&(self.target_height as u16).to_be_bytes());
        self.send_buffer[8..12].copy_from_slice(&self.sequence.to_be_bytes());
        self.send_buffer[12..12 + pixel_count]
            .copy_from_slice(&self.downsample_buffer[..pixel_count]);

        let frame_len = 4 + total_len as usize;

        // Write with timeout. On failure, drop the connection.
        let write_result = smol::future::or(
            async {
                stream.write_all(&self.send_buffer[..frame_len]).await
            },
            async {
                smol::Timer::after(WRITE_TIMEOUT).await;
                Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "debug write timeout",
                ))
            },
        )
        .await;

        match write_result {
            Ok(()) => {
                self.sequence = self.sequence.wrapping_add(1);
            }
            Err(e) => {
                debug!(error = %e, "Debug viewer write failed, disconnecting");
                self.stream = None;
            }
        }
    }

    /// Attempt to reconnect to the viewer if disconnected.
    ///
    /// Rate-limited to one attempt per `RECONNECT_INTERVAL`.
    pub async fn try_reconnect(&mut self) {
        if self.stream.is_some() {
            return;
        }
        if self.last_reconnect.elapsed() < RECONNECT_INTERVAL {
            return;
        }
        self.last_reconnect = Instant::now();

        match try_connect(&self.address).await {
            Ok(s) => {
                info!(address = self.address, "Reconnected to debug viewer");
                self.stream = Some(s);
                self.sequence = 0;
            }
            Err(e) => {
                debug!(
                    address = self.address,
                    error = %e,
                    "Debug viewer still unavailable"
                );
            }
        }
    }

    /// Whether the sender currently has a live connection.
    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }
}

/// Downsample a grayscale mask by `factor` using max-pooling.
///
/// Each output pixel is the maximum value of the corresponding `factor x factor`
/// block in the source. Max (not average) preserves motion signals: a single
/// bright pixel in a block means "motion here".
///
/// # Arguments
/// * `source` — source mask pixels, row-major, `source_width * source_height` bytes.
/// * `source_width`, `source_height` — dimensions of the source mask.
/// * `factor` — downsample factor (must be >= 1).
/// * `target` — pre-allocated output buffer, at least
///   `(source_width / factor) * (source_height / factor)` bytes.
pub fn downsample_mask_max(
    source:        &[u8],
    source_width:  u32,
    source_height: u32,
    factor:        u32,
    target:        &mut [u8],
) {
    assert!(factor >= 1);
    let target_width  = source_width  / factor;
    let target_height = source_height / factor;
    let target_len    = (target_width * target_height) as usize;
    assert!(source.len() >= (source_width * source_height) as usize);
    assert!(target.len() >= target_len);

    for ty in 0..target_height {
        for tx in 0..target_width {
            let mut max_value: u8 = 0;
            let base_y = ty * factor;
            let base_x = tx * factor;

            for dy in 0..factor {
                let row = (base_y + dy) * source_width + base_x;
                for dx in 0..factor {
                    let v = source[(row + dx) as usize];
                    if v > max_value {
                        max_value = v;
                    }
                }
            }

            target[(ty * target_width + tx) as usize] = max_value;
        }
    }
}

/// Attempt a TCP connection with a timeout and set TCP_NODELAY.
async fn try_connect(address: &str) -> std::io::Result<TcpStream> {
    let stream = smol::future::or(
        TcpStream::connect(address),
        async {
            smol::Timer::after(CONNECT_TIMEOUT).await;
            Err(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "debug connect timeout",
            ))
        },
    )
    .await?;

    stream.set_nodelay(true)?;
    Ok(stream)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_downsample_factor_1() {
        // Factor 1 is identity.
        let source = vec![10, 20, 30, 40];
        let mut target = vec![0u8; 4];
        downsample_mask_max(&source, 2, 2, 1, &mut target);
        assert_eq!(target, vec![10, 20, 30, 40]);
    }

    #[test]
    fn test_downsample_factor_2() {
        // 4x4 → 2x2, max of each 2x2 block.
        #[rustfmt::skip]
        let source = vec![
             1,  2,  3,  4,
             5,  6,  7,  8,
             9, 10, 11, 12,
            13, 14, 15, 16,
        ];
        let mut target = vec![0u8; 4];
        downsample_mask_max(&source, 4, 4, 2, &mut target);
        // Block (0,0): max(1,2,5,6)=6; Block (1,0): max(3,4,7,8)=8
        // Block (0,1): max(9,10,13,14)=14; Block (1,1): max(11,12,15,16)=16
        assert_eq!(target, vec![6, 8, 14, 16]);
    }

    #[test]
    fn test_downsample_preserves_motion() {
        // A single bright pixel in a block should survive max-pooling.
        let mut source = vec![0u8; 16];
        source[5] = 255; // Pixel at (1,1) in a 4x4 image.
        let mut target = vec![0u8; 4];
        downsample_mask_max(&source, 4, 4, 2, &mut target);
        // Block (0,0) contains pixel (1,1) → 255.
        assert_eq!(target[0], 255);
        assert_eq!(target[1], 0);
        assert_eq!(target[2], 0);
        assert_eq!(target[3], 0);
    }

    #[test]
    fn test_downsample_large_factor() {
        // 6x6 → 2x2 with factor 3.
        let mut source = vec![0u8; 36];
        source[0] = 100;  // Top-left block.
        source[35] = 200; // Bottom-right block.
        let mut target = vec![0u8; 4];
        downsample_mask_max(&source, 6, 6, 3, &mut target);
        assert_eq!(target[0], 100);
        assert_eq!(target[1], 0);
        assert_eq!(target[2], 0);
        assert_eq!(target[3], 200);
    }
}
