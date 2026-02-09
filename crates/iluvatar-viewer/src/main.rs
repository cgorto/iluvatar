// Debug viewer for Iluvatar camera difference masks.
//
// Listens on a TCP port for incoming camera connections and renders the
// downsampled difference mask in a minifb window. No dependency on
// iluvatar-core, postcard, or serde — reads the bare wire protocol directly.
//
// Wire protocol (camera → viewer):
//
//   [total_len: u32 BE][width: u16 BE][height: u16 BE][sequence: u32 BE][data: width*height bytes]
//    └── 4 bytes ──┘   └────────────────── total_len bytes ──────────────────┘

use minifb::{Key, Window, WindowOptions};
use std::io::{self, Read};
use std::net::{TcpListener, TcpStream};
use std::time::Instant;

/// Maximum frame dimensions we accept (sanity guard).
const MAX_WIDTH:  u16 = 4096;
const MAX_HEIGHT: u16 = 4096;

/// Maximum payload size: header (8 bytes) + MAX_WIDTH * MAX_HEIGHT.
const MAX_PAYLOAD: u32 = 8 + (MAX_WIDTH as u32) * (MAX_HEIGHT as u32);

fn main() {
    let listen_address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "0.0.0.0:9100".to_string());

    println!("Iluvatar Debug Viewer");
    println!("Listening on {listen_address}");

    let listener = TcpListener::bind(&listen_address)
        .unwrap_or_else(|e| {
            eprintln!("Failed to bind {listen_address}: {e}");
            std::process::exit(1);
        });

    // Accept loop: handle one camera at a time. When a camera disconnects,
    // we close the window and go back to waiting.
    loop {
        println!("Waiting for camera connection...");
        match listener.accept() {
            Ok((stream, addr)) => {
                println!("Camera connected from {addr}");
                stream.set_nodelay(true).ok();
                if let Err(e) = handle_stream(stream) {
                    println!("Camera disconnected: {e}");
                }
            }
            Err(e) => {
                eprintln!("Accept error: {e}");
            }
        }
    }
}

/// Read frames from a camera stream and render them in a window.
fn handle_stream(mut stream: TcpStream) -> io::Result<()> {
    // Set a read timeout so we can poll the window for events even when
    // no frames are arriving. 16ms ≈ 60 Hz window refresh.
    stream.set_read_timeout(Some(std::time::Duration::from_millis(16)))?;

    // Read the first frame header to learn the resolution.
    let (width, height, sequence, first_frame) = read_first_frame(&mut stream)?;

    let title = format!(
        "Iluvatar Viewer — {}x{} (seq {})",
        width, height, sequence
    );
    let mut window = Window::new(
        &title,
        width as usize,
        height as usize,
        WindowOptions {
            resize: false,
            scale_mode: minifb::ScaleMode::AspectRatioStretch,
            ..WindowOptions::default()
        },
    )
    .map_err(|e| io::Error::other(e.to_string()))?;

    // Limit update rate.
    window.set_target_fps(60);

    let pixel_count = (width as usize) * (height as usize);
    let mut rgb_buffer: Vec<u32> = vec![0; pixel_count];
    let mut frame_data: Vec<u8>  = vec![0; pixel_count];

    // Render the first frame.
    frame_data[..first_frame.len()].copy_from_slice(&first_frame);
    grayscale_to_rgb32(&frame_data, &mut rgb_buffer);
    window
        .update_with_buffer(&rgb_buffer, width as usize, height as usize)
        .ok();

    let mut frame_count: u64 = 1;
    let mut fps_timer = Instant::now();
    let mut fps_frames: u64 = 0;
    let mut current_fps: f64;

    // Read buffer for the length prefix (4 bytes) and header (8 bytes).
    let mut len_buf = [0u8; 4];
    let mut hdr_buf = [0u8; 8];

    loop {
        // Check window events (ESC, close button).
        if !window.is_open() || window.is_key_down(Key::Escape) {
            println!("Window closed");
            return Ok(());
        }

        // Try to read the next frame. On timeout, just refresh the window.
        match read_exact_or_timeout(&mut stream, &mut len_buf) {
            Ok(true) => {}
            Ok(false) => {
                // Timeout — no data available. Refresh window and retry.
                window
                    .update_with_buffer(
                        &rgb_buffer,
                        width as usize,
                        height as usize,
                    )
                    .ok();
                continue;
            }
            Err(e) => return Err(e),
        }

        let total_len = u32::from_be_bytes(len_buf);
        if !(8..=MAX_PAYLOAD).contains(&total_len) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid frame length: {total_len}"),
            ));
        }

        // Read header.
        stream.read_exact(&mut hdr_buf)?;
        let frame_width  = u16::from_be_bytes([hdr_buf[0], hdr_buf[1]]);
        let frame_height = u16::from_be_bytes([hdr_buf[2], hdr_buf[3]]);
        let frame_seq    = u32::from_be_bytes([hdr_buf[4], hdr_buf[5], hdr_buf[6], hdr_buf[7]]);

        let data_len = (total_len - 8) as usize;
        let expected  = (frame_width as usize) * (frame_height as usize);
        if data_len != expected {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "data length mismatch: got {data_len}, expected {expected} \
                     for {}x{}",
                    frame_width, frame_height
                ),
            ));
        }

        // If resolution changed, we'd need to resize — for now, just bail.
        if frame_width as u32 != width || frame_height as u32 != height {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "resolution changed mid-stream",
            ));
        }

        // Read pixel data.
        stream.read_exact(&mut frame_data[..data_len])?;

        // Convert and display.
        grayscale_to_rgb32(&frame_data[..data_len], &mut rgb_buffer);
        window
            .update_with_buffer(&rgb_buffer, width as usize, height as usize)
            .ok();

        frame_count += 1;
        fps_frames += 1;

        // Update FPS in title every second.
        let fps_elapsed = fps_timer.elapsed();
        if fps_elapsed.as_secs() >= 1 {
            current_fps =
                fps_frames as f64 / fps_elapsed.as_secs_f64();
            let title = format!(
                "Iluvatar Viewer — {}x{} | {:.1} fps | seq {} | {} frames",
                width, height, current_fps, frame_seq, frame_count
            );
            window.set_title(&title);
            fps_timer = Instant::now();
            fps_frames = 0;
        }
    }
}

/// Read the first complete frame (header + data) from the stream.
///
/// Returns (width, height, sequence, pixel_data).
fn read_first_frame(stream: &mut TcpStream) -> io::Result<(u32, u32, u32, Vec<u8>)> {
    // Temporarily use blocking reads for the first frame.
    stream.set_read_timeout(None)?;

    let mut len_buf = [0u8; 4];
    stream.read_exact(&mut len_buf)?;
    let total_len = u32::from_be_bytes(len_buf);

    if !(8..=MAX_PAYLOAD).contains(&total_len) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("invalid first frame length: {total_len}"),
        ));
    }

    let mut hdr_buf = [0u8; 8];
    stream.read_exact(&mut hdr_buf)?;
    let width    = u16::from_be_bytes([hdr_buf[0], hdr_buf[1]]);
    let height   = u16::from_be_bytes([hdr_buf[2], hdr_buf[3]]);
    let sequence = u32::from_be_bytes([hdr_buf[4], hdr_buf[5], hdr_buf[6], hdr_buf[7]]);

    if width == 0 || height == 0 || width > MAX_WIDTH || height > MAX_HEIGHT {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("invalid dimensions: {width}x{height}"),
        ));
    }

    let data_len = (total_len - 8) as usize;
    let expected = (width as usize) * (height as usize);
    if data_len != expected {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "first frame data length mismatch: got {data_len}, expected \
                 {expected} for {width}x{height}"
            ),
        ));
    }

    let mut data = vec![0u8; data_len];
    stream.read_exact(&mut data)?;

    // Restore non-blocking timeout for the render loop.
    stream.set_read_timeout(Some(std::time::Duration::from_millis(16)))?;

    println!("First frame: {width}x{height}, sequence {sequence}");

    Ok((width as u32, height as u32, sequence, data))
}

/// Try to read exactly `buf.len()` bytes, returning `Ok(false)` on timeout
/// (WouldBlock) instead of an error. Any other error is propagated.
fn read_exact_or_timeout(stream: &mut TcpStream, buf: &mut [u8]) -> io::Result<bool> {
    let mut filled = 0;
    while filled < buf.len() {
        match stream.read(&mut buf[filled..]) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "camera disconnected",
                ));
            }
            Ok(n) => {
                filled += n;
            }
            Err(e)
                if e.kind() == io::ErrorKind::WouldBlock
                    || e.kind() == io::ErrorKind::TimedOut =>
            {
                if filled == 0 {
                    // No data at all — pure timeout.
                    return Ok(false);
                }
                // Partial read — keep trying (we already started a frame).
                continue;
            }
            Err(e) => return Err(e),
        }
    }
    Ok(true)
}

/// Convert grayscale bytes to 0x00RRGGBB u32 values for minifb.
///
/// Uses a green-tinted palette: brighter values get more green, with
/// a hint of blue and red to create a "night vision" look that makes
/// motion easy to spot against the dark background.
fn grayscale_to_rgb32(gray: &[u8], rgb: &mut [u32]) {
    assert!(rgb.len() >= gray.len());
    for (i, &g) in gray.iter().enumerate() {
        // Hot colormap: black → green → yellow → white.
        let (r, green, b) = if g == 0 {
            (0u8, 0u8, 0u8)
        } else {
            // Scale: green comes first, red ramps up later, blue last.
            let green = g;
            let r = g.saturating_sub(64).saturating_mul(2);
            let b = g.saturating_sub(192).saturating_mul(4);
            (r, green, b)
        };
        rgb[i] = ((r as u32) << 16) | ((green as u32) << 8) | (b as u32);
    }
}
