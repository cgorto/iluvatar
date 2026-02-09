//! TCP frame server for streaming rendered camera frames to `iluvatar-camera` processes.
//!
//! ## Architecture
//!
//! Each render camera that has a `stream_port` configured gets:
//! 1. A Bevy `Readback` on its render target image (RGBA → grayscale conversion)
//! 2. A TCP listener on the configured port
//! 3. A system that sends the latest grayscale frame to connected clients
//!
//! ## Wire Protocol
//!
//! ```text
//! [4 bytes: frame_number  (u32 BE)]
//! [4 bytes: width         (u32 BE)]
//! [4 bytes: height        (u32 BE)]
//! [8 bytes: timestamp_us  (u64 BE)]
//! [width * height bytes: grayscale pixel data]
//! ```
//!
//! The server uses latest-frame-wins semantics: if the camera process is slow,
//! frames are dropped and only the most recent frame is sent.

use std::collections::HashMap;
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};

use bevy::{
    prelude::*,
    render::gpu_readback::{Readback, ReadbackComplete},
};

use crate::render_camera::RenderCamera;
use crate::sim_config::SimulatorTomlConfig;

/// Plugin that sets up TCP frame streaming for render cameras.
pub struct FrameServerPlugin;

impl Plugin for FrameServerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FrameServerState>()
            .add_systems(PostStartup, setup_frame_readbacks)
            .add_systems(PostUpdate, send_frames_to_clients);
    }
}

/// Per-camera frame data ready to send over TCP.
#[derive(Clone)]
struct FrameData {
    frame_number: u32,
    width: u32,
    height: u32,
    timestamp_us: u64,
    grayscale: Vec<u8>,
}

/// Shared state for the frame server, accessible from Bevy systems and TCP threads.
#[derive(Resource, Default)]
struct FrameServerState {
    /// camera_id → latest frame (written by readback observers, read by send system)
    frames: HashMap<u32, FrameData>,
    /// camera_id → TCP client connection
    clients: HashMap<u32, TcpStream>,
    /// camera_id → frame counter
    counters: HashMap<u32, u32>,
}

/// After render cameras are spawned, set up readbacks and TCP listeners for each
/// camera that has a `stream_port` in the TOML config.
fn setup_frame_readbacks(
    mut commands: Commands,
    cameras: Query<(Entity, &RenderCamera)>,
    sim_toml: Option<Res<SimulatorTomlConfig>>,
    mut state: ResMut<FrameServerState>,
) {
    let Some(toml) = sim_toml.as_ref() else {
        return;
    };

    // Build a map from camera_id → config entry
    let port_map: HashMap<u32, &crate::sim_config::CameraEntry> =
        toml.cameras.iter().map(|e| (e.id, e)).collect();

    for (entity, render_cam) in cameras.iter() {
        let Some(entry) = port_map.get(&render_cam.camera_id) else {
            continue;
        };

        let camera_id = render_cam.camera_id;
        let port = entry.stream_port;
        let resolution = entry.resolution_uvec2();
        let width = resolution.x;
        let height = resolution.y;

        // Add a readback on the render target image.
        // Use a closure observer that captures camera metadata so we don't need
        // to look up the entity from the event (which Bevy 0.18 `On` doesn't support).
        commands
            .entity(entity)
            .insert(Readback::texture(render_cam.render_target.clone()));

        commands.entity(entity).observe(
            move |event: On<ReadbackComplete>, mut state: ResMut<FrameServerState>| {
                handle_frame_readback_inner(&event, camera_id, width, height, &mut state);
            },
        );

        // Start a TCP listener in a background thread.
        std::thread::spawn(move || {
            run_tcp_acceptor(camera_id, port);
        });

        state.counters.insert(camera_id, 0);

        info!(
            "Frame server: camera {} streaming on TCP port {} ({}x{})",
            camera_id, port, width, height
        );
    }
}

/// Background thread that accepts TCP connections for one camera.
/// Stores accepted connections in a global map that the Bevy system reads from.
fn run_tcp_acceptor(camera_id: u32, port: u16) {
    let addr = format!("0.0.0.0:{}", port);
    let listener = match TcpListener::bind(&addr) {
        Ok(l) => {
            info!(
                "Frame server: camera {} listening on {}",
                camera_id, addr
            );
            l
        }
        Err(e) => {
            error!(
                "Frame server: camera {} failed to bind {}: {}",
                camera_id, addr, e
            );
            return;
        }
    };

    // Accept connections in a loop (reconnection support)
    for stream_result in listener.incoming() {
        match stream_result {
            Ok(stream) => {
                info!(
                    "Frame server: camera {} client connected from {:?}",
                    camera_id,
                    stream.peer_addr()
                );
                let _ = stream.set_nodelay(true);
                PENDING_CONNECTIONS
                    .lock()
                    .unwrap()
                    .insert(camera_id, stream);
            }
            Err(e) => {
                warn!(
                    "Frame server: camera {} accept error: {}",
                    camera_id, e
                );
            }
        }
    }
}

/// Global map for TCP acceptor threads to hand connections to the Bevy world.
static PENDING_CONNECTIONS: std::sync::LazyLock<Mutex<HashMap<u32, TcpStream>>> =
    std::sync::LazyLock::new(|| Mutex::new(HashMap::new()));

/// Process a readback completion: convert RGBA → grayscale and store the frame.
fn handle_frame_readback_inner(
    rgba_data: &[u8],
    camera_id: u32,
    width: u32,
    height: u32,
    state: &mut FrameServerState,
) {
    let expected_rgba = (width * height * 4) as usize;

    if rgba_data.len() < expected_rgba {
        warn!(
            "Frame readback for camera {}: expected {} RGBA bytes, got {}",
            camera_id, expected_rgba, rgba_data.len()
        );
        return;
    }

    // RGBA → grayscale (luminance: 0.299R + 0.587G + 0.114B)
    let pixel_count = (width * height) as usize;
    let mut grayscale = vec![0u8; pixel_count];
    for i in 0..pixel_count {
        let r = rgba_data[i * 4] as f32;
        let g = rgba_data[i * 4 + 1] as f32;
        let b = rgba_data[i * 4 + 2] as f32;
        grayscale[i] = (0.299 * r + 0.587 * g + 0.114 * b).min(255.0) as u8;
    }

    let counter = state.counters.entry(camera_id).or_insert(0);
    *counter += 1;

    let timestamp_us = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64;

    state.frames.insert(
        camera_id,
        FrameData {
            frame_number: *counter,
            width,
            height,
            timestamp_us,
            grayscale,
        },
    );
}

/// System that checks for new TCP connections and sends latest frames to connected clients.
fn send_frames_to_clients(mut state: ResMut<FrameServerState>) {
    // Pick up any new connections from acceptor threads
    if let Ok(mut pending) = PENDING_CONNECTIONS.try_lock() {
        for (camera_id, stream) in pending.drain() {
            state.clients.insert(camera_id, stream);
        }
    }

    // Collect camera IDs that need sending (avoids borrow conflict)
    let camera_ids: Vec<u32> = state.clients.keys().copied().collect();
    let mut disconnected = Vec::new();

    for camera_id in camera_ids {
        // Clone the frame to avoid borrowing state.frames while state.clients is mutably borrowed
        let frame = match state.frames.get(&camera_id) {
            Some(f) => f.clone(),
            None => continue,
        };

        let client = state.clients.get_mut(&camera_id).unwrap();
        if let Err(_e) = send_frame(client, &frame) {
            warn!("Frame server: camera {} client disconnected", camera_id);
            disconnected.push(camera_id);
        }
    }

    for id in disconnected {
        state.clients.remove(&id);
    }
}

/// Serialize and send a single frame over TCP.
fn send_frame(stream: &mut TcpStream, frame: &FrameData) -> std::io::Result<()> {
    let mut header = [0u8; 20];
    header[0..4].copy_from_slice(&frame.frame_number.to_be_bytes());
    header[4..8].copy_from_slice(&frame.width.to_be_bytes());
    header[8..12].copy_from_slice(&frame.height.to_be_bytes());
    header[12..20].copy_from_slice(&frame.timestamp_us.to_be_bytes());

    stream.write_all(&header)?;
    stream.write_all(&frame.grayscale)?;
    stream.flush()?;
    Ok(())
}
