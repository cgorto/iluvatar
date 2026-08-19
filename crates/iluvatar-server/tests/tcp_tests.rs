/// TCP listener integration tests.
///
/// Validates that the TCP transport accepts cameras using the same
/// length-prefixed postcard protocol as QUIC. Each test spins up
/// a TcpServer on a random port, connects a raw TCP client, and
/// verifies the registration handshake.
use std::net::SocketAddr;
use std::sync::Arc;

use futures_lite::io::{AsyncReadExt, AsyncWriteExt};
use parking_lot::RwLock;
use smol::net::TcpStream;

use iluvatar_core::{
    CameraCapabilities, CameraId, CameraIntrinsics, CameraMessage, CameraPose, CameraRegistration,
    CoordinateMode, DistortionModel, Fov, FrameFormat, GeoPosition, GridConfigMessage,
    LocalizationStatus, PoseUncertainty, ServerMessage,
    protocol::{self, MAX_MESSAGE_SIZE},
};
use iluvatar_server::camera_mgmt::CameraRegistry;
use iluvatar_server::tcp::TcpServer;

fn test_grid_config() -> GridConfigMessage {
    GridConfigMessage {
        origin_lat: 47.6,
        origin_lon: -122.3,
        origin_alt: 0.0,
        dimensions: [100, 100, 50],
        voxel_size: 0.25,
        coordinate_mode: CoordinateMode::Gps,
    }
}

fn test_registration(camera_id: CameraId) -> CameraRegistration {
    CameraRegistration {
        version: 2,
        camera_id,
        intrinsics: CameraIntrinsics {
            focal_length: glam::Vec2::new(600.0, 600.0),
            principal_point: glam::Vec2::new(640.0, 360.0),
            resolution: glam::UVec2::new(1280, 720),
            fov: Fov {
                horizontal: 1.2,
                vertical: 0.7,
            },
            distortion: DistortionModel::None,
        },
        initial_pose: CameraPose {
            position: GeoPosition::new(47.6, -122.3, 10.0),
            orientation: glam::Quat::IDENTITY,
            timestamp: 0,
            uncertainty: PoseUncertainty::default(),
            status: LocalizationStatus::Nominal,
        },
        capabilities: CameraCapabilities {
            motion_frames: true,
            rle_encoding: false,
            flags: 0,
        },
    }
}

/// Read one length-prefixed frame from a TCP stream (test helper).
async fn read_framed(stream: &mut TcpStream) -> Vec<u8> {
    let mut len_buf = [0u8; 4];
    stream.read_exact(&mut len_buf).await.unwrap();
    let len = u32::from_be_bytes(len_buf) as usize;
    assert!(len <= MAX_MESSAGE_SIZE);
    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf).await.unwrap();
    buf
}

/// Write one length-prefixed frame to a TCP stream (test helper).
async fn write_framed(stream: &mut TcpStream, data: &[u8]) {
    let len = data.len() as u32;
    stream.write_all(&len.to_be_bytes()).await.unwrap();
    stream.write_all(data).await.unwrap();
}

/// Spawn a TcpServer on an ephemeral port and return the address plus
/// the message channel receiver for inspecting forwarded messages.
async fn spawn_server() -> (SocketAddr, async_channel::Receiver<CameraMessage>) {
    let (msg_tx, msg_rx) = async_channel::bounded::<CameraMessage>(100);
    let registry = Arc::new(RwLock::new(CameraRegistry::new()));
    let grid_config = test_grid_config();

    // Bind to port 0 to get an OS-assigned ephemeral port.
    let server = TcpServer::bind("127.0.0.1:0".parse().unwrap())
        .await
        .unwrap();
    let addr = server.local_addr();

    smol::spawn(async move {
        let _ = server.run(msg_tx, registry, grid_config).await;
    })
    .detach();

    (addr, msg_rx)
}

#[test]
fn test_tcp_registration_motion_camera() {
    smol::block_on(async {
        let (addr, msg_rx) = spawn_server().await;

        let mut stream = TcpStream::connect(addr).await.unwrap();

        // Send Register message.
        let reg = CameraMessage::Register(test_registration(42));
        let reg_bytes = protocol::serialize(&reg).unwrap();
        write_framed(&mut stream, &reg_bytes).await;

        // Server should respond with RegisteredWithPrefs, then GridConfig.
        let prefs_data = read_framed(&mut stream).await;
        let prefs: ServerMessage = protocol::deserialize(&prefs_data).unwrap();
        match prefs {
            ServerMessage::RegisteredWithPrefs {
                camera_id,
                preferences,
            } => {
                assert_eq!(camera_id, 42);
                assert_eq!(preferences.preferred_format, FrameFormat::MotionPixels);
            }
            other => panic!("Expected RegisteredWithPrefs, got {:?}", other),
        }

        let grid_data = read_framed(&mut stream).await;
        let grid_msg: ServerMessage = protocol::deserialize(&grid_data).unwrap();
        match grid_msg {
            ServerMessage::GridConfig(gc) => {
                assert_eq!(gc.dimensions, [100, 100, 50]);
                assert!((gc.voxel_size - 0.25).abs() < f32::EPSILON);
            }
            other => panic!("Expected GridConfig, got {:?}", other),
        }

        // Verify the registration was forwarded to the message channel.
        let forwarded = msg_rx.recv().await.unwrap();
        match forwarded {
            CameraMessage::Register(r) => assert_eq!(r.camera_id, 42),
            other => panic!("Expected Register on channel, got {:?}", other),
        }
    });
}

#[test]
fn test_tcp_registration_contribution_camera() {
    smol::block_on(async {
        let (addr, _msg_rx) = spawn_server().await;

        let mut stream = TcpStream::connect(addr).await.unwrap();

        // Version-2 camera selecting contribution delivery.
        let mut reg = test_registration(7);
        reg.capabilities.motion_frames = false;
        let reg_bytes = protocol::serialize(&CameraMessage::Register(reg)).unwrap();
        write_framed(&mut stream, &reg_bytes).await;

        // Server should respond with GridConfig only (no RegisteredWithPrefs).
        let grid_data = read_framed(&mut stream).await;
        let grid_msg: ServerMessage = protocol::deserialize(&grid_data).unwrap();
        match grid_msg {
            ServerMessage::GridConfig(gc) => {
                assert_eq!(gc.dimensions, [100, 100, 50]);
            }
            other => panic!("Expected GridConfig, got {:?}", other),
        }
    });
}

#[test]
fn test_tcp_frame_forwarding() {
    use iluvatar_core::{MotionData, MotionFrame, MotionPixel};

    smol::block_on(async {
        let (addr, msg_rx) = spawn_server().await;

        let mut stream = TcpStream::connect(addr).await.unwrap();

        // Register first.
        let reg = CameraMessage::Register(test_registration(1));
        let reg_bytes = protocol::serialize(&reg).unwrap();
        write_framed(&mut stream, &reg_bytes).await;

        // Drain registration responses.
        let _ = read_framed(&mut stream).await; // RegisteredWithPrefs
        let _ = read_framed(&mut stream).await; // GridConfig

        // Drain the forwarded Register message.
        let _ = msg_rx.recv().await.unwrap();

        // Send a MotionFrame.
        let motion = CameraMessage::Motion(MotionFrame {
            camera_id: 1,
            sequence: 0,
            timestamp: 1000,
            pose: test_registration(1).initial_pose,
            motion: MotionData::Sparse(vec![
                MotionPixel::new(100, 200, 255),
                MotionPixel::new(101, 200, 128),
            ]),
        });
        let motion_bytes = protocol::serialize(&motion).unwrap();
        write_framed(&mut stream, &motion_bytes).await;

        // Verify it arrives on the channel.
        let forwarded = msg_rx.recv().await.unwrap();
        match forwarded {
            CameraMessage::Motion(mf) => {
                assert_eq!(mf.camera_id, 1);
                assert_eq!(mf.sequence, 0);
                assert_eq!(mf.motion.pixel_count(), 2);
            }
            other => panic!("Expected Motion on channel, got {:?}", other),
        }
    });
}
