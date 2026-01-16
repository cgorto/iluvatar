import type { TrackedObject, CameraStatus } from './viewer';

interface UIUpdate {
  objects?: TrackedObject[];
  cameras?: CameraStatus[];
  system?: SystemStatus;
}

interface SystemStatus {
  active_cameras: number;
  total_cameras: number;
  tracked_objects: number;
  voxels_active: number;
  uptime_seconds: number;
}

export function updateUI(data: UIUpdate): void {
  if (data.objects) {
    updateObjectList(data.objects);
  }

  if (data.cameras) {
    updateCameraGrid(data.cameras);
  }

  if (data.system) {
    updateSystemStatus(data.system);
  }
}

function updateObjectList(objects: TrackedObject[]): void {
  const container = document.getElementById('objectList');
  const countEl = document.getElementById('objectCount');

  if (!container) return;

  if (countEl) {
    countEl.textContent = objects.length.toString();
  }

  container.innerHTML = objects
    .map(
      (obj) => `
      <div class="object-item">
        <div class="id">OBJ-${obj.id}</div>
        <div class="details">
          Position: ${obj.position[0].toFixed(4)}°N, ${obj.position[1].toFixed(4)}°W, ${obj.position[2].toFixed(0)}m
          ${obj.velocity ? `<br>Velocity: ${formatVelocity(obj.velocity)}` : ''}
        </div>
      </div>
    `
    )
    .join('');
}

function updateCameraGrid(cameras: CameraStatus[]): void {
  const container = document.getElementById('cameraGrid');
  if (!container) return;

  container.innerHTML = cameras
    .map(
      (cam) => `
      <div class="camera-status ${cam.connected ? 'online' : ''}">
        ${cam.camera_id}
      </div>
    `
    )
    .join('');
}

function updateSystemStatus(status: SystemStatus): void {
  const uptimeEl = document.getElementById('uptime');
  const voxelEl = document.getElementById('voxelCount');

  if (uptimeEl) {
    uptimeEl.textContent = formatUptime(status.uptime_seconds);
  }

  if (voxelEl) {
    voxelEl.textContent = status.voxels_active.toLocaleString();
  }
}

function formatUptime(seconds: number): string {
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  const secs = seconds % 60;
  return `${hours}h ${minutes}m ${secs}s`;
}

function formatVelocity(velocity: [number, number, number]): string {
  const speed = Math.sqrt(
    velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2
  );
  return `${speed.toFixed(1)} m/s`;
}
