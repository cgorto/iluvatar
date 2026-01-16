# iluvatar-client Implementation Guide

This is the web-based visualization client for Iluvatar, built with TypeScript, CesiumJS for 3D globe visualization, and WebSocket for real-time updates.

## Overview

```
iluvatar-client/
├── src/
│   ├── main.ts          # Application entry point
│   ├── viewer.ts        # CesiumJS 3D visualization
│   ├── websocket.ts     # Server connection
│   └── ui.ts            # UI updates
├── index.html           # Main HTML page
├── package.json         # Dependencies
├── tsconfig.json        # TypeScript config
└── vite.config.ts       # Build config
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         BROWSER                                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   ┌─────────────┐      ┌─────────────────┐      ┌─────────────┐    │
│   │  WebSocket  │─────▶│   State Store   │─────▶│   Viewer    │    │
│   │   Client    │      │  (objects,      │      │  (CesiumJS) │    │
│   │             │      │   cameras)      │      │             │    │
│   └─────────────┘      └────────┬────────┘      └─────────────┘    │
│                                 │                                   │
│                                 ▼                                   │
│                        ┌─────────────────┐                          │
│                        │   UI Updates    │                          │
│                        │  (object list,  │                          │
│                        │   status bar)   │                          │
│                        └─────────────────┘                          │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Setup & Running

```bash
# Install dependencies
cd crates/iluvatar-client
npm install

# Development server (with hot reload)
npm run dev

# Production build
npm run build

# Preview production build
npm run preview
```

---

## Module Implementation Details

### main.ts - Application Entry Point

#### Current State
- Initializes viewer and WebSocket client
- Routes messages to appropriate handlers

#### TODO: State Management

```typescript
// src/state.ts

export interface AppState {
  connection: ConnectionState;
  objects: Map<string, TrackedObject>;
  cameras: Map<number, CameraStatus>;
  systemStatus: SystemStatus | null;
  gridBounds: BoundingBox | null;
  selectedObjectId: string | null;
  viewMode: ViewMode;
}

export enum ConnectionState {
  Disconnected = 'disconnected',
  Connecting = 'connecting',
  Connected = 'connected',
  Reconnecting = 'reconnecting',
}

export enum ViewMode {
  Tracking = 'tracking',     // Follow tracked objects
  Overview = 'overview',     // Show entire grid
  Camera = 'camera',         // View from camera perspective
}

class StateStore {
  private state: AppState;
  private listeners: Set<(state: AppState) => void> = new Set();

  constructor() {
    this.state = {
      connection: ConnectionState.Disconnected,
      objects: new Map(),
      cameras: new Map(),
      systemStatus: null,
      gridBounds: null,
      selectedObjectId: null,
      viewMode: ViewMode.Overview,
    };
  }

  getState(): AppState {
    return this.state;
  }

  updateObjects(objects: TrackedObject[]): void {
    const newObjects = new Map<string, TrackedObject>();
    for (const obj of objects) {
      newObjects.set(obj.id, obj);
    }
    this.state = { ...this.state, objects: newObjects };
    this.notify();
  }

  updateCameras(cameras: CameraStatus[]): void {
    const newCameras = new Map<number, CameraStatus>();
    for (const cam of cameras) {
      newCameras.set(cam.camera_id, cam);
    }
    this.state = { ...this.state, cameras: newCameras };
    this.notify();
  }

  setConnectionState(state: ConnectionState): void {
    this.state = { ...this.state, connection: state };
    this.notify();
  }

  selectObject(id: string | null): void {
    this.state = { ...this.state, selectedObjectId: id };
    this.notify();
  }

  subscribe(listener: (state: AppState) => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  private notify(): void {
    for (const listener of this.listeners) {
      listener(this.state);
    }
  }
}

export const store = new StateStore();
```

#### TODO: Enhanced Main Entry

```typescript
// src/main.ts

import { IluvatarViewer } from './viewer';
import { WebSocketClient } from './websocket';
import { initializeUI, updateUI } from './ui';
import { store, ConnectionState } from './state';

async function main() {
  // Initialize UI
  initializeUI();

  // Create viewer
  const viewer = new IluvatarViewer('cesiumContainer');

  // Create WebSocket client
  const serverUrl = getServerUrl();
  const wsClient = new WebSocketClient(serverUrl);

  // Subscribe viewer to state changes
  store.subscribe((state) => {
    viewer.updateFromState(state);
    updateUI(state);
  });

  // Handle WebSocket messages
  wsClient.onMessage((msg) => {
    switch (msg.type) {
      case 'Snapshot':
        store.updateObjects(msg.data.objects);
        store.updateCameras(msg.data.camera_states);
        if (msg.data.grid_bounds) {
          store.setGridBounds(msg.data.grid_bounds);
        }
        break;

      case 'Update':
        store.updateObjects(msg.data.objects);
        break;

      case 'CameraStatus':
        store.updateCameras(msg.data.cameras);
        break;

      case 'SystemStatus':
        store.setSystemStatus(msg.data);
        break;
    }
  });

  // Handle connection state
  wsClient.onConnectionChange((connected) => {
    store.setConnectionState(
      connected ? ConnectionState.Connected : ConnectionState.Disconnected
    );
  });

  // Connect
  store.setConnectionState(ConnectionState.Connecting);
  wsClient.connect();

  // Handle keyboard shortcuts
  document.addEventListener('keydown', (e) => {
    handleKeyboardShortcut(e, viewer, store);
  });
}

function getServerUrl(): string {
  // Allow override via query param or environment
  const params = new URLSearchParams(window.location.search);
  return params.get('server') || 'ws://localhost:8080/ws';
}

function handleKeyboardShortcut(
  e: KeyboardEvent,
  viewer: IluvatarViewer,
  store: StateStore
) {
  switch (e.key) {
    case 'Escape':
      store.selectObject(null);
      break;
    case 'h':
      viewer.flyToHome();
      break;
    case 'f':
      const selected = store.getState().selectedObjectId;
      if (selected) {
        viewer.flyToObject(selected);
      }
      break;
  }
}

main().catch(console.error);
```

---

### viewer.ts - CesiumJS Visualization

#### Current State
- Basic Cesium viewer initialization
- Object point rendering
- Camera status tracking

#### TODO: Enhanced Object Visualization

```typescript
// src/viewer.ts

import * as Cesium from 'cesium';
import type { AppState, TrackedObject, CameraStatus } from './state';

interface ObjectVisual {
  point: Cesium.Entity;
  trail?: Cesium.Entity;
  boundingBox?: Cesium.Entity;
  label: Cesium.Entity;
}

export class IluvatarViewer {
  private viewer: Cesium.Viewer;
  private objectVisuals: Map<string, ObjectVisual> = new Map();
  private cameraVisuals: Map<number, Cesium.Entity> = new Map();
  private gridEntity: Cesium.Entity | null = null;

  private trailsEnabled = true;
  private boundingBoxesEnabled = false;
  private cameraFrustumsEnabled = true;

  constructor(containerId: string) {
    // Initialize Cesium with access token
    Cesium.Ion.defaultAccessToken = 'YOUR_CESIUM_TOKEN';

    this.viewer = new Cesium.Viewer(containerId, {
      terrain: Cesium.Terrain.fromWorldTerrain(),
      timeline: false,
      animation: false,
      baseLayerPicker: true,
      geocoder: false,
      homeButton: true,
      sceneModePicker: true,
      navigationHelpButton: false,
      selectionIndicator: true,
      infoBox: true,
    });

    // Enable lighting
    this.viewer.scene.globe.enableLighting = true;

    // Set default view
    this.flyToHome();

    // Handle entity selection
    this.viewer.selectedEntityChanged.addEventListener((entity) => {
      if (entity && entity.id?.startsWith('object-')) {
        const objectId = entity.id.replace('object-', '');
        // Emit selection event
        this.onObjectSelected?.(objectId);
      }
    });
  }

  onObjectSelected?: (objectId: string) => void;

  updateFromState(state: AppState): void {
    this.updateObjects(Array.from(state.objects.values()));
    this.updateCameras(Array.from(state.cameras.values()));

    if (state.gridBounds) {
      this.updateGridVisualization(state.gridBounds);
    }
  }

  updateObjects(objects: TrackedObject[]): void {
    const currentIds = new Set(objects.map((o) => o.id));

    // Remove old objects
    for (const [id, visual] of this.objectVisuals) {
      if (!currentIds.has(id)) {
        this.viewer.entities.remove(visual.point);
        if (visual.trail) this.viewer.entities.remove(visual.trail);
        if (visual.boundingBox) this.viewer.entities.remove(visual.boundingBox);
        this.viewer.entities.remove(visual.label);
        this.objectVisuals.delete(id);
      }
    }

    // Update or create objects
    for (const obj of objects) {
      const position = Cesium.Cartesian3.fromDegrees(
        obj.position[1], // lon
        obj.position[0], // lat
        obj.position[2]  // alt
      );

      if (this.objectVisuals.has(obj.id)) {
        this.updateExistingObject(obj, position);
      } else {
        this.createNewObject(obj, position);
      }
    }
  }

  private createNewObject(obj: TrackedObject, position: Cesium.Cartesian3): void {
    // Main point
    const point = this.viewer.entities.add({
      id: `object-${obj.id}`,
      position,
      point: {
        pixelSize: 14,
        color: this.getObjectColor(obj),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        heightReference: Cesium.HeightReference.NONE,
      },
      description: this.getObjectDescription(obj),
    });

    // Label
    const label = this.viewer.entities.add({
      id: `object-label-${obj.id}`,
      position,
      label: {
        text: `OBJ-${obj.id}`,
        font: '12px monospace',
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        outlineWidth: 2,
        outlineColor: Cesium.Color.BLACK,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -20),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });

    const visual: ObjectVisual = { point, label };

    // Trail (polyline showing recent positions)
    if (this.trailsEnabled && obj.velocity) {
      visual.trail = this.createTrail(obj, position);
    }

    // Bounding box
    if (this.boundingBoxesEnabled) {
      visual.boundingBox = this.createBoundingBox(obj);
    }

    this.objectVisuals.set(obj.id, visual);
  }

  private updateExistingObject(obj: TrackedObject, position: Cesium.Cartesian3): void {
    const visual = this.objectVisuals.get(obj.id)!;

    // Update position
    (visual.point.position as Cesium.ConstantPositionProperty).setValue(position);
    (visual.label.position as Cesium.ConstantPositionProperty).setValue(position);

    // Update color based on confidence
    (visual.point.point!.color as Cesium.ConstantProperty).setValue(
      this.getObjectColor(obj)
    );

    // Update description
    visual.point.description = new Cesium.ConstantProperty(
      this.getObjectDescription(obj)
    );

    // Update trail
    if (visual.trail && obj.velocity) {
      this.updateTrail(visual.trail, obj, position);
    }
  }

  private getObjectColor(obj: TrackedObject): Cesium.Color {
    // Color based on confidence: red (low) -> yellow -> green (high)
    const confidence = Math.min(obj.confidence, 1.0);
    if (confidence < 0.5) {
      return Cesium.Color.fromCssColorString(
        `rgb(255, ${Math.floor(confidence * 2 * 255)}, 0)`
      );
    } else {
      return Cesium.Color.fromCssColorString(
        `rgb(${Math.floor((1 - confidence) * 2 * 255)}, 255, 0)`
      );
    }
  }

  private getObjectDescription(obj: TrackedObject): string {
    let desc = `<h2>Object ${obj.id}</h2>`;
    desc += `<p><b>Position:</b> ${obj.position[0].toFixed(6)}°N, ${obj.position[1].toFixed(6)}°W</p>`;
    desc += `<p><b>Altitude:</b> ${obj.position[2].toFixed(1)} m</p>`;
    desc += `<p><b>Confidence:</b> ${(obj.confidence * 100).toFixed(1)}%</p>`;

    if (obj.velocity) {
      const speed = Math.sqrt(
        obj.velocity[0] ** 2 + obj.velocity[1] ** 2 + obj.velocity[2] ** 2
      );
      desc += `<p><b>Speed:</b> ${speed.toFixed(1)} m/s</p>`;
    }

    return desc;
  }

  private createTrail(obj: TrackedObject, currentPos: Cesium.Cartesian3): Cesium.Entity {
    // Create a short trail behind the object based on velocity
    const velocity = obj.velocity!;
    const trailLength = 5; // seconds

    const positions = [currentPos];
    for (let t = 1; t <= 10; t++) {
      const dt = (t / 10) * trailLength;
      const pastPos = Cesium.Cartesian3.fromDegrees(
        obj.position[1] - velocity[1] * dt * 0.00001, // Approximate conversion
        obj.position[0] - velocity[0] * dt * 0.00001,
        obj.position[2] - velocity[2] * dt
      );
      positions.push(pastPos);
    }

    return this.viewer.entities.add({
      polyline: {
        positions,
        width: 3,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.2,
          color: Cesium.Color.YELLOW.withAlpha(0.5),
        }),
      },
    });
  }

  private createBoundingBox(obj: TrackedObject): Cesium.Entity {
    const bb = obj.bounding_box;
    const center = [
      (bb.min[0] + bb.max[0]) / 2,
      (bb.min[1] + bb.max[1]) / 2,
      (bb.min[2] + bb.max[2]) / 2,
    ];
    const dimensions = [
      bb.max[0] - bb.min[0],
      bb.max[1] - bb.min[1],
      bb.max[2] - bb.min[2],
    ];

    return this.viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(center[1], center[0], center[2]),
      box: {
        dimensions: new Cesium.Cartesian3(dimensions[0], dimensions[1], dimensions[2]),
        material: Cesium.Color.CYAN.withAlpha(0.2),
        outline: true,
        outlineColor: Cesium.Color.CYAN,
      },
    });
  }

  updateCameras(cameras: CameraStatus[]): void {
    // Update camera status indicators
    for (const cam of cameras) {
      if (!this.cameraVisuals.has(cam.camera_id)) {
        // Would need camera positions from server to visualize
        // For now, just track status
      }
    }
  }

  private updateGridVisualization(bounds: BoundingBox): void {
    if (this.gridEntity) {
      this.viewer.entities.remove(this.gridEntity);
    }

    // Show grid bounds as a box outline
    const center = [
      (bounds.min[0] + bounds.max[0]) / 2,
      (bounds.min[1] + bounds.max[1]) / 2,
      (bounds.min[2] + bounds.max[2]) / 2,
    ];

    this.gridEntity = this.viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(center[1], center[0], center[2]),
      box: {
        dimensions: new Cesium.Cartesian3(
          bounds.max[0] - bounds.min[0],
          bounds.max[1] - bounds.min[1],
          bounds.max[2] - bounds.min[2]
        ),
        material: Cesium.Color.WHITE.withAlpha(0.05),
        outline: true,
        outlineColor: Cesium.Color.WHITE.withAlpha(0.3),
      },
    });
  }

  flyToHome(): void {
    this.viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(-122.3321, 47.6062, 5000),
      orientation: {
        heading: 0,
        pitch: Cesium.Math.toRadians(-45),
        roll: 0,
      },
    });
  }

  flyToObject(objectId: string): void {
    const visual = this.objectVisuals.get(objectId);
    if (visual) {
      this.viewer.flyTo(visual.point);
    }
  }

  setTrailsEnabled(enabled: boolean): void {
    this.trailsEnabled = enabled;
    // Would need to recreate all trails
  }

  setBoundingBoxesEnabled(enabled: boolean): void {
    this.boundingBoxesEnabled = enabled;
    // Would need to recreate all bounding boxes
  }
}
```

---

### websocket.ts - Server Connection

#### Current State
- Basic WebSocket connection
- Automatic reconnection
- Message routing

#### TODO: Connection Quality Monitoring

```typescript
// src/websocket.ts

export interface ConnectionMetrics {
  latency: number;           // ms
  messagesPerSecond: number;
  bytesPerSecond: number;
  lastMessageTime: number;
}

export class WebSocketClient {
  private ws: WebSocket | null = null;
  private url: string;
  private messageHandlers: ((msg: ServerMessage) => void)[] = [];
  private connectionHandlers: ((connected: boolean) => void)[] = [];

  // Reconnection state
  private reconnectDelay = 1000;
  private maxReconnectDelay = 30000;
  private currentDelay = 1000;
  private reconnectAttempts = 0;

  // Metrics
  private metrics: ConnectionMetrics = {
    latency: 0,
    messagesPerSecond: 0,
    bytesPerSecond: 0,
    lastMessageTime: 0,
  };
  private messageCount = 0;
  private byteCount = 0;
  private lastMetricsUpdate = Date.now();

  constructor(url: string) {
    this.url = url;

    // Update metrics every second
    setInterval(() => this.updateMetrics(), 1000);
  }

  connect(): void {
    try {
      this.ws = new WebSocket(this.url);

      this.ws.onopen = () => {
        console.log('WebSocket connected');
        this.currentDelay = this.reconnectDelay;
        this.reconnectAttempts = 0;
        this.notifyConnectionChange(true);

        // Subscribe to updates
        this.send({ type: 'Subscribe' });

        // Request initial snapshot
        this.send({ type: 'RequestSnapshot' });

        // Start heartbeat
        this.startHeartbeat();
      };

      this.ws.onmessage = (event) => {
        this.handleMessage(event);
      };

      this.ws.onclose = (event) => {
        console.log('WebSocket disconnected:', event.code, event.reason);
        this.notifyConnectionChange(false);
        this.stopHeartbeat();
        this.scheduleReconnect();
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error);
      };
    } catch (e) {
      console.error('Failed to create WebSocket:', e);
      this.scheduleReconnect();
    }
  }

  private handleMessage(event: MessageEvent): void {
    const data = event.data;
    this.byteCount += typeof data === 'string' ? data.length : data.byteLength;
    this.messageCount++;

    try {
      const msg = JSON.parse(data) as ServerMessage;
      this.metrics.lastMessageTime = Date.now();

      // Handle pong for latency calculation
      if (msg.type === 'Pong' && msg.data?.timestamp) {
        this.metrics.latency = Date.now() - msg.data.timestamp;
        return;
      }

      this.notifyMessage(msg);
    } catch (e) {
      console.error('Failed to parse message:', e);
    }
  }

  private heartbeatInterval: number | null = null;

  private startHeartbeat(): void {
    this.heartbeatInterval = window.setInterval(() => {
      this.send({ type: 'Ping', timestamp: Date.now() });
    }, 5000);
  }

  private stopHeartbeat(): void {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  private updateMetrics(): void {
    const now = Date.now();
    const elapsed = (now - this.lastMetricsUpdate) / 1000;

    this.metrics.messagesPerSecond = this.messageCount / elapsed;
    this.metrics.bytesPerSecond = this.byteCount / elapsed;

    this.messageCount = 0;
    this.byteCount = 0;
    this.lastMetricsUpdate = now;
  }

  getMetrics(): ConnectionMetrics {
    return { ...this.metrics };
  }

  private scheduleReconnect(): void {
    this.reconnectAttempts++;
    console.log(
      `Reconnecting in ${this.currentDelay}ms (attempt ${this.reconnectAttempts})...`
    );

    setTimeout(() => {
      this.connect();
    }, this.currentDelay);

    this.currentDelay = Math.min(this.currentDelay * 2, this.maxReconnectDelay);
  }

  send(data: unknown): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    }
  }

  onMessage(handler: (msg: ServerMessage) => void): void {
    this.messageHandlers.push(handler);
  }

  onConnectionChange(handler: (connected: boolean) => void): void {
    this.connectionHandlers.push(handler);
  }

  private notifyMessage(msg: ServerMessage): void {
    for (const handler of this.messageHandlers) {
      handler(msg);
    }
  }

  private notifyConnectionChange(connected: boolean): void {
    for (const handler of this.connectionHandlers) {
      handler(connected);
    }
  }

  disconnect(): void {
    this.stopHeartbeat();
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}
```

---

### ui.ts - User Interface

#### Current State
- Basic object list updates
- Camera status grid
- System status display

#### TODO: Enhanced UI Components

```typescript
// src/ui.ts

import type { AppState, TrackedObject, CameraStatus, SystemStatus } from './state';
import { ConnectionState } from './state';

export function initializeUI(): void {
  // Initialize any interactive elements
  setupViewModeButtons();
  setupFilterControls();
  setupKeyboardShortcuts();
}

export function updateUI(state: AppState): void {
  updateConnectionStatus(state.connection);
  updateObjectList(Array.from(state.objects.values()), state.selectedObjectId);
  updateCameraGrid(Array.from(state.cameras.values()));

  if (state.systemStatus) {
    updateSystemStatus(state.systemStatus);
  }
}

function updateConnectionStatus(status: ConnectionState): void {
  const dot = document.getElementById('connectionStatus');
  const text = document.getElementById('connectionText');

  if (!dot || !text) return;

  dot.classList.remove('connected', 'connecting', 'disconnected');

  switch (status) {
    case ConnectionState.Connected:
      dot.classList.add('connected');
      text.textContent = 'Connected';
      break;
    case ConnectionState.Connecting:
      dot.classList.add('connecting');
      text.textContent = 'Connecting...';
      break;
    case ConnectionState.Reconnecting:
      dot.classList.add('connecting');
      text.textContent = 'Reconnecting...';
      break;
    default:
      dot.classList.add('disconnected');
      text.textContent = 'Disconnected';
  }
}

function updateObjectList(objects: TrackedObject[], selectedId: string | null): void {
  const container = document.getElementById('objectList');
  const countEl = document.getElementById('objectCount');

  if (!container) return;

  if (countEl) {
    countEl.textContent = objects.length.toString();
  }

  // Sort by confidence (highest first)
  const sorted = [...objects].sort((a, b) => b.confidence - a.confidence);

  container.innerHTML = sorted
    .map((obj) => {
      const isSelected = obj.id === selectedId;
      const speed = obj.velocity
        ? Math.sqrt(obj.velocity[0] ** 2 + obj.velocity[1] ** 2 + obj.velocity[2] ** 2)
        : 0;

      const confidenceClass =
        obj.confidence >= 0.8 ? 'high' : obj.confidence >= 0.5 ? 'medium' : 'low';

      return `
        <div class="object-item ${isSelected ? 'selected' : ''}"
             data-object-id="${obj.id}"
             onclick="selectObject('${obj.id}')">
          <div class="object-header">
            <span class="id">OBJ-${obj.id}</span>
            <span class="confidence ${confidenceClass}">
              ${(obj.confidence * 100).toFixed(0)}%
            </span>
          </div>
          <div class="details">
            <div class="position">
              ${obj.position[0].toFixed(4)}°N, ${Math.abs(obj.position[1]).toFixed(4)}°W
            </div>
            <div class="altitude">Alt: ${obj.position[2].toFixed(0)}m</div>
            ${speed > 0 ? `<div class="speed">${speed.toFixed(1)} m/s</div>` : ''}
          </div>
        </div>
      `;
    })
    .join('');
}

function updateCameraGrid(cameras: CameraStatus[]): void {
  const container = document.getElementById('cameraGrid');
  if (!container) return;

  // Sort by camera ID
  const sorted = [...cameras].sort((a, b) => a.camera_id - b.camera_id);

  container.innerHTML = sorted
    .map((cam) => {
      const statusClass = cam.connected ? 'online' : 'offline';
      const fps = cam.connected ? cam.frames_per_second.toFixed(0) : '--';

      return `
        <div class="camera-status ${statusClass}" title="Camera ${cam.camera_id}: ${fps} FPS">
          <span class="camera-id">${cam.camera_id}</span>
          <span class="camera-fps">${fps}</span>
        </div>
      `;
    })
    .join('');
}

function updateSystemStatus(status: SystemStatus): void {
  const uptimeEl = document.getElementById('uptime');
  const voxelEl = document.getElementById('voxelCount');
  const camerasEl = document.getElementById('cameraCount');
  const objectsEl = document.getElementById('trackedCount');

  if (uptimeEl) {
    uptimeEl.textContent = formatUptime(status.uptime_seconds);
  }

  if (voxelEl) {
    voxelEl.textContent = formatNumber(status.voxels_active);
  }

  if (camerasEl) {
    camerasEl.textContent = `${status.active_cameras}/${status.total_cameras}`;
  }

  if (objectsEl) {
    objectsEl.textContent = status.tracked_objects.toString();
  }
}

function formatUptime(seconds: number): string {
  const days = Math.floor(seconds / 86400);
  const hours = Math.floor((seconds % 86400) / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);

  if (days > 0) {
    return `${days}d ${hours}h ${minutes}m`;
  } else if (hours > 0) {
    return `${hours}h ${minutes}m`;
  } else {
    return `${minutes}m ${seconds % 60}s`;
  }
}

function formatNumber(n: number): string {
  if (n >= 1000000) {
    return (n / 1000000).toFixed(1) + 'M';
  } else if (n >= 1000) {
    return (n / 1000).toFixed(1) + 'K';
  }
  return n.toString();
}

function setupViewModeButtons(): void {
  // Add event listeners for view mode toggles
}

function setupFilterControls(): void {
  // Add event listeners for filtering options
}

function setupKeyboardShortcuts(): void {
  // Register keyboard shortcuts help
}

// Global function for object selection (called from onclick)
(window as any).selectObject = (id: string) => {
  // Dispatch selection event
  window.dispatchEvent(new CustomEvent('objectSelected', { detail: { id } }));
};
```

---

## Styling

#### TODO: Enhanced CSS

```css
/* styles.css */

:root {
  --bg-primary: #1a1a2e;
  --bg-secondary: #16213e;
  --bg-tertiary: #0f3460;
  --text-primary: #eee;
  --text-secondary: #94a3b8;
  --accent: #e94560;
  --success: #4ade80;
  --warning: #fbbf24;
}

.object-item {
  background: var(--bg-primary);
  padding: 0.75rem;
  margin-bottom: 0.5rem;
  border-radius: 6px;
  border-left: 3px solid var(--accent);
  cursor: pointer;
  transition: all 0.2s ease;
}

.object-item:hover {
  background: var(--bg-tertiary);
}

.object-item.selected {
  border-left-color: var(--success);
  background: rgba(74, 222, 128, 0.1);
}

.object-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.25rem;
}

.confidence {
  font-size: 0.75rem;
  padding: 0.1rem 0.4rem;
  border-radius: 4px;
}

.confidence.high {
  background: rgba(74, 222, 128, 0.2);
  color: var(--success);
}

.confidence.medium {
  background: rgba(251, 191, 36, 0.2);
  color: var(--warning);
}

.confidence.low {
  background: rgba(233, 69, 96, 0.2);
  color: var(--accent);
}

.camera-status {
  position: relative;
  padding: 0.5rem;
  background: var(--bg-primary);
  border-radius: 4px;
  text-align: center;
}

.camera-status::before {
  content: '';
  position: absolute;
  top: 4px;
  right: 4px;
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: var(--accent);
}

.camera-status.online::before {
  background: var(--success);
}

.camera-fps {
  display: block;
  font-size: 0.7rem;
  color: var(--text-secondary);
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

.status-dot.connecting {
  animation: pulse 1s infinite;
  background: var(--warning);
}
```

---

## Implementation Priority

1. **Phase 1: Basic Visualization**
   - CesiumJS setup with terrain
   - Object points and labels
   - WebSocket connection

2. **Phase 2: Enhanced Objects**
   - Motion trails
   - Confidence-based coloring
   - Bounding box visualization
   - Object selection and info

3. **Phase 3: Camera Visualization**
   - Camera positions
   - Frustum visualization
   - Coverage overlay

4. **Phase 4: Polish**
   - Connection quality metrics
   - Filtering and search
   - Keyboard shortcuts
   - Responsive design
   - Dark/light theme

---

## Build & Deployment

```bash
# Development
npm run dev

# Production build
npm run build

# Output is in dist/ - can be served by any static file server
# or integrated into the Rust server to serve files

# Docker deployment
docker build -t iluvatar-client .
docker run -p 3000:3000 iluvatar-client
```
