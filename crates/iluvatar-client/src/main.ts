import { IluvatarViewer } from './viewer';
import { WebSocketClient } from './websocket';
import { updateUI } from './ui';

const viewer = new IluvatarViewer('cesiumContainer');
const wsClient = new WebSocketClient('ws://localhost:8080/ws');

wsClient.onMessage((msg) => {
  switch (msg.type) {
    case 'Snapshot':
      viewer.updateObjects(msg.data.objects);
      viewer.updateCameras(msg.data.camera_states);
      updateUI(msg.data);
      break;
    case 'Update':
      viewer.updateObjects(msg.data.objects);
      updateUI({ objects: msg.data.objects });
      break;
    case 'CameraStatus':
      viewer.updateCameras(msg.data.cameras);
      break;
    case 'SystemStatus':
      updateUI({ system: msg.data });
      break;
  }
});

wsClient.onConnectionChange((connected) => {
  const dot = document.getElementById('connectionStatus');
  const text = document.getElementById('connectionText');
  if (dot && text) {
    dot.classList.toggle('connected', connected);
    text.textContent = connected ? 'Connected' : 'Disconnected';
  }
});

wsClient.connect();
