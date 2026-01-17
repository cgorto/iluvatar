import { create } from 'zustand';
import { type TrackedObject } from './lib/protocol';

interface AppState {
  objects: Map<bigint, TrackedObject>;
  connected: boolean;
  lastUpdate: number;
  connect: () => void;
  disconnect: () => void;
}

const WS_URL = 'ws://localhost:9000';

export const useStore = create<AppState>((set, get) => {
  let socket: WebSocket | null = null;
  let reconnectTimeout: number | null = null;

  const handleMessage = (event: MessageEvent) => {
    try {
      // We expect JSON now
      const data = JSON.parse(event.data);
      
      // Assume data is an array of TrackedObjects or has an 'objects' field
      // Adjusting based on common patterns. If it's a list:
      const objectList = Array.isArray(data) ? data : (data.objects || []);
      
      const newObjects = new Map<bigint, TrackedObject>();
      
      for (const item of objectList) {
        // Handle ID conversion if needed (JSON numbers vs BigInt)
        const id = typeof item.id === 'string' || typeof item.id === 'number' 
          ? BigInt(item.id) 
          : BigInt(0); // Fallback

        // Map JSON structure to TrackedObject if necessary
        // Assuming direct mapping for now, but being safe with vector3s
        const obj: TrackedObject = {
          id,
          centroid: item.centroid || { x: 0, y: 0, z: 0 },
          boundingBox: item.boundingBox || { min: {x:0,y:0,z:0}, max: {x:0,y:0,z:0} },
          pointCount: item.pointCount || 0,
          totalIntensity: item.totalIntensity || 0,
          velocity: item.velocity || null,
          confidence: item.confidence || 0
        };
        
        newObjects.set(id, obj);
      }

      set({ objects: newObjects, lastUpdate: Date.now() });

    } catch (e) {
      console.error("Parse error", e);
    }
  };

  return {
    objects: new Map(),
    connected: false,
    lastUpdate: 0,

    connect: () => {
      if (socket) return;
      
      console.log("Connecting to", WS_URL);
      socket = new WebSocket(WS_URL);

      socket.onopen = () => {
        set({ connected: true });
        console.log("Connected via JSON/9000");
      };

      socket.onclose = () => {
        set({ connected: false });
        socket = null;
        reconnectTimeout = setTimeout(() => get().connect(), 1000);
      };

      socket.onerror = (e) => {
        console.error("WS Error", e);
      };

      socket.onmessage = handleMessage;
    },

    disconnect: () => {
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      socket?.close();
      socket = null;
      set({ connected: false });
    }
  };
});
