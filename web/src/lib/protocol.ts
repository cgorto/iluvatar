export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface BoundingBox {
  min: Vector3;
  max: Vector3;
}

export interface TrackedObject {
  id: bigint;
  centroid: Vector3;
  boundingBox: BoundingBox;
  pointCount: number;
  totalIntensity: number;
  velocity: Vector3 | null;
  confidence: number;
}

export const MessageType = {
  Snapshot: 0,
  Update: 1,
  CameraStatus: 2,
  SystemStatus: 3,
} as const;
