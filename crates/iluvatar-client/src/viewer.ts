import * as Cesium from 'cesium';

export interface TrackedObject {
  id: string;
  position: [number, number, number]; // lat, lon, alt
  bounding_box: { min: number[]; max: number[] };
  velocity?: [number, number, number];
  confidence: number;
}

export interface CameraStatus {
  camera_id: number;
  connected: boolean;
  frames_per_second: number;
}

export class IluvatarViewer {
  private viewer: Cesium.Viewer;
  private objectEntities: Map<string, Cesium.Entity> = new Map();
  private cameraEntities: Map<number, Cesium.Entity> = new Map();
  private trailEntities: Map<string, Cesium.Entity> = new Map();

  constructor(containerId: string) {
    this.viewer = new Cesium.Viewer(containerId, {
      terrain: Cesium.Terrain.fromWorldTerrain(),
      timeline: false,
      animation: false,
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
    });

    // Set default view
    this.viewer.camera.setView({
      destination: Cesium.Cartesian3.fromDegrees(-122.3321, 47.6062, 5000),
      orientation: {
        heading: 0,
        pitch: Cesium.Math.toRadians(-45),
        roll: 0,
      },
    });
  }

  updateObjects(objects: TrackedObject[]): void {
    const currentIds = new Set(objects.map((o) => o.id));

    // Remove entities for objects no longer tracked
    for (const [id, entity] of this.objectEntities) {
      if (!currentIds.has(id)) {
        this.viewer.entities.remove(entity);
        this.objectEntities.delete(id);

        const trail = this.trailEntities.get(id);
        if (trail) {
          this.viewer.entities.remove(trail);
          this.trailEntities.delete(id);
        }
      }
    }

    // Update or create entities
    for (const obj of objects) {
      const position = Cesium.Cartesian3.fromDegrees(
        obj.position[1], // lon
        obj.position[0], // lat
        obj.position[2]  // alt
      );

      if (this.objectEntities.has(obj.id)) {
        const entity = this.objectEntities.get(obj.id)!;
        (entity.position as Cesium.ConstantPositionProperty).setValue(position);
      } else {
        const entity = this.viewer.entities.add({
          id: `object-${obj.id}`,
          position,
          point: {
            pixelSize: 12,
            color: Cesium.Color.RED,
            outlineColor: Cesium.Color.WHITE,
            outlineWidth: 2,
          },
          label: {
            text: `OBJ-${obj.id}`,
            font: '12px sans-serif',
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            outlineWidth: 2,
            verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
            pixelOffset: new Cesium.Cartesian2(0, -15),
          },
        });
        this.objectEntities.set(obj.id, entity);
      }
    }
  }

  updateCameras(cameras: CameraStatus[]): void {
    // For now, just track which cameras are connected
    // Full frustum visualization would require camera positions from server
    for (const cam of cameras) {
      if (!this.cameraEntities.has(cam.camera_id)) {
        // Would add camera visualization here if we had positions
      }
    }
  }

  flyTo(lon: number, lat: number, height: number): void {
    this.viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(lon, lat, height),
    });
  }
}
