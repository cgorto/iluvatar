import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Text, Line } from '@react-three/drei';
import { EffectComposer, Bloom, Noise, Vignette, Scanline } from '@react-three/postprocessing';
import { useStore } from '../store';
import { useMemo } from 'react';
import * as THREE from 'three';
import { BlendFunction } from 'postprocessing';
import { type TrackedObject } from '../lib/protocol';

const TrackedEntity = ({ object }: { object: TrackedObject }) => {
  // Map coordinates: ENU (x=East, y=North, z=Up).
  // Three.js: x=Right, y=Up, z=Forward(Screen out).
  // We want a top-down view or iso view.
  // Let's map ENU(x,y,z) -> Three(x, z, -y) to lay it flat on the grid?
  // Or Three(x, z, -y) where Y is up in ThreeJS.
  // Let's say: 
  // R3F X = ENU X
  // R3F Y = ENU Z (Height)
  // R3F Z = -ENU Y (North goes into screen)
  
  const position = useMemo(() => 
    [object.centroid.x, object.centroid.z, -object.centroid.y] as [number, number, number], 
    [object.centroid]
  );
  
  const points = useMemo(() => [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, -object.centroid.z, 0)
  ], [object.centroid.z]);

  return (
    <group position={position}>
      {/* The Object Marker */}
      <mesh>
        <boxGeometry args={[0.4, 0.4, 0.4]} />
        <meshBasicMaterial color="#ffb000" wireframe />
      </mesh>
      
      {/* Inner solid core */}
      <mesh>
        <boxGeometry args={[0.2, 0.2, 0.2]} />
        <meshBasicMaterial color="#ffb000" />
      </mesh>

      {/* Label */}
      <Text 
        position={[0.5, 0.5, 0]} 
        fontSize={0.3} 
        color="#ffb000"
        anchorX="left"
        font="https://fonts.gstatic.com/s/sharetechmono/v15/J7aHnp1uDWRCCytEsefQwNM3.woff"
      >
        ID:{object.id.toString()}
      </Text>

      {/* Height Line */}
      <Line points={points} color="#5c4000" lineWidth={1} />
    </group>
  );
};

const Scene = () => {
  const objects = useStore(state => state.objects);
  const objectList = Array.from(objects.values());

  return (
    <>
      <color attach="background" args={['#050505']} />
      
      <OrbitControls makeDefault maxPolarAngle={Math.PI / 2.1} />
      
      <ambientLight intensity={0.5} />
      <pointLight position={[10, 10, 10]} intensity={1} />

      <Grid 
        args={[100, 100]} 
        cellSize={1} 
        cellThickness={1} 
        cellColor="#1a1a1a" 
        sectionSize={10} 
        sectionThickness={1.5} 
        sectionColor="#333" 
        fadeDistance={50} 
      />

      {objectList.map(obj => (
        <TrackedEntity key={obj.id.toString()} object={obj} />
      ))}

      <EffectComposer>
        <Bloom 
          luminanceThreshold={0.2} 
          mipmapBlur 
          intensity={1.5} 
          radius={0.6}
        />
        <Noise opacity={0.15} blendFunction={BlendFunction.OVERLAY} />
        <Vignette eskil={false} offset={0.1} darkness={1.1} />
        <Scanline density={1.5} opacity={0.1} />
      </EffectComposer>
    </>
  );
};

export default function RadarScreen() {
  return (
    <div style={{ width: '100vw', height: '100vh' }}>
      <Canvas camera={{ position: [10, 10, 10], fov: 45 }}>
        <Scene />
      </Canvas>
    </div>
  );
}
