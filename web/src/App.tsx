import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Text, Line, Float } from '@react-three/drei';
import { EffectComposer, Bloom, Noise, Vignette, Scanline } from '@react-three/postprocessing';
import { BlendFunction } from 'postprocessing';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';

import { useStore } from './store';
import { type TrackedObject } from './lib/protocol';
import { ConcreteGridMaterial } from './materials/ConcreteGridMaterial';
import { Dither, CRT } from './effects';

// Ensure the material is registered
console.log(ConcreteGridMaterial);

const TrackedEntity = ({ object }: { object: TrackedObject }) => {
  // Coordinate transform:
  // R3F X = ENU X
  // R3F Y = ENU Z (Height)
  // R3F Z = -ENU Y (North goes into screen)
  const position = useMemo(() => 
    [object.centroid.x, object.centroid.z, -object.centroid.y] as [number, number, number], 
    [object.centroid]
  );
  
  const points = useMemo(() => [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, -object.centroid.z, 0) // Drop line to floor
  ], [object.centroid.z]);

  return (
    <group position={position}>
      {/* The Object Marker - Wireframe Cube */}
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
  const { objects, connect, connected } = useStore(state => ({
    objects: state.objects,
    connect: state.connect,
    connected: state.connected
  }));

  const objectList = Array.from(objects.values());
  const materialRef = useRef<any>(null);

  // Connect on mount
  useMemo(() => {
    connect();
  }, [connect]);

  useFrame((state) => {
    if (materialRef.current) {
      materialRef.current.time = state.clock.elapsedTime;
    }
  });

  return (
    <>
      <color attach="background" args={['#050505']} />
      
      <OrbitControls makeDefault maxPolarAngle={Math.PI / 2.1} />
      
      <ambientLight intensity={0.5} />
      <pointLight position={[10, 10, 10]} intensity={1} />

      {/* The Concrete Floor */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
        <planeGeometry args={[100, 100]} />
        <concreteGridMaterial 
          ref={materialRef} 
          gridScale={20.0} 
          concreteColor={new THREE.Color('#1a1a1a')} 
          gridColor={new THREE.Color('#3a3a3a')} 
        />
      </mesh>

      {/* Render Tracked Objects */}
      {objectList.map(obj => (
        <TrackedEntity key={obj.id.toString()} object={obj} />
      ))}

      {/* Status Text */}
      <Float speed={1} floatIntensity={0.2}>
         <Text 
            position={[0, 4, -5]} 
            fontSize={1} 
            color="#dddddd"
            font="https://fonts.gstatic.com/s/jetbrainsmono/v18/tDbY2o-flEEny0FZhsfKu5WU4zr3E_BX0Pn5.woff"
        >
            ILUVATAR_SYS
        </Text>
        <Text 
            position={[0, 3.2, -5]} 
            fontSize={0.3} 
            color={connected ? "#00ff00" : "#ff0000"}
            font="https://fonts.gstatic.com/s/jetbrainsmono/v18/tDbY2o-flEEny0FZhsfKu5WU4zr3E_BX0Pn5.woff"
        >
            [{connected ? "LINK ESTABLISHED" : "SIGNAL LOST"}]
        </Text>
      </Float>

      {/* Post-Processing Pipeline */}
      <EffectComposer>
        {/* Agent 1 Effects */}
        <Bloom 
          luminanceThreshold={0.2} 
          mipmapBlur 
          intensity={1.5} 
          radius={0.6}
        />
        <Noise opacity={0.15} blendFunction={BlendFunction.OVERLAY} />
        <Vignette eskil={false} offset={0.1} darkness={1.1} />
        
        {/* Agent 2 Effects */}
        <Dither scale={1.0} />
        <CRT curvature={0.2} aberration={0.8} vignette={1.2} />
      </EffectComposer>
    </>
  );
};

export default function App() {
  return (
    <div style={{ width: '100vw', height: '100vh', background: '#000' }}>
      <Canvas camera={{ position: [0, 5, 10], fov: 45 }}>
        <Scene />
      </Canvas>
    </div>
  );
}
