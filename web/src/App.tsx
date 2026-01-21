import React, { useEffect, useState, useRef } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import * as THREE from 'three'

// Types
interface TrackedObject {
  id: number
  position: { x: number; y: number; z: number }
}

// Ghost component - renders a single tracked object
function Ghost({ position }: { position: { x: number; y: number; z: number } }) {
  const meshRef = useRef<THREE.Mesh>(null)
  
  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.rotation.y += 0.01
      meshRef.current.rotation.x += 0.005
    }
  })
  
  return (
    <mesh ref={meshRef} position={[position.x, position.z, position.y]}>
      <octahedronGeometry args={[2, 0]} />
      <meshBasicMaterial color="#00ff88" wireframe />
    </mesh>
  )
}

// Grid floor
function Grid() {
  return (
    <gridHelper args={[200, 40, '#333333', '#222222']} />
  )
}

// Main App
export default function App() {
  const [objects, setObjects] = useState<Map<number, TrackedObject>>(new Map())
  const [status, setStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting')
  
  useEffect(() => {
    console.log('[WS] Connecting to ws://localhost:8080...')
    const ws = new WebSocket('ws://localhost:8080')
    
    ws.onopen = () => {
      console.log('[WS] Connected!')
      setStatus('connected')
    }
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        
        // Handle the Update envelope
        const update = data.Update || data.Snapshot || data
        if (update.objects) {
          setObjects(prev => {
            const next = new Map(prev)
            for (const obj of update.objects) {
              // Parse centroid - could be array [x,y,z] or object {x,y,z}
              let pos: { x: number; y: number; z: number }
              if (Array.isArray(obj.centroid)) {
                pos = { x: obj.centroid[0], y: obj.centroid[1], z: obj.centroid[2] }
              } else if (obj.centroid) {
                pos = obj.centroid
              } else if (obj.position) {
                pos = obj.position
              } else {
                console.warn('[WS] Object has no position:', obj)
                continue
              }
              
              next.set(obj.id, { id: obj.id, position: pos })
            }
            return next
          })
        }
      } catch (e) {
        console.error('[WS] Parse error:', e)
      }
    }
    
    ws.onerror = (e) => {
      console.error('[WS] Error:', e)
      setStatus('disconnected')
    }
    
    ws.onclose = () => {
      console.log('[WS] Disconnected')
      setStatus('disconnected')
    }
    
    return () => ws.close()
  }, [])
  
  const objectList = Array.from(objects.values())
  
  return (
    <>
      {/* Status overlay */}
      <div style={{
        position: 'absolute',
        top: 20,
        left: 20,
        color: status === 'connected' ? '#00ff88' : '#ff4444',
        fontFamily: 'monospace',
        fontSize: 14,
        zIndex: 1000,
        textShadow: '0 0 10px currentColor',
      }}>
        :: {status.toUpperCase()} :: {objectList.length} OBJECTS
      </div>
      
      {/* 3D Scene */}
      <Canvas
        camera={{ position: [100, 80, 100], fov: 50, far: 2000 }}
        style={{ background: '#0a0a0a' }}
      >
        <ambientLight intensity={0.5} />
        <Grid />
        
        {/* Render all tracked objects */}
        {objectList.map(obj => (
          <Ghost key={obj.id} position={obj.position} />
        ))}
        
        {/* Origin marker */}
        <mesh position={[0, 0, 0]}>
          <sphereGeometry args={[1, 8, 8]} />
          <meshBasicMaterial color="#ff0000" wireframe />
        </mesh>
      </Canvas>
    </>
  )
}
