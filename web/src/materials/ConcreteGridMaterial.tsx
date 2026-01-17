import { shaderMaterial } from '@react-three/drei'
import { extend } from '@react-three/fiber'
import * as THREE from 'three'

const ConcreteGridMaterial = shaderMaterial(
  {
    time: 0,
    resolution: new THREE.Vector2(),
    gridScale: 10.0,
    concreteColor: new THREE.Color('#2a2a2a'),
    gridColor: new THREE.Color('#4a4a4a'),
  },
  // Vertex Shader
  `
    varying vec2 vUv;
    varying vec3 vPosition;
    void main() {
      vUv = uv;
      vPosition = position;
      gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }
  `,
  // Fragment Shader
  `
    uniform float time;
    uniform float gridScale;
    uniform vec3 concreteColor;
    uniform vec3 gridColor;
    varying vec2 vUv;
    varying vec3 vPosition;

    // Pseudo-random function
    float random(vec2 st) {
        return fract(sin(dot(st.xy, vec2(12.9898,78.233))) * 43758.5453123);
    }

    // Simple noise
    float noise(vec2 st) {
        vec2 i = floor(st);
        vec2 f = fract(st);
        float a = random(i);
        float b = random(i + vec2(1.0, 0.0));
        float c = random(i + vec2(0.0, 1.0));
        float d = random(i + vec2(1.0, 1.0));
        vec2 u = f * f * (3.0 - 2.0 * f);
        return mix(a, b, u.x) + (c - a)* u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
    }

    void main() {
        // Concrete noise texture
        float n = noise(vUv * 100.0 + time * 0.01);
        float n2 = noise(vUv * 200.0 - time * 0.02);
        float grain = (n + n2) * 0.5;
        
        // Grid pattern
        vec2 grid = fract(vUv * gridScale);
        float line = step(0.98, grid.x) + step(0.98, grid.y);
        
        // "Data made solid" - interference
        float interference = smoothstep(0.4, 0.5, sin(vPosition.y * 20.0 + time * 2.0) * sin(vPosition.x * 10.0));
        
        vec3 color = mix(concreteColor, gridColor, line);
        
        // Add grain
        color += (grain - 0.5) * 0.1;
        
        // Add subtle data interference glow
        color += vec3(0.1, 0.1, 0.15) * interference * 0.2;

        gl_FragColor = vec4(color, 1.0);
    }
  `
)

extend({ ConcreteGridMaterial })

declare module '@react-three/fiber' {
  interface ThreeElements {
    concreteGridMaterial: any
  }
}

export { ConcreteGridMaterial }
