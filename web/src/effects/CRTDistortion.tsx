import { Effect } from 'postprocessing'
import { Uniform } from 'three'

const fragmentShader = `
uniform float time;
uniform float curvature;
uniform float aberration;
uniform float vignette;

vec2 curve(vec2 uv) {
    uv = (uv - 0.5) * 2.0;
    uv *= 1.1;	
    uv.x *= 1.0 + pow((abs(uv.y) / 5.0), 2.0) * curvature;
    uv.y *= 1.0 + pow((abs(uv.x) / 4.0), 2.0) * curvature;
    uv  = (uv / 2.0) + 0.5;
    uv =  uv * 0.92 + 0.04;
    return uv;
}

void mainImage(const in vec4 inputColor, const in vec2 uv, out vec4 outputColor) {
    vec2 curvedUV = curve(uv);
    
    // Check bounds
    if (curvedUV.x < 0.0 || curvedUV.x > 1.0 || curvedUV.y < 0.0 || curvedUV.y > 1.0) {
        outputColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // Aberration based on distance from center
    float dist = distance(curvedUV, vec2(0.5));
    float offset = aberration * dist * 0.02;
    
    // Jitter
    float jitter = sin(time * 50.0) * 0.0005;
    curvedUV.x += jitter;
    
    vec4 r = texture2D(inputBuffer, curvedUV + vec2(offset, 0.0));
    vec4 g = texture2D(inputBuffer, curvedUV);
    vec4 b = texture2D(inputBuffer, curvedUV - vec2(offset, 0.0));
    
    vec3 color = vec3(r.r, g.g, b.b);
    
    // Scanlines
    float scanline = sin(curvedUV.y * 800.0 + time * 5.0) * 0.04;
    color -= scanline;

    // Claustrophobic vignette
    float v = dist * vignette;
    color *= 1.0 - v * v;
    
    outputColor = vec4(color, inputColor.a);
}
`

export class CRTDistortion extends Effect {
  constructor({ curvature = 1.0, aberration = 1.0, vignette = 1.5 } = {}) {
    super('CRTDistortion', fragmentShader, {
      uniforms: new Map([
        ['curvature', new Uniform(curvature)],
        ['aberration', new Uniform(aberration)],
        ['vignette', new Uniform(vignette)],
        ['time', new Uniform(0.0)],
      ]),
    })
  }

  update(_renderer: any, _inputBuffer: any, deltaTime: number) {
    const time = this.uniforms.get('time')
    if (time) {
        time.value += deltaTime;
    }
  }
}
