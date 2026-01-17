import { Effect } from 'postprocessing'
import { Uniform } from 'three'

const fragmentShader = `
uniform float scale;

float dither8x8(vec2 position, float brightness) {
    int x = int(mod(position.x, 8.0));
    int y = int(mod(position.y, 8.0));
    int index = x + y * 8;
    float limit = 0.0;
    
    if (index < 1) limit = 0.015625;
    else if (index < 2) limit = 0.515625;
    else if (index < 3) limit = 0.140625;
    else if (index < 4) limit = 0.640625;
    else if (index < 5) limit = 0.046875;
    else if (index < 6) limit = 0.546875;
    else if (index < 7) limit = 0.171875;
    else if (index < 8) limit = 0.671875;
    else if (index < 9) limit = 0.765625;
    else if (index < 10) limit = 0.265625;
    else if (index < 11) limit = 0.890625;
    else if (index < 12) limit = 0.390625;
    else if (index < 13) limit = 0.796875;
    else if (index < 14) limit = 0.296875;
    else if (index < 15) limit = 0.921875;
    else if (index < 16) limit = 0.421875;
    else if (index < 17) limit = 0.203125;
    else if (index < 18) limit = 0.703125;
    else if (index < 19) limit = 0.078125;
    else if (index < 20) limit = 0.578125;
    else if (index < 21) limit = 0.234375;
    else if (index < 22) limit = 0.734375;
    else if (index < 23) limit = 0.109375;
    else if (index < 24) limit = 0.609375;
    else if (index < 25) limit = 0.953125;
    else if (index < 26) limit = 0.453125;
    else if (index < 27) limit = 0.828125;
    else if (index < 28) limit = 0.328125;
    else if (index < 29) limit = 0.984375;
    else if (index < 30) limit = 0.484375;
    else if (index < 31) limit = 0.859375;
    else if (index < 32) limit = 0.359375;
    else if (index < 33) limit = 0.0625;
    else if (index < 34) limit = 0.5625;
    else if (index < 35) limit = 0.1875;
    else if (index < 36) limit = 0.6875;
    else if (index < 37) limit = 0.03125;
    else if (index < 38) limit = 0.53125;
    else if (index < 39) limit = 0.15625;
    else if (index < 40) limit = 0.65625;
    else if (index < 41) limit = 0.8125;
    else if (index < 42) limit = 0.3125;
    else if (index < 43) limit = 0.9375;
    else if (index < 44) limit = 0.4375;
    else if (index < 45) limit = 0.78125;
    else if (index < 46) limit = 0.28125;
    else if (index < 47) limit = 0.90625;
    else if (index < 48) limit = 0.40625;
    else if (index < 49) limit = 0.25;
    else if (index < 50) limit = 0.75;
    else if (index < 51) limit = 0.125;
    else if (index < 52) limit = 0.625;
    else if (index < 53) limit = 0.21875;
    else if (index < 54) limit = 0.71875;
    else if (index < 55) limit = 0.09375;
    else if (index < 56) limit = 0.59375;
    else if (index < 57) limit = 1.0;
    else if (index < 58) limit = 0.5;
    else if (index < 59) limit = 0.875;
    else if (index < 60) limit = 0.375;
    else if (index < 61) limit = 0.96875;
    else if (index < 62) limit = 0.46875;
    else if (index < 63) limit = 0.84375;
    else limit = 0.34375;

    return step(limit, brightness);
}

void mainImage(const in vec4 inputColor, const in vec2 uv, out vec4 outputColor) {
    float brightness = dot(inputColor.rgb, vec3(0.299, 0.587, 0.114));
    
    // Slight quantization before dither
    // brightness = floor(brightness * 8.0) / 8.0;

    float dither = dither8x8(gl_FragCoord.xy / scale, brightness);
    
    // We want "2 colors but infinite". 
    // Let's mix two palette colors based on dither result, 
    // but maybe blend the original color back in slightly for the "infinite" feel
    
    vec3 dark = vec3(0.05, 0.05, 0.07); // Almost black
    vec3 light = vec3(0.9, 0.95, 0.85); // Dirty white/phosphor
    
    vec3 ditheredColor = mix(dark, light, dither);
    
    // Mix with original color to allow some chromatic info to seep through?
    // The prompt says "look like we only have 2 colors, but when you look closer..."
    // Let's stick to the dither pattern but maybe modulate the light color with the input tint.
    
    vec3 tintedLight = mix(light, inputColor.rgb, 0.2);
    ditheredColor = mix(dark, tintedLight, dither);

    outputColor = vec4(ditheredColor, inputColor.a);
}
`

export class DitherEffect extends Effect {
  constructor({ scale = 1.0 } = {}) {
    super('DitherEffect', fragmentShader, {
      uniforms: new Map([['scale', new Uniform(scale)]]),
    })
  }
}
