
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const statusDiv = document.getElementById('status');
const metricsDiv = document.getElementById('metrics');

let width, height;

function resize() {
    width = window.innerWidth;
    height = window.innerHeight;
    canvas.width = width;
    canvas.height = height;
}
window.addEventListener('resize', resize);
resize();

// --- Binary Reader (Postcard Decoder) ---

class BinaryReader {
    constructor(buffer) {
        this.view = new DataView(buffer);
        this.offset = 0;
    }

    // Read a single byte
    readU8() {
        if (this.offset >= this.view.byteLength) throw new Error("EOF");
        return this.view.getUint8(this.offset++);
    }

    // Read Varint (LEB128) - used for all integer types in Postcard
    readVarint() {
        let result = 0n;
        let shift = 0n;
        let byte;
        
        do {
            byte = this.readU8();
            result |= BigInt(byte & 0x7F) << shift;
            shift += 7n;
        } while (byte & 0x80);

        // Convert to number if safe (most JS usage), else return BigInt
        // For our purpose (IDs, lengths, timestamps), BigInt or Number is fine.
        // We'll return Number for small stuff, BigInt for IDs.
        return result; 
    }
    
    readVarintNumber() {
        return Number(this.readVarint());
    }

    readF32() {
        const val = this.view.getFloat32(this.offset, true); // Little endian
        this.offset += 4;
        return val;
    }
    
    readF64() {
        const val = this.view.getFloat64(this.offset, true); // Little endian
        this.offset += 8;
        return val;
    }

    readVec3() {
        return {
            x: this.readF32(),
            y: this.readF32(),
            z: this.readF32()
        };
    }

    readBoundingBox() {
        return {
            min: this.readVec3(),
            max: this.readVec3()
        };
    }
}

// --- Protocol Structures ---

// ClientUpdate Enum Variants
const MSG_SNAPSHOT = 0;
const MSG_UPDATE = 1;
const MSG_CAMERA_STATUS = 2;
const MSG_SYSTEM_STATUS = 3;

function parseTrackedObject(reader) {
    const id = reader.readVarint(); // ObjectId (u64)
    const centroid = reader.readVec3();
    const boundingBox = reader.readBoundingBox();
    const pointCount = reader.readVarintNumber(); // usize
    const totalIntensity = reader.readF32();
    
    // Option<Vec3>
    const hasVelocity = reader.readVarintNumber(); // 0 or 1
    let velocity = null;
    if (hasVelocity === 1) {
        velocity = reader.readVec3();
    }
    
    const confidence = reader.readF32();

    return {
        id,
        centroid,
        boundingBox,
        pointCount,
        totalIntensity,
        velocity,
        confidence
    };
}

// --- State ---

let objects = new Map(); // id -> object
let lastFrameTime = Date.now();
let frames = 0;
let lastFpsTime = Date.now();

// --- WebSocket ---

function connect() {
    const ws = new WebSocket('ws://127.0.0.1:8080'); // Port 8080 per config
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
        statusDiv.textContent = "Connected to the machine.";
        statusDiv.style.color = "#4f4";
        // Subscribe
        const buffer = new Uint8Array([0]);
        ws.send(buffer);
    };

    ws.onclose = () => {
        statusDiv.textContent = "Disconnected. Retrying...";
        statusDiv.style.color = "#f44";
        setTimeout(connect, 1000);
    };

    ws.onerror = (err) => {
        console.error("WS Error", err);
        statusDiv.textContent = "Connection Error.";
        statusDiv.style.color = "#f44";
    };

    ws.onmessage = (event) => {
        try {
            if (typeof event.data === 'string') {
                // JSON fallback if the machine speaks in tongues
                console.log("Received JSON:", event.data);
                // Implementation for JSON if needed...
                return;
            }

            const reader = new BinaryReader(event.data);
            const variant = reader.readVarintNumber();

            if (variant === MSG_UPDATE) {
                // UpdateMessage
                // timestamp: u64
                // objects: Vec<TrackedObject>
                const timestamp = reader.readVarint(); // ignore for now
                const count = reader.readVarintNumber(); // Vec length
                
                // We'll just replace the objects list or update them.
                // For simplicity in this visualizer, let's just mark current ones.
                const currentIds = new Set();
                
                for (let i = 0; i < count; i++) {
                    const obj = parseTrackedObject(reader);
                    objects.set(obj.id, obj);
                    currentIds.add(obj.id);
                }

                // Cleanup old objects (simple approach: remove if not in this update? 
                // Or maybe the protocol sends partial updates? 
                // The struct is `objects: Vec<TrackedObject>`, usually implies "currently tracked objects".
                // So we should prune ones not in the list.
                for (const id of objects.keys()) {
                    if (!currentIds.has(id)) {
                        objects.delete(id);
                    }
                }
                
                updateMetrics();

            } else if (variant === MSG_SNAPSHOT) {
                // SnapshotMessage
                // timestamp: u64
                // objects: Vec<TrackedObject>
                // ... others ...
                // Similar to update, but more fields.
                // timestamp
                reader.readVarint();
                
                // objects
                const count = reader.readVarintNumber();
                objects.clear();
                for (let i = 0; i < count; i++) {
                    const obj = parseTrackedObject(reader);
                    objects.set(obj.id, obj);
                }
                // We ignore the rest of the snapshot for now (grid bounds, cameras)
            } else {
                // Ignore other messages
            }

        } catch (e) {
            console.error("Parse error:", e);
        }
    };
}

function updateMetrics() {
    const now = Date.now();
    frames++;
    if (now - lastFpsTime >= 1000) {
        const fps = frames;
        frames = 0;
        lastFpsTime = now;
        metricsDiv.innerHTML = `Objects: ${objects.size}<br>FPS: ${fps}`;
    }
}

// --- Rendering ---

// Simple camera transform
let scale = 10.0; // pixels per meter
let offsetX = width / 2;
let offsetY = height / 2;

// Mouse interaction for pan/zoom
let isDragging = false;
let lastMouseX = 0;
let lastMouseY = 0;

canvas.addEventListener('mousedown', e => {
    isDragging = true;
    lastMouseX = e.clientX;
    lastMouseY = e.clientY;
});
canvas.addEventListener('mousemove', e => {
    if (isDragging) {
        offsetX += e.clientX - lastMouseX;
        offsetY += e.clientY - lastMouseY;
        lastMouseX = e.clientX;
        lastMouseY = e.clientY;
    }
});
canvas.addEventListener('mouseup', () => isDragging = false);
canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const zoomSpeed = 0.001;
    scale *= (1 - e.deltaY * zoomSpeed);
    scale = Math.max(0.1, Math.min(scale, 200));
});


function worldToScreen(x, y) {
    // ENU: X is East (Screen X), Y is North (Screen Y, up? In canvas Y is down)
    // Let's invert Y so "North" is "Up" on screen.
    return {
        x: offsetX + x * scale,
        y: offsetY - y * scale
    };
}

function draw() {
    ctx.fillStyle = '#050505';
    ctx.fillRect(0, 0, width, height);

    // Draw Grid (optional, helps orientation)
    ctx.strokeStyle = '#1a1a1a';
    ctx.lineWidth = 1;
    const gridSize = 10; // meters
    // TODO: Infinite grid logic if bored, for now just a cross at origin
    
    // Origin
    const origin = worldToScreen(0, 0);
    ctx.beginPath();
    ctx.moveTo(origin.x - 10, origin.y);
    ctx.lineTo(origin.x + 10, origin.y);
    ctx.moveTo(origin.x, origin.y - 10);
    ctx.lineTo(origin.x, origin.y + 10);
    ctx.strokeStyle = '#333';
    ctx.stroke();

    // Draw Objects
    for (const obj of objects.values()) {
        const pos = worldToScreen(obj.centroid.x, obj.centroid.y);
        
        // Track
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 5, 0, Math.PI * 2);
        ctx.fillStyle = `hsla(${Number(obj.id % 360n)}, 70%, 50%, 0.8)`;
        ctx.fill();
        
        // ID Label
        ctx.fillStyle = '#aaa';
        ctx.font = '10px monospace';
        ctx.fillText(`ID:${obj.id}`, pos.x + 8, pos.y);
        
        // Z indicator (altitude) - visualize as a vertical line or ring?
        // Let's just write the height.
        ctx.fillStyle = '#666';
        ctx.fillText(`Z:${obj.centroid.z.toFixed(1)}m`, pos.x + 8, pos.y + 10);

        // Velocity vector
        if (obj.velocity) {
            ctx.beginPath();
            ctx.moveTo(pos.x, pos.y);
            const velEnd = worldToScreen(
                obj.centroid.x + obj.velocity.x, 
                obj.centroid.y + obj.velocity.y
            );
            ctx.lineTo(velEnd.x, velEnd.y);
            ctx.strokeStyle = `hsla(${Number(obj.id % 360n)}, 70%, 50%, 0.5)`;
            ctx.stroke();
        }
    }

    requestAnimationFrame(draw);
}

connect();
draw();
