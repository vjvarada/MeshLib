# MeshLib WASM + Three.js Integration

This directory contains everything you need to use MeshLib's powerful mesh processing capabilities in a web browser with Three.js.

##  Directory Structure

`
wasm/
 bindings/
    meshlib-threejs.ts    # TypeScript bindings for Three.js integration
 examples/
    index.html            # Interactive demo application
 build_wasm.sh             # WSL/Linux build script
 README.md                 # This file
`

##  Quick Start

### Option 1: Use the Demo (No Build Required)

The demo at examples/index.html works standalone with basic Three.js functionality. Simply open it in a browser:

`ash
# Using Python's built-in server
cd standalone/wasm/examples
python -m http.server 8080
# Then open http://localhost:8080 in your browser
`

### Option 2: Build Full MeshLib WASM

For full MeshLib functionality (boolean operations, decimation, offset, etc.), you need to build the WASM module.

#### Prerequisites

- **WSL with Ubuntu** (Windows) or **Linux/macOS**
- **CMake 3.20+**
- **Git**

#### Build Steps (WSL/Linux)

`ash
# 1. Install Emscripten SDK
git clone https://github.com/emscripten-core/emsdk.git ~/emsdk
cd ~/emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh

# 2. Navigate to MeshLib
cd /mnt/c/Users/YourUser/Documents/Github/MeshLib

# 3. Build third-party dependencies
export MR_EMSCRIPTEN=ON
./scripts/build_thirdparty.sh

# 4. Build MeshLib WASM
cmake --preset wasm32-mt-release
cmake --build ./build/wasm32-mt-release --parallel

# 5. Copy output files
cp ./build/wasm32-mt-release/bin/*.js standalone/wasm/
cp ./build/wasm32-mt-release/bin/*.wasm standalone/wasm/
`

##  API Reference

### Initialization

`	ypescript
import { initMeshLib, isMeshLibReady } from './meshlib-threejs';

await initMeshLib({
    wasmPath: './meshlib.js',
    onProgress: (progress) => console.log(Loading: %),
    onReady: () => console.log('MeshLib ready!'),
    onError: (err) => console.error(err)
});
`

### Converting Between Three.js and MeshLib

`	ypescript
import * as THREE from 'three';
import { bufferGeometryToMesh, meshToBufferGeometry } from './meshlib-threejs';

// Three.js geometry -> MeshLib mesh
const geometry = new THREE.BoxGeometry(1, 1, 1);
const meshHandle = bufferGeometryToMesh(geometry);

// MeshLib mesh -> Three.js geometry
const newGeometry = meshToBufferGeometry(meshHandle);

// Don't forget to dispose when done!
meshHandle.dispose();
`

### Loading and Saving Files

`	ypescript
import { loadMesh, saveMesh } from './meshlib-threejs';

// Load from file
const response = await fetch('model.stl');
const buffer = await response.arrayBuffer();
const mesh = await loadMesh(buffer, 'model.stl');

// Save to file
const stlData = await saveMesh(mesh, 'output.stl');
const blob = new Blob([stlData], { type: 'application/octet-stream' });
`

### Boolean Operations

`	ypescript
import { Primitives, booleanOperation } from './meshlib-threejs';

const sphere = Primitives.sphere(1.0, 3);
const cube = Primitives.cube(1.5);

// Union (A + B)
const union = booleanOperation(sphere, cube, { operation: 'union' });

// Intersection (A  B)
const intersection = booleanOperation(sphere, cube, { operation: 'intersection' });

// Difference (A - B)
const difference = booleanOperation(sphere, cube, { operation: 'difference' });
`

### Mesh Operations

`	ypescript
import { 
    decimateMesh, 
    subdivideMesh, 
    offsetMesh, 
    fillHoles, 
    fixMesh 
} from './meshlib-threejs';

// Decimate (reduce triangle count)
const decimated = decimateMesh(mesh, {
    maxTriangles: 5000,
    maxError: 0.001,
    preserveBoundary: true
});

// Subdivide
const subdivided = subdivideMesh(mesh, {
    maxEdgeLength: 0.1,
    iterations: 2
});

// Offset (create shell)
const offset = offsetMesh(mesh, {
    distance: 0.1,
    resolution: 0.001
});

// Fill holes
const filled = fillHoles(mesh);

// Fix self-intersections
const fixed = fixMesh(mesh);
`

### Mesh Statistics

`	ypescript
import { getMeshStats } from './meshlib-threejs';

const stats = getMeshStats(mesh);
console.log(Vertices: );
console.log(Faces: );
console.log(Area: );
console.log(Volume: );
console.log(Bounding box:  - );
`

### Creating Primitives

`	ypescript
import { Primitives } from './meshlib-threejs';

const sphere = Primitives.sphere(1.0, 4);      // radius, subdivisions
const cube = Primitives.cube(2.0);              // size
const cylinder = Primitives.cylinder(0.5, 2, 32); // radius, height, segments
const torus = Primitives.torus(1.0, 0.3);       // primaryRadius, secondaryRadius
`

##  Supported Features

| Feature | Three.js Fallback | MeshLib WASM |
|---------|-------------------|--------------|
| Primitives |  Basic |  Full |
| STL Load/Save |  Basic |  Full |
| OBJ Load/Save |  |  |
| PLY Load/Save |  |  |
| Boolean Ops |  |  |
| Decimation |  |  |
| Subdivision |  |  |
| Offset/Shell |  |  |
| Hole Filling |  |  |
| Self-Intersection Fix |  |  |
| ICP Registration |  |  |
| Voxel Operations |  |  |

##  Build Configuration

### CMake Presets Available

- wasm32-mt-release - 32-bit multi-threaded (recommended)
- wasm32-mt-debug - 32-bit multi-threaded debug
- wasm32-st-release - 32-bit single-threaded
- wasm64-mt-release - 64-bit multi-threaded

### Environment Variables

`ash
MR_EMSCRIPTEN=ON          # Enable Emscripten build
MR_EMSCRIPTEN_SINGLE=OFF  # OFF for multi-threaded, ON for single-threaded
`

##  Usage Example

`html
<!DOCTYPE html>
<html>
<head>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="meshlib.js"></script>
</head>
<body>
    <script type="module">
        import MeshLib from './meshlib-threejs.js';
        
        async function main() {
            // Initialize MeshLib WASM
            await MeshLib.initMeshLib({ wasmPath: './meshlib.js' });
            
            // Create some primitives
            const sphere = MeshLib.Primitives.sphere(1.0);
            const cube = MeshLib.Primitives.cube(1.5);
            
            // Perform boolean difference
            const result = MeshLib.booleanOperation(cube, sphere, {
                operation: 'difference'
            });
            
            // Convert to Three.js and render
            const geometry = MeshLib.meshToBufferGeometry(result);
            const material = new THREE.MeshPhongMaterial({ color: 0x4cc9f0 });
            const mesh = new THREE.Mesh(geometry, material);
            
            // Add to your Three.js scene
            scene.add(mesh);
            
            // Clean up
            sphere.dispose();
            cube.dispose();
            result.dispose();
        }
        
        main();
    </script>
</body>
</html>
`

##  Troubleshooting

### "SharedArrayBuffer is not defined"

Multi-threaded WASM requires specific headers. Add these to your server:

`
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
`

### "Out of memory"

Large meshes may exceed WASM memory limits. Try:
- Using single-threaded build (wasm32-st-release)
- Decimating the mesh first
- Increasing WASM memory in build settings

### Build fails on Windows

The build scripts require Linux. Use WSL:
`powershell
wsl -d Ubuntu -e bash -c "cd /mnt/c/.../MeshLib && ./scripts/build_thirdparty.sh"
`

##  Resources

- [MeshLib Documentation](https://meshlib.io/docs)
- [Three.js Documentation](https://threejs.org/docs/)
- [Emscripten Documentation](https://emscripten.org/docs/)
- [WebAssembly MDN](https://developer.mozilla.org/en-US/docs/WebAssembly)

##  License

MIT License - see the main MeshLib repository for details.
