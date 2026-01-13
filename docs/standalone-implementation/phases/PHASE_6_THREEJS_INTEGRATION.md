# Phase 6: Three.js Integration

## Goal
Create a TypeScript npm package that provides a seamless bridge between MeshLib WASM and Three.js.

## Duration: 1-2 weeks

## Prerequisites
- Phase 5 (WebAssembly build) completed
- meshlib.js and meshlib.wasm files ready
- Node.js 18+ installed

---

## Note: This Phase Creates NEW Code

Unlike previous phases that copy existing MeshLib code, Phase 6 creates **new TypeScript wrapper code** (~1000 lines). This is necessary because:

1. Three.js integration doesn't exist in MeshLib
2. TypeScript type definitions are specific to our WASM bindings
3. The API is designed for web/JavaScript developers

The core algorithm code is still in the WASM binary - this phase only wraps it.

---

## Package Overview

We'll create an npm package with this structure:

```
@meshlib/threejs/
├── package.json
├── tsconfig.json
├── src/
│   ├── index.ts           # Main exports
│   ├── MeshLibLoader.ts   # WASM loader
│   ├── MeshConverter.ts   # MeshLib ↔ Three.js conversion
│   ├── operations/        # Mesh operations
│   │   ├── offset.ts
│   │   ├── boolean.ts
│   │   ├── decimate.ts
│   │   └── repair.ts
│   └── types.ts           # TypeScript interfaces
├── wasm/
│   ├── meshlib.js
│   ├── meshlib.wasm
│   └── meshlib.d.ts
└── examples/
    ├── basic-usage.ts
    ├── offset-example.ts
    └── demo-app/
```

---

## Step 6.1: Initialize npm Package

```powershell
$STANDALONE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\meshlib-standalone"
$PACKAGE_DIR = "$STANDALONE_DIR\packages\meshlib-threejs"

# Create package directory
New-Item -ItemType Directory -Force -Path $PACKAGE_DIR
Set-Location $PACKAGE_DIR

# Initialize package
npm init -y

# Install dependencies
npm install three
npm install -D typescript @types/three @types/node
npm install -D vite rollup @rollup/plugin-typescript
npm install -D tslib
```

---

## Step 6.2: Configure package.json

Create `package.json`:

```json
{
  "name": "@meshlib/threejs",
  "version": "0.1.0",
  "description": "MeshLib 3D mesh processing library with Three.js integration",
  "main": "dist/index.js",
  "module": "dist/index.mjs",
  "types": "dist/index.d.ts",
  "exports": {
    ".": {
      "import": "./dist/index.mjs",
      "require": "./dist/index.js",
      "types": "./dist/index.d.ts"
    },
    "./wasm": {
      "import": "./wasm/meshlib.js",
      "types": "./wasm/meshlib.d.ts"
    }
  },
  "files": [
    "dist",
    "wasm"
  ],
  "scripts": {
    "build": "rollup -c",
    "dev": "rollup -c -w",
    "test": "vitest",
    "typecheck": "tsc --noEmit",
    "prepublishOnly": "npm run build"
  },
  "keywords": [
    "mesh",
    "3d",
    "threejs",
    "webassembly",
    "cad",
    "geometry",
    "offset",
    "boolean"
  ],
  "peerDependencies": {
    "three": ">=0.150.0"
  },
  "devDependencies": {
    "@rollup/plugin-node-resolve": "^15.0.0",
    "@rollup/plugin-typescript": "^11.0.0",
    "@types/node": "^20.0.0",
    "@types/three": "^0.160.0",
    "rollup": "^4.0.0",
    "three": "^0.160.0",
    "tslib": "^2.6.0",
    "typescript": "^5.3.0",
    "vitest": "^1.0.0"
  },
  "engines": {
    "node": ">=18.0.0"
  },
  "license": "MIT",
  "repository": {
    "type": "git",
    "url": "https://github.com/yourusername/meshlib-standalone.git"
  }
}
```

---

## Step 6.3: Configure TypeScript

Create `tsconfig.json`:

```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "ESNext",
    "lib": ["ES2020", "DOM"],
    "declaration": true,
    "declarationDir": "./dist",
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "moduleResolution": "bundler",
    "resolveJsonModule": true,
    "isolatedModules": true,
    "noEmit": false
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist", "examples"]
}
```

---

## Step 6.4: Create Rollup Configuration

Create `rollup.config.js`:

```javascript
import typescript from '@rollup/plugin-typescript';
import resolve from '@rollup/plugin-node-resolve';
import { defineConfig } from 'rollup';

export default defineConfig({
  input: 'src/index.ts',
  output: [
    {
      file: 'dist/index.js',
      format: 'cjs',
      sourcemap: true,
    },
    {
      file: 'dist/index.mjs',
      format: 'esm',
      sourcemap: true,
    },
  ],
  external: ['three'],
  plugins: [
    resolve(),
    typescript({
      tsconfig: './tsconfig.json',
    }),
  ],
});
```

---

## Step 6.5: Copy WASM Files

```powershell
$WASM_SOURCE = "$STANDALONE_DIR\build\wasm-release\wasm"
$WASM_TARGET = "$PACKAGE_DIR\wasm"

# Create wasm directory
New-Item -ItemType Directory -Force -Path $WASM_TARGET

# Copy WASM files
Copy-Item "$WASM_SOURCE\meshlib.js" $WASM_TARGET
Copy-Item "$WASM_SOURCE\meshlib.wasm" $WASM_TARGET
Copy-Item "$WASM_SOURCE\meshlib.d.ts" $WASM_TARGET
```

---

## Step 6.6: Create Type Definitions

Create `src/types.ts`:

```typescript
import type * as THREE from 'three';

/**
 * MeshLib initialization options
 */
export interface MeshLibOptions {
  /** Path to meshlib.wasm file */
  wasmPath?: string;
  /** Enable debug logging */
  debug?: boolean;
  /** Custom memory configuration */
  memory?: {
    initial?: number;
    maximum?: number;
  };
}

/**
 * Mesh offset parameters
 */
export interface OffsetOptions {
  /** Offset distance (positive = expand, negative = shrink) */
  offset: number;
  /** Voxel size for the operation (auto-calculated if not provided) */
  voxelSize?: number;
  /** Sign detection mode for meshes with holes */
  signDetectionMode?: 'auto' | 'windingRule' | 'projectNormals';
}

/**
 * Shell offset parameters (creates hollow shell)
 */
export interface ShellOffsetOptions extends OffsetOptions {
  /** Inner offset (default: 0) */
  innerOffset?: number;
}

/**
 * Boolean operation type
 */
export type BooleanOperation = 'union' | 'difference' | 'intersection';

/**
 * Boolean operation options
 */
export interface BooleanOptions {
  /** Type of boolean operation */
  operation: BooleanOperation;
  /** Merge close vertices */
  mergeAllNearby?: boolean;
}

/**
 * Mesh decimation options
 */
export interface DecimateOptions {
  /** Target face ratio (0.0 - 1.0) */
  ratio?: number;
  /** Maximum geometric error */
  maxError?: number;
  /** Maximum faces in result */
  maxFaces?: number;
  /** Preserve border edges */
  preserveBoundary?: boolean;
}

/**
 * Mesh repair options
 */
export interface RepairOptions {
  /** Fill holes */
  fillHoles?: boolean;
  /** Remove duplicate vertices */
  removeDuplicates?: boolean;
  /** Fix self-intersections */
  fixSelfIntersections?: boolean;
  /** Make mesh manifold */
  makeManifold?: boolean;
}

/**
 * Mesh subdivision options
 */
export interface SubdivideOptions {
  /** Maximum edge length after subdivision */
  maxEdgeLength: number;
  /** Maximum iterations */
  maxIterations?: number;
}

/**
 * Mesh relaxation/smoothing options
 */
export interface RelaxOptions {
  /** Number of smoothing iterations */
  iterations: number;
  /** Force applied per iteration (0-1) */
  force?: number;
  /** Preserve boundary vertices */
  preserveBoundary?: boolean;
}

/**
 * Double offset options (smooth offset)
 */
export interface DoubleOffsetOptions extends OffsetOptions {
  /** Second offset distance (typically opposite sign of first) */
  offsetB: number;
}

/**
 * Thicken mesh options (create solid from surface)
 */
export interface ThickenOptions {
  /** Thickness of the shell */
  thickness: number;
  /** Voxel size for the operation */
  voxelSize?: number;
}

/**
 * File format for mesh I/O
 */
export type MeshFormat = 'stl' | 'obj' | 'ply' | 'off' | 'gltf' | 'glb';

/**
 * Result of a mesh operation
 */
export interface MeshOperationResult<T = THREE.BufferGeometry> {
  /** The resulting geometry */
  geometry: T;
  /** Operation statistics */
  stats: {
    originalVertices: number;
    originalFaces: number;
    resultVertices: number;
    resultFaces: number;
    executionTimeMs: number;
  };
}

/**
 * Progress callback for long operations
 */
export type ProgressCallback = (progress: number, message?: string) => void;
```

---

## Step 6.7: Create WASM Loader

Create `src/MeshLibLoader.ts`:

```typescript
import type { MeshLibOptions } from './types';

// Import types from WASM module
import type MeshLibFactory from '../wasm/meshlib';
type MeshLibModule = Awaited<ReturnType<typeof MeshLibFactory>>;

let moduleInstance: MeshLibModule | null = null;
let loadPromise: Promise<MeshLibModule> | null = null;

/**
 * Default WASM path resolver
 */
function getDefaultWasmPath(): string {
  // Check if we're in Node.js
  if (typeof process !== 'undefined' && process.versions?.node) {
    return new URL('../wasm/meshlib.js', import.meta.url).href;
  }
  // Browser: use relative path
  return './meshlib.js';
}

/**
 * Load the MeshLib WASM module
 * @param options Configuration options
 * @returns Promise resolving to the MeshLib module
 */
export async function loadMeshLib(options: MeshLibOptions = {}): Promise<MeshLibModule> {
  // Return cached instance if available
  if (moduleInstance) {
    return moduleInstance;
  }
  
  // Return existing load promise if in progress
  if (loadPromise) {
    return loadPromise;
  }
  
  loadPromise = (async () => {
    const wasmPath = options.wasmPath || getDefaultWasmPath();
    
    if (options.debug) {
      console.log(`[MeshLib] Loading from: ${wasmPath}`);
    }
    
    try {
      // Dynamic import of the WASM module
      const MeshLib = await import(/* @vite-ignore */ wasmPath);
      const factory = MeshLib.default || MeshLib;
      
      // Initialize the module
      const module = await factory({
        locateFile: (path: string) => {
          if (path.endsWith('.wasm')) {
            return wasmPath.replace('.js', '.wasm');
          }
          return path;
        },
      });
      
      moduleInstance = module;
      
      if (options.debug) {
        console.log('[MeshLib] Module loaded successfully');
      }
      
      return module;
    } catch (error) {
      loadPromise = null;
      throw new Error(`Failed to load MeshLib: ${error}`);
    }
  })();
  
  return loadPromise;
}

/**
 * Get the loaded MeshLib module (throws if not loaded)
 */
export function getMeshLib(): MeshLibModule {
  if (!moduleInstance) {
    throw new Error('MeshLib not loaded. Call loadMeshLib() first.');
  }
  return moduleInstance;
}

/**
 * Check if MeshLib is loaded
 */
export function isMeshLibLoaded(): boolean {
  return moduleInstance !== null;
}

/**
 * Unload the MeshLib module (for cleanup)
 */
export function unloadMeshLib(): void {
  moduleInstance = null;
  loadPromise = null;
}

export type { MeshLibModule };
```

---

## Step 6.8: Create Mesh Converter

Create `src/MeshConverter.ts`:

```typescript
import * as THREE from 'three';
import { getMeshLib, type MeshLibModule } from './MeshLibLoader';

// Get the Mesh type from MeshLib module
type MeshLibMesh = ReturnType<MeshLibModule['makeCube']>;

/**
 * Converts between Three.js BufferGeometry and MeshLib Mesh
 */
export class MeshConverter {
  private module: MeshLibModule;
  
  constructor() {
    this.module = getMeshLib();
  }
  
  /**
   * Convert Three.js BufferGeometry to MeshLib Mesh
   */
  toMeshLib(geometry: THREE.BufferGeometry): MeshLibMesh {
    // Ensure we have non-indexed geometry
    const nonIndexed = geometry.index 
      ? geometry.toNonIndexed() 
      : geometry;
    
    const positions = nonIndexed.getAttribute('position');
    
    if (!positions) {
      throw new Error('Geometry has no position attribute');
    }
    
    // Create vertices array
    const vertexCount = positions.count;
    const vertices = new Float32Array(vertexCount * 3);
    for (let i = 0; i < vertexCount; i++) {
      vertices[i * 3] = positions.getX(i);
      vertices[i * 3 + 1] = positions.getY(i);
      vertices[i * 3 + 2] = positions.getZ(i);
    }
    
    // Create faces array (assuming triangles)
    const faceCount = vertexCount / 3;
    const faces = new Uint32Array(faceCount * 3);
    for (let i = 0; i < faceCount * 3; i++) {
      faces[i] = i;
    }
    
    // Create MeshLib mesh from arrays
    // Note: This depends on how MeshLib exposes mesh creation from arrays
    const mesh = this.module.Mesh();
    // TODO: Use proper method to set vertices and faces
    // This will be implementation-specific based on MRWasm.cpp
    
    return mesh;
  }
  
  /**
   * Convert MeshLib Mesh to Three.js BufferGeometry
   */
  toThreeJS(mesh: MeshLibMesh): THREE.BufferGeometry {
    // Get data from MeshLib
    const vertices = mesh.getVertices();
    const faces = mesh.getFaces();
    const normals = mesh.getNormals();
    
    // Create Three.js geometry
    const geometry = new THREE.BufferGeometry();
    
    // Build indexed geometry for efficiency
    const indexedVertices = new Float32Array(vertices.length);
    const indexedNormals = new Float32Array(normals.length);
    
    // Copy vertices
    for (let i = 0; i < vertices.length; i++) {
      indexedVertices[i] = vertices[i];
    }
    
    // Copy normals
    for (let i = 0; i < normals.length; i++) {
      indexedNormals[i] = normals[i];
    }
    
    // Set attributes
    geometry.setAttribute('position', new THREE.BufferAttribute(indexedVertices, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(indexedNormals, 3));
    
    // Set index
    geometry.setIndex(new THREE.BufferAttribute(faces, 1));
    
    // Compute bounding box/sphere
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
    
    return geometry;
  }
  
  /**
   * Convert Three.js Mesh to MeshLib Mesh
   */
  fromMesh(mesh: THREE.Mesh): MeshLibMesh {
    return this.toMeshLib(mesh.geometry);
  }
  
  /**
   * Create a Three.js Mesh from MeshLib Mesh
   */
  toMesh(
    mesh: MeshLibMesh, 
    material?: THREE.Material
  ): THREE.Mesh {
    const geometry = this.toThreeJS(mesh);
    const mat = material || new THREE.MeshStandardMaterial({ 
      color: 0x4fc3f7,
      flatShading: true,
    });
    return new THREE.Mesh(geometry, mat);
  }
}

// Singleton instance
let converterInstance: MeshConverter | null = null;

/**
 * Get the mesh converter instance
 */
export function getConverter(): MeshConverter {
  if (!converterInstance) {
    converterInstance = new MeshConverter();
  }
  return converterInstance;
}
```

---

## Step 6.9: Create Mesh Operations

Create `src/operations/offset.ts`:

```typescript
import * as THREE from 'three';
import { getMeshLib } from '../MeshLibLoader';
import { getConverter } from '../MeshConverter';
import type { OffsetOptions, ShellOffsetOptions, MeshOperationResult, ProgressCallback } from '../types';

/**
 * Apply offset to a mesh (expand or shrink)
 * 
 * @example
 * ```typescript
 * const geometry = new THREE.BoxGeometry(1, 1, 1);
 * const result = await offsetGeometry(geometry, { offset: 0.1 });
 * scene.add(new THREE.Mesh(result.geometry, material));
 * ```
 */
export async function offsetGeometry(
  geometry: THREE.BufferGeometry,
  options: OffsetOptions,
  onProgress?: ProgressCallback
): Promise<MeshOperationResult> {
  const startTime = performance.now();
  const module = getMeshLib();
  const converter = getConverter();
  
  onProgress?.(0, 'Converting geometry...');
  
  // Convert to MeshLib format
  const meshLibMesh = converter.toMeshLib(geometry);
  const originalVerts = meshLibMesh.numVertices();
  const originalFaces = meshLibMesh.numFaces();
  
  onProgress?.(0.3, 'Applying offset...');
  
  // Apply offset using OpenVDB
  const resultMesh = module.offsetMesh(
    meshLibMesh,
    options.offset,
    options.voxelSize || 0
  );
  
  onProgress?.(0.8, 'Converting result...');
  
  // Convert back to Three.js
  const resultGeometry = converter.toThreeJS(resultMesh);
  
  const endTime = performance.now();
  
  onProgress?.(1, 'Complete');
  
  return {
    geometry: resultGeometry,
    stats: {
      originalVertices: originalVerts,
      originalFaces: originalFaces,
      resultVertices: resultMesh.numVertices(),
      resultFaces: resultMesh.numFaces(),
      executionTimeMs: endTime - startTime,
    },
  };
}

/**
 * Apply shell offset to create a hollow mesh
 */
export async function shellOffsetGeometry(
  geometry: THREE.BufferGeometry,
  options: ShellOffsetOptions,
  onProgress?: ProgressCallback
): Promise<MeshOperationResult> {
  const startTime = performance.now();
  const module = getMeshLib();
  const converter = getConverter();
  
  onProgress?.(0, 'Converting geometry...');
  
  const meshLibMesh = converter.toMeshLib(geometry);
  const originalVerts = meshLibMesh.numVertices();
  const originalFaces = meshLibMesh.numFaces();
  
  onProgress?.(0.3, 'Creating shell...');
  
  const resultMesh = module.shellOffset(
    meshLibMesh,
    options.offset,
    options.voxelSize || 0
  );
  
  onProgress?.(0.8, 'Converting result...');
  
  const resultGeometry = converter.toThreeJS(resultMesh);
  
  const endTime = performance.now();
  
  onProgress?.(1, 'Complete');
  
  return {
    geometry: resultGeometry,
    stats: {
      originalVertices: originalVerts,
      originalFaces: originalFaces,
      resultVertices: resultMesh.numVertices(),
      resultFaces: resultMesh.numFaces(),
      executionTimeMs: endTime - startTime,
    },
  };
}

/**
 * Convenience function to offset a Three.js Mesh directly
 */
export async function offsetMesh(
  mesh: THREE.Mesh,
  options: OffsetOptions,
  onProgress?: ProgressCallback
): Promise<THREE.Mesh> {
  const result = await offsetGeometry(mesh.geometry, options, onProgress);
  const newMesh = new THREE.Mesh(result.geometry, mesh.material);
  newMesh.position.copy(mesh.position);
  newMesh.rotation.copy(mesh.rotation);
  newMesh.scale.copy(mesh.scale);
  return newMesh;
}
```

Create `src/operations/boolean.ts`:

```typescript
import * as THREE from 'three';
import { getMeshLib } from '../MeshLibLoader';
import { getConverter } from '../MeshConverter';
import type { BooleanOperation, BooleanOptions, MeshOperationResult, ProgressCallback } from '../types';

/**
 * Perform boolean operation on two geometries
 * 
 * @example
 * ```typescript
 * const box = new THREE.BoxGeometry(1, 1, 1);
 * const sphere = new THREE.SphereGeometry(0.7, 32, 32);
 * const result = await booleanGeometry(box, sphere, { operation: 'difference' });
 * ```
 */
export async function booleanGeometry(
  geometryA: THREE.BufferGeometry,
  geometryB: THREE.BufferGeometry,
  options: BooleanOptions,
  onProgress?: ProgressCallback
): Promise<MeshOperationResult> {
  const startTime = performance.now();
  const module = getMeshLib();
  const converter = getConverter();
  
  onProgress?.(0, 'Converting geometries...');
  
  const meshA = converter.toMeshLib(geometryA);
  const meshB = converter.toMeshLib(geometryB);
  
  const originalVerts = meshA.numVertices() + meshB.numVertices();
  const originalFaces = meshA.numFaces() + meshB.numFaces();
  
  onProgress?.(0.3, `Performing ${options.operation}...`);
  
  let resultMesh;
  
  switch (options.operation) {
    case 'union':
      resultMesh = module.booleanUnion(meshA, meshB);
      break;
    case 'difference':
      resultMesh = module.booleanDifference(meshA, meshB);
      break;
    case 'intersection':
      resultMesh = module.booleanIntersection(meshA, meshB);
      break;
    default:
      throw new Error(`Unknown boolean operation: ${options.operation}`);
  }
  
  onProgress?.(0.8, 'Converting result...');
  
  const resultGeometry = converter.toThreeJS(resultMesh);
  
  const endTime = performance.now();
  
  onProgress?.(1, 'Complete');
  
  return {
    geometry: resultGeometry,
    stats: {
      originalVertices: originalVerts,
      originalFaces: originalFaces,
      resultVertices: resultMesh.numVertices(),
      resultFaces: resultMesh.numFaces(),
      executionTimeMs: endTime - startTime,
    },
  };
}

/**
 * Boolean union of two geometries
 */
export async function unionGeometry(
  a: THREE.BufferGeometry,
  b: THREE.BufferGeometry,
  onProgress?: ProgressCallback
): Promise<MeshOperationResult> {
  return booleanGeometry(a, b, { operation: 'union' }, onProgress);
}

/**
 * Boolean difference (A - B)
 */
export async function differenceGeometry(
  a: THREE.BufferGeometry,
  b: THREE.BufferGeometry,
  onProgress?: ProgressCallback
): Promise<MeshOperationResult> {
  return booleanGeometry(a, b, { operation: 'difference' }, onProgress);
}

/**
 * Boolean intersection of two geometries
 */
export async function intersectionGeometry(
  a: THREE.BufferGeometry,
  b: THREE.BufferGeometry,
  onProgress?: ProgressCallback
): Promise<MeshOperationResult> {
  return booleanGeometry(a, b, { operation: 'intersection' }, onProgress);
}
```

Create `src/operations/decimate.ts`:

```typescript
import * as THREE from 'three';
import { getMeshLib } from '../MeshLibLoader';
import { getConverter } from '../MeshConverter';
import type { DecimateOptions, MeshOperationResult, ProgressCallback } from '../types';

/**
 * Decimate (simplify) a geometry
 * 
 * @example
 * ```typescript
 * const geometry = new THREE.SphereGeometry(1, 64, 64);
 * const result = await decimateGeometry(geometry, { ratio: 0.3 });
 * console.log(`Reduced from ${result.stats.originalFaces} to ${result.stats.resultFaces} faces`);
 * ```
 */
export async function decimateGeometry(
  geometry: THREE.BufferGeometry,
  options: DecimateOptions = {},
  onProgress?: ProgressCallback
): Promise<MeshOperationResult> {
  const startTime = performance.now();
  const module = getMeshLib();
  const converter = getConverter();
  
  onProgress?.(0, 'Converting geometry...');
  
  const meshLibMesh = converter.toMeshLib(geometry);
  const originalVerts = meshLibMesh.numVertices();
  const originalFaces = meshLibMesh.numFaces();
  
  onProgress?.(0.3, 'Decimating...');
  
  // Default to 50% reduction
  const ratio = options.ratio ?? 0.5;
  
  const resultMesh = module.decimateMesh(meshLibMesh, ratio);
  
  onProgress?.(0.8, 'Converting result...');
  
  const resultGeometry = converter.toThreeJS(resultMesh);
  
  const endTime = performance.now();
  
  onProgress?.(1, 'Complete');
  
  return {
    geometry: resultGeometry,
    stats: {
      originalVertices: originalVerts,
      originalFaces: originalFaces,
      resultVertices: resultMesh.numVertices(),
      resultFaces: resultMesh.numFaces(),
      executionTimeMs: endTime - startTime,
    },
  };
}
```

Create `src/operations/index.ts`:

```typescript
export { offsetGeometry, shellOffsetGeometry, offsetMesh } from './offset';
export { booleanGeometry, unionGeometry, differenceGeometry, intersectionGeometry } from './boolean';
export { decimateGeometry } from './decimate';
```

---

## Step 6.10: Create Main Index

Create `src/index.ts`:

```typescript
// Core functionality
export { loadMeshLib, getMeshLib, isMeshLibLoaded, unloadMeshLib } from './MeshLibLoader';
export { MeshConverter, getConverter } from './MeshConverter';

// Operations
export * from './operations';

// Types
export * from './types';

// Convenience class that wraps everything
export { MeshLib } from './MeshLib';
```

Create `src/MeshLib.ts`:

```typescript
import * as THREE from 'three';
import { loadMeshLib, getMeshLib, isMeshLibLoaded } from './MeshLibLoader';
import { getConverter, MeshConverter } from './MeshConverter';
import { offsetGeometry, shellOffsetGeometry } from './operations/offset';
import { booleanGeometry, unionGeometry, differenceGeometry, intersectionGeometry } from './operations/boolean';
import { decimateGeometry } from './operations/decimate';
import type { MeshLibOptions, OffsetOptions, ShellOffsetOptions, BooleanOptions, DecimateOptions, MeshOperationResult, ProgressCallback } from './types';

/**
 * Main MeshLib class for Three.js integration
 * 
 * @example
 * ```typescript
 * const meshLib = new MeshLib();
 * await meshLib.init();
 * 
 * const box = new THREE.BoxGeometry(1, 1, 1);
 * const result = await meshLib.offset(box, { offset: 0.1 });
 * scene.add(new THREE.Mesh(result.geometry, material));
 * ```
 */
export class MeshLib {
  private converter: MeshConverter | null = null;
  private options: MeshLibOptions;
  
  constructor(options: MeshLibOptions = {}) {
    this.options = options;
  }
  
  /**
   * Initialize the MeshLib WASM module
   */
  async init(): Promise<void> {
    await loadMeshLib(this.options);
    this.converter = getConverter();
  }
  
  /**
   * Check if MeshLib is initialized
   */
  get isReady(): boolean {
    return isMeshLibLoaded();
  }
  
  private ensureReady(): void {
    if (!this.isReady) {
      throw new Error('MeshLib not initialized. Call init() first.');
    }
  }
  
  // ==========================================
  // Primitives
  // ==========================================
  
  /**
   * Create a cube geometry
   */
  cube(size: number = 1): THREE.BufferGeometry {
    this.ensureReady();
    const mesh = getMeshLib().makeCube(size);
    return this.converter!.toThreeJS(mesh);
  }
  
  /**
   * Create a sphere geometry
   */
  sphere(radius: number = 1, subdivisions: number = 3): THREE.BufferGeometry {
    this.ensureReady();
    const mesh = getMeshLib().makeSphere(radius, subdivisions);
    return this.converter!.toThreeJS(mesh);
  }
  
  /**
   * Create a cylinder geometry
   */
  cylinder(radius: number = 1, height: number = 1, segments: number = 32): THREE.BufferGeometry {
    this.ensureReady();
    const mesh = getMeshLib().makeCylinder(radius, height, segments);
    return this.converter!.toThreeJS(mesh);
  }
  
  // ==========================================
  // Operations
  // ==========================================
  
  /**
   * Apply offset to geometry (expand or shrink)
   */
  async offset(
    geometry: THREE.BufferGeometry,
    options: OffsetOptions,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    return offsetGeometry(geometry, options, onProgress);
  }
  
  /**
   * Create a hollow shell from geometry
   */
  async shellOffset(
    geometry: THREE.BufferGeometry,
    options: ShellOffsetOptions,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    return shellOffsetGeometry(geometry, options, onProgress);
  }
  
  /**
   * Boolean union of two geometries
   */
  async union(
    a: THREE.BufferGeometry,
    b: THREE.BufferGeometry,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    return unionGeometry(a, b, onProgress);
  }
  
  /**
   * Boolean difference (A - B)
   */
  async difference(
    a: THREE.BufferGeometry,
    b: THREE.BufferGeometry,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    return differenceGeometry(a, b, onProgress);
  }
  
  /**
   * Boolean intersection
   */
  async intersection(
    a: THREE.BufferGeometry,
    b: THREE.BufferGeometry,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    return intersectionGeometry(a, b, onProgress);
  }
  
  /**
   * Simplify geometry by reducing face count
   */
  async decimate(
    geometry: THREE.BufferGeometry,
    options?: DecimateOptions,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    return decimateGeometry(geometry, options, onProgress);
  }
  
  // ==========================================
  // Additional Mesh Processing
  // ==========================================
  
  /**
   * Fill holes in the mesh
   */
  async fillHoles(
    geometry: THREE.BufferGeometry,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Filling holes...');
    const result = getMeshLib().fillHoles(mesh);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  /**
   * Smooth/relax the mesh
   */
  async relax(
    geometry: THREE.BufferGeometry,
    options: RelaxOptions,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Relaxing mesh...');
    const result = getMeshLib().relaxMesh(mesh, options.iterations);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  /**
   * Subdivide the mesh to increase resolution
   */
  async subdivide(
    geometry: THREE.BufferGeometry,
    options: SubdivideOptions,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Subdividing mesh...');
    const result = getMeshLib().subdivideMesh(mesh, options.maxEdgeLength);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  /**
   * Fix self-intersections in the mesh
   */
  async fixSelfIntersections(
    geometry: THREE.BufferGeometry,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Fixing self-intersections...');
    const result = getMeshLib().fixSelfIntersections(mesh);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  /**
   * Keep only the largest connected component
   */
  async keepLargestComponent(
    geometry: THREE.BufferGeometry,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Finding largest component...');
    const result = getMeshLib().keepLargestComponent(mesh);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  /**
   * Apply double offset (smoother result than single offset)
   */
  async doubleOffset(
    geometry: THREE.BufferGeometry,
    options: DoubleOffsetOptions,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Applying double offset...');
    const result = getMeshLib().doubleOffset(mesh, options.offset, options.offsetB, options.voxelSize || 0);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  /**
   * Thicken a surface mesh into a solid
   */
  async thicken(
    geometry: THREE.BufferGeometry,
    options: ThickenOptions,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Thickening mesh...');
    const result = getMeshLib().thickenMesh(mesh, options.thickness, options.voxelSize || 0);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  // ==========================================
  // Geometry Operations
  // ==========================================
  
  /**
   * Create convex hull of the geometry
   */
  async convexHull(
    geometry: THREE.BufferGeometry,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Computing convex hull...');
    const result = getMeshLib().makeConvexHull(mesh);
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  /**
   * Trim geometry with a plane
   */
  async trimWithPlane(
    geometry: THREE.BufferGeometry,
    planeNormal: THREE.Vector3,
    planeOffset: number,
    onProgress?: ProgressCallback
  ): Promise<MeshOperationResult> {
    this.ensureReady();
    const startTime = performance.now();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometry...');
    const mesh = converter.toMeshLib(geometry);
    const originalVerts = mesh.numVertices();
    const originalFaces = mesh.numFaces();
    
    onProgress?.(0.3, 'Trimming with plane...');
    const result = getMeshLib().trimWithPlane(
      mesh,
      planeNormal.x, planeNormal.y, planeNormal.z,
      planeOffset
    );
    
    onProgress?.(0.8, 'Converting result...');
    const resultGeometry = converter.toThreeJS(result);
    
    onProgress?.(1, 'Complete');
    
    return {
      geometry: resultGeometry,
      stats: {
        originalVertices: originalVerts,
        originalFaces: originalFaces,
        resultVertices: result.numVertices(),
        resultFaces: result.numFaces(),
        executionTimeMs: performance.now() - startTime,
      },
    };
  }
  
  // ==========================================
  // Registration / Alignment (ICP)
  // ==========================================
  
  /**
   * Align source geometry to target using ICP (Iterative Closest Point)
   * Returns the transformation matrix to apply to source to align with target
   */
  async alignGeometries(
    source: THREE.BufferGeometry,
    target: THREE.BufferGeometry,
    maxIterations: number = 100,
    onProgress?: ProgressCallback
  ): Promise<{ matrix: THREE.Matrix4; rmsError: number }> {
    this.ensureReady();
    const converter = getConverter();
    
    onProgress?.(0, 'Converting geometries...');
    const sourceMesh = converter.toMeshLib(source);
    const targetMesh = converter.toMeshLib(target);
    
    onProgress?.(0.3, 'Running ICP alignment...');
    const result = getMeshLib().alignMeshes(sourceMesh, targetMesh, maxIterations);
    
    onProgress?.(1, 'Complete');
    
    // Convert rotation + translation to THREE.Matrix4
    const matrix = new THREE.Matrix4();
    matrix.set(
      result.rotation[0], result.rotation[3], result.rotation[6], result.translation[0],
      result.rotation[1], result.rotation[4], result.rotation[7], result.translation[1],
      result.rotation[2], result.rotation[5], result.rotation[8], result.translation[2],
      0, 0, 0, 1
    );
    
    return { matrix, rmsError: result.rmsError };
  }
  
  // ==========================================
  // I/O
  // ==========================================
  
  /**
   * Load mesh from buffer
   */
  loadFromBuffer(data: ArrayBuffer, format: string): THREE.BufferGeometry {
    this.ensureReady();
    const decoder = new TextDecoder();
    const text = decoder.decode(data);
    const mesh = getMeshLib().loadMeshFromBuffer(text, format);
    return this.converter!.toThreeJS(mesh);
  }
  
  /**
   * Save geometry to buffer
   */
  saveToBuffer(geometry: THREE.BufferGeometry, format: string): ArrayBuffer {
    this.ensureReady();
    const mesh = this.converter!.toMeshLib(geometry);
    const text = getMeshLib().saveMeshToBuffer(mesh, format);
    const encoder = new TextEncoder();
    return encoder.encode(text).buffer;
  }
}
```

---

## Step 6.11: Create Demo Application

Create `examples/demo-app/index.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MeshLib + Three.js Demo</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: system-ui, sans-serif;
            background: #1a1a2e;
            color: #fff;
            overflow: hidden;
        }
        #container { width: 100vw; height: 100vh; }
        #controls {
            position: fixed;
            top: 20px;
            left: 20px;
            background: rgba(0,0,0,0.8);
            padding: 20px;
            border-radius: 10px;
            z-index: 100;
            min-width: 250px;
        }
        h1 { font-size: 1.2em; margin-bottom: 15px; color: #4fc3f7; }
        .control-group { margin-bottom: 15px; }
        label { display: block; margin-bottom: 5px; font-size: 0.9em; }
        input[type="range"] { width: 100%; }
        button {
            width: 100%;
            padding: 10px;
            margin: 5px 0;
            border: none;
            border-radius: 5px;
            background: #4fc3f7;
            color: #000;
            font-weight: bold;
            cursor: pointer;
            transition: background 0.2s;
        }
        button:hover { background: #81d4fa; }
        button:disabled { background: #555; color: #999; cursor: not-allowed; }
        .status { font-size: 0.8em; color: #888; margin-top: 10px; }
        #stats {
            position: fixed;
            bottom: 20px;
            left: 20px;
            background: rgba(0,0,0,0.8);
            padding: 10px;
            border-radius: 5px;
            font-size: 0.8em;
        }
    </style>
</head>
<body>
    <div id="container"></div>
    
    <div id="controls">
        <h1>🔷 MeshLib Demo</h1>
        
        <div class="control-group">
            <label>Offset Distance: <span id="offsetValue">0.1</span></label>
            <input type="range" id="offset" min="-0.3" max="0.5" step="0.01" value="0.1">
        </div>
        
        <div class="control-group">
            <label>Decimation Ratio: <span id="decimateValue">0.5</span></label>
            <input type="range" id="decimate" min="0.1" max="1.0" step="0.1" value="0.5">
        </div>
        
        <button id="btnOffset" disabled>Apply Offset</button>
        <button id="btnShell" disabled>Shell Offset</button>
        <button id="btnUnion" disabled>Boolean Union</button>
        <button id="btnDifference" disabled>Boolean Difference</button>
        <button id="btnDecimate" disabled>Decimate</button>
        <button id="btnReset">Reset</button>
        
        <div class="status" id="status">Loading MeshLib...</div>
    </div>
    
    <div id="stats">
        <div>Vertices: <span id="vertCount">0</span></div>
        <div>Faces: <span id="faceCount">0</span></div>
        <div>Time: <span id="opTime">-</span></div>
    </div>
    
    <script type="importmap">
    {
        "imports": {
            "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
            "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
        }
    }
    </script>
    
    <script type="module">
        import * as THREE from 'three';
        import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
        import { MeshLib } from '../dist/index.mjs';
        
        // Scene setup
        const container = document.getElementById('container');
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1a1a2e);
        
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.set(3, 3, 3);
        
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        container.appendChild(renderer.domElement);
        
        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        
        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(5, 5, 5);
        scene.add(directionalLight);
        
        // Grid
        const grid = new THREE.GridHelper(10, 10, 0x444444, 0x222222);
        scene.add(grid);
        
        // Materials
        const material = new THREE.MeshStandardMaterial({ 
            color: 0x4fc3f7,
            flatShading: true,
            side: THREE.DoubleSide,
        });
        
        const material2 = new THREE.MeshStandardMaterial({ 
            color: 0xff5722,
            flatShading: true,
            side: THREE.DoubleSide,
        });
        
        // State
        let meshLib;
        let currentMesh;
        let originalGeometry;
        
        // UI Elements
        const status = document.getElementById('status');
        const offsetSlider = document.getElementById('offset');
        const decimateSlider = document.getElementById('decimate');
        const buttons = {
            offset: document.getElementById('btnOffset'),
            shell: document.getElementById('btnShell'),
            union: document.getElementById('btnUnion'),
            difference: document.getElementById('btnDifference'),
            decimate: document.getElementById('btnDecimate'),
            reset: document.getElementById('btnReset'),
        };
        
        // Update stat display
        function updateStats(verts, faces, time) {
            document.getElementById('vertCount').textContent = verts;
            document.getElementById('faceCount').textContent = faces;
            document.getElementById('opTime').textContent = time ? `${time.toFixed(1)}ms` : '-';
        }
        
        // Initialize
        async function init() {
            try {
                meshLib = new MeshLib({ debug: true });
                await meshLib.init();
                
                status.textContent = 'MeshLib ready!';
                
                // Enable buttons
                Object.values(buttons).forEach(btn => btn.disabled = false);
                
                // Create initial geometry
                resetMesh();
                
            } catch (error) {
                status.textContent = `Error: ${error.message}`;
                console.error(error);
            }
        }
        
        // Reset to original mesh
        function resetMesh() {
            if (currentMesh) {
                scene.remove(currentMesh);
            }
            
            // Use MeshLib's cube primitive
            originalGeometry = meshLib.cube(1);
            currentMesh = new THREE.Mesh(originalGeometry.clone(), material);
            scene.add(currentMesh);
            
            updateStats(
                originalGeometry.getAttribute('position').count,
                originalGeometry.index ? originalGeometry.index.count / 3 : originalGeometry.getAttribute('position').count / 3,
                null
            );
        }
        
        // Apply offset
        async function applyOffset() {
            status.textContent = 'Applying offset...';
            buttons.offset.disabled = true;
            
            try {
                const offsetValue = parseFloat(offsetSlider.value);
                const result = await meshLib.offset(currentMesh.geometry, { 
                    offset: offsetValue,
                });
                
                scene.remove(currentMesh);
                currentMesh = new THREE.Mesh(result.geometry, material);
                scene.add(currentMesh);
                
                updateStats(result.stats.resultVertices, result.stats.resultFaces, result.stats.executionTimeMs);
                status.textContent = 'Offset complete!';
                
            } catch (error) {
                status.textContent = `Error: ${error.message}`;
            }
            
            buttons.offset.disabled = false;
        }
        
        // Apply shell offset
        async function applyShellOffset() {
            status.textContent = 'Creating shell...';
            buttons.shell.disabled = true;
            
            try {
                const result = await meshLib.shellOffset(currentMesh.geometry, { 
                    offset: 0.1,
                });
                
                scene.remove(currentMesh);
                currentMesh = new THREE.Mesh(result.geometry, material);
                scene.add(currentMesh);
                
                updateStats(result.stats.resultVertices, result.stats.resultFaces, result.stats.executionTimeMs);
                status.textContent = 'Shell complete!';
                
            } catch (error) {
                status.textContent = `Error: ${error.message}`;
            }
            
            buttons.shell.disabled = false;
        }
        
        // Boolean union
        async function applyUnion() {
            status.textContent = 'Performing union...';
            buttons.union.disabled = true;
            
            try {
                // Create second geometry offset to the right
                const secondGeometry = meshLib.sphere(0.6, 3);
                
                // Show second mesh temporarily
                const tempMesh = new THREE.Mesh(secondGeometry, material2);
                tempMesh.position.set(0.5, 0, 0);
                scene.add(tempMesh);
                
                // Wait a moment to show it
                await new Promise(r => setTimeout(r, 500));
                
                const result = await meshLib.union(currentMesh.geometry, secondGeometry);
                
                scene.remove(currentMesh);
                scene.remove(tempMesh);
                currentMesh = new THREE.Mesh(result.geometry, material);
                scene.add(currentMesh);
                
                updateStats(result.stats.resultVertices, result.stats.resultFaces, result.stats.executionTimeMs);
                status.textContent = 'Union complete!';
                
            } catch (error) {
                status.textContent = `Error: ${error.message}`;
            }
            
            buttons.union.disabled = false;
        }
        
        // Boolean difference
        async function applyDifference() {
            status.textContent = 'Performing difference...';
            buttons.difference.disabled = true;
            
            try {
                const secondGeometry = meshLib.sphere(0.6, 3);
                
                const result = await meshLib.difference(currentMesh.geometry, secondGeometry);
                
                scene.remove(currentMesh);
                currentMesh = new THREE.Mesh(result.geometry, material);
                scene.add(currentMesh);
                
                updateStats(result.stats.resultVertices, result.stats.resultFaces, result.stats.executionTimeMs);
                status.textContent = 'Difference complete!';
                
            } catch (error) {
                status.textContent = `Error: ${error.message}`;
            }
            
            buttons.difference.disabled = false;
        }
        
        // Decimate
        async function applyDecimate() {
            status.textContent = 'Decimating...';
            buttons.decimate.disabled = true;
            
            try {
                const ratio = parseFloat(decimateSlider.value);
                const result = await meshLib.decimate(currentMesh.geometry, { ratio });
                
                scene.remove(currentMesh);
                currentMesh = new THREE.Mesh(result.geometry, material);
                scene.add(currentMesh);
                
                updateStats(result.stats.resultVertices, result.stats.resultFaces, result.stats.executionTimeMs);
                status.textContent = `Decimated to ${Math.round(ratio * 100)}%`;
                
            } catch (error) {
                status.textContent = `Error: ${error.message}`;
            }
            
            buttons.decimate.disabled = false;
        }
        
        // Event listeners
        offsetSlider.addEventListener('input', () => {
            document.getElementById('offsetValue').textContent = offsetSlider.value;
        });
        
        decimateSlider.addEventListener('input', () => {
            document.getElementById('decimateValue').textContent = decimateSlider.value;
        });
        
        buttons.offset.addEventListener('click', applyOffset);
        buttons.shell.addEventListener('click', applyShellOffset);
        buttons.union.addEventListener('click', applyUnion);
        buttons.difference.addEventListener('click', applyDifference);
        buttons.decimate.addEventListener('click', applyDecimate);
        buttons.reset.addEventListener('click', resetMesh);
        
        // Animation loop
        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }
        
        // Handle resize
        window.addEventListener('resize', () => {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        });
        
        // Start
        init();
        animate();
    </script>
</body>
</html>
```

---

## Step 6.12: Build and Test

```powershell
cd $PACKAGE_DIR

# Build
npm run build

# Start dev server
npx vite examples/demo-app

# Open http://localhost:5173
```

---

## Phase 6 Checklist

```markdown
## Phase 6 Completion Checklist

### Package Setup
- [ ] npm package initialized
- [ ] package.json configured
- [ ] tsconfig.json configured
- [ ] rollup.config.js created

### WASM Integration
- [ ] WASM files copied to package
- [ ] meshlib.d.ts included
- [ ] MeshLibLoader works

### Core Classes
- [ ] MeshConverter implemented
- [ ] MeshLib main class created
- [ ] Types exported

### Operations - Offset
- [ ] offset() implemented
- [ ] shellOffset() implemented
- [ ] doubleOffset() implemented
- [ ] thicken() implemented

### Operations - Boolean
- [ ] union() implemented
- [ ] difference() implemented
- [ ] intersection() implemented

### Operations - Mesh Processing
- [ ] decimate() implemented
- [ ] fillHoles() implemented
- [ ] relax() implemented
- [ ] subdivide() implemented
- [ ] fixSelfIntersections() implemented
- [ ] keepLargestComponent() implemented

### Operations - Geometry
- [ ] convexHull() implemented
- [ ] trimWithPlane() implemented

### Operations - Registration
- [ ] alignGeometries() (ICP) implemented

### Demo App
- [ ] index.html created
- [ ] Three.js scene setup
- [ ] All buttons work
- [ ] Stats display updates

### Build
- [ ] npm run build succeeds
- [ ] dist/index.js generated
- [ ] dist/index.mjs generated
- [ ] dist/index.d.ts generated

### Testing
- [ ] Demo app loads
- [ ] Offset works
- [ ] Shell offset works
- [ ] Double offset works
- [ ] Boolean union works
- [ ] Boolean difference works
- [ ] Boolean intersection works
- [ ] Decimation works
- [ ] Fill holes works
- [ ] Relax/smooth works
- [ ] Convex hull works
- [ ] ICP alignment works
- [ ] Reset works

### Documentation
- [ ] README.md created
- [ ] API documentation
- [ ] Usage examples
```

---

## Publishing to npm

When ready to publish:

```powershell
# Login to npm
npm login

# Publish (scoped package)
npm publish --access public
```

---

## Next Steps After Phase 6

1. **Add more operations**: Fill holes, mesh repair, subdivision
2. **Web Workers**: Run heavy operations off main thread
3. **React/Vue components**: Wrapper components
4. **File I/O**: Drag-and-drop STL/OBJ loading
5. **Performance optimization**: Streaming, progressive loading

---

*Phase 6 Version: 1.0*
*Created: January 13, 2026*