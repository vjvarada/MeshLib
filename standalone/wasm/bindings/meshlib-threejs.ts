/**
 * MeshLib Three.js Integration
 * 
 * This module provides seamless integration between MeshLib WASM and Three.js,
 * allowing you to use MeshLib's powerful mesh operations in browser applications.
 * 
 * Features:
 * - Load/save mesh files (STL, OBJ, PLY, etc.)
 * - Boolean operations (union, intersection, difference)
 * - Mesh decimation and subdivision
 * - Offset/shell operations
 * - Hole filling
 * - Mesh fixing and cleaning
 * - Convert between MeshLib and Three.js formats
 * 
 * @author MeshLib Standalone
 * @license MIT
 */

// Type definitions for TypeScript support
export interface MeshLibModule {
    ccall: (name: string, returnType: string, argTypes: string[], args: any[]) => any;
    cwrap: (name: string, returnType: string, argTypes: string[]) => Function;
    _malloc: (size: number) => number;
    _free: (ptr: number) => void;
    HEAPF32: Float32Array;
    HEAPU32: Uint32Array;
    HEAPU8: Uint8Array;
    FS: any;
}

export interface MeshLibConfig {
    wasmPath?: string;
    onProgress?: (progress: number) => void;
    onReady?: () => void;
    onError?: (error: Error) => void;
}

export interface BooleanOptions {
    operation: 'union' | 'intersection' | 'difference';
}

export interface DecimateOptions {
    maxTriangles?: number;
    maxError?: number;
    preserveBoundary?: boolean;
}

export interface OffsetOptions {
    distance: number;
    resolution?: number;
}

export interface SubdivideOptions {
    maxEdgeLength?: number;
    iterations?: number;
}

// Global module reference
let Module: MeshLibModule | null = null;
let isReady = false;

/**
 * Initialize the MeshLib WASM module
 */
export async function initMeshLib(config: MeshLibConfig = {}): Promise<void> {
    const {
        wasmPath = './meshlib.js',
        onProgress = () => {},
        onReady = () => {},
        onError = () => {}
    } = config;

    return new Promise((resolve, reject) => {
        // Create script element to load WASM module
        const script = document.createElement('script');
        script.src = wasmPath;
        
        // Set up module initialization
        (window as any).Module = {
            onRuntimeInitialized: () => {
                Module = (window as any).Module;
                isReady = true;
                onReady();
                resolve();
            },
            setStatus: (text: string) => {
                const match = text.match(/([\\d.]+)\\/([\\d.]+)/);
                if (match) {
                    const progress = parseInt(match[1]) / parseInt(match[2]);
                    onProgress(progress);
                }
            },
            print: console.log,
            printErr: console.error
        };

        script.onerror = (e) => {
            const error = new Error('Failed to load MeshLib WASM module');
            onError(error);
            reject(error);
        };

        document.head.appendChild(script);
    });
}

/**
 * Check if MeshLib is ready
 */
export function isMeshLibReady(): boolean {
    return isReady && Module !== null;
}

/**
 * MeshLib mesh handle
 */
export class MeshHandle {
    private _ptr: number;
    private _disposed: boolean = false;

    constructor(ptr: number) {
        this._ptr = ptr;
    }

    get ptr(): number {
        if (this._disposed) {
            throw new Error('Mesh has been disposed');
        }
        return this._ptr;
    }

    get isDisposed(): boolean {
        return this._disposed;
    }

    dispose(): void {
        if (!this._disposed && Module) {
            Module.ccall('mrMeshFree', null, ['number'], [this._ptr]);
            this._disposed = true;
        }
    }
}

/**
 * Convert Three.js BufferGeometry to MeshLib mesh
 */
export function bufferGeometryToMesh(geometry: THREE.BufferGeometry): MeshHandle {
    if (!Module) throw new Error('MeshLib not initialized');

    const positions = geometry.getAttribute('position');
    const indices = geometry.getIndex();

    if (!positions) {
        throw new Error('BufferGeometry must have position attribute');
    }

    const vertexCount = positions.count;
    const positionArray = positions.array as Float32Array;

    // Allocate memory for vertices (3 floats per vertex)
    const vertexPtr = Module._malloc(vertexCount * 3 * 4);
    Module.HEAPF32.set(positionArray, vertexPtr / 4);

    let meshPtr: number;

    if (indices) {
        // Indexed geometry
        const indexCount = indices.count;
        const triangleCount = indexCount / 3;
        const indexArray = indices.array;

        // Allocate memory for triangles (3 uint32 per triangle)
        const trianglePtr = Module._malloc(triangleCount * 3 * 4);
        const heap32 = new Uint32Array(Module.HEAPU32.buffer, trianglePtr, triangleCount * 3);
        
        for (let i = 0; i < indexCount; i++) {
            heap32[i] = indexArray[i];
        }

        meshPtr = Module.ccall('mrMeshFromTriangles', 'number', 
            ['number', 'number', 'number', 'number'],
            [vertexPtr, vertexCount, trianglePtr, triangleCount]);

        Module._free(trianglePtr);
    } else {
        // Non-indexed geometry - treat every 3 vertices as a triangle
        const triangleCount = vertexCount / 3;
        const trianglePtr = Module._malloc(triangleCount * 3 * 4);
        const heap32 = new Uint32Array(Module.HEAPU32.buffer, trianglePtr, triangleCount * 3);
        
        for (let i = 0; i < vertexCount; i++) {
            heap32[i] = i;
        }

        meshPtr = Module.ccall('mrMeshFromTriangles', 'number',
            ['number', 'number', 'number', 'number'],
            [vertexPtr, vertexCount, trianglePtr, triangleCount]);

        Module._free(trianglePtr);
    }

    Module._free(vertexPtr);

    if (meshPtr === 0) {
        throw new Error('Failed to create mesh from BufferGeometry');
    }

    return new MeshHandle(meshPtr);
}

/**
 * Convert MeshLib mesh to Three.js BufferGeometry
 */
export function meshToBufferGeometry(mesh: MeshHandle): THREE.BufferGeometry {
    if (!Module) throw new Error('MeshLib not initialized');
    if (typeof THREE === 'undefined') throw new Error('Three.js not loaded');

    const meshPtr = mesh.ptr;

    // Get vertex count and pointer
    const vertexCount = Module.ccall('mrMeshPointsNum', 'number', ['number'], [meshPtr]);
    const vertexPtr = Module.ccall('mrMeshPoints', 'number', ['number'], [meshPtr]);

    // Get triangulation
    const triangulationPtr = Module.ccall('mrMeshGetTriangulation', 'number', ['number'], [meshPtr]);
    
    // Get topology to count faces
    const topologyPtr = Module.ccall('mrMeshTopology', 'number', ['number'], [meshPtr]);
    const faceCount = Module.ccall('mrMeshTopologyNumValidFaces', 'number', ['number'], [topologyPtr]);

    // Create Three.js geometry
    const geometry = new THREE.BufferGeometry();

    // Copy vertices
    const positions = new Float32Array(vertexCount * 3);
    const srcPositions = new Float32Array(Module.HEAPF32.buffer, vertexPtr, vertexCount * 3);
    positions.set(srcPositions);

    // Copy indices from triangulation
    const indices = new Uint32Array(faceCount * 3);
    const srcIndices = new Uint32Array(Module.HEAPU32.buffer, triangulationPtr, faceCount * 3);
    indices.set(srcIndices);

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));

    // Compute normals
    geometry.computeVertexNormals();

    // Free triangulation
    Module.ccall('mrTriangulationFree', null, ['number'], [triangulationPtr]);

    return geometry;
}

/**
 * Load mesh from file data
 */
export async function loadMesh(data: ArrayBuffer, filename: string): Promise<MeshHandle> {
    if (!Module) throw new Error('MeshLib not initialized');

    // Write file to virtual filesystem
    const uint8Array = new Uint8Array(data);
    Module.FS.writeFile(filename, uint8Array);

    // Load mesh based on extension
    const ext = filename.split('.').pop()?.toLowerCase();
    let meshPtr: number;

    switch (ext) {
        case 'stl':
            meshPtr = Module.ccall('mrMeshLoadFromAnySupportedFormat', 'number', 
                ['string'], [filename]);
            break;
        case 'obj':
            meshPtr = Module.ccall('mrMeshLoadFromAnySupportedFormat', 'number',
                ['string'], [filename]);
            break;
        case 'ply':
            meshPtr = Module.ccall('mrMeshLoadFromAnySupportedFormat', 'number',
                ['string'], [filename]);
            break;
        default:
            meshPtr = Module.ccall('mrMeshLoadFromAnySupportedFormat', 'number',
                ['string'], [filename]);
    }

    // Clean up virtual file
    Module.FS.unlink(filename);

    if (meshPtr === 0) {
        throw new Error(\Failed to load mesh from \\);
    }

    return new MeshHandle(meshPtr);
}

/**
 * Save mesh to file data
 */
export async function saveMesh(mesh: MeshHandle, filename: string): Promise<ArrayBuffer> {
    if (!Module) throw new Error('MeshLib not initialized');

    const success = Module.ccall('mrMeshSaveToAnySupportedFormat', 'number',
        ['number', 'string'], [mesh.ptr, filename]);

    if (!success) {
        throw new Error(\Failed to save mesh to \\);
    }

    const data = Module.FS.readFile(filename);
    Module.FS.unlink(filename);

    return data.buffer;
}

/**
 * Boolean operation between two meshes
 */
export function booleanOperation(meshA: MeshHandle, meshB: MeshHandle, options: BooleanOptions): MeshHandle {
    if (!Module) throw new Error('MeshLib not initialized');

    const operationType = {
        'union': 0,
        'intersection': 1,
        'difference': 2
    }[options.operation];

    const resultPtr = Module.ccall('mrBoolean', 'number',
        ['number', 'number', 'number'],
        [meshA.ptr, meshB.ptr, operationType]);

    if (resultPtr === 0) {
        throw new Error(\Boolean \ operation failed\);
    }

    return new MeshHandle(resultPtr);
}

/**
 * Decimate mesh (reduce triangle count)
 */
export function decimateMesh(mesh: MeshHandle, options: DecimateOptions = {}): MeshHandle {
    if (!Module) throw new Error('MeshLib not initialized');

    const {
        maxTriangles = 10000,
        maxError = 0.001,
        preserveBoundary = true
    } = options;

    // Create a copy to modify
    const copyPtr = Module.ccall('mrMeshCopy', 'number', ['number'], [mesh.ptr]);

    // Apply decimation
    Module.ccall('mrDecimateMesh', null,
        ['number', 'number', 'number', 'number'],
        [copyPtr, maxTriangles, maxError, preserveBoundary ? 1 : 0]);

    return new MeshHandle(copyPtr);
}

/**
 * Offset mesh (create shell)
 */
export function offsetMesh(mesh: MeshHandle, options: OffsetOptions): MeshHandle {
    if (!Module) throw new Error('MeshLib not initialized');

    const {
        distance,
        resolution = 0.001
    } = options;

    const resultPtr = Module.ccall('mrOffsetMesh', 'number',
        ['number', 'number', 'number'],
        [mesh.ptr, distance, resolution]);

    if (resultPtr === 0) {
        throw new Error('Offset operation failed');
    }

    return new MeshHandle(resultPtr);
}

/**
 * Subdivide mesh
 */
export function subdivideMesh(mesh: MeshHandle, options: SubdivideOptions = {}): MeshHandle {
    if (!Module) throw new Error('MeshLib not initialized');

    const {
        maxEdgeLength = 0.1,
        iterations = 1
    } = options;

    // Create a copy to modify
    const copyPtr = Module.ccall('mrMeshCopy', 'number', ['number'], [mesh.ptr]);

    // Apply subdivision
    Module.ccall('mrSubdivideMesh', null,
        ['number', 'number', 'number'],
        [copyPtr, maxEdgeLength, iterations]);

    return new MeshHandle(copyPtr);
}

/**
 * Fill holes in mesh
 */
export function fillHoles(mesh: MeshHandle): MeshHandle {
    if (!Module) throw new Error('MeshLib not initialized');

    // Create a copy to modify
    const copyPtr = Module.ccall('mrMeshCopy', 'number', ['number'], [mesh.ptr]);

    // Fill holes
    Module.ccall('mrFillHoles', null, ['number'], [copyPtr]);

    return new MeshHandle(copyPtr);
}

/**
 * Fix mesh (resolve self-intersections, degenerate triangles, etc.)
 */
export function fixMesh(mesh: MeshHandle): MeshHandle {
    if (!Module) throw new Error('MeshLib not initialized');

    // Create a copy to modify
    const copyPtr = Module.ccall('mrMeshCopy', 'number', ['number'], [mesh.ptr]);

    // Fix self-intersections
    Module.ccall('mrFixSelfIntersections', null, ['number'], [copyPtr]);

    return new MeshHandle(copyPtr);
}

/**
 * Get mesh statistics
 */
export function getMeshStats(mesh: MeshHandle): {
    vertexCount: number;
    faceCount: number;
    edgeCount: number;
    area: number;
    volume: number;
    boundingBox: { min: [number, number, number]; max: [number, number, number] };
} {
    if (!Module) throw new Error('MeshLib not initialized');

    const meshPtr = mesh.ptr;
    const topologyPtr = Module.ccall('mrMeshTopology', 'number', ['number'], [meshPtr]);

    const vertexCount = Module.ccall('mrMeshPointsNum', 'number', ['number'], [meshPtr]);
    const faceCount = Module.ccall('mrMeshTopologyNumValidFaces', 'number', ['number'], [topologyPtr]);
    const edgeCount = Module.ccall('mrMeshTopologyNumValidVerts', 'number', ['number'], [topologyPtr]);
    const area = Module.ccall('mrMeshArea', 'number', ['number', 'number'], [meshPtr, 0]);
    const volume = Module.ccall('mrMeshVolume', 'number', ['number', 'number'], [meshPtr, 0]);

    // Get bounding box
    const boxPtr = Module._malloc(24); // 6 floats
    Module.ccall('mrMeshComputeBoundingBox', null, ['number', 'number', 'number'], [meshPtr, 0, boxPtr]);
    const box = new Float32Array(Module.HEAPF32.buffer, boxPtr, 6);

    const result = {
        vertexCount,
        faceCount,
        edgeCount,
        area,
        volume,
        boundingBox: {
            min: [box[0], box[1], box[2]] as [number, number, number],
            max: [box[3], box[4], box[5]] as [number, number, number]
        }
    };

    Module._free(boxPtr);

    return result;
}

/**
 * Create primitive shapes
 */
export const Primitives = {
    sphere(radius: number, subdivisions: number = 3): MeshHandle {
        if (!Module) throw new Error('MeshLib not initialized');
        const ptr = Module.ccall('mrMakeSphere', 'number', ['number', 'number'], [radius, subdivisions]);
        return new MeshHandle(ptr);
    },

    cube(size: number): MeshHandle {
        if (!Module) throw new Error('MeshLib not initialized');
        const ptr = Module.ccall('mrMakeCube', 'number', ['number', 'number', 'number'], [size, size, size]);
        return new MeshHandle(ptr);
    },

    cylinder(radius: number, height: number, segments: number = 32): MeshHandle {
        if (!Module) throw new Error('MeshLib not initialized');
        const ptr = Module.ccall('mrMakeCylinder', 'number', 
            ['number', 'number', 'number'], [radius, height, segments]);
        return new MeshHandle(ptr);
    },

    torus(primaryRadius: number, secondaryRadius: number): MeshHandle {
        if (!Module) throw new Error('MeshLib not initialized');
        const ptr = Module.ccall('mrMakeTorus', 'number',
            ['number', 'number'], [primaryRadius, secondaryRadius]);
        return new MeshHandle(ptr);
    }
};

// Default export
export default {
    initMeshLib,
    isMeshLibReady,
    bufferGeometryToMesh,
    meshToBufferGeometry,
    loadMesh,
    saveMesh,
    booleanOperation,
    decimateMesh,
    offsetMesh,
    subdivideMesh,
    fillHoles,
    fixMesh,
    getMeshStats,
    Primitives,
    MeshHandle
};
