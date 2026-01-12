/**
 * MeshLib Core - Three.js Integration Helpers
 * 
 * Utility functions for seamless integration between MeshLib Core (WASM) and Three.js.
 */

import * as THREE from 'three';
import type { MeshLibModule, Mesh as MeshLibMesh, PointCloud as MeshLibPointCloud } from './meshlib';

/**
 * Convert a MeshLib mesh to a Three.js BufferGeometry
 */
export function meshToThreeGeometry(mesh: MeshLibMesh): THREE.BufferGeometry {
    const geometry = new THREE.BufferGeometry();
    
    // Get vertex data
    const vertices = mesh.getVertices();
    const indices = mesh.getIndices();
    const normals = mesh.getNormals();
    
    // Set position attribute
    geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    
    // Set index
    if (indices.length > 0) {
        geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    }
    
    // Set normals
    if (normals.length > 0) {
        geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    } else {
        geometry.computeVertexNormals();
    }
    
    return geometry;
}

/**
 * Convert a Three.js BufferGeometry to a MeshLib mesh
 */
export function threeGeometryToMesh(geometry: THREE.BufferGeometry, meshlib: MeshLibModule): MeshLibMesh {
    const mesh = new meshlib.Mesh();
    
    // Get position data
    const positionAttr = geometry.getAttribute('position');
    if (!positionAttr) {
        throw new Error('Geometry has no position attribute');
    }
    
    const vertices = new Float32Array(positionAttr.array);
    
    // Get indices
    let indices: Uint32Array;
    if (geometry.index) {
        indices = new Uint32Array(geometry.index.array);
    } else {
        // Generate indices for non-indexed geometry
        indices = new Uint32Array(positionAttr.count);
        for (let i = 0; i < positionAttr.count; i++) {
            indices[i] = i;
        }
    }
    
    // TODO: Set mesh data from vertices and indices
    // This requires exposing appropriate mesh construction methods in bindings
    
    return mesh;
}

/**
 * Convert a MeshLib PointCloud to Three.js Points
 */
export function pointCloudToThreePoints(
    pointCloud: MeshLibPointCloud, 
    material?: THREE.PointsMaterial
): THREE.Points {
    const geometry = new THREE.BufferGeometry();
    
    // Get point data
    // TODO: Add getPoints() method to PointCloud bindings
    // const points = pointCloud.getPoints();
    // geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
    
    const defaultMaterial = material || new THREE.PointsMaterial({
        size: 0.01,
        sizeAttenuation: true,
        color: 0x00ff00
    });
    
    return new THREE.Points(geometry, defaultMaterial);
}

/**
 * Create a Three.js Mesh from MeshLib mesh with standard material
 */
export function createThreeMesh(
    meshlibMesh: MeshLibMesh, 
    material?: THREE.Material
): THREE.Mesh {
    const geometry = meshToThreeGeometry(meshlibMesh);
    
    const defaultMaterial = material || new THREE.MeshStandardMaterial({
        color: 0x808080,
        metalness: 0.3,
        roughness: 0.7,
        side: THREE.DoubleSide
    });
    
    return new THREE.Mesh(geometry, defaultMaterial);
}

/**
 * Load mesh file and return Three.js mesh
 */
export async function loadMeshFile(
    meshlib: MeshLibModule,
    url: string,
    material?: THREE.Material
): Promise<THREE.Mesh> {
    // Fetch the file
    const response = await fetch(url);
    const buffer = await response.arrayBuffer();
    
    // Determine file extension
    const extension = url.split('.').pop()?.toLowerCase() || 'stl';
    
    // Load with MeshLib
    const meshlibMesh = meshlib.loadMeshFromBuffer(buffer, extension);
    
    // Convert to Three.js
    return createThreeMesh(meshlibMesh, material);
}

/**
 * Download mesh as file
 */
export function downloadMesh(
    meshlib: MeshLibModule,
    mesh: MeshLibMesh,
    filename: string,
    format: string = 'stl'
): void {
    // Save to buffer
    const buffer = meshlib.saveMeshToBuffer(mesh, format);
    
    // Create blob and download
    const blob = new Blob([buffer], { type: 'application/octet-stream' });
    const url = URL.createObjectURL(blob);
    
    const link = document.createElement('a');
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    
    URL.revokeObjectURL(url);
}

/**
 * Update Three.js geometry from MeshLib mesh (for animations/modifications)
 */
export function updateThreeGeometry(
    geometry: THREE.BufferGeometry,
    mesh: MeshLibMesh
): void {
    const vertices = mesh.getVertices();
    const positionAttr = geometry.getAttribute('position') as THREE.BufferAttribute;
    
    positionAttr.array.set(vertices);
    positionAttr.needsUpdate = true;
    
    // Update normals
    const normals = mesh.getNormals();
    if (normals.length > 0) {
        const normalAttr = geometry.getAttribute('normal') as THREE.BufferAttribute;
        if (normalAttr) {
            normalAttr.array.set(normals);
            normalAttr.needsUpdate = true;
        }
    } else {
        geometry.computeVertexNormals();
    }
    
    // Update bounding box
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
}

/**
 * MeshLib to Three.js bounding box conversion
 */
export function toThreeBox3(box: ReturnType<MeshLibMesh['getBoundingBox']>): THREE.Box3 {
    const min = box.min;
    const max = box.max;
    
    return new THREE.Box3(
        new THREE.Vector3(min.x, min.y, min.z),
        new THREE.Vector3(max.x, max.y, max.z)
    );
}

/**
 * Fit camera to view mesh
 */
export function fitCameraToMesh(
    camera: THREE.PerspectiveCamera,
    mesh: MeshLibMesh,
    controls?: { target: THREE.Vector3 }
): void {
    const box = mesh.getBoundingBox();
    const center = box.center();
    const size = box.diagonal();
    
    // Calculate camera distance
    const fov = camera.fov * (Math.PI / 180);
    const distance = size / (2 * Math.tan(fov / 2));
    
    // Position camera
    camera.position.set(
        center.x + distance,
        center.y + distance * 0.5,
        center.z + distance
    );
    
    // Look at center
    camera.lookAt(center.x, center.y, center.z);
    
    // Update controls if provided (e.g., OrbitControls)
    if (controls) {
        controls.target.set(center.x, center.y, center.z);
    }
    
    // Clean up
    center.delete();
    box.delete();
}

/**
 * Interactive mesh processing helper class
 */
export class MeshProcessor {
    private meshlib: MeshLibModule;
    private mesh: MeshLibMesh;
    private threeMesh: THREE.Mesh;
    
    constructor(meshlib: MeshLibModule, mesh: MeshLibMesh, material?: THREE.Material) {
        this.meshlib = meshlib;
        this.mesh = mesh;
        this.threeMesh = createThreeMesh(mesh, material);
    }
    
    get threeObject(): THREE.Mesh {
        return this.threeMesh;
    }
    
    /**
     * Decimate mesh and update Three.js geometry
     */
    decimate(maxError: number): void {
        const settings = new this.meshlib.DecimateSettings();
        settings.maxError = maxError;
        settings.packMesh = true;
        
        this.meshlib.decimateMesh(this.mesh, settings);
        settings.delete();
        
        this.updateGeometry();
    }
    
    /**
     * Subdivide mesh and update Three.js geometry
     */
    subdivide(maxEdgeLen: number, maxSplits: number = 10000): void {
        const settings = new this.meshlib.SubdivideSettings();
        settings.maxEdgeLen = maxEdgeLen;
        settings.maxEdgeSplits = maxSplits;
        
        this.meshlib.subdivideMesh(this.mesh, settings);
        settings.delete();
        
        this.updateGeometry();
    }
    
    /**
     * Smooth/relax mesh
     */
    smooth(iterations: number = 3, force: number = 0.5): void {
        const params = new this.meshlib.MeshRelaxParams();
        params.iterations = iterations;
        params.force = force;
        
        this.meshlib.relaxMesh(this.mesh, params);
        params.delete();
        
        this.updateGeometry();
    }
    
    /**
     * Boolean operation with another mesh
     */
    boolean(other: MeshLibMesh, operation: number): boolean {
        const result = this.meshlib.boolean(this.mesh, other, operation);
        
        if (result.valid()) {
            // Replace mesh with result
            this.mesh.delete();
            this.mesh = result.mesh;
            this.updateGeometry();
            return true;
        }
        
        console.error('Boolean operation failed:', result.errorString);
        result.delete();
        return false;
    }
    
    /**
     * Update Three.js geometry from current mesh state
     */
    private updateGeometry(): void {
        // Dispose old geometry
        this.threeMesh.geometry.dispose();
        
        // Create new geometry
        this.threeMesh.geometry = meshToThreeGeometry(this.mesh);
    }
    
    /**
     * Clean up resources
     */
    dispose(): void {
        this.mesh.delete();
        this.threeMesh.geometry.dispose();
        if (this.threeMesh.material instanceof THREE.Material) {
            this.threeMesh.material.dispose();
        }
    }
}
