/**
 * MeshLib Core WebAssembly TypeScript Definitions
 * 
 * Type definitions for using MeshLib Core in TypeScript/JavaScript applications.
 */

export interface MeshLibModule {
    // Version info
    getVersion(): string;
    hasParallelSupport(): boolean;
    hasVoxelSupport(): boolean;
    
    // I/O
    loadMeshFromBuffer(buffer: ArrayBuffer, extension: string): Mesh;
    saveMeshToBuffer(mesh: Mesh, extension: string): ArrayBuffer;
    getSupportedLoadFormats(): string[];
    getSupportedSaveFormats(): string[];
    
    // Primitives
    makeUVSphere(radius: number, hSegments: number, vSegments: number): Mesh;
    makeCube(size: Vector3f): Mesh;
    makeCylinder(radius: number, height: number, segments: number): Mesh;
    makeTorus(majorRadius: number, minorRadius: number, majorSegments: number, minorSegments: number): Mesh;
    copyMesh(mesh: Mesh): Mesh;
    
    // Algorithms
    decimateMesh(mesh: Mesh, settings: DecimateSettings): DecimateResult;
    subdivideMesh(mesh: Mesh, settings: SubdivideSettings): number;
    boolean(meshA: Mesh, meshB: Mesh, operation: BooleanOperation, transform?: AffineXf3f): BooleanResult;
    relaxMesh(mesh: Mesh, params: MeshRelaxParams): boolean;
    
    // Collision
    meshesCollide(meshA: Mesh, meshB: Mesh): boolean;
    hasSelfIntersections(mesh: Mesh): boolean;
    
    // Classes
    Vector3f: Vector3fConstructor;
    Box3f: Box3fConstructor;
    AffineXf3f: AffineXf3fConstructor;
    Mesh: MeshConstructor;
    PointCloud: PointCloudConstructor;
    DecimateSettings: DecimateSettingsConstructor;
    SubdivideSettings: SubdivideSettingsConstructor;
    FillHoleParams: FillHoleParamsConstructor;
    ICPProperties: ICPPropertiesConstructor;
    MeshRelaxParams: MeshRelaxParamsConstructor;
    
    // Enums
    DecimateStrategy: typeof DecimateStrategy;
    BooleanOperation: typeof BooleanOperation;
    ICPMethod: typeof ICPMethod;
}

// =============================================================================
// Vector3f
// =============================================================================

export interface Vector3f {
    x: number;
    y: number;
    z: number;
    length(): number;
    lengthSq(): number;
    normalized(): Vector3f;
    delete(): void;
}

export interface Vector3fConstructor {
    new(): Vector3f;
    new(x: number, y: number, z: number): Vector3f;
}

// =============================================================================
// Box3f
// =============================================================================

export interface Box3f {
    min: Vector3f;
    max: Vector3f;
    center(): Vector3f;
    size(): Vector3f;
    diagonal(): number;
    valid(): boolean;
    delete(): void;
}

export interface Box3fConstructor {
    new(): Box3f;
}

// =============================================================================
// AffineXf3f
// =============================================================================

export interface AffineXf3f {
    delete(): void;
}

export interface AffineXf3fConstructor {
    new(): AffineXf3f;
    translation(v: Vector3f): AffineXf3f;
    xfAround(matrix: any, center: Vector3f): AffineXf3f;
}

// =============================================================================
// Mesh
// =============================================================================

export interface Mesh {
    numVertices(): number;
    numFaces(): number;
    getBoundingBox(): Box3f;
    computeBoundingBox(): Box3f;
    transform(xf: AffineXf3f): void;
    
    // Three.js integration
    getVertices(): Float32Array;
    getIndices(): Uint32Array;
    getNormals(): Float32Array;
    
    delete(): void;
}

export interface MeshConstructor {
    new(): Mesh;
}

// =============================================================================
// PointCloud
// =============================================================================

export interface PointCloud {
    numValidPoints(): number;
    hasNormals(): boolean;
    getBoundingBox(): Box3f;
    delete(): void;
}

export interface PointCloudConstructor {
    new(): PointCloud;
}

// =============================================================================
// Enums
// =============================================================================

export enum DecimateStrategy {
    MinimizeError = 0,
    ShortestEdgeFirst = 1
}

export enum BooleanOperation {
    Union = 0,
    Intersection = 1,
    DifferenceAB = 2,
    DifferenceBA = 3
}

export enum ICPMethod {
    Combined = 0,
    PointToPoint = 1,
    PointToPlane = 2
}

// =============================================================================
// Settings Classes
// =============================================================================

export interface DecimateSettings {
    strategy: DecimateStrategy;
    maxError: number;
    maxEdgeLen: number;
    maxTriangleAspectRatio: number;
    maxDeletedVertices: number;
    maxDeletedFaces: number;
    packMesh: boolean;
    delete(): void;
}

export interface DecimateSettingsConstructor {
    new(): DecimateSettings;
}

export interface DecimateResult {
    vertsDeleted: number;
    facesDeleted: number;
    errorIntroduced: number;
}

export interface SubdivideSettings {
    maxEdgeLen: number;
    maxEdgeSplits: number;
    maxDeviationAfterFlip: number;
    delete(): void;
}

export interface SubdivideSettingsConstructor {
    new(): SubdivideSettings;
}

export interface FillHoleParams {
    delete(): void;
}

export interface FillHoleParamsConstructor {
    new(): FillHoleParams;
}

export interface ICPProperties {
    method: ICPMethod;
    p2plAngleLimit: number;
    p2plScaleLimit: number;
    cosTreshold: number;
    distTresholdSq: number;
    farDistFactor: number;
    iterLimit: number;
    delete(): void;
}

export interface ICPPropertiesConstructor {
    new(): ICPProperties;
}

export interface MeshRelaxParams {
    iterations: number;
    force: number;
    delete(): void;
}

export interface MeshRelaxParamsConstructor {
    new(): MeshRelaxParams;
}

// =============================================================================
// Result Types
// =============================================================================

export interface BooleanResult {
    mesh: Mesh;
    errorString: string;
    valid(): boolean;
    delete(): void;
}

// =============================================================================
// Module Loader
// =============================================================================

/**
 * Load the MeshLib Core WebAssembly module
 */
export default function MeshLibCore(): Promise<MeshLibModule>;
