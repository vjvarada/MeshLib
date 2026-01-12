/**
 * @file bindings.cpp
 * @brief Embind bindings for WebAssembly
 * 
 * This file exposes MeshLib Core C++ API to JavaScript via Emscripten's Embind.
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <meshlib/meshlib.h>
#include <vector>
#include <string>

using namespace emscripten;
using namespace meshlib;

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * Load mesh from a JavaScript ArrayBuffer
 */
Mesh loadMeshFromBuffer(const val& buffer, const std::string& extension) {
    // Get buffer data
    std::vector<uint8_t> data = vecFromJSArray<uint8_t>(buffer);
    
    // Create a memory stream and load
    // Note: Actual implementation would use MeshLoad functions
    Mesh mesh;
    // TODO: Implement actual loading from memory buffer
    return mesh;
}

/**
 * Convert mesh to JavaScript ArrayBuffer for saving
 */
val saveMeshToBuffer(const Mesh& mesh, const std::string& extension) {
    std::vector<uint8_t> data;
    // TODO: Implement actual saving to memory buffer
    
    // Return as ArrayBuffer
    return val(typed_memory_view(data.size(), data.data()));
}

/**
 * Get mesh vertices as Float32Array for Three.js
 */
val getMeshVertices(const Mesh& mesh) {
    const auto& points = mesh.points;
    std::vector<float> vertices;
    vertices.reserve(points.size() * 3);
    
    for (const auto& p : points) {
        vertices.push_back(p.x);
        vertices.push_back(p.y);
        vertices.push_back(p.z);
    }
    
    return val(typed_memory_view(vertices.size(), vertices.data()));
}

/**
 * Get mesh face indices as Uint32Array for Three.js
 */
val getMeshIndices(const Mesh& mesh) {
    std::vector<uint32_t> indices;
    // TODO: Extract face indices from mesh topology
    
    return val(typed_memory_view(indices.size(), indices.data()));
}

/**
 * Get mesh normals as Float32Array for Three.js
 */
val getMeshNormals(const Mesh& mesh) {
    std::vector<float> normals;
    // TODO: Compute and return normals
    
    return val(typed_memory_view(normals.size(), normals.data()));
}

// =============================================================================
// Vector3f Bindings
// =============================================================================

EMSCRIPTEN_BINDINGS(vector3f) {
    class_<Vector3f>("Vector3f")
        .constructor<>()
        .constructor<float, float, float>()
        .property("x", &Vector3f::x)
        .property("y", &Vector3f::y)
        .property("z", &Vector3f::z)
        .function("length", &Vector3f::length)
        .function("lengthSq", &Vector3f::lengthSq)
        .function("normalized", &Vector3f::normalized)
        ;
}

// =============================================================================
// Box3f Bindings
// =============================================================================

EMSCRIPTEN_BINDINGS(box3f) {
    class_<Box3f>("Box3f")
        .constructor<>()
        .property("min", &Box3f::min)
        .property("max", &Box3f::max)
        .function("center", &Box3f::center)
        .function("size", &Box3f::size)
        .function("diagonal", &Box3f::diagonal)
        .function("valid", &Box3f::valid)
        ;
}

// =============================================================================
// AffineXf3f Bindings
// =============================================================================

EMSCRIPTEN_BINDINGS(affine_xf) {
    class_<AffineXf3f>("AffineXf3f")
        .constructor<>()
        .class_function("translation", &AffineXf3f::translation)
        .class_function("xfAround", &AffineXf3f::xfAround)
        // Add more transformations as needed
        ;
}

// =============================================================================
// Mesh Bindings
// =============================================================================

EMSCRIPTEN_BINDINGS(mesh) {
    class_<Mesh>("Mesh")
        .constructor<>()
        // Basic properties
        .function("numVertices", &Mesh::topology.numValidVerts)
        .function("numFaces", &Mesh::topology.numValidFaces)
        
        // Bounding box
        .function("getBoundingBox", &Mesh::getBoundingBox)
        .function("computeBoundingBox", select_overload<Box3f() const>(&Mesh::computeBoundingBox))
        
        // Transformations
        .function("transform", select_overload<void(const AffineXf3f&)>(&Mesh::transform))
        
        // For Three.js integration
        .function("getVertices", &getMeshVertices)
        .function("getIndices", &getMeshIndices)
        .function("getNormals", &getMeshNormals)
        ;
}

// =============================================================================
// PointCloud Bindings
// =============================================================================

EMSCRIPTEN_BINDINGS(pointcloud) {
    class_<PointCloud>("PointCloud")
        .constructor<>()
        .function("numValidPoints", &PointCloud::calcNumValidPoints)
        .function("hasNormals", &PointCloud::hasNormals)
        .function("getBoundingBox", &PointCloud::getBoundingBox)
        ;
}

// =============================================================================
// Decimation Settings & Algorithm
// =============================================================================

EMSCRIPTEN_BINDINGS(decimate) {
    enum_<DecimateStrategy>("DecimateStrategy")
        .value("MinimizeError", DecimateStrategy::MinimizeError)
        .value("ShortestEdgeFirst", DecimateStrategy::ShortestEdgeFirst)
        ;
    
    class_<DecimateSettings>("DecimateSettings")
        .constructor<>()
        .property("strategy", &DecimateSettings::strategy)
        .property("maxError", &DecimateSettings::maxError)
        .property("maxEdgeLen", &DecimateSettings::maxEdgeLen)
        .property("maxTriangleAspectRatio", &DecimateSettings::maxTriangleAspectRatio)
        .property("maxDeletedVertices", &DecimateSettings::maxDeletedVertices)
        .property("maxDeletedFaces", &DecimateSettings::maxDeletedFaces)
        .property("packMesh", &DecimateSettings::packMesh)
        ;
    
    function("decimateMesh", &decimateMesh);
}

// =============================================================================
// Subdivision Settings & Algorithm
// =============================================================================

EMSCRIPTEN_BINDINGS(subdivide) {
    class_<SubdivideSettings>("SubdivideSettings")
        .constructor<>()
        .property("maxEdgeLen", &SubdivideSettings::maxEdgeLen)
        .property("maxEdgeSplits", &SubdivideSettings::maxEdgeSplits)
        .property("maxDeviationAfterFlip", &SubdivideSettings::maxDeviationAfterFlip)
        ;
    
    function("subdivideMesh", select_overload<int(Mesh&, const SubdivideSettings&)>(&subdivideMesh));
}

// =============================================================================
// Boolean Operations
// =============================================================================

EMSCRIPTEN_BINDINGS(boolean_ops) {
    enum_<BooleanOperation>("BooleanOperation")
        .value("Union", BooleanOperation::Union)
        .value("Intersection", BooleanOperation::Intersection)
        .value("DifferenceAB", BooleanOperation::DifferenceAB)
        .value("DifferenceBA", BooleanOperation::DifferenceBA)
        ;
    
    class_<BooleanResult>("BooleanResult")
        .constructor<>()
        .property("mesh", &BooleanResult::mesh)
        .property("errorString", &BooleanResult::errorString)
        .function("valid", &BooleanResult::valid)
        ;
    
    function("boolean", select_overload<BooleanResult(const Mesh&, const Mesh&, BooleanOperation, const AffineXf3f*)>(
        &boolean));
}

// =============================================================================
// Hole Filling
// =============================================================================

EMSCRIPTEN_BINDINGS(fill_hole) {
    class_<FillHoleParams>("FillHoleParams")
        .constructor<>()
        ;
    
    // Note: fillHole takes EdgeId which requires more bindings
    // Simplified version for WASM
}

// =============================================================================
// ICP (Iterative Closest Point)
// =============================================================================

EMSCRIPTEN_BINDINGS(icp) {
    enum_<ICPMethod>("ICPMethod")
        .value("Combined", ICPMethod::Combined)
        .value("PointToPoint", ICPMethod::PointToPoint)
        .value("PointToPlane", ICPMethod::PointToPlane)
        ;
    
    class_<ICPProperties>("ICPProperties")
        .constructor<>()
        .property("method", &ICPProperties::method)
        .property("p2plAngleLimit", &ICPProperties::p2plAngleLimit)
        .property("p2plScaleLimit", &ICPProperties::p2plScaleLimit)
        .property("cosTreshold", &ICPProperties::cosTreshold)
        .property("distTresholdSq", &ICPProperties::distTresholdSq)
        .property("farDistFactor", &ICPProperties::farDistFactor)
        .property("iterLimit", &ICPProperties::iterLimit)
        ;
    
    // Simplified ICP function binding
    // Full ICP class would require more complex bindings
}

// =============================================================================
// Collision Detection
// =============================================================================

EMSCRIPTEN_BINDINGS(collision) {
    // Simple collision check function
    function("meshesCollide", [](const Mesh& a, const Mesh& b) -> bool {
        auto result = findCollidingTriangles(MeshPart{a}, MeshPart{b}, nullptr, true);
        return !result.empty();
    });
    
    function("hasSelfIntersections", [](const Mesh& mesh) -> bool {
        auto result = findSelfCollidingTriangles(MeshPart{mesh}, nullptr, nullptr);
        return result.has_value() && result.value();
    });
}

// =============================================================================
// Mesh Relaxation
// =============================================================================

EMSCRIPTEN_BINDINGS(relax) {
    class_<MeshRelaxParams>("MeshRelaxParams")
        .constructor<>()
        .property("iterations", &MeshRelaxParams::iterations)
        .property("force", &MeshRelaxParams::force)
        ;
    
    function("relaxMesh", select_overload<bool(Mesh&, const MeshRelaxParams&, ProgressCallback)>(&relax));
}

// =============================================================================
// I/O Functions
// =============================================================================

EMSCRIPTEN_BINDINGS(io) {
    // Load mesh from buffer
    function("loadMeshFromBuffer", &loadMeshFromBuffer);
    
    // Save mesh to buffer
    function("saveMeshToBuffer", &saveMeshToBuffer);
    
    // Supported formats
    function("getSupportedLoadFormats", []() -> val {
        std::vector<std::string> formats = {"stl", "obj", "ply", "off"};
        return val::array(formats);
    });
    
    function("getSupportedSaveFormats", []() -> val {
        std::vector<std::string> formats = {"stl", "obj", "ply", "off"};
        return val::array(formats);
    });
}

// =============================================================================
// Utility Functions
// =============================================================================

EMSCRIPTEN_BINDINGS(utils) {
    // Create primitive meshes
    function("makeUVSphere", &makeUVSphere);
    function("makeCube", &makeCube);
    function("makeCylinder", &makeCylinder);
    function("makeTorus", &makeTorus);
    
    // Copy mesh
    function("copyMesh", [](const Mesh& mesh) -> Mesh {
        return mesh; // Uses copy constructor
    });
}

// =============================================================================
// Version Info
// =============================================================================

EMSCRIPTEN_BINDINGS(version) {
    function("getVersion", []() -> std::string {
        return MESHLIB_CORE_VERSION;
    });
    
    function("hasParallelSupport", []() -> bool {
        return MESHLIB_PARALLEL_ENABLED;
    });
    
    function("hasVoxelSupport", []() -> bool {
#ifdef MESHLIB_HAS_OPENVDB
        return true;
#else
        return false;
#endif
    });
}
