// MRMeshOperationsBindings.cpp - Python bindings for mesh operations
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "MRMesh/MRMesh.h"
#include "MRMesh/MRCube.h"
#include "MRMesh/MRTorus.h"
#include "MRMesh/MRMakeSphereMesh.h"
#include "MRMesh/MRCylinder.h"
#include "MRMesh/MRMeshDecimate.h"
#include "MRMesh/MRMeshSubdivide.h"
#include "MRMesh/MRMeshBoolean.h"
#include "MRMesh/MRMeshCollide.h"
#include "MRMesh/MRConvexHull.h"
#include "MRMesh/MRMeshFixer.h"
#include "MRMesh/MRMeshLoad.h"
#include "MRMesh/MRMeshSave.h"
#include "MRMesh/MRMeshFillHole.h"
#include "MRMesh/MRMeshComponents.h"
#include "MRMesh/MRRegionBoundary.h"
#include "MRMesh/MRMeshNormals.h"
#include "MRMesh/MRMeshRelax.h"
#include "MRMesh/MRICP.h"
#include "MRMesh/MRPointCloud.h"
#include "MRMesh/MRPointsLoad.h"
#include "MRMesh/MRMeshBuilder.h"
#include "MRMesh/MRPointCloudTriangulation.h"
#include "MRMesh/MRMeshMetrics.h"

namespace py = pybind11;
using namespace MR;

void bindMeshOperations(py::module_& m) {
    // TEST FUNCTION - to verify bindings are working
    m.def("testNewBindings", []() { 
        return "New bindings working!"; 
    }, "Test function to verify new bindings are being registered");
    
    // Primitive creation
    m.def("makeCube", [](const Vector3f& size, const Vector3f& base) {
        return makeCube(size, base);
    }, py::arg("size") = Vector3f::diagonal(1.0f), 
       py::arg("base") = Vector3f(),
       "Create a cube mesh");
       
    m.def("makeTorus", [](float primaryRadius, float secondaryRadius, 
                          int primaryResolution, int secondaryResolution) {
        return makeTorus(primaryRadius, secondaryRadius, primaryResolution, secondaryResolution);
    }, py::arg("primaryRadius") = 1.0f, py::arg("secondaryRadius") = 0.1f,
       py::arg("primaryResolution") = 32, py::arg("secondaryResolution") = 32,
       "Create a torus mesh");
       
    // Mesh loading
    m.def("loadStl", [](const std::string& path) {
        auto result = loadStl(path);
        if (!result.has_value())
            throw std::runtime_error(result.error());
        return result.value();
    }, py::arg("path"), "Load mesh from STL file");
       
    m.def("makeUVSphere", [](float radius, int numMeridians, int numParallels) {
        return makeUVSphere(radius, numMeridians, numParallels);
    }, py::arg("radius") = 1.0f, py::arg("numMeridians") = 32, py::arg("numParallels") = 32,
       "Create a UV sphere mesh");
       
    // Decimation settings
    py::class_<DecimateSettings>(m, "DecimateSettings", "Settings for mesh decimation")
        .def(py::init<>())
        .def_readwrite("maxError", &DecimateSettings::maxError,
                       "Maximum permitted deviation from original mesh")
        .def_readwrite("maxDeletedFaces", &DecimateSettings::maxDeletedFaces,
                       "Maximum number of faces to delete (0 = no limit)")
        .def_readwrite("maxDeletedVertices", &DecimateSettings::maxDeletedVertices,
                       "Maximum number of vertices to delete (0 = no limit)")
        .def_readwrite("strategy", &DecimateSettings::strategy,
                       "Decimation strategy");
                       
    py::enum_<DecimateStrategy>(m, "DecimateStrategy")
        .value("MinimizeError", DecimateStrategy::MinimizeError)
        .value("ShortestEdgeFirst", DecimateStrategy::ShortestEdgeFirst);
                       
    // Decimation result
    py::class_<DecimateResult>(m, "DecimateResult", "Result of mesh decimation")
        .def_readonly("vertsDeleted", &DecimateResult::vertsDeleted)
        .def_readonly("facesDeleted", &DecimateResult::facesDeleted)
        .def_readonly("errorIntroduced", &DecimateResult::errorIntroduced);
                       
    // Decimation function
    m.def("decimateMesh", [](Mesh& mesh, const DecimateSettings& settings) {
        return decimateMesh(mesh, settings);
    }, py::arg("mesh"), py::arg("settings") = DecimateSettings(),
       "Decimate mesh (simplify by reducing triangle count)");
       
    // Boolean operations
    py::enum_<BooleanOperation>(m, "BooleanOperation")
        .value("Union", BooleanOperation::Union)
        .value("Intersection", BooleanOperation::Intersection)
        .value("DifferenceAB", BooleanOperation::DifferenceAB)
        .value("DifferenceBA", BooleanOperation::DifferenceBA);
        
    m.def("boolean", [](const Mesh& meshA, const Mesh& meshB, BooleanOperation op) {
        BooleanResult result = boolean(meshA, meshB, op);
        if (result.valid())
            return result.mesh;
        throw std::runtime_error(result.errorString);
    }, py::arg("meshA"), py::arg("meshB"), py::arg("operation"),
       "Perform boolean operation on two meshes");
       
    // Collision detection
    m.def("findCollidingTriangles", [](const Mesh& a, const Mesh& b, 
                                        const AffineXf3f* rigidB2A, bool firstIntersectionOnly) {
        return findCollidingTriangles(a, b, rigidB2A, firstIntersectionOnly);
    }, py::arg("a"), py::arg("b"), py::arg("rigidB2A") = nullptr, 
       py::arg("firstIntersectionOnly") = false,
       "Find colliding triangles between two meshes");
       
    // Convex hull
    m.def("makeConvexHull", py::overload_cast<const Mesh&>(&makeConvexHull),
          py::arg("mesh"), "Create convex hull of a mesh");
          
    // Mesh fixing/validation functions
    m.def("findDegenerateFaces", [](const Mesh& mesh, float criticalAspectRatio) {
        auto result = findDegenerateFaces(mesh, criticalAspectRatio);
        if (!result.has_value())
            throw std::runtime_error(result.error());
        return result.value();
    }, py::arg("mesh"), py::arg("criticalAspectRatio") = FLT_MAX,
       "Find faces with aspect ratio >= criticalAspectRatio (default: all degenerate faces)");
       
    m.def("findMultipleEdges", [](const MeshTopology& topology) {
        auto result = findMultipleEdges(topology);
        if (!result.has_value())
            throw std::runtime_error(result.error());
        return result.value();
    }, py::arg("topology"), "Find multiple edges in the mesh topology");
    
    m.def("fixMultipleEdges", py::overload_cast<Mesh&>(&fixMultipleEdges),
          py::arg("mesh"), "Find and resolve multiple edges in the mesh");
          
    m.def("hasMultipleEdges", &hasMultipleEdges,
          py::arg("topology"), "Check if the mesh has multiple edges");
          
    // Region boundary functions
    m.def("findRightBoundary", 
          py::overload_cast<const MeshTopology&, const FaceBitSet*>(&findRightBoundary),
          py::arg("topology"), py::arg("region") = nullptr,
          "Find right boundary loops of a region (or entire mesh if region is None)");
          
    // ============================================================================
    // Mesh Saving Functions
    // ============================================================================
    m.def("saveStl", [](const Mesh& mesh, const std::string& path) {
        auto result = MeshSave::toAnySupportedFormat(mesh, path);
        if (!result.has_value())
            throw std::runtime_error(result.error());
    }, py::arg("mesh"), py::arg("path"), "Save mesh to STL file");
    
    m.def("saveMesh", [](const Mesh& mesh, const std::string& path) {
        auto result = MeshSave::toAnySupportedFormat(mesh, path);
        if (!result.has_value())
            throw std::runtime_error(result.error());
    }, py::arg("mesh"), py::arg("path"), 
       "Save mesh to file (format auto-detected from extension: .stl, .obj, .ply, .off, .ctm)");
    
    // ============================================================================
    // Mesh Loading Functions (extended)
    // ============================================================================
    m.def("loadMesh", [](const std::string& path) {
        auto result = MeshLoad::fromAnySupportedFormat(path);
        if (!result.has_value())
            throw std::runtime_error(result.error());
        return result.value();
    }, py::arg("path"), 
       "Load mesh from file (format auto-detected from extension)");
    
    // ============================================================================
    // More Primitive Creation Functions  
    // ============================================================================
    m.def("makeCylinder", [](float radius, float length, int resolution) {
        return makeCylinder(radius, length, resolution);
    }, py::arg("radius") = 1.0f, py::arg("length") = 1.0f, py::arg("resolution") = 32,
       "Create a cylinder mesh");
    
    // ============================================================================
    // Mesh Subdivision
    // ============================================================================
    py::class_<SubdivideSettings>(m, "SubdivideSettings", "Settings for mesh subdivision")
        .def(py::init<>())
        .def_readwrite("maxEdgeLen", &SubdivideSettings::maxEdgeLen,
                       "Maximum edge length (edges longer than this will be split)")
        .def_readwrite("maxEdgeSplits", &SubdivideSettings::maxEdgeSplits,
                       "Maximum number of edge splits")
        .def_readwrite("maxDeviationAfterFlip", &SubdivideSettings::maxDeviationAfterFlip,
                       "Maximum deviation after edge flip");
                       
    m.def("subdivideMesh", [](Mesh& mesh, const SubdivideSettings& settings) {
        return subdivideMesh(mesh, settings);
    }, py::arg("mesh"), py::arg("settings") = SubdivideSettings(),
       "Subdivide mesh by splitting long edges, returns number of edge splits performed");
       
    // ============================================================================
    // Mesh Components
    // ============================================================================
    m.def("getAllComponentsVertices", [](const Mesh& mesh) {
        return MeshComponents::getAllComponentsVerts(mesh);
    }, py::arg("mesh"), "Get vertex sets for all connected components");
    
    m.def("getNumComponents", [](const Mesh& mesh) {
        auto verts = MeshComponents::getAllComponentsVerts(mesh);
        return verts.size();
    }, py::arg("mesh"), "Get number of connected components");
    
    m.def("getLargestComponent", [](const Mesh& mesh) {
        return MeshComponents::getLargestComponent(mesh);
    }, py::arg("mesh"), "Get face bitset of the largest connected component");
    
    // ============================================================================
    // Hole Filling
    // ============================================================================
    py::class_<FillHoleParams>(m, "FillHoleParams", "Parameters for hole filling")
        .def(py::init<>())
        .def_readwrite("metric", &FillHoleParams::metric, "Quality metric for triangulation");
        
    m.def("fillHole", [](Mesh& mesh, EdgeId holeEdge) {
        FillHoleParams params;
        fillHole(mesh, holeEdge, params);
    }, py::arg("mesh"), py::arg("holeEdge"),
       "Fill a hole starting from the given boundary edge");
       
    m.def("fillAllHoles", [](Mesh& mesh) {
        std::vector<EdgeId> holes = mesh.topology.findHoleRepresentiveEdges();
        for (EdgeId e : holes) {
            FillHoleParams params;
            fillHole(mesh, e, params);
        }
        return holes.size();
    }, py::arg("mesh"), "Fill all holes in the mesh, returns number of holes filled");
    
    // ============================================================================
    // Mesh Normals
    // ============================================================================
    m.def("computePerVertexNormals", [](const Mesh& mesh) {
        return computePerVertNormals(mesh);
    }, py::arg("mesh"), "Compute per-vertex normals");
    
    m.def("computePerFaceNormals", [](const Mesh& mesh) {
        return computePerFaceNormals(mesh);
    }, py::arg("mesh"), "Compute per-face normals");
    
    // ============================================================================
    // Mesh Smoothing/Relaxation
    // ============================================================================
    py::class_<MeshRelaxParams>(m, "MeshRelaxParams", "Parameters for mesh relaxation")
        .def(py::init<>())
        .def_readwrite("iterations", &MeshRelaxParams::iterations, "Number of iterations");
        
    m.def("relax", [](Mesh& mesh, const MeshRelaxParams& params) {
        return relax(mesh, params);
    }, py::arg("mesh"), py::arg("params") = MeshRelaxParams(),
       "Relax mesh (smooth vertices)");
    
    // ============================================================================
    // ICP Alignment
    // ============================================================================
    py::class_<ICPProperties>(m, "ICPProperties", "Properties for ICP alignment")
        .def(py::init<>())
        .def_readwrite("distThresholdSq", &ICPProperties::distThresholdSq,
                       "Maximum squared distance for point pairs")
        .def_readwrite("exitVal", &ICPProperties::exitVal,
                       "Target root mean squared deviation to exit")
        .def_readwrite("method", &ICPProperties::method,
                       "Algorithm method to use");
                       
    py::enum_<ICPMethod>(m, "ICPMethod", "ICP algorithm method")
        .value("PointToPoint", ICPMethod::PointToPoint)
        .value("PointToPlane", ICPMethod::PointToPlane)
        .value("Combined", ICPMethod::Combined);
        
    py::class_<ICP>(m, "ICP", "Iterative Closest Point alignment class")
        .def(py::init<const MeshPart&, const MeshPart&, const AffineXf3f&, const AffineXf3f&, float>(),
             py::arg("fltMesh"), py::arg("refMesh"), 
             py::arg("fltXf") = AffineXf3f(), py::arg("refXf") = AffineXf3f(),
             py::arg("samplingVoxelSize") = 0.0f,
             "Initialize ICP with floating and reference meshes")
        .def("setParams", &ICP::setParams, py::arg("params"),
             "Set ICP properties")
        .def("calculateTransformation", &ICP::calculateTransformation,
             "Calculate transformation to align meshes")
        .def("getStatusInfo", &ICP::getStatusInfo,
             "Get information string about ICP status");
             
    // ============================================================================
    // Point Cloud Loading and Operations
    // ============================================================================
    m.def("loadPoints", [](const std::string& path) {
        auto result = PointsLoad::fromAnySupportedFormat(path);
        if (!result.has_value())
            throw std::runtime_error(result.error());
        return result.value();
    }, py::arg("path"), "Load point cloud from file (PLY, OBJ, etc.)");
    
    m.def("triangulatePointCloud", [](const PointCloud& cloud) {
        return triangulatePointCloud(cloud);
    }, py::arg("pointCloud"), "Triangulate point cloud to create mesh");
    
    // ============================================================================
    // Mesh Merging and Stitching
    // ============================================================================
    m.def("mergeMeshes", [](const std::vector<const Mesh*>& meshes) {
        Mesh result;
        for (const auto* mesh : meshes) {
            if (mesh) {
                #ifdef _MSC_VER
                #pragma warning(push)
                #pragma warning(disable: 4996)
                #endif
                result.addPart(*mesh);
                #ifdef _MSC_VER
                #pragma warning(pop)
                #endif
            }
        }
        return result;
    }, py::arg("meshes"), "Merge multiple meshes into one");
    
    py::class_<StitchHolesParams>(m, "StitchHolesParams", "Parameters for hole stitching")
        .def(py::init<>())
        .def_readwrite("metric", &StitchHolesParams::metric, "Edge metric for stitching");
        
    m.def("buildCylinderBetweenTwoHoles", [](Mesh& mesh, EdgeId a, EdgeId b, const StitchHolesParams& params) {
        buildCylinderBetweenTwoHoles(mesh, a, b, params);
    }, py::arg("mesh"), py::arg("edgeA"), py::arg("edgeB"), py::arg("params") = StitchHolesParams(),
       "Stitch two holes together with a cylindrical surface");
       
    m.def("getUniversalMetric", [](const Mesh& mesh) {
        return getUniversalMetric(mesh);
    }, py::arg("mesh"), "Get universal edge metric for mesh");
}

