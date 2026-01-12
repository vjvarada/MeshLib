/**
 * @file bindings.cpp
 * @brief pybind11 bindings for MeshLib Core
 * 
 * Python bindings using pybind11 for the MeshLib Core library.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>

#include "meshlib/meshlib.h"

namespace py = pybind11;
using namespace meshlib;

// =============================================================================
// NumPy Conversion Helpers
// =============================================================================

/**
 * Get mesh vertices as NumPy array (Nx3)
 */
py::array_t<float> getMeshVerticesNumpy(const Mesh& mesh) {
    const auto& vertices = mesh.vertices();
    
    py::array_t<float> result({static_cast<py::ssize_t>(vertices.size()), static_cast<py::ssize_t>(3)});
    auto buf = result.mutable_unchecked<2>();
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        buf(i, 0) = vertices[i].x;
        buf(i, 1) = vertices[i].y;
        buf(i, 2) = vertices[i].z;
    }
    
    return result;
}

/**
 * Get mesh triangles as NumPy array (Mx3)
 */
py::array_t<int> getMeshTrianglesNumpy(const Mesh& mesh) {
    const auto& triangles = mesh.triangles();
    
    py::array_t<int> result({static_cast<py::ssize_t>(triangles.size()), static_cast<py::ssize_t>(3)});
    auto buf = result.mutable_unchecked<2>();
    
    for (size_t i = 0; i < triangles.size(); ++i) {
        buf(i, 0) = triangles[i][0];
        buf(i, 1) = triangles[i][1];
        buf(i, 2) = triangles[i][2];
    }
    
    return result;
}

/**
 * Create Mesh from NumPy arrays
 */
Mesh createMeshFromNumpy(py::array_t<float> vertices, py::array_t<int> triangles) {
    auto vBuf = vertices.unchecked<2>();
    auto tBuf = triangles.unchecked<2>();
    
    std::vector<Vector3f> verts;
    verts.reserve(vBuf.shape(0));
    for (py::ssize_t i = 0; i < vBuf.shape(0); ++i) {
        verts.push_back({vBuf(i, 0), vBuf(i, 1), vBuf(i, 2)});
    }
    
    std::vector<Triangle> tris;
    tris.reserve(tBuf.shape(0));
    for (py::ssize_t i = 0; i < tBuf.shape(0); ++i) {
        tris.push_back({tBuf(i, 0), tBuf(i, 1), tBuf(i, 2)});
    }
    
    return Mesh::fromTriangles(verts, tris);
}

/**
 * Get point cloud points as NumPy array (Nx3)
 */
py::array_t<float> getPointCloudPointsNumpy(const PointCloud& cloud) {
    const auto& points = cloud.points();
    
    py::array_t<float> result({static_cast<py::ssize_t>(points.size()), static_cast<py::ssize_t>(3)});
    auto buf = result.mutable_unchecked<2>();
    
    for (size_t i = 0; i < points.size(); ++i) {
        buf(i, 0) = points[i].x;
        buf(i, 1) = points[i].y;
        buf(i, 2) = points[i].z;
    }
    
    return result;
}

// =============================================================================
// PYBIND11 MODULE DEFINITION
// =============================================================================

PYBIND11_MODULE(meshlib_core, m) {
    m.doc() = "MeshLib Core - 3D Mesh Processing Library";
    
    // =========================================================================
    // Core Types
    // =========================================================================
    
    // Vector2f
    py::class_<Vector2f>(m, "Vector2f")
        .def(py::init<>())
        .def(py::init<float, float>())
        .def_readwrite("x", &Vector2f::x)
        .def_readwrite("y", &Vector2f::y)
        .def("__getitem__", [](const Vector2f& v, int i) { return v[i]; })
        .def("__setitem__", [](Vector2f& v, int i, float val) { v[i] = val; })
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * float())
        .def(py::self / float())
        .def("dot", &Vector2f::dot)
        .def("length", &Vector2f::length)
        .def("lengthSq", &Vector2f::lengthSq)
        .def("normalized", &Vector2f::normalized)
        .def_static("zero", &Vector2f::zero)
        .def_static("unitX", &Vector2f::unitX)
        .def_static("unitY", &Vector2f::unitY)
        .def("__repr__", [](const Vector2f& v) {
            return "Vector2f(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
        });
    
    // Vector3f
    py::class_<Vector3f>(m, "Vector3f")
        .def(py::init<>())
        .def(py::init<float, float, float>())
        .def_readwrite("x", &Vector3f::x)
        .def_readwrite("y", &Vector3f::y)
        .def_readwrite("z", &Vector3f::z)
        .def("__getitem__", [](const Vector3f& v, int i) { return v[i]; })
        .def("__setitem__", [](Vector3f& v, int i, float val) { v[i] = val; })
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * float())
        .def(py::self / float())
        .def(-py::self)
        .def("dot", &Vector3f::dot)
        .def("cross", &Vector3f::cross)
        .def("length", &Vector3f::length)
        .def("lengthSq", &Vector3f::lengthSq)
        .def("normalized", &Vector3f::normalized)
        .def_static("zero", &Vector3f::zero)
        .def_static("unitX", &Vector3f::unitX)
        .def_static("unitY", &Vector3f::unitY)
        .def_static("unitZ", &Vector3f::unitZ)
        .def("__repr__", [](const Vector3f& v) {
            return "Vector3f(" + std::to_string(v.x) + ", " + 
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });
    
    // Box3f
    py::class_<Box3f>(m, "Box3f")
        .def(py::init<>())
        .def(py::init<const Vector3f&, const Vector3f&>())
        .def_readwrite("min", &Box3f::min)
        .def_readwrite("max", &Box3f::max)
        .def("valid", &Box3f::valid)
        .def("center", &Box3f::center)
        .def("size", &Box3f::size)
        .def("diagonal", &Box3f::diagonal)
        .def("volume", &Box3f::volume)
        .def("surfaceArea", &Box3f::surfaceArea)
        .def("contains", py::overload_cast<const Vector3f&>(&Box3f::contains, py::const_))
        .def("intersects", &Box3f::intersects)
        .def("expanded", py::overload_cast<float>(&Box3f::expanded, py::const_))
        .def("__repr__", [](const Box3f& b) {
            return "Box3f(min=" + std::to_string(b.min.x) + "," + std::to_string(b.min.y) + 
                   "," + std::to_string(b.min.z) + ", max=" + std::to_string(b.max.x) + 
                   "," + std::to_string(b.max.y) + "," + std::to_string(b.max.z) + ")";
        });
    
    // Matrix3f
    py::class_<Matrix3f>(m, "Matrix3f")
        .def(py::init<>())
        .def_static("identity", &Matrix3f::identity)
        .def_static("zero", &Matrix3f::zero)
        .def_static("rotationX", &Matrix3f::rotationX)
        .def_static("rotationY", &Matrix3f::rotationY)
        .def_static("rotationZ", &Matrix3f::rotationZ)
        .def_static("scale", py::overload_cast<float, float, float>(&Matrix3f::scale))
        .def("__call__", [](const Matrix3f& m, int r, int c) { return m(r, c); })
        .def("__mul__", [](const Matrix3f& m, const Vector3f& v) { return m * v; })
        .def("transposed", &Matrix3f::transposed);
    
    // AffineTransform3f
    py::class_<AffineTransform3f>(m, "AffineTransform3f")
        .def(py::init<>())
        .def_readwrite("linear", &AffineTransform3f::linear)
        .def_readwrite("translation", &AffineTransform3f::translation)
        .def_static("identity", &AffineTransform3f::identity)
        .def_static("translation", &AffineTransform3f::translation)
        .def_static("rotationX", &AffineTransform3f::rotationX)
        .def_static("rotationY", &AffineTransform3f::rotationY)
        .def_static("rotationZ", &AffineTransform3f::rotationZ)
        .def_static("scale", py::overload_cast<float>(&AffineTransform3f::scale))
        .def("apply", &AffineTransform3f::apply)
        .def("applyToDirection", &AffineTransform3f::applyToDirection);
    
    // ErrorCode enum
    py::enum_<ErrorCode>(m, "ErrorCode")
        .value("Success", ErrorCode::Success)
        .value("Failed", ErrorCode::Failed)
        .value("InvalidInput", ErrorCode::InvalidInput)
        .value("FileNotFound", ErrorCode::FileNotFound)
        .value("IOError", ErrorCode::IOError)
        .value("EmptyMesh", ErrorCode::EmptyMesh)
        .value("Cancelled", ErrorCode::Cancelled)
        .value("NotImplemented", ErrorCode::NotImplemented)
        .export_values();
    
    // Status
    py::class_<Status>(m, "Status")
        .def(py::init<>())
        .def_readwrite("code", &Status::code)
        .def_readwrite("message", &Status::message)
        .def("ok", [](const Status& s) { return s.code == ErrorCode::Success; });
    
    // =========================================================================
    // Mesh Class
    // =========================================================================
    
    py::class_<Mesh>(m, "Mesh")
        .def(py::init<>())
        .def_static("from_triangles", &Mesh::fromTriangles)
        .def_static("from_numpy", &createMeshFromNumpy)
        .def("empty", &Mesh::empty)
        .def("vertex_count", &Mesh::vertexCount)
        .def("triangle_count", &Mesh::triangleCount)
        .def("vertices", &Mesh::vertices)
        .def("triangles", &Mesh::triangles)
        .def("vertices_numpy", &getMeshVerticesNumpy)
        .def("triangles_numpy", &getMeshTrianglesNumpy)
        .def("vertex_normals", &Mesh::vertexNormals)
        .def("face_normals", &Mesh::faceNormals)
        .def("bounding_box", &Mesh::boundingBox)
        .def("centroid", &Mesh::centroid)
        .def("area", &Mesh::area)
        .def("volume", &Mesh::volume)
        .def("face_normal", &Mesh::faceNormal)
        .def("face_area", &Mesh::faceArea)
        .def("face_centroid", &Mesh::faceCentroid)
        .def("is_closed", &Mesh::isClosed)
        .def("transform", &Mesh::transform)
        .def("translate", &Mesh::translate)
        .def("scale", py::overload_cast<float>(&Mesh::scale, py::const_))
        .def("flip_faces", &Mesh::flipFaces)
        .def("merge_close_vertices", &Mesh::mergeCloseVertices)
        .def("__repr__", [](const Mesh& m) {
            return "<Mesh vertices=" + std::to_string(m.vertexCount()) + 
                   " triangles=" + std::to_string(m.triangleCount()) + ">";
        });
    
    // =========================================================================
    // PointCloud Class
    // =========================================================================
    
    py::class_<PointCloud>(m, "PointCloud")
        .def(py::init<>())
        .def(py::init<const std::vector<Vector3f>&>())
        .def("empty", &PointCloud::empty)
        .def("size", &PointCloud::size)
        .def("points", &PointCloud::points)
        .def("points_numpy", &getPointCloudPointsNumpy)
        .def("has_normals", &PointCloud::hasNormals)
        .def("has_colors", &PointCloud::hasColors)
        .def("normals", &PointCloud::normals)
        .def("colors", &PointCloud::colors)
        .def("bounding_box", &PointCloud::boundingBox)
        .def("centroid", &PointCloud::centroid)
        .def("add_point", py::overload_cast<const Vector3f&>(&PointCloud::addPoint))
        .def("transform", &PointCloud::transform)
        .def("estimate_normals", &PointCloud::estimateNormals, py::arg("k") = 6)
        .def("subsampled", &PointCloud::subsampled)
        .def_static("merge", &PointCloud::merge)
        .def("__repr__", [](const PointCloud& c) {
            return "<PointCloud size=" + std::to_string(c.size()) + ">";
        });
    
    // =========================================================================
    // Primitives
    // =========================================================================
    
    m.def("create_sphere", &createSphere, 
          py::arg("radius") = 1.0f, 
          py::arg("segments") = 32, 
          py::arg("rings") = 16,
          "Create a UV sphere mesh");
    
    m.def("create_icosphere", &createIcosphere,
          py::arg("radius") = 1.0f,
          py::arg("subdivisions") = 2,
          "Create an icosphere mesh");
    
    m.def("create_box", &createBox,
          py::arg("size"),
          "Create a box mesh");
    
    m.def("create_cylinder", &createCylinder,
          py::arg("radius") = 1.0f,
          py::arg("height") = 2.0f,
          py::arg("segments") = 32,
          "Create a cylinder mesh");
    
    m.def("create_cone", &createCone,
          py::arg("radius") = 1.0f,
          py::arg("height") = 2.0f,
          py::arg("segments") = 32,
          "Create a cone mesh");
    
    m.def("create_torus", &createTorus,
          py::arg("major_radius") = 1.0f,
          py::arg("minor_radius") = 0.3f,
          py::arg("major_segments") = 32,
          py::arg("minor_segments") = 16,
          "Create a torus mesh");
    
    m.def("create_plane", py::overload_cast<float, float, int, int>(&createPlane),
          py::arg("width") = 1.0f,
          py::arg("height") = 1.0f,
          py::arg("divisions_x") = 1,
          py::arg("divisions_y") = 1,
          "Create a plane mesh");
    
    m.def("create_disc", &createDisc,
          py::arg("radius") = 1.0f,
          py::arg("segments") = 32,
          "Create a disc mesh");
    
    // =========================================================================
    // I/O
    // =========================================================================
    
    py::enum_<MeshFormat>(m, "MeshFormat")
        .value("Unknown", MeshFormat::Unknown)
        .value("OBJ", MeshFormat::OBJ)
        .value("STL", MeshFormat::STL)
        .value("PLY", MeshFormat::PLY)
        .export_values();
    
    m.def("detect_format", &detectFormat, "Detect mesh file format from path");
    
    m.def("load_mesh", [](const std::string& path) {
        auto [mesh, status] = loadMesh(path);
        if (status.code != ErrorCode::Success) {
            throw std::runtime_error(status.message);
        }
        return mesh;
    }, py::arg("path"), "Load mesh from file");
    
    m.def("save_mesh", [](const Mesh& mesh, const std::string& path) {
        auto status = saveMesh(mesh, path);
        if (status.code != ErrorCode::Success) {
            throw std::runtime_error(status.message);
        }
    }, py::arg("mesh"), py::arg("path"), "Save mesh to file");
    
    // =========================================================================
    // Algorithms - Decimate
    // =========================================================================
    
    py::class_<DecimateParams>(m, "DecimateParams")
        .def(py::init<>())
        .def_readwrite("target_triangle_count", &DecimateParams::targetTriangleCount)
        .def_readwrite("max_error", &DecimateParams::maxError)
        .def_readwrite("preserve_boundary", &DecimateParams::preserveBoundary);
    
    py::class_<DecimateResult>(m, "DecimateResult")
        .def_readonly("mesh", &DecimateResult::mesh)
        .def_readonly("status", &DecimateResult::status);
    
    m.def("decimate", &decimate, py::arg("mesh"), py::arg("params") = DecimateParams());
    m.def("decimate_to_count", &decimateToCount);
    m.def("decimate_by_ratio", &decimateByRatio);
    m.def("generate_lods", &generateLODs);
    
    // =========================================================================
    // Algorithms - Smooth
    // =========================================================================
    
    py::enum_<SmoothingMethod>(m, "SmoothingMethod")
        .value("Laplacian", SmoothingMethod::Laplacian)
        .value("Taubin", SmoothingMethod::Taubin)
        .value("HCLaplacian", SmoothingMethod::HCLaplacian)
        .value("Cotangent", SmoothingMethod::Cotangent)
        .export_values();
    
    py::class_<SmoothParams>(m, "SmoothParams")
        .def(py::init<>())
        .def_readwrite("method", &SmoothParams::method)
        .def_readwrite("iterations", &SmoothParams::iterations)
        .def_readwrite("lambda_", &SmoothParams::lambda)
        .def_readwrite("mu", &SmoothParams::mu);
    
    m.def("smooth", &smooth);
    m.def("smooth_laplacian", &smoothLaplacian,
          py::arg("mesh"), py::arg("iterations") = 3, py::arg("lambda_") = 0.5f);
    m.def("smooth_taubin", &smoothTaubin,
          py::arg("mesh"), py::arg("iterations") = 3, 
          py::arg("lambda_") = 0.5f, py::arg("mu") = -0.53f);
    m.def("relax", &relax, py::arg("mesh"), py::arg("iterations") = 3);
    
    // =========================================================================
    // Algorithms - Subdivide
    // =========================================================================
    
    py::enum_<SubdivisionScheme>(m, "SubdivisionScheme")
        .value("Loop", SubdivisionScheme::Loop)
        .value("Midpoint", SubdivisionScheme::Midpoint)
        .value("CatmullClark", SubdivisionScheme::CatmullClark)
        .value("Butterfly", SubdivisionScheme::Butterfly)
        .value("Sqrt3", SubdivisionScheme::Sqrt3)
        .export_values();
    
    py::class_<SubdivideParams>(m, "SubdivideParams")
        .def(py::init<>())
        .def_readwrite("scheme", &SubdivideParams::scheme)
        .def_readwrite("iterations", &SubdivideParams::iterations);
    
    m.def("subdivide", &subdivide);
    m.def("subdivide_loop", &subdivideLoop);
    m.def("subdivide_midpoint", &subdivideMidpoint);
    m.def("subdivide_adaptive", &subdivideAdaptive,
          py::arg("mesh"), py::arg("max_edge_length"), py::arg("max_iterations") = 10);
    
    // =========================================================================
    // Algorithms - Fill Hole
    // =========================================================================
    
    py::enum_<FillHoleMethod>(m, "FillHoleMethod")
        .value("Fan", FillHoleMethod::Fan)
        .value("Planar", FillHoleMethod::Planar)
        .value("Smooth", FillHoleMethod::Smooth)
        .value("MinimalArea", FillHoleMethod::MinimalArea)
        .export_values();
    
    py::class_<HoleInfo>(m, "HoleInfo")
        .def_readonly("boundary_vertices", &HoleInfo::boundaryVertices)
        .def_readonly("perimeter", &HoleInfo::perimeter)
        .def_readonly("area", &HoleInfo::area);
    
    py::class_<FillHoleParams>(m, "FillHoleParams")
        .def(py::init<>())
        .def_readwrite("method", &FillHoleParams::method)
        .def_readwrite("max_hole_size", &FillHoleParams::maxHoleSize);
    
    py::class_<FillHoleResult>(m, "FillHoleResult")
        .def_readonly("mesh", &FillHoleResult::mesh)
        .def_readonly("holes", &FillHoleResult::holes)
        .def_readonly("status", &FillHoleResult::status);
    
    m.def("find_holes", &findHoles);
    m.def("fill_holes", &fillHoles, py::arg("mesh"), py::arg("params") = FillHoleParams());
    
    // =========================================================================
    // Algorithms - ICP
    // =========================================================================
    
    py::enum_<ICPMethod>(m, "ICPMethod")
        .value("PointToPoint", ICPMethod::PointToPoint)
        .value("PointToPlane", ICPMethod::PointToPlane)
        .export_values();
    
    py::enum_<ICPSampling>(m, "ICPSampling")
        .value("All", ICPSampling::All)
        .value("Random", ICPSampling::Random)
        .value("Uniform", ICPSampling::Uniform)
        .export_values();
    
    py::class_<ICPParams>(m, "ICPParams")
        .def(py::init<>())
        .def_readwrite("method", &ICPParams::method)
        .def_readwrite("max_iterations", &ICPParams::maxIterations)
        .def_readwrite("tolerance", &ICPParams::tolerance)
        .def_readwrite("max_correspondence_distance", &ICPParams::maxCorrespondenceDistance)
        .def_readwrite("sampling", &ICPParams::sampling)
        .def_readwrite("max_correspondences", &ICPParams::maxCorrespondences);
    
    py::class_<ICPResult>(m, "ICPResult")
        .def_readonly("transform", &ICPResult::transform)
        .def_readonly("final_error", &ICPResult::finalError)
        .def_readonly("iterations", &ICPResult::iterations)
        .def_readonly("converged", &ICPResult::converged)
        .def_readonly("status", &ICPResult::status);
    
    m.def("icp", py::overload_cast<const Mesh&, const Mesh&, const ICPParams&>(&icp),
          py::arg("source"), py::arg("target"), py::arg("params") = ICPParams());
    m.def("icp", py::overload_cast<const PointCloud&, const PointCloud&, const ICPParams&>(&icp),
          py::arg("source"), py::arg("target"), py::arg("params") = ICPParams());
    m.def("align_mesh", &alignMesh,
          py::arg("source"), py::arg("target"), py::arg("params") = ICPParams());
    m.def("align_point_cloud", &alignPointCloud,
          py::arg("source"), py::arg("target"), py::arg("params") = ICPParams());
    
    // =========================================================================
    // Algorithms - Boolean
    // =========================================================================
    
    py::enum_<BooleanOperation>(m, "BooleanOperation")
        .value("Union", BooleanOperation::Union)
        .value("Intersection", BooleanOperation::Intersection)
        .value("Difference", BooleanOperation::Difference)
        .value("SymmetricDifference", BooleanOperation::SymmetricDifference)
        .export_values();
    
    py::class_<BooleanParams>(m, "BooleanParams")
        .def(py::init<>());
    
    py::class_<BooleanResult>(m, "BooleanResult")
        .def_readonly("mesh", &BooleanResult::mesh)
        .def_readonly("status", &BooleanResult::status);
    
    m.def("boolean", &boolean,
          py::arg("mesh_a"), py::arg("mesh_b"), py::arg("operation"),
          py::arg("params") = BooleanParams());
    m.def("boolean_union", &booleanUnion,
          py::arg("mesh_a"), py::arg("mesh_b"), py::arg("params") = BooleanParams());
    m.def("boolean_intersection", &booleanIntersection,
          py::arg("mesh_a"), py::arg("mesh_b"), py::arg("params") = BooleanParams());
    m.def("boolean_difference", &booleanDifference,
          py::arg("mesh_a"), py::arg("mesh_b"), py::arg("params") = BooleanParams());
    
    // =========================================================================
    // Version Info
    // =========================================================================
    
    m.attr("__version__") = "1.0.0";
    m.attr("VERSION_MAJOR") = 1;
    m.attr("VERSION_MINOR") = 0;
    m.attr("VERSION_PATCH") = 0;
}
