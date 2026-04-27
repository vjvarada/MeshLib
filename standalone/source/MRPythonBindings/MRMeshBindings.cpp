// MRMeshBindings.cpp - Python bindings for Mesh class
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "MRMesh/MRMesh.h"
#include "MRMesh/MRMeshTopology.h"
#include "MRMesh/MRBox.h"
#include "MRMesh/MRAffineXf3.h"
#include "MRMesh/MRId.h"
#include "MRMesh/MRBitSet.h"
#include "MRMesh/MRPointCloud.h"

namespace py = pybind11;
using namespace MR;

void bindMesh(py::module_& m) {
    // FaceId
    py::class_<FaceId>(m, "FaceId", "Face identifier")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("id"))
        .def("valid", &FaceId::valid)
        .def("__int__", [](const FaceId& id) { return (int)id; })
        .def("__repr__", [](const FaceId& id) {
            return "FaceId(" + std::to_string((int)id) + ")";
        });
        
    // VertId
    py::class_<VertId>(m, "VertId", "Vertex identifier")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("id"))
        .def("valid", &VertId::valid)
        .def("__int__", [](const VertId& id) { return (int)id; })
        .def("__repr__", [](const VertId& id) {
            return "VertId(" + std::to_string((int)id) + ")";
        });
        
    // EdgeId
    py::class_<EdgeId>(m, "EdgeId", "Edge identifier")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("id"))
        .def("valid", &EdgeId::valid)
        .def("__int__", [](const EdgeId& id) { return (int)id; })
        .def("__repr__", [](const EdgeId& id) {
            return "EdgeId(" + std::to_string((int)id) + ")";
        });
        
    // FaceBitSet
    py::class_<FaceBitSet>(m, "FaceBitSet", "Bit set for faces")
        .def(py::init<>())
        .def("resize", [](FaceBitSet& bs, size_t size, bool value) { bs.resize(size, value); },
             py::arg("size"), py::arg("value") = false)
        .def("set", [](FaceBitSet& bs, FaceId id, bool value) { bs.set(id, value); },
             py::arg("id"), py::arg("value") = true)
        .def("test", [](const FaceBitSet& bs, FaceId id) { return bs.test(id); },
             py::arg("id"))
        .def("count", &FaceBitSet::count)
        .def("size", &FaceBitSet::size);
        
    // AffineXf3f
    py::class_<AffineXf3f>(m, "AffineXf3f", "3D affine transformation")
        .def(py::init<>())
        .def_static("translation", &AffineXf3f::translation, py::arg("translation"),
                    "Create a translation transformation")
        .def_static("linear", &AffineXf3f::linear, py::arg("matrix"),
                    "Create a linear transformation")
        .def("__call__", [](const AffineXf3f& xf, const Vector3f& v) { return xf(v); },
             py::arg("point"), "Transform a point");

    // MeshTopology
    py::class_<MeshTopology>(m, "MeshTopology", "Mesh topology (connectivity information)")
        .def("numValidVerts", &MeshTopology::numValidVerts, 
             "Return number of valid vertices")
        .def("numValidFaces", &MeshTopology::numValidFaces,
             "Return number of valid faces")
        .def("getValidFaces", &MeshTopology::getValidFaces, 
             py::return_value_policy::reference_internal,
             "Return bitset of valid faces")
        .def("getValidVerts", &MeshTopology::getValidVerts,
             py::return_value_policy::reference_internal,
             "Return bitset of valid vertices")
        .def("hasFace", &MeshTopology::hasFace, py::arg("f"),
             "Check if a face is valid")
        .def("hasVert", &MeshTopology::hasVert, py::arg("v"),
             "Check if a vertex is valid")
        .def("edgeWithLeft", &MeshTopology::edgeWithLeft, py::arg("f"),
             "Get an edge with given left face")
        .def("org", &MeshTopology::org, py::arg("e"),
             "Get origin vertex of edge")
        .def("dest", &MeshTopology::dest, py::arg("e"),
             "Get destination vertex of edge")
        .def("next", &MeshTopology::next, py::arg("e"),
             "Get next edge in face loop")
        .def("prev", &MeshTopology::prev, py::arg("e"),
             "Get previous edge in face loop")
        .def("isClosed", [](const MeshTopology& t) { return t.isClosed(); },
             "Check if mesh is closed (no boundary)")
        .def("findHoleRepresentiveEdges", &MeshTopology::findHoleRepresentiveEdges,
             "Find representative edges for all holes")
        .def("deleteFaces", [](MeshTopology& t, const FaceBitSet& fs) {
            t.deleteFaces(fs);
        }, py::arg("faces"),
             "Delete specified faces from the mesh")
        .def("flipOrientation", &MeshTopology::flipOrientation,
             "Flip orientation of all faces");
        
    // Mesh
    py::class_<Mesh>(m, "Mesh", "Triangle mesh with vertices and faces")
        .def(py::init<>(), "Create an empty mesh")
        .def_readwrite("topology", &Mesh::topology, "Mesh topology (connectivity)")
        .def_readwrite("points", &Mesh::points, "Vertex positions")
        .def("getBoundingBox", &Mesh::getBoundingBox, "Get axis-aligned bounding box")
        .def("volume", [](const Mesh& m) { return m.volume(); },
             "Compute volume (for closed meshes)")
        .def("area", [](const Mesh& m) { return m.area(); },
             "Compute surface area")
        .def("addPart", [](Mesh& mesh, const Mesh& other) {
            #pragma warning(push)
            #pragma warning(disable: 4996)
            mesh.addPart(other);
            #pragma warning(pop)
        }, py::arg("other"), "Add another mesh to this mesh")
        .def("invalidateCaches", [](Mesh& m) { m.invalidateCaches(); },
             "Invalidate cached data (call after modifying mesh)")
        .def("transform", [](Mesh& mesh, const AffineXf3f& xf) {
            mesh.transform(xf);
        }, py::arg("xf"), "Apply affine transformation to mesh");
        
    // PointCloud  
    py::class_<PointCloud>(m, "PointCloud", "3D point cloud")
        .def(py::init<>(), "Create an empty point cloud")
        .def_readwrite("points", &PointCloud::points, "Point positions")
        .def_readwrite("normals", &PointCloud::normals, "Point normals (optional)")
        .def_readwrite("validPoints", &PointCloud::validPoints, "Bitset of valid points")
        .def("getBoundingBox", &PointCloud::getBoundingBox, "Get axis-aligned bounding box");
}

