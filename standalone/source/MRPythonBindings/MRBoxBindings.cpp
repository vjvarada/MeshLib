// MRBoxBindings.cpp - Python bindings for Box3f
#include <pybind11/pybind11.h>
#include "MRMesh/MRBox.h"
#include "MRMesh/MRVector3.h"

namespace py = pybind11;
using namespace MR;

void bindBox3(py::module_& m) {
    py::class_<Box3f>(m, "Box3f", "3D axis-aligned bounding box")
        .def(py::init<>(), "Default constructor (invalid box)")
        .def(py::init<const Vector3f&, const Vector3f&>(), 
             py::arg("min"), py::arg("max"),
             "Construct from min and max corners")
        .def_readwrite("min", &Box3f::min, "Minimum corner")
        .def_readwrite("max", &Box3f::max, "Maximum corner")
        .def("valid", &Box3f::valid, "Check if the box is valid (min <= max)")
        .def("center", &Box3f::center, "Return the center of the box")
        .def("size", &Box3f::size, "Return the size (max - min)")
        .def("diagonal", &Box3f::diagonal, "Return the diagonal length")
        .def("volume", &Box3f::volume, "Return the volume of the box")
        .def("include", py::overload_cast<const Vector3f&>(&Box3f::include),
             py::arg("point"), "Expand box to include the point")
        .def("contains", py::overload_cast<const Vector3f&>(&Box3f::contains, py::const_),
             py::arg("point"), "Check if point is inside the box")
        .def("__repr__", [](const Box3f& b) {
            return "Box3f(min=(" + std::to_string(b.min.x) + ", " + 
                   std::to_string(b.min.y) + ", " + std::to_string(b.min.z) + "), max=(" +
                   std::to_string(b.max.x) + ", " + std::to_string(b.max.y) + ", " +
                   std::to_string(b.max.z) + "))";
        });
        
    py::class_<Box3i>(m, "Box3i", "3D axis-aligned bounding box with integer coordinates")
        .def(py::init<>(), "Default constructor (invalid box)")
        .def(py::init<const Vector3i&, const Vector3i&>(),
             py::arg("min"), py::arg("max"),
             "Construct from min and max corners")
        .def_readwrite("min", &Box3i::min, "Minimum corner")
        .def_readwrite("max", &Box3i::max, "Maximum corner")
        .def("valid", &Box3i::valid, "Check if the box is valid");
}
