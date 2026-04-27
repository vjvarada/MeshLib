// MRVectorBindings.cpp - Python bindings for Vector3f
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "MRMesh/MRVector3.h"
#include "MRMesh/MRVector2.h"
#include "MRMesh/MRVector.h"
#include "MRMesh/MRId.h"

namespace py = pybind11;
using namespace MR;

void bindVector3(py::module_& m) {
    // VertCoords binding (Vector of Vector3f with VertId indexing)
    py::class_<VertCoords>(m, "VertCoords", "Container of vertex positions")
        .def("__len__", [](const VertCoords& v) { return v.size(); })
        .def("__getitem__", [](const VertCoords& v, size_t i) -> const Vector3f& {
            if (i >= v.size()) {
                throw py::index_error("Index out of range");
            }
            return v[VertId(i)];
        }, py::return_value_policy::reference_internal)
        .def("__setitem__", [](VertCoords& v, size_t i, const Vector3f& val) {
            if (i >= v.size()) {
                throw py::index_error("Index out of range");
            }
            v[VertId(i)] = val;
        })
        .def("resize", [](VertCoords& v, size_t size) { v.resize(size); })
        .def("size", &VertCoords::size);
    
    // Vector3f binding
    py::class_<Vector3f>(m, "Vector3f", "3D vector with float components")
        .def(py::init<>(), "Default constructor (0, 0, 0)")
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"),
             "Construct from x, y, z components")
        .def_readwrite("x", &Vector3f::x, "X component")
        .def_readwrite("y", &Vector3f::y, "Y component")
        .def_readwrite("z", &Vector3f::z, "Z component")
        .def_static("diagonal", &Vector3f::diagonal, py::arg("value"),
                    "Create vector with all components equal to value")
        .def("length", &Vector3f::length, "Return the length of the vector")
        .def("lengthSq", &Vector3f::lengthSq, "Return the squared length of the vector")
        .def("normalized", &Vector3f::normalized, "Return a normalized copy of the vector")
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * float())
        .def(float() * py::self)
        .def(py::self / float())
        .def(-py::self)
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= float())
        .def(py::self /= float())
        .def("__repr__", [](const Vector3f& v) {
            return "Vector3f(" + std::to_string(v.x) + ", " + 
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });
    
    // Vector2f binding
    py::class_<Vector2f>(m, "Vector2f", "2D vector with float components")
        .def(py::init<>(), "Default constructor (0, 0)")
        .def(py::init<float, float>(), py::arg("x"), py::arg("y"),
             "Construct from x, y components")
        .def_readwrite("x", &Vector2f::x, "X component")
        .def_readwrite("y", &Vector2f::y, "Y component")
        .def("length", &Vector2f::length, "Return the length of the vector")
        .def("lengthSq", &Vector2f::lengthSq, "Return the squared length of the vector")
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * float())
        .def("__repr__", [](const Vector2f& v) {
            return "Vector2f(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
        });
        
    // Vector3i binding
    py::class_<Vector3i>(m, "Vector3i", "3D vector with integer components")
        .def(py::init<>(), "Default constructor (0, 0, 0)")
        .def(py::init<int, int, int>(), py::arg("x"), py::arg("y"), py::arg("z"),
             "Construct from x, y, z components")
        .def_readwrite("x", &Vector3i::x, "X component")
        .def_readwrite("y", &Vector3i::y, "Y component")
        .def_readwrite("z", &Vector3i::z, "Z component")
        .def("__repr__", [](const Vector3i& v) {
            return "Vector3i(" + std::to_string(v.x) + ", " + 
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });
        
    // Utility functions
    m.def("dot", py::overload_cast<const Vector3f&, const Vector3f&>(&MR::dot<float>),
          py::arg("a"), py::arg("b"), "Compute dot product of two Vector3f");
    m.def("cross", py::overload_cast<const Vector3f&, const Vector3f&>(&MR::cross<float>),
          py::arg("a"), py::arg("b"), "Compute cross product of two Vector3f");
}
