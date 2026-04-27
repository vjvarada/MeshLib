// MRMeshPy - Main Python module for MRMesh
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

namespace py = pybind11;

// Forward declarations for binding functions
void bindVector3(py::module_& m);
void bindBox3(py::module_& m);
void bindMesh(py::module_& m);
void bindMeshOperations(py::module_& m);

PYBIND11_MODULE(mrmeshpy, m) {
    m.doc() = "MeshLib Python bindings - Industrial-grade mesh processing library";
    
    // Bind types in dependency order
    bindVector3(m);
    bindBox3(m);
    bindMesh(m);
    bindMeshOperations(m);
    
    // Version info
    m.attr("__version__") = "1.0.0";
}
