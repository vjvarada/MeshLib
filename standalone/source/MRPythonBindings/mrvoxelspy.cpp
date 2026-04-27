// MRVoxelsPy - Python module for MRVoxels
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Forward declarations
void bindVoxelsOperations(py::module_& m);

PYBIND11_MODULE(mrvoxelspy, m) {
    m.doc() = "MeshLib Voxels Python bindings - OpenVDB-based voxel operations";
    
    bindVoxelsOperations(m);
    
    m.attr("__version__") = "1.0.0";
}
