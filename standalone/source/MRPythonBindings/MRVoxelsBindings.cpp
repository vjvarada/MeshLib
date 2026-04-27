// MRVoxelsBindings.cpp - Python bindings for voxel operations
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "MRMesh/MRMesh.h"
#include "MRMesh/MRVector3.h"
#include "MRMesh/MRExpected.h"
#include "MRVoxels/MROffset.h"
#include "MRVoxels/MRVDBConversions.h"
#include "MRVoxels/MRVoxelsVolume.h"
#include "MRVoxels/MRFloatGrid.h"

namespace py = pybind11;
using namespace MR;

void bindVoxelsOperations(py::module_& m) {
    // SignDetectionMode enum
    py::enum_<SignDetectionMode>(m, "SignDetectionMode", "Mode for detecting sign of distance field")
        .value("Unsigned", SignDetectionMode::Unsigned, "No sign detection (shell)")
        .value("OpenVDB", SignDetectionMode::OpenVDB, "Use OpenVDB for sign detection")
        .value("ProjectionNormal", SignDetectionMode::ProjectionNormal, "Use projection normal")
        .value("WindingRule", SignDetectionMode::WindingRule, "Use winding rule")
        .value("HoleWindingRule", SignDetectionMode::HoleWindingRule, "Use hole winding rule");
        
    // OffsetParameters
    py::class_<OffsetParameters>(m, "OffsetParameters", "Parameters for mesh offset operations")
        .def(py::init<>())
        .def_readwrite("voxelSize", &OffsetParameters::voxelSize,
                       "Size of voxels in the grid")
        .def_readwrite("signDetectionMode", &OffsetParameters::signDetectionMode,
                       "How to detect inside/outside");
                       
    // GeneralOffsetParameters (extends OffsetParameters)
    py::class_<GeneralOffsetParameters, OffsetParameters>(m, "GeneralOffsetParameters", 
                                                           "Extended parameters for general offset operations")
        .def(py::init<>())
        .def_readwrite("minNewVertDev", &GeneralOffsetParameters::minNewVertDev,
                       "Minimum deviation for new vertices")
        .def_readwrite("maxNewRank2VertDev", &GeneralOffsetParameters::maxNewRank2VertDev,
                       "Maximum deviation for rank 2 vertices")
        .def_readwrite("maxNewRank3VertDev", &GeneralOffsetParameters::maxNewRank3VertDev,
                       "Maximum deviation for rank 3 vertices");
                       
    // suggestVoxelSize function
    m.def("suggestVoxelSize", [](const Mesh& mesh, float approxNumVoxels) {
        return suggestVoxelSize(mesh, approxNumVoxels);
    }, py::arg("mesh"), py::arg("approxNumVoxels"),
       "Suggest appropriate voxel size for given mesh and target voxel count");
       
    // offsetMesh function
    m.def("offsetMesh", [](const Mesh& mesh, float offset, const OffsetParameters& params) {
        auto result = offsetMesh(mesh, offset, params);
        if (result.has_value())
            return result.value();
        throw std::runtime_error(result.error());
    }, py::arg("mesh"), py::arg("offset"), py::arg("params") = OffsetParameters(),
       "Create an offset shell around the mesh");
       
    // generalOffsetMesh function  
    m.def("generalOffsetMesh", [](const Mesh& mesh, float offset, const GeneralOffsetParameters& params) {
        auto result = generalOffsetMesh(mesh, offset, params);
        if (result.has_value())
            return result.value();
        throw std::runtime_error(result.error());
    }, py::arg("mesh"), py::arg("offset"), py::arg("params") = GeneralOffsetParameters(),
       "Create a general offset mesh with advanced parameters");
       
    // thickenMesh function
    m.def("thickenMesh", [](const Mesh& mesh, float offset, const GeneralOffsetParameters& params) {
        auto result = thickenMesh(mesh, offset, params);
        if (result.has_value())
            return result.value();
        throw std::runtime_error(result.error());
    }, py::arg("mesh"), py::arg("offset"), py::arg("params") = GeneralOffsetParameters(),
       "Thicken an open mesh by creating a shell");
       
    // doubleOffsetMesh function
    m.def("doubleOffsetMesh", [](const Mesh& mesh, float offsetA, float offsetB, const OffsetParameters& params) {
        auto result = doubleOffsetMesh(mesh, offsetA, offsetB, params);
        if (result.has_value())
            return result.value();
        throw std::runtime_error(result.error());
    }, py::arg("mesh"), py::arg("offsetA"), py::arg("offsetB"), py::arg("params") = OffsetParameters(),
       "Apply double offset (offset then inset) for smoothing");
       
    // MeshToVolumeParams
    py::class_<MeshToVolumeParams>(m, "MeshToVolumeParams", "Parameters for mesh to volume conversion")
        .def(py::init<>())
        .def_readwrite("voxelSize", &MeshToVolumeParams::voxelSize, "Voxel size")
        .def_readwrite("surfaceOffset", &MeshToVolumeParams::surfaceOffset,
                       "Number of voxels around surface to calculate distance");
                       
    // meshToVolume function
    m.def("meshToVolume", [](const Mesh& mesh, const MeshToVolumeParams& params) {
        auto result = meshToVolume(mesh, params);
        if (result.has_value()) {
            // Return a dict with the volume info
            py::dict info;
            info["dims"] = py::make_tuple(result->dims.x, result->dims.y, result->dims.z);
            info["voxelSize"] = py::make_tuple(result->voxelSize.x, result->voxelSize.y, result->voxelSize.z);
            info["min"] = result->min;
            info["max"] = result->max;
            return info;
        }
        throw std::runtime_error(result.error());
    }, py::arg("mesh"), py::arg("params") = MeshToVolumeParams(),
       "Convert mesh to VDB volume (returns volume info dict)");
}
