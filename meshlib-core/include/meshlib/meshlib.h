#pragma once

/**
 * @file meshlib.h
 * @brief Main umbrella header for meshlib
 * 
 * Include this header to get access to all meshlib functionality.
 */

// Version information
#define MESHLIB_VERSION_MAJOR 0
#define MESHLIB_VERSION_MINOR 3
#define MESHLIB_VERSION_PATCH 0
#define MESHLIB_VERSION_STRING "0.3.0"

// Core types
#include "core/Types.h"
#include "core/Id.h"
#include "core/Vector.h"
#include "core/Matrix.h"
#include "core/Box.h"
#include "core/Quaternion.h"
#include "core/AffineXf.h"
#include "core/Line.h"
#include "core/Plane.h"
#include "core/Sphere.h"
#include "core/BitSet.h"
#include "core/IdVector.h"
#include "core/UnionFind.h"
#include "core/Expected.h"

// Mesh location types (Phase 7)
#include "core/MeshTriPoint.h"
#include "core/MeshEdgePoint.h"

// Mesh query algorithms (Phase 7)
#include "core/MeshProject.h"
#include "core/MeshComponents.h"
#include "core/MeshNormals.h"
#include "core/MeshDistance.h"

// Mesh types
#include "mesh/Mesh.h"
#include "mesh/MeshTopology.h"
#include "mesh/PointCloud.h"
#include "mesh/Primitives.h"

// Spatial indexing
#include "spatial/AABBTree.h"

// Algorithms
#include "algorithms/Boolean.h"
#include "algorithms/Decimate.h"
#include "algorithms/Subdivide.h"
#include "algorithms/FillHole.h"
#include "algorithms/ICP.h"
#include "algorithms/Smooth.h"

// I/O
#include "io/MeshIO.h"

/**
 * @namespace meshlib
 * @brief Core mesh processing library namespace
 */
namespace meshlib {

/**
 * @brief Get library version string
 * @return Version string in format "major.minor.patch"
 */
inline const char* version() {
    return MESHLIB_VERSION_STRING;
}

/**
 * @brief Get library version components
 */
inline void versionComponents(int& major, int& minor, int& patch) {
    major = MESHLIB_VERSION_MAJOR;
    minor = MESHLIB_VERSION_MINOR;
    patch = MESHLIB_VERSION_PATCH;
}

} // namespace meshlib
