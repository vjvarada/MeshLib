#pragma once

/**
 * @file Subdivide.h
 * @brief Mesh subdivision algorithms
 */

#include "meshlib/config.h"
#include "../mesh/Mesh.h"
#include "../core/Types.h"

namespace meshlib {

/**
 * @brief Subdivision scheme
 */
enum class SubdivisionScheme {
    Loop,           ///< Loop subdivision (smooth, for triangular meshes)
    CatmullClark,   ///< Catmull-Clark subdivision (smooth, for quad meshes)
    MidPoint,       ///< Simple midpoint subdivision (linear)
    Butterfly,      ///< Modified butterfly (interpolating)
    Sqrt3           ///< Sqrt(3) subdivision
};

/**
 * @brief Parameters for mesh subdivision
 */
struct MESHLIB_API SubdivideParams {
    /// Number of subdivision iterations
    int iterations = 1;
    
    /// Subdivision scheme to use
    SubdivisionScheme scheme = SubdivisionScheme::Loop;
    
    /// Maximum edge length - subdivide until all edges are shorter
    float maxEdgeLength = 0.0f;  // 0 = don't use edge length criterion
    
    /// Maximum number of resulting triangles (0 = no limit)
    size_t maxTriangles = 0;
    
    /// Progress callback
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Result of subdivision operation
 */
struct MESHLIB_API SubdivideResult {
    Mesh mesh;                  ///< Subdivided mesh
    Result status;              ///< Operation status
    int actualIterations;       ///< Number of iterations actually performed
    
    bool ok() const { return status.ok(); }
    explicit operator bool() const { return ok(); }
};

// ==================== Subdivision Functions ====================

/**
 * @brief Subdivide a mesh using the specified parameters
 * @param mesh Input mesh
 * @param params Subdivision parameters
 * @return Result containing subdivided mesh
 */
MESHLIB_API SubdivideResult subdivide(const Mesh& mesh, const SubdivideParams& params = {});

/**
 * @brief Perform Loop subdivision
 * @param mesh Input mesh
 * @param iterations Number of subdivision iterations
 * @return Subdivided mesh
 */
MESHLIB_API Mesh subdivideLoop(const Mesh& mesh, int iterations = 1);

/**
 * @brief Perform simple midpoint subdivision (linear)
 * @param mesh Input mesh
 * @param iterations Number of subdivision iterations
 * @return Subdivided mesh
 */
MESHLIB_API Mesh subdivideMidpoint(const Mesh& mesh, int iterations = 1);

/**
 * @brief Perform adaptive subdivision based on edge length
 * @param mesh Input mesh
 * @param maxEdgeLength Maximum allowed edge length
 * @return Subdivided mesh
 */
MESHLIB_API Mesh subdivideAdaptive(const Mesh& mesh, float maxEdgeLength);

/**
 * @brief Subdivide only selected faces
 * @param mesh Input mesh
 * @param faceIndices Indices of faces to subdivide
 * @param iterations Number of subdivision iterations
 * @return Subdivided mesh
 */
MESHLIB_API Mesh subdivideSelected(const Mesh& mesh, const std::vector<int>& faceIndices, int iterations = 1);

} // namespace meshlib
