#pragma once

/**
 * @file FillHole.h
 * @brief Hole filling algorithms for meshes
 */

#include "meshlib/config.h"
#include "../mesh/Mesh.h"
#include "../core/Types.h"

namespace meshlib {

/**
 * @brief Strategy for filling holes
 */
enum class FillHoleMethod {
    Planar,         ///< Simple planar triangulation (fast, for planar holes)
    Minimal,        ///< Minimal area triangulation
    Smooth,         ///< Smooth surface fitting
    Curvature       ///< Curvature-aware filling
};

/**
 * @brief Parameters for hole filling
 */
struct MESHLIB_API FillHoleParams {
    /// Method for filling holes
    FillHoleMethod method = FillHoleMethod::Smooth;
    
    /// Maximum hole perimeter to fill (0 = no limit)
    float maxPerimeter = 0.0f;
    
    /// Maximum number of edges in hole boundary (0 = no limit)
    int maxBoundaryEdges = 0;
    
    /// Number of smoothing iterations for the filled region
    int smoothingIterations = 3;
    
    /// Whether to refine the filled triangles to match surrounding density
    bool refine = true;
    
    /// Target edge length for refinement (0 = auto-detect)
    float targetEdgeLength = 0.0f;
    
    /// Progress callback
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Information about a hole in a mesh
 */
struct MESHLIB_API HoleInfo {
    std::vector<int> boundaryVertices;  ///< Vertices forming the hole boundary
    float perimeter;                    ///< Length of the hole boundary
    float area;                         ///< Approximate area of the hole
    Vector3f centroid;                  ///< Center of the hole
    Vector3f normal;                    ///< Estimated normal of the hole
};

/**
 * @brief Result of hole filling operation
 */
struct MESHLIB_API FillHoleResult {
    Mesh mesh;                  ///< Mesh with holes filled
    Result status;              ///< Operation status
    int holesFilled;            ///< Number of holes filled
    int trianglesAdded;         ///< Number of triangles added
    
    bool ok() const { return status.ok(); }
    explicit operator bool() const { return ok(); }
};

// ==================== Hole Detection ====================

/**
 * @brief Find all holes (boundary loops) in a mesh
 * @param mesh Input mesh
 * @return Vector of hole information
 */
MESHLIB_API std::vector<HoleInfo> findHoles(const Mesh& mesh);

/**
 * @brief Get the number of holes in a mesh
 * @param mesh Input mesh
 * @return Number of holes
 */
MESHLIB_API int countHoles(const Mesh& mesh);

/**
 * @brief Check if a mesh is closed (has no holes)
 * @param mesh Input mesh
 * @return True if mesh is watertight
 */
MESHLIB_API bool isClosed(const Mesh& mesh);

// ==================== Hole Filling ====================

/**
 * @brief Fill all holes in a mesh
 * @param mesh Input mesh
 * @param params Filling parameters
 * @return Result containing mesh with holes filled
 */
MESHLIB_API FillHoleResult fillHoles(const Mesh& mesh, const FillHoleParams& params = {});

/**
 * @brief Fill a specific hole
 * @param mesh Input mesh
 * @param holeIndex Index of the hole to fill (from findHoles)
 * @param params Filling parameters
 * @return Result containing mesh with hole filled
 */
MESHLIB_API FillHoleResult fillHole(const Mesh& mesh, int holeIndex, const FillHoleParams& params = {});

/**
 * @brief Fill a hole defined by boundary vertices
 * @param mesh Input mesh
 * @param boundaryVertices Vertices forming the hole boundary (in order)
 * @param params Filling parameters
 * @return Result containing mesh with hole filled
 */
MESHLIB_API FillHoleResult fillHoleByBoundary(
    const Mesh& mesh, 
    const std::vector<int>& boundaryVertices,
    const FillHoleParams& params = {});

/**
 * @brief Simple planar hole filling
 * @param mesh Input mesh
 * @return Mesh with all holes filled using planar triangulation
 */
MESHLIB_API Mesh fillHolesPlanar(const Mesh& mesh);

/**
 * @brief Fill the largest hole in a mesh
 * @param mesh Input mesh
 * @param params Filling parameters
 * @return Result containing mesh with largest hole filled
 */
MESHLIB_API FillHoleResult fillLargestHole(const Mesh& mesh, const FillHoleParams& params = {});

} // namespace meshlib
