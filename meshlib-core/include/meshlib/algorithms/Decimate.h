#pragma once

/**
 * @file Decimate.h
 * @brief Mesh simplification (decimation) algorithms
 */

#include "meshlib/config.h"
#include "../mesh/Mesh.h"
#include "../core/Types.h"

namespace meshlib {

/**
 * @brief Strategy for mesh decimation
 */
enum class DecimateStrategy {
    QuadricError,    ///< Quadric error metrics (QEM) - best quality
    EdgeLength,      ///< Remove shortest edges first
    Uniform          ///< Uniform vertex removal
};

/**
 * @brief Parameters for mesh decimation
 */
struct MESHLIB_API DecimateParams {
    /// Target number of triangles (0 = use ratio instead)
    size_t targetTriangleCount = 0;
    
    /// Target ratio of triangles to keep (0.0 - 1.0, used if targetTriangleCount is 0)
    float targetRatio = 0.5f;
    
    /// Maximum geometric error allowed
    float maxError = std::numeric_limits<float>::max();
    
    /// Maximum edge length in result (prevents overly long edges)
    float maxEdgeLength = std::numeric_limits<float>::max();
    
    /// Minimum face angle to preserve (in radians) - prevents flat triangles
    float minTriangleAngle = 0.0f;
    
    /// Decimation strategy
    DecimateStrategy strategy = DecimateStrategy::QuadricError;
    
    /// Preserve mesh boundary (don't collapse boundary edges)
    bool preserveBoundary = false;
    
    /// Preserve sharp features (edges with high dihedral angle)
    bool preserveSharpFeatures = false;
    
    /// Angle threshold for sharp features (in radians)
    float sharpAngleThreshold = 0.5f;  // ~30 degrees
    
    /// Weight for boundary edge preservation
    float boundaryWeight = 1.0f;
    
    /// Progress callback
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Result of decimation operation
 */
struct MESHLIB_API DecimateResult {
    Mesh mesh;                  ///< Simplified mesh
    Result status;              ///< Operation status
    size_t removedTriangles;    ///< Number of triangles removed
    size_t removedVertices;     ///< Number of vertices removed
    float maxErrorAchieved;     ///< Maximum error in the result
    
    bool ok() const { return status.ok(); }
    explicit operator bool() const { return ok(); }
};

// ==================== Decimation Functions ====================

/**
 * @brief Decimate a mesh using the specified parameters
 * @param mesh Input mesh
 * @param params Decimation parameters
 * @return Result containing simplified mesh
 */
MESHLIB_API DecimateResult decimate(const Mesh& mesh, const DecimateParams& params = {});

/**
 * @brief Decimate a mesh to a target triangle count
 * @param mesh Input mesh
 * @param targetTriangles Target number of triangles
 * @return Simplified mesh
 */
MESHLIB_API Mesh decimateToCount(const Mesh& mesh, size_t targetTriangles);

/**
 * @brief Decimate a mesh by a ratio
 * @param mesh Input mesh
 * @param ratio Ratio of triangles to keep (0.0 - 1.0)
 * @return Simplified mesh
 */
MESHLIB_API Mesh decimateByRatio(const Mesh& mesh, float ratio);

/**
 * @brief Decimate mesh until error threshold is reached
 * @param mesh Input mesh
 * @param maxError Maximum allowed geometric error
 * @return Simplified mesh
 */
MESHLIB_API Mesh decimateToError(const Mesh& mesh, float maxError);

// ==================== LOD Generation ====================

/**
 * @brief Generate multiple levels of detail
 * @param mesh Input mesh
 * @param levels Number of LOD levels to generate
 * @param baseRatio Ratio for first LOD (subsequent LODs halve the triangle count)
 * @return Vector of meshes from highest to lowest detail
 */
MESHLIB_API std::vector<Mesh> generateLODs(const Mesh& mesh, int levels = 4, float baseRatio = 0.5f);

} // namespace meshlib
