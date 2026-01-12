#pragma once

/**
 * @file Smooth.h
 * @brief Mesh smoothing algorithms
 */

#include "meshlib/config.h"
#include "../mesh/Mesh.h"
#include "../core/Types.h"

namespace meshlib {

/**
 * @brief Smoothing algorithm type
 */
enum class SmoothingMethod {
    Laplacian,      ///< Laplacian smoothing (simple averaging)
    Taubin,         ///< Taubin smoothing (volume preserving)
    HCLaplacian,    ///< HC Laplacian smoothing (feature preserving)
    Cotangent       ///< Cotangent weight smoothing
};

/**
 * @brief Parameters for mesh smoothing
 */
struct MESHLIB_API SmoothParams {
    /// Smoothing method
    SmoothingMethod method = SmoothingMethod::Taubin;
    
    /// Number of smoothing iterations
    int iterations = 3;
    
    /// Smoothing factor (0 = no smoothing, 1 = full smoothing)
    float lambda = 0.5f;
    
    /// For Taubin smoothing: contraction factor (should be negative, |mu| > lambda)
    float mu = -0.53f;
    
    /// Preserve boundary vertices
    bool preserveBoundary = true;
    
    /// Preserve sharp features
    bool preserveSharpFeatures = false;
    
    /// Angle threshold for sharp features (radians)
    float sharpAngleThreshold = 0.5f;
    
    /// Progress callback
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Result of smoothing operation
 */
struct MESHLIB_API SmoothResult {
    Mesh mesh;          ///< Smoothed mesh
    Result status;      ///< Operation status
    
    bool ok() const { return status.ok(); }
    explicit operator bool() const { return ok(); }
};

// ==================== Smoothing Functions ====================

/**
 * @brief Smooth a mesh using the specified parameters
 * @param mesh Input mesh
 * @param params Smoothing parameters
 * @return Result containing smoothed mesh
 */
MESHLIB_API SmoothResult smooth(const Mesh& mesh, const SmoothParams& params = {});

/**
 * @brief Laplacian smoothing
 * @param mesh Input mesh
 * @param iterations Number of iterations
 * @param lambda Smoothing factor
 * @return Smoothed mesh
 */
MESHLIB_API Mesh smoothLaplacian(const Mesh& mesh, int iterations = 3, float lambda = 0.5f);

/**
 * @brief Taubin smoothing (volume-preserving)
 * @param mesh Input mesh
 * @param iterations Number of iterations
 * @param lambda Forward smoothing factor
 * @param mu Backward smoothing factor (should be negative)
 * @return Smoothed mesh
 */
MESHLIB_API Mesh smoothTaubin(const Mesh& mesh, int iterations = 3, float lambda = 0.5f, float mu = -0.53f);

/**
 * @brief Smooth only selected vertices
 * @param mesh Input mesh
 * @param vertexIndices Indices of vertices to smooth
 * @param params Smoothing parameters
 * @return Smoothed mesh
 */
MESHLIB_API Mesh smoothSelected(const Mesh& mesh, const std::vector<int>& vertexIndices, const SmoothParams& params = {});

/**
 * @brief Relax mesh (move vertices to improve triangle quality)
 * @param mesh Input mesh
 * @param iterations Number of relaxation iterations
 * @return Relaxed mesh
 */
Mesh relax(const Mesh& mesh, int iterations = 3);

} // namespace meshlib
