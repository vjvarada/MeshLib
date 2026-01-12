#pragma once

/**
 * @file ICP.h
 * @brief Iterative Closest Point algorithm for mesh/point cloud alignment
 */

#include "meshlib/config.h"
#include "../mesh/Mesh.h"
#include "../mesh/PointCloud.h"
#include "../core/Matrix.h"
#include "../core/Types.h"

namespace meshlib {

/**
 * @brief ICP point selection method
 */
enum class ICPMethod {
    PointToPoint,   ///< Minimize point-to-point distances
    PointToPlane    ///< Minimize point-to-plane distances (requires normals)
};

/**
 * @brief ICP sampling strategy
 */
enum class ICPSampling {
    All,            ///< Use all points
    Uniform,        ///< Uniform random sampling
    Grid            ///< Grid-based sampling
};

/**
 * @brief Parameters for ICP alignment
 */
struct MESHLIB_API ICPParams {
    /// ICP method
    ICPMethod method = ICPMethod::PointToPlane;
    
    /// Sampling strategy
    ICPSampling sampling = ICPSampling::Uniform;
    
    /// Number of sample points (0 = use all)
    size_t sampleCount = 5000;
    
    /// Maximum iterations
    int maxIterations = 100;
    
    /// Convergence threshold for transformation change
    float convergenceThreshold = 1e-6f;
    
    /// Maximum correspondence distance (farther points are ignored)
    float maxCorrespondenceDistance = std::numeric_limits<float>::max();
    
    /// Distance threshold as ratio of bounding box diagonal (if maxCorrespondenceDistance not set)
    float correspondenceDistanceRatio = 0.1f;
    
    /// Reject outliers with distance > mean + outlierRejectionStdDev * stdDev
    float outlierRejectionStdDev = 3.0f;
    
    /// Whether to use outlier rejection
    bool rejectOutliers = true;
    
    /// Whether to allow scaling in transformation
    bool allowScaling = false;
    
    /// Progress callback
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Result of ICP alignment
 */
struct MESHLIB_API ICPResult {
    /// Transformation that aligns source to target
    AffineTransform3f transform;
    
    /// Rotation part of the transformation
    Matrix3f rotation;
    
    /// Translation part of the transformation
    Vector3f translation;
    
    /// Scale factor (1.0 if scaling not allowed)
    float scale = 1.0f;
    
    /// Operation status
    Result status;
    
    /// Final mean squared error
    float meanSquaredError = 0.0f;
    
    /// Final root mean squared error
    float rmse = 0.0f;
    
    /// Number of iterations performed
    int iterations = 0;
    
    /// Number of valid correspondences in final iteration
    size_t correspondenceCount = 0;
    
    /// Whether algorithm converged
    bool converged = false;
    
    bool ok() const { return status.ok(); }
    explicit operator bool() const { return ok(); }
};

// ==================== ICP Alignment ====================

/**
 * @brief Align a source mesh to a target mesh using ICP
 * @param source Mesh to transform
 * @param target Reference mesh
 * @param params ICP parameters
 * @return Result containing transformation
 */
MESHLIB_API ICPResult icp(
    const Mesh& source,
    const Mesh& target,
    const ICPParams& params = {});

/**
 * @brief Align a source point cloud to a target point cloud using ICP
 * @param source Point cloud to transform
 * @param target Reference point cloud
 * @param params ICP parameters
 * @return Result containing transformation
 */
MESHLIB_API ICPResult icp(
    const PointCloud& source,
    const PointCloud& target,
    const ICPParams& params = {});

/**
 * @brief Align a point cloud to a mesh using ICP
 * @param source Point cloud to transform
 * @param target Reference mesh
 * @param params ICP parameters
 * @return Result containing transformation
 */
MESHLIB_API ICPResult icp(
    const PointCloud& source,
    const Mesh& target,
    const ICPParams& params = {});

/**
 * @brief Align a mesh to a point cloud using ICP
 * @param source Mesh to transform
 * @param target Reference point cloud
 * @param params ICP parameters
 * @return Result containing transformation
 */
MESHLIB_API ICPResult icp(
    const Mesh& source,
    const PointCloud& target,
    const ICPParams& params = {});

// ==================== Convenience Functions ====================

/**
 * @brief Simple ICP alignment (point-to-plane)
 * @param source Mesh to transform
 * @param target Reference mesh
 * @param maxIterations Maximum iterations
 * @return Transformation matrix
 */
MESHLIB_API AffineTransform3f alignMeshes(
    const Mesh& source,
    const Mesh& target,
    int maxIterations = 50);

/**
 * @brief Compute alignment error between two meshes
 * @param meshA First mesh
 * @param meshB Second mesh
 * @return Mean distance between meshes
 */
MESHLIB_API float alignmentError(const Mesh& meshA, const Mesh& meshB);

/**
 * @brief Get the transformation to align mesh1 to mesh2 using their bounding boxes
 * @param source Source mesh
 * @param target Target mesh
 * @return Rough alignment transformation
 */
MESHLIB_API AffineTransform3f roughAlignByBoundingBox(const Mesh& source, const Mesh& target);

} // namespace meshlib
