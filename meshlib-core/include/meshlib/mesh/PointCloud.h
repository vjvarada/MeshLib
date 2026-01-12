#pragma once

/**
 * @file PointCloud.h
 * @brief Point cloud data structure
 */

#include "meshlib/config.h"
#include "../core/Vector.h"
#include "../core/Matrix.h"
#include "../core/Box.h"
#include <vector>
#include <optional>

namespace meshlib {

/**
 * @brief 3D point cloud with optional normals and colors
 */
class MESHLIB_API PointCloud {
public:
    /// Default constructor
    PointCloud() = default;
    
    /// Construct from points
    explicit PointCloud(std::vector<Vector3f> points) 
        : points_(std::move(points)) {}
    
    /// Construct from points and normals
    PointCloud(std::vector<Vector3f> points, std::vector<Vector3f> normals)
        : points_(std::move(points)), normals_(std::move(normals)) {}
    
    // ==================== Accessors ====================
    
    /// Get number of points
    size_t size() const { return points_.size(); }
    
    /// Check if empty
    bool empty() const { return points_.empty(); }
    
    /// Check if normals are present
    bool hasNormals() const { return !normals_.empty() && normals_.size() == points_.size(); }
    
    /// Check if colors are present
    bool hasColors() const { return !colors_.empty() && colors_.size() == points_.size(); }
    
    /// Get all points
    const std::vector<Vector3f>& points() const { return points_; }
    std::vector<Vector3f>& points() { invalidateCache(); return points_; }
    
    /// Get all normals
    const std::vector<Vector3f>& normals() const { return normals_; }
    std::vector<Vector3f>& normals() { return normals_; }
    
    /// Get all colors (RGB, 0-1 range)
    const std::vector<Vector3f>& colors() const { return colors_; }
    std::vector<Vector3f>& colors() { return colors_; }
    
    /// Get a specific point
    const Vector3f& point(size_t index) const { return points_[index]; }
    Vector3f& point(size_t index) { invalidateCache(); return points_[index]; }
    
    /// Get a specific normal
    const Vector3f& normal(size_t index) const { return normals_[index]; }
    Vector3f& normal(size_t index) { return normals_[index]; }
    
    // ==================== Geometry ====================
    
    /// Compute the axis-aligned bounding box
    Box3f boundingBox() const;
    
    /// Compute the centroid (average of all points)
    Vector3f centroid() const;
    
    // ==================== Modification ====================
    
    /// Add a point
    void addPoint(const Vector3f& point);
    
    /// Add a point with normal
    void addPoint(const Vector3f& point, const Vector3f& normal);
    
    /// Reserve capacity
    void reserve(size_t capacity);
    
    /// Clear all data
    void clear();
    
    /// Transform all points by an affine transformation
    void transform(const AffineTransform3f& xf);
    
    /// Estimate normals from local neighborhoods
    void estimateNormals(int neighborCount = 10);
    
    /// Remove points with invalid coordinates (NaN, Inf)
    int removeInvalidPoints();
    
    /// Subsample to approximately the given number of points
    PointCloud subsampled(size_t targetCount) const;
    
    /// Merge another point cloud into this one
    void merge(const PointCloud& other);
    
private:
    std::vector<Vector3f> points_;
    std::vector<Vector3f> normals_;
    std::vector<Vector3f> colors_;
    
    mutable std::optional<Box3f> cachedBoundingBox_;
    
    void invalidateCache() { cachedBoundingBox_.reset(); }
};

} // namespace meshlib
