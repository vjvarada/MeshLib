/**
 * @file PointCloud.cpp
 * @brief Implementation of the PointCloud class
 */

#include "meshlib/mesh/PointCloud.h"
#include <algorithm>
#include <random>
#include <cmath>
#include <numeric>

namespace meshlib {

Box3f PointCloud::boundingBox() const {
    if (cachedBoundingBox_.has_value()) {
        return *cachedBoundingBox_;
    }
    
    Box3f box;
    for (const auto& p : points_) {
        box.include(p);
    }
    
    cachedBoundingBox_ = box;
    return box;
}

Vector3f PointCloud::centroid() const {
    if (points_.empty()) {
        return Vector3f::zero();
    }
    
    Vector3f sum = Vector3f::zero();
    for (const auto& p : points_) {
        sum += p;
    }
    
    return sum / static_cast<float>(points_.size());
}

void PointCloud::addPoint(const Vector3f& point) {
    points_.push_back(point);
    invalidateCache();
}

void PointCloud::addPoint(const Vector3f& point, const Vector3f& normal) {
    points_.push_back(point);
    normals_.push_back(normal);
    invalidateCache();
}

void PointCloud::reserve(size_t capacity) {
    points_.reserve(capacity);
    if (!normals_.empty()) {
        normals_.reserve(capacity);
    }
    if (!colors_.empty()) {
        colors_.reserve(capacity);
    }
}

void PointCloud::clear() {
    points_.clear();
    normals_.clear();
    colors_.clear();
    invalidateCache();
}

void PointCloud::transform(const AffineTransform3f& xf) {
    for (auto& p : points_) {
        p = xf(p);
    }
    
    if (hasNormals()) {
        // Transform normals (rotation only, no translation)
        for (auto& n : normals_) {
            n = xf.transformDirection(n).normalized();
        }
    }
    
    invalidateCache();
}

void PointCloud::estimateNormals(int neighborCount) {
    if (points_.size() < 3) {
        return;
    }
    
    normals_.resize(points_.size());
    
    // Simple normal estimation using local PCA
    // For a more robust implementation, use a KD-tree for neighbor queries
    
    for (size_t i = 0; i < points_.size(); ++i) {
        const Vector3f& p = points_[i];
        
        // Find nearest neighbors (brute force for simplicity)
        std::vector<std::pair<float, size_t>> distances;
        distances.reserve(points_.size());
        
        for (size_t j = 0; j < points_.size(); ++j) {
            if (i != j) {
                float dist = distanceSq(p, points_[j]);
                distances.push_back({dist, j});
            }
        }
        
        std::partial_sort(distances.begin(), 
                         distances.begin() + std::min(static_cast<size_t>(neighborCount), distances.size()),
                         distances.end());
        
        // Compute covariance matrix
        Vector3f mean = p;
        int count = std::min(neighborCount, static_cast<int>(distances.size()));
        
        for (int k = 0; k < count; ++k) {
            mean += points_[distances[k].second];
        }
        mean = mean / static_cast<float>(count + 1);
        
        // Simplified: use cross product of two vectors to neighbors
        if (count >= 2) {
            Vector3f v1 = points_[distances[0].second] - p;
            Vector3f v2 = points_[distances[1].second] - p;
            normals_[i] = cross(v1, v2).normalized();
        } else {
            normals_[i] = Vector3f::unitZ();
        }
    }
}

int PointCloud::removeInvalidPoints() {
    int removed = 0;
    
    for (size_t i = 0; i < points_.size(); ) {
        if (!points_[i].isFinite()) {
            // Swap with last and pop
            std::swap(points_[i], points_.back());
            points_.pop_back();
            
            if (hasNormals()) {
                std::swap(normals_[i], normals_.back());
                normals_.pop_back();
            }
            
            if (hasColors()) {
                std::swap(colors_[i], colors_.back());
                colors_.pop_back();
            }
            
            ++removed;
        } else {
            ++i;
        }
    }
    
    if (removed > 0) {
        invalidateCache();
    }
    
    return removed;
}

PointCloud PointCloud::subsampled(size_t targetCount) const {
    if (points_.size() <= targetCount) {
        return *this;
    }
    
    PointCloud result;
    result.reserve(targetCount);
    
    // Random subsampling
    std::vector<size_t> indices(points_.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);
    
    for (size_t i = 0; i < targetCount; ++i) {
        result.points_.push_back(points_[indices[i]]);
        
        if (hasNormals()) {
            result.normals_.push_back(normals_[indices[i]]);
        }
        
        if (hasColors()) {
            result.colors_.push_back(colors_[indices[i]]);
        }
    }
    
    return result;
}

void PointCloud::merge(const PointCloud& other) {
    size_t oldSize = points_.size();
    
    points_.insert(points_.end(), other.points_.begin(), other.points_.end());
    
    if (hasNormals() && other.hasNormals()) {
        normals_.insert(normals_.end(), other.normals_.begin(), other.normals_.end());
    } else if (hasNormals()) {
        normals_.resize(oldSize); // Keep only original normals
    }
    
    if (hasColors() && other.hasColors()) {
        colors_.insert(colors_.end(), other.colors_.begin(), other.colors_.end());
    } else if (hasColors()) {
        colors_.resize(oldSize);
    }
    
    invalidateCache();
}

} // namespace meshlib
