#pragma once

/**
 * @file Box.h
 * @brief Axis-aligned bounding box
 */

#include "Vector.h"
#include <limits>
#include <algorithm>

namespace meshlib {

/**
 * @brief 3D axis-aligned bounding box
 */
template <typename T>
struct Box3 {
    Vector3<T> min;
    Vector3<T> max;
    
    /// Default constructor - creates an invalid (inverted) box
    constexpr Box3() noexcept 
        : min(std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max())
        , max(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest()) 
    {}
    
    /// Construct from min and max points
    constexpr Box3(const Vector3<T>& minPt, const Vector3<T>& maxPt) noexcept
        : min(minPt), max(maxPt) {}
    
    /// Check if the box is valid (min <= max in all dimensions)
    constexpr bool valid() const noexcept {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }
    
    /// Check if box is empty (invalid)
    constexpr bool empty() const noexcept { return !valid(); }
    
    /// Get the center of the box
    constexpr Vector3<T> center() const noexcept {
        return (min + max) * T(0.5);
    }
    
    /// Get the size (dimensions) of the box
    constexpr Vector3<T> size() const noexcept {
        return valid() ? max - min : Vector3<T>::zero();
    }
    
    /// Get the diagonal length of the box
    T diagonal() const noexcept {
        return size().length();
    }
    
    /// Get the volume of the box
    constexpr T volume() const noexcept {
        if (!valid()) return T(0);
        Vector3<T> s = size();
        return s.x * s.y * s.z;
    }
    
    /// Get the surface area of the box
    constexpr T surfaceArea() const noexcept {
        if (!valid()) return T(0);
        Vector3<T> s = size();
        return T(2) * (s.x * s.y + s.y * s.z + s.z * s.x);
    }
    
    /// Expand the box to include a point
    void include(const Vector3<T>& p) noexcept {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }
    
    /// Expand the box to include another box
    void include(const Box3& other) noexcept {
        if (other.valid()) {
            include(other.min);
            include(other.max);
        }
    }
    
    /// Check if a point is inside the box
    constexpr bool contains(const Vector3<T>& p) const noexcept {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }
    
    /// Check if this box intersects another box
    constexpr bool intersects(const Box3& other) const noexcept {
        if (!valid() || !other.valid()) return false;
        return max.x >= other.min.x && min.x <= other.max.x &&
               max.y >= other.min.y && min.y <= other.max.y &&
               max.z >= other.min.z && min.z <= other.max.z;
    }
    
    /// Get intersection of two boxes (may be invalid if no intersection)
    constexpr Box3 intersection(const Box3& other) const noexcept {
        return {
            {std::max(min.x, other.min.x), std::max(min.y, other.min.y), std::max(min.z, other.min.z)},
            {std::min(max.x, other.max.x), std::min(max.y, other.max.y), std::min(max.z, other.max.z)}
        };
    }
    
    /// Expand the box by a margin in all directions
    constexpr Box3 expanded(T margin) const noexcept {
        return {
            {min.x - margin, min.y - margin, min.z - margin},
            {max.x + margin, max.y + margin, max.z + margin}
        };
    }
    
    /// Get one of the 8 corners of the box
    constexpr Vector3<T> corner(int index) const noexcept {
        return {
            (index & 1) ? max.x : min.x,
            (index & 2) ? max.y : min.y,
            (index & 4) ? max.z : min.z
        };
    }
    
    // Comparison
    constexpr bool operator==(const Box3& other) const noexcept {
        return min == other.min && max == other.max;
    }
    
    constexpr bool operator!=(const Box3& other) const noexcept {
        return !(*this == other);
    }
    
    // Static constructors
    
    /// Create a box from a center point and half-extents
    static constexpr Box3 fromCenterHalfSize(const Vector3<T>& center, const Vector3<T>& halfSize) noexcept {
        return {center - halfSize, center + halfSize};
    }
    
    /// Create a box from a center point and full size
    static constexpr Box3 fromCenterSize(const Vector3<T>& center, const Vector3<T>& size) noexcept {
        return fromCenterHalfSize(center, size * T(0.5));
    }
    
    /// Create a box containing a single point
    static constexpr Box3 fromPoint(const Vector3<T>& p) noexcept {
        return {p, p};
    }
    
    /// Create a box from an array of points
    template <typename Iterator>
    static Box3 fromPoints(Iterator begin, Iterator end) noexcept {
        Box3 box;
        for (auto it = begin; it != end; ++it) {
            box.include(*it);
        }
        return box;
    }
    
    /// Compute signed distance from a point to the box surface
    /// Negative inside, positive outside
    T signedDistance(const Vector3<T>& p) const noexcept {
        if (!valid()) return std::numeric_limits<T>::max();
        
        // Compute distance to each face
        Vector3<T> d1 = p - min;  // distance to min faces
        Vector3<T> d2 = max - p;  // distance to max faces
        
        if (contains(p)) {
            // Inside: return negative of distance to nearest face
            T minDist = std::min({d1.x, d1.y, d1.z, d2.x, d2.y, d2.z});
            return -minDist;
        }
        
        // Outside: compute distance to nearest point on box
        Vector3<T> closest{
            std::clamp(p.x, min.x, max.x),
            std::clamp(p.y, min.y, max.y),
            std::clamp(p.z, min.z, max.z)
        };
        return (p - closest).length();
    }
    
    /// Compute squared distance from a point to the box surface
    T squaredDistanceToPoint(const Vector3<T>& p) const noexcept {
        if (!valid()) return std::numeric_limits<T>::max();
        
        Vector3<T> closest{
            std::clamp(p.x, min.x, max.x),
            std::clamp(p.y, min.y, max.y),
            std::clamp(p.z, min.z, max.z)
        };
        return (p - closest).lengthSq();
    }
};

// Type aliases
using Box3f = Box3<float>;
using Box3d = Box3<double>;
using Box3i = Box3<int>;

} // namespace meshlib
