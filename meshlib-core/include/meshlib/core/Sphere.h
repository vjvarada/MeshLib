#pragma once

/**
 * @file Sphere.h
 * @brief Sphere and other primitive shapes
 * 
 * Industrial-strength port of MeshLib's geometric primitives
 */

#include "meshlib/config.h"
#include "Vector.h"
#include "Line.h"
#include <cmath>
#include <optional>
#include <array>

namespace meshlib {

/**
 * @brief Sphere in 3D space
 */
template <typename T>
struct Sphere3 {
    Vector3<T> center;
    T radius = 1;
    
    /// Default constructor - unit sphere at origin
    constexpr Sphere3() noexcept = default;
    
    /// Construct from center and radius
    constexpr Sphere3(const Vector3<T>& c, T r) noexcept
        : center(c), radius(r) {}
    
    // ==================== Queries ====================
    
    /// Check if point is inside sphere
    constexpr bool contains(const Vector3<T>& point) const noexcept {
        return (point - center).lengthSq() <= radius * radius;
    }
    
    /// Signed distance from point to sphere surface (negative inside)
    T signedDistance(const Vector3<T>& point) const noexcept {
        return (point - center).length() - radius;
    }
    
    /// Distance from point to sphere surface (always positive)
    T distance(const Vector3<T>& point) const noexcept {
        return std::abs(signedDistance(point));
    }
    
    /// Project point onto sphere surface
    Vector3<T> project(const Vector3<T>& point) const noexcept {
        Vector3<T> dir = point - center;
        T len = dir.length();
        if (len < T(1e-10)) {
            return center + Vector3<T>(radius, 0, 0);
        }
        return center + dir * (radius / len);
    }
    
    /// Get surface normal at point (assumes point is on sphere)
    Vector3<T> normal(const Vector3<T>& point) const noexcept {
        return (point - center).normalized();
    }
    
    /// Surface area
    constexpr T area() const noexcept {
        return T(4) * T(3.14159265358979323846) * radius * radius;
    }
    
    /// Volume
    constexpr T volume() const noexcept {
        return T(4.0 / 3.0) * T(3.14159265358979323846) * radius * radius * radius;
    }
    
    // ==================== Intersection ====================
    
    /// Ray-sphere intersection, returns distances to entry and exit points
    /// Returns nullopt if ray misses sphere
    std::optional<std::array<T, 2>> intersect(const Line3<T>& ray) const noexcept {
        Vector3<T> oc = ray.p - center;
        T a = dot(ray.d, ray.d);
        T b = 2 * dot(oc, ray.d);
        T c = dot(oc, oc) - radius * radius;
        T discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return std::nullopt;
        
        T sqrtD = std::sqrt(discriminant);
        T t1 = (-b - sqrtD) / (2 * a);
        T t2 = (-b + sqrtD) / (2 * a);
        
        return std::array<T, 2>{t1, t2};
    }
    
    /// Check if ray intersects sphere
    bool rayHits(const Line3<T>& ray, T maxDist = std::numeric_limits<T>::max()) const noexcept {
        auto result = intersect(ray);
        if (!result) return false;
        return (*result)[0] <= maxDist || (*result)[1] <= maxDist;
    }
    
    /// Check if two spheres intersect
    bool intersects(const Sphere3& other) const noexcept {
        T dist2 = (center - other.center).lengthSq();
        T radSum = radius + other.radius;
        return dist2 <= radSum * radSum;
    }
    
    // ==================== Bounding ====================
    
    /// Bounding sphere from points
    static Sphere3 boundingSphere(const Vector3<T>* points, size_t count) noexcept;
    
    /// Minimum bounding sphere (Welzl's algorithm)
    static Sphere3 minBoundingSphere(const Vector3<T>* points, size_t count) noexcept;
};

/**
 * @brief Cylinder in 3D space (finite, capped)
 */
template <typename T>
struct Cylinder3 {
    Vector3<T> center;   ///< Center of the cylinder
    Vector3<T> axis;     ///< Axis direction (normalized)
    T radius = 1;        ///< Radius
    T height = 1;        ///< Full height (extends height/2 in each direction from center)
    
    constexpr Cylinder3() noexcept : center(), axis(0, 0, 1), radius(1), height(1) {}
    
    constexpr Cylinder3(const Vector3<T>& c, const Vector3<T>& ax, T r, T h) noexcept
        : center(c), axis(ax.normalized()), radius(r), height(h) {}
    
    /// Check if point is inside cylinder
    bool contains(const Vector3<T>& point) const noexcept {
        Vector3<T> d = point - center;
        T axial = dot(d, axis);
        if (std::abs(axial) > height / 2) return false;
        
        Vector3<T> radial = d - axis * axial;
        return radial.lengthSq() <= radius * radius;
    }
    
    /// Get bottom cap center
    constexpr Vector3<T> bottomCenter() const noexcept {
        return center - axis * (height / 2);
    }
    
    /// Get top cap center
    constexpr Vector3<T> topCenter() const noexcept {
        return center + axis * (height / 2);
    }
    
    /// Volume
    constexpr T volume() const noexcept {
        return T(3.14159265358979323846) * radius * radius * height;
    }
};

/**
 * @brief Cone in 3D space
 */
template <typename T>
struct Cone3 {
    Vector3<T> apex;     ///< Apex (tip) of the cone
    Vector3<T> axis;     ///< Direction from apex to base (normalized)
    T angle;             ///< Half-angle at apex (radians)
    T height;            ///< Height from apex to base
    
    constexpr Cone3() noexcept 
        : apex(), axis(0, 0, -1), angle(T(0.5)), height(1) {}
    
    constexpr Cone3(const Vector3<T>& a, const Vector3<T>& ax, T ang, T h) noexcept
        : apex(a), axis(ax.normalized()), angle(ang), height(h) {}
    
    /// Radius at the base
    T baseRadius() const noexcept {
        return height * std::tan(angle);
    }
    
    /// Check if point is inside cone
    bool contains(const Vector3<T>& point) const noexcept {
        Vector3<T> d = point - apex;
        T axial = dot(d, axis);
        if (axial < 0 || axial > height) return false;
        
        T maxRadius = axial * std::tan(angle);
        Vector3<T> radial = d - axis * axial;
        return radial.lengthSq() <= maxRadius * maxRadius;
    }
    
    /// Volume
    T volume() const noexcept {
        T r = baseRadius();
        return T(3.14159265358979323846 / 3.0) * r * r * height;
    }
};

/**
 * @brief Capsule (cylinder with hemispherical caps)
 */
template <typename T>
struct Capsule3 {
    Vector3<T> a;     ///< First endpoint of axis
    Vector3<T> b;     ///< Second endpoint of axis
    T radius;         ///< Radius
    
    constexpr Capsule3() noexcept 
        : a(0, 0, -0.5f), b(0, 0, 0.5f), radius(1) {}
    
    constexpr Capsule3(const Vector3<T>& p1, const Vector3<T>& p2, T r) noexcept
        : a(p1), b(p2), radius(r) {}
    
    /// Signed distance from point to capsule surface
    T signedDistance(const Vector3<T>& point) const noexcept {
        LineSegm3<T> seg(a, b);
        return seg.distance(point) - radius;
    }
    
    /// Check if point is inside capsule
    bool contains(const Vector3<T>& point) const noexcept {
        return signedDistance(point) <= 0;
    }
    
    /// Height of cylinder portion
    T cylinderHeight() const noexcept {
        return (b - a).length();
    }
    
    /// Volume
    T volume() const noexcept {
        T h = cylinderHeight();
        T r2 = radius * radius;
        // Cylinder volume + 2 hemisphere volumes
        return T(3.14159265358979323846) * r2 * h + 
               T(4.0 / 3.0 * 3.14159265358979323846) * r2 * radius;
    }
};

// ==================== Type Aliases ====================

using Sphere3f = Sphere3<float>;
using Sphere3d = Sphere3<double>;

using Cylinder3f = Cylinder3<float>;
using Cylinder3d = Cylinder3<double>;

using Cone3f = Cone3<float>;
using Cone3d = Cone3<double>;

using Capsule3f = Capsule3<float>;
using Capsule3d = Capsule3<double>;

} // namespace meshlib
