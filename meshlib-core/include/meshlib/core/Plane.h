#pragma once

/**
 * @file Plane.h
 * @brief Plane representation in 3D space
 * 
 * Industrial-strength port of MeshLib's MRPlane3.h
 */

#include "meshlib/config.h"
#include "Vector.h"
#include "Line.h"
#include <cmath>
#include <optional>

namespace meshlib {

/**
 * @brief Plane in 3D space
 * 
 * Represented as n · p = d, where:
 * - n is the unit normal vector
 * - d is the signed distance from origin to plane
 * 
 * Points p on the plane satisfy: dot(n, p) = d
 */
template <typename T>
struct Plane3 {
    Vector3<T> n;  ///< Normal vector (should be normalized)
    T d = 0;       ///< Signed distance from origin
    
    /// Default constructor - XY plane
    constexpr Plane3() noexcept : n(0, 0, 1), d(0) {}
    
    /// Construct from normal and distance
    constexpr Plane3(const Vector3<T>& normal, T distance) noexcept
        : n(normal), d(distance) {}
    
    /// Construct from normal and point on plane
    Plane3(const Vector3<T>& normal, const Vector3<T>& point) noexcept
        : n(normal.normalized()), d(dot(n, point)) {}
    
    /// Construct from three points (counter-clockwise order for normal)
    static Plane3 fromPoints(const Vector3<T>& a, const Vector3<T>& b, const Vector3<T>& c) noexcept {
        Vector3<T> normal = cross(b - a, c - a).normalized();
        return Plane3(normal, dot(normal, a));
    }
    
    /// Construct from point and two direction vectors
    static Plane3 fromPointAndVectors(
        const Vector3<T>& point,
        const Vector3<T>& u,
        const Vector3<T>& v) noexcept {
        return Plane3(cross(u, v).normalized(), point);
    }
    
    /// Best-fit plane through a set of points (least squares)
    static Plane3 bestFit(const Vector3<T>* points, size_t count) noexcept;
    
    // ==================== Queries ====================
    
    /// Signed distance from point to plane (positive = same side as normal)
    constexpr T signedDistance(const Vector3<T>& point) const noexcept {
        return dot(n, point) - d;
    }
    
    /// Absolute distance from point to plane
    T distance(const Vector3<T>& point) const noexcept {
        return std::abs(signedDistance(point));
    }
    
    /// Project point onto plane
    constexpr Vector3<T> project(const Vector3<T>& point) const noexcept {
        return point - n * signedDistance(point);
    }
    
    /// Reflect point across plane
    constexpr Vector3<T> reflect(const Vector3<T>& point) const noexcept {
        return point - n * (2 * signedDistance(point));
    }
    
    /// Check which side of plane a point is on
    /// Returns: >0 = positive side (normal direction), <0 = negative side, 0 = on plane
    constexpr int side(const Vector3<T>& point, T tolerance = 0) const noexcept {
        T dist = signedDistance(point);
        if (dist > tolerance) return 1;
        if (dist < -tolerance) return -1;
        return 0;
    }
    
    /// Get a point on the plane (closest to origin)
    constexpr Vector3<T> pointOnPlane() const noexcept {
        return n * d;
    }
    
    // ==================== Intersection ====================
    
    /// Intersect plane with line, returns parameter t where line(t) hits plane
    /// Returns nullopt if line is parallel to plane
    std::optional<T> intersect(const Line3<T>& line) const noexcept {
        T denom = dot(n, line.d);
        if (std::abs(denom) < T(1e-10)) return std::nullopt;
        return (d - dot(n, line.p)) / denom;
    }
    
    /// Intersect plane with line segment, returns point if intersection exists
    std::optional<Vector3<T>> intersect(const LineSegm3<T>& seg) const noexcept {
        T d0 = signedDistance(seg.a);
        T d1 = signedDistance(seg.b);
        
        // Both on same side?
        if (d0 * d1 > 0) return std::nullopt;
        
        // One or both on plane?
        T denom = d0 - d1;
        if (std::abs(denom) < T(1e-10)) {
            return seg.a;  // Segment is on or parallel to plane
        }
        
        T t = d0 / denom;
        return seg(t);
    }
    
    /// Intersect two planes, returns line of intersection
    /// Returns nullopt if planes are parallel
    std::optional<Line3<T>> intersect(const Plane3& other) const noexcept {
        Vector3<T> dir = cross(n, other.n);
        T dirLen2 = dir.lengthSq();
        
        if (dirLen2 < T(1e-10)) return std::nullopt;  // Parallel planes
        
        // Find a point on the intersection line
        // Solve the 2x2 system for a point
        Vector3<T> point = (cross(dir, other.n) * d + cross(n, dir) * other.d) / dirLen2;
        
        return Line3<T>(point, dir);
    }
    
    /// Intersect three planes, returns point if unique
    static std::optional<Vector3<T>> intersect3(
        const Plane3& p1, const Plane3& p2, const Plane3& p3) noexcept {
        
        Vector3<T> n1xn2 = cross(p1.n, p2.n);
        T det = dot(n1xn2, p3.n);
        
        if (std::abs(det) < T(1e-10)) return std::nullopt;
        
        Vector3<T> point = (
            cross(p3.n, p2.n) * p1.d +
            cross(p1.n, p3.n) * p2.d +
            n1xn2 * p3.d
        ) / det;
        
        return point;
    }
    
    // ==================== Transformation ====================
    
    /// Flip the plane (reverse normal)
    constexpr Plane3 flipped() const noexcept {
        return Plane3(-n, -d);
    }
    
    /// Offset the plane by distance (positive = along normal)
    constexpr Plane3 offset(T amount) const noexcept {
        return Plane3(n, d + amount);
    }
    
    // ==================== Comparison ====================
    
    constexpr bool operator==(const Plane3& other) const noexcept {
        return n == other.n && d == other.d;
    }
    
    constexpr bool operator!=(const Plane3& other) const noexcept {
        return !(*this == other);
    }
    
    // ==================== Standard Planes ====================
    
    static constexpr Plane3 xy() noexcept { return Plane3(Vector3<T>(0, 0, 1), T(0)); }
    static constexpr Plane3 xz() noexcept { return Plane3(Vector3<T>(0, 1, 0), T(0)); }
    static constexpr Plane3 yz() noexcept { return Plane3(Vector3<T>(1, 0, 0), T(0)); }
};

// ==================== Type Aliases ====================

using Plane3f = Plane3<float>;
using Plane3d = Plane3<double>;

} // namespace meshlib
