#pragma once

/**
 * @file Line.h
 * @brief Line and line segment representations
 * 
 * Industrial-strength port of MeshLib's MRLine.h and MRLineSegm.h
 */

#include "meshlib/config.h"
#include "Vector.h"
#include <cmath>
#include <algorithm>

namespace meshlib {

/**
 * @brief Infinite line in 3D space
 * 
 * Represented as point + direction: L(t) = p + t * d
 */
template <typename T>
struct Line3 {
    Vector3<T> p;  ///< Point on the line
    Vector3<T> d;  ///< Direction (not necessarily normalized)
    
    /// Default constructor
    constexpr Line3() noexcept : p(), d(0, 0, 1) {}
    
    /// Construct from point and direction
    constexpr Line3(const Vector3<T>& point, const Vector3<T>& direction) noexcept
        : p(point), d(direction) {}
    
    /// Get point at parameter t
    constexpr Vector3<T> operator()(T t) const noexcept {
        return p + d * t;
    }
    
    /// Project a point onto the line, returning parameter t
    T project(const Vector3<T>& point) const noexcept {
        T denom = dot(d, d);
        if (denom < T(1e-10)) return T(0);
        return dot(point - p, d) / denom;
    }
    
    /// Get closest point on line to given point
    Vector3<T> closestPoint(const Vector3<T>& point) const noexcept {
        return (*this)(project(point));
    }
    
    /// Squared distance from point to line
    T distanceSq(const Vector3<T>& point) const noexcept {
        return (point - closestPoint(point)).lengthSq();
    }
    
    /// Distance from point to line
    T distance(const Vector3<T>& point) const noexcept {
        return std::sqrt(distanceSq(point));
    }
    
    /// Get normalized version of this line
    Line3 normalized() const noexcept {
        return Line3(p, d.normalized());
    }
    
    /// Create line from two points
    static Line3 fromPoints(const Vector3<T>& a, const Vector3<T>& b) noexcept {
        return Line3(a, b - a);
    }
};

/**
 * @brief Line segment in 3D space
 * 
 * Defined by two endpoints a and b
 */
template <typename T>
struct LineSegm3 {
    Vector3<T> a;  ///< First endpoint
    Vector3<T> b;  ///< Second endpoint
    
    /// Default constructor
    constexpr LineSegm3() noexcept = default;
    
    /// Construct from endpoints
    constexpr LineSegm3(const Vector3<T>& start, const Vector3<T>& end) noexcept
        : a(start), b(end) {}
    
    /// Get direction vector (b - a)
    constexpr Vector3<T> dir() const noexcept { return b - a; }
    
    /// Get length of segment
    T length() const noexcept { return dir().length(); }
    
    /// Get squared length
    constexpr T lengthSq() const noexcept { return dir().lengthSq(); }
    
    /// Get midpoint
    constexpr Vector3<T> midpoint() const noexcept { return (a + b) * T(0.5); }
    
    /// Get point at parameter t (t=0 -> a, t=1 -> b)
    constexpr Vector3<T> operator()(T t) const noexcept {
        return a + (b - a) * t;
    }
    
    /// Project point onto segment, returning clamped parameter t in [0,1]
    T project(const Vector3<T>& point) const noexcept {
        Vector3<T> d = b - a;
        T denom = dot(d, d);
        if (denom < T(1e-10)) return T(0);
        T t = dot(point - a, d) / denom;
        return std::clamp(t, T(0), T(1));
    }
    
    /// Get closest point on segment to given point
    Vector3<T> closestPoint(const Vector3<T>& point) const noexcept {
        return (*this)(project(point));
    }
    
    /// Squared distance from point to segment
    T distanceSq(const Vector3<T>& point) const noexcept {
        return (point - closestPoint(point)).lengthSq();
    }
    
    /// Distance from point to segment
    T distance(const Vector3<T>& point) const noexcept {
        return std::sqrt(distanceSq(point));
    }
    
    /// Convert to infinite line
    Line3<T> toLine() const noexcept {
        return Line3<T>(a, b - a);
    }
};

/**
 * @brief Infinite line in 2D space
 */
template <typename T>
struct Line2 {
    Vector2<T> p;  ///< Point on the line
    Vector2<T> d;  ///< Direction
    
    constexpr Line2() noexcept : p(), d(1, 0) {}
    constexpr Line2(const Vector2<T>& point, const Vector2<T>& direction) noexcept
        : p(point), d(direction) {}
    
    constexpr Vector2<T> operator()(T t) const noexcept {
        return p + d * t;
    }
    
    T project(const Vector2<T>& point) const noexcept {
        T denom = dot(d, d);
        if (denom < T(1e-10)) return T(0);
        return dot(point - p, d) / denom;
    }
    
    static Line2 fromPoints(const Vector2<T>& a, const Vector2<T>& b) noexcept {
        return Line2(a, b - a);
    }
};

/**
 * @brief Line segment in 2D space
 */
template <typename T>
struct LineSegm2 {
    Vector2<T> a;
    Vector2<T> b;
    
    constexpr LineSegm2() noexcept = default;
    constexpr LineSegm2(const Vector2<T>& start, const Vector2<T>& end) noexcept
        : a(start), b(end) {}
    
    constexpr Vector2<T> dir() const noexcept { return b - a; }
    T length() const noexcept { return dir().length(); }
    constexpr Vector2<T> midpoint() const noexcept { return (a + b) * T(0.5); }
    
    constexpr Vector2<T> operator()(T t) const noexcept {
        return a + (b - a) * t;
    }
};

// ==================== Type Aliases ====================

using Line3f = Line3<float>;
using Line3d = Line3<double>;
using Line2f = Line2<float>;
using Line2d = Line2<double>;

using LineSegm3f = LineSegm3<float>;
using LineSegm3d = LineSegm3<double>;
using LineSegm2f = LineSegm2<float>;
using LineSegm2d = LineSegm2<double>;

} // namespace meshlib
