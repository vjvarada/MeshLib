// MeshEdgePoint.h - Point location on mesh edges with parametric coordinates
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/Vector.h"
#include "meshlib/core/MeshTriPoint.h"
#include <cmath>
#include <cassert>
#include <optional>

namespace meshlib {

/// A point on an edge, represented by the edge ID and a parametric value t in [0, 1]
/// When t = 0, point is at org(edge); when t = 1, point is at dest(edge)
template<typename T>
struct MeshEdgePoint {
    EdgeId edge;    ///< The edge this point lies on (half-edge)
    T t{0};         ///< Parametric position: 0 = origin, 1 = destination
    
    constexpr MeshEdgePoint() noexcept = default;
    constexpr MeshEdgePoint(EdgeId e, T t_) noexcept : edge(e), t(t_) {}
    
    /// Check if this is a valid point (has a valid edge)
    [[nodiscard]] constexpr bool valid() const noexcept { return edge.valid(); }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return valid(); }
    
    /// Check if point is at the origin vertex (t ~ 0)
    [[nodiscard]] constexpr bool atOrg(T eps = T(1e-6)) const noexcept {
        return t <= eps;
    }
    
    /// Check if point is at the destination vertex (t ~ 1)
    [[nodiscard]] constexpr bool atDest(T eps = T(1e-6)) const noexcept {
        return t >= T(1) - eps;
    }
    
    /// Check if point is at either endpoint
    [[nodiscard]] constexpr bool atVertex(T eps = T(1e-6)) const noexcept {
        return atOrg(eps) || atDest(eps);
    }
    
    /// Check if point is strictly in the interior of the edge
    [[nodiscard]] constexpr bool inInterior(T eps = T(1e-6)) const noexcept {
        return t > eps && t < T(1) - eps;
    }
    
    /// Get the same point represented on the symmetric (opposite) half-edge
    [[nodiscard]] constexpr MeshEdgePoint symmetric() const noexcept {
        return {edge.sym(), T(1) - t};
    }
    
    /// Clamp t to valid range [0, 1]
    [[nodiscard]] constexpr MeshEdgePoint clamped() const noexcept {
        return {edge, std::max(T(0), std::min(T(1), t))};
    }
    
    /// Interpolate position from edge endpoints
    [[nodiscard]] Vector3<T> interpolate(
        const Vector3<T>& pOrg,
        const Vector3<T>& pDest
    ) const noexcept {
        return pOrg * (T(1) - t) + pDest * t;
    }
    
    /// Interpolate any attribute from edge endpoints
    template<typename V>
    [[nodiscard]] V interpolate(const V& vOrg, const V& vDest) const noexcept {
        return vOrg * (T(1) - t) + vDest * t;
    }
    
    /// Create an edge point at the origin
    [[nodiscard]] static constexpr MeshEdgePoint atOrigin(EdgeId e) noexcept {
        return {e, T(0)};
    }
    
    /// Create an edge point at the destination
    [[nodiscard]] static constexpr MeshEdgePoint atDestination(EdgeId e) noexcept {
        return {e, T(1)};
    }
    
    /// Create an edge point at the middle
    [[nodiscard]] static constexpr MeshEdgePoint atMidpoint(EdgeId e) noexcept {
        return {e, T(0.5)};
    }
    
    bool operator==(const MeshEdgePoint& other) const noexcept = default;
    
    /// Compare two edge points on the same undirected edge
    /// Returns true if this point comes before other along the edge direction
    [[nodiscard]] bool operator<(const MeshEdgePoint& other) const noexcept {
        if (edge == other.edge) return t < other.t;
        if (edge == other.edge.sym()) return t < (T(1) - other.t);
        return edge < other.edge;
    }
};

using MeshEdgePointf = MeshEdgePoint<float>;
using MeshEdgePointd = MeshEdgePoint<double>;

/// Project a point onto an edge segment
template<typename T>
[[nodiscard]] MeshEdgePoint<T> projectOntoEdge(
    const Vector3<T>& p,
    EdgeId edge,
    const Vector3<T>& pOrg,
    const Vector3<T>& pDest
) noexcept {
    Vector3<T> edgeVec = pDest - pOrg;
    T edgeLenSq = edgeVec.lengthSq();
    
    if (edgeLenSq < T(1e-12)) {
        // Degenerate edge - return midpoint
        return MeshEdgePoint<T>::atMidpoint(edge);
    }
    
    // Project point onto the infinite line
    T t = dot(p - pOrg, edgeVec) / edgeLenSq;
    
    // Clamp to [0, 1]
    t = std::max(T(0), std::min(T(1), t));
    
    return {edge, t};
}

/// Compute distance from point to edge segment
template<typename T>
[[nodiscard]] T distanceToEdge(
    const Vector3<T>& p,
    const Vector3<T>& pOrg,
    const Vector3<T>& pDest
) noexcept {
    auto proj = projectOntoEdge(p, EdgeId{0}, pOrg, pDest);
    Vector3<T> closest = proj.interpolate(pOrg, pDest);
    return (p - closest).length();
}

/// Compute squared distance from point to edge segment (faster, no sqrt)
template<typename T>
[[nodiscard]] T distanceSquaredToEdge(
    const Vector3<T>& p,
    const Vector3<T>& pOrg,
    const Vector3<T>& pDest
) noexcept {
    auto proj = projectOntoEdge(p, EdgeId{0}, pOrg, pDest);
    Vector3<T> closest = proj.interpolate(pOrg, pDest);
    return (p - closest).lengthSq();
}

/// Compute the closest pair of points between two edge segments
/// Returns (t1, t2) parameters for the closest points on edge1 and edge2
template<typename T>
[[nodiscard]] std::pair<T, T> closestPointsOnEdges(
    const Vector3<T>& a0, const Vector3<T>& a1,  // Edge 1: a0 to a1
    const Vector3<T>& b0, const Vector3<T>& b1,  // Edge 2: b0 to b1
    T eps = T(1e-12)
) noexcept {
    Vector3<T> d1 = a1 - a0;  // Direction of edge 1
    Vector3<T> d2 = b1 - b0;  // Direction of edge 2
    Vector3<T> r = a0 - b0;
    
    T a = dot(d1, d1);  // |d1|^2
    T e = dot(d2, d2);  // |d2|^2
    T f = dot(d2, r);
    
    T s, t;
    
    // Check if both segments degenerate to points
    if (a <= eps && e <= eps) {
        return {T(0), T(0)};
    }
    
    if (a <= eps) {
        // Segment 1 degenerates to a point
        s = T(0);
        t = std::max(T(0), std::min(T(1), f / e));
    } else {
        T c = dot(d1, r);
        if (e <= eps) {
            // Segment 2 degenerates to a point
            t = T(0);
            s = std::max(T(0), std::min(T(1), -c / a));
        } else {
            // General non-degenerate case
            T b = dot(d1, d2);
            T denom = a * e - b * b;
            
            if (denom != T(0)) {
                s = std::max(T(0), std::min(T(1), (b * f - c * e) / denom));
            } else {
                s = T(0);  // Parallel segments, pick arbitrary point
            }
            
            // Compute closest point on segment 2 to the closest point on segment 1
            t = (b * s + f) / e;
            
            // If t outside [0,1], clamp and recompute s
            if (t < T(0)) {
                t = T(0);
                s = std::max(T(0), std::min(T(1), -c / a));
            } else if (t > T(1)) {
                t = T(1);
                s = std::max(T(0), std::min(T(1), (b - c) / a));
            }
        }
    }
    
    return {s, t};
}

/// Compute distance between two edge segments
template<typename T>
[[nodiscard]] T distanceBetweenEdges(
    const Vector3<T>& a0, const Vector3<T>& a1,
    const Vector3<T>& b0, const Vector3<T>& b1
) noexcept {
    auto [s, t] = closestPointsOnEdges(a0, a1, b0, b1);
    Vector3<T> c1 = a0 + (a1 - a0) * s;
    Vector3<T> c2 = b0 + (b1 - b0) * t;
    return (c1 - c2).length();
}

/// Result of an edge-edge intersection test
template<typename T>
struct EdgeIntersection {
    bool intersects{false};  ///< Whether edges intersect
    T t1{0};                 ///< Parameter on first edge
    T t2{0};                 ///< Parameter on second edge
    Vector3<T> point;        ///< Intersection point (if intersects)
    
    [[nodiscard]] explicit operator bool() const noexcept { return intersects; }
};

/// Test if two 2D line segments intersect
template<typename T>
[[nodiscard]] std::optional<std::pair<T, T>> intersectEdges2D(
    const Vector2<T>& a0, const Vector2<T>& a1,
    const Vector2<T>& b0, const Vector2<T>& b1,
    T eps = T(1e-10)
) noexcept {
    Vector2<T> d1 = a1 - a0;
    Vector2<T> d2 = b1 - b0;
    
    // Cross product of directions (2D "pseudo-determinant")
    T cross = d1.x * d2.y - d1.y * d2.x;
    
    if (std::abs(cross) < eps) {
        // Parallel or coincident
        return std::nullopt;
    }
    
    Vector2<T> r = b0 - a0;
    T t = (r.x * d2.y - r.y * d2.x) / cross;
    T s = (r.x * d1.y - r.y * d1.x) / cross;
    
    // Check if intersection is within both segments
    if (t >= -eps && t <= T(1) + eps && s >= -eps && s <= T(1) + eps) {
        return std::make_pair(
            std::max(T(0), std::min(T(1), t)),
            std::max(T(0), std::min(T(1), s))
        );
    }
    
    return std::nullopt;
}

/// Represent an edge by its two half-edges
struct UndirectedEdgeId {
    EdgeId e;  ///< One of the two half-edges (the one with smaller value)
    
    constexpr UndirectedEdgeId() noexcept = default;
    constexpr explicit UndirectedEdgeId(EdgeId edge) noexcept 
        : e(edge.undirected()) {}
    
    [[nodiscard]] constexpr bool valid() const noexcept { return e.valid(); }
    [[nodiscard]] constexpr EdgeId directed() const noexcept { return e; }
    [[nodiscard]] constexpr EdgeId sym() const noexcept { return e.sym(); }
    
    bool operator==(const UndirectedEdgeId& other) const noexcept = default;
    bool operator<(const UndirectedEdgeId& other) const noexcept { return e < other.e; }
};

/// Point on an undirected edge (works with either half-edge)
template<typename T>
struct UndirectedEdgePoint {
    UndirectedEdgeId edge;
    T t{0};  ///< Parameter from edge.directed() origin
    
    constexpr UndirectedEdgePoint() noexcept = default;
    constexpr UndirectedEdgePoint(UndirectedEdgeId e, T t_) noexcept : edge(e), t(t_) {}
    constexpr explicit UndirectedEdgePoint(const MeshEdgePoint<T>& ep) noexcept
        : edge(UndirectedEdgeId(ep.edge))
        , t(ep.edge == edge.directed() ? ep.t : T(1) - ep.t) {}
    
    [[nodiscard]] constexpr bool valid() const noexcept { return edge.valid(); }
    
    /// Convert to directed edge point using the canonical direction
    [[nodiscard]] constexpr MeshEdgePoint<T> toDirected() const noexcept {
        return {edge.directed(), t};
    }
    
    /// Convert to directed edge point using a specific half-edge
    [[nodiscard]] constexpr MeshEdgePoint<T> toDirected(EdgeId e) const noexcept {
        if (e == edge.directed()) {
            return {e, t};
        } else if (e == edge.sym()) {
            return {e, T(1) - t};
        }
        return {};  // Invalid - edge doesn't match
    }
    
    bool operator==(const UndirectedEdgePoint& other) const noexcept = default;
};

using UndirectedEdgePointf = UndirectedEdgePoint<float>;
using UndirectedEdgePointd = UndirectedEdgePoint<double>;

} // namespace meshlib
