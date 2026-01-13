// MeshTriPoint.h - Point location on mesh triangles using barycentric coordinates
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/Vector.h"
#include <cmath>
#include <cassert>
#include <optional>

namespace meshlib {

/// Barycentric coordinates within a triangle
/// Weights (a, b) represent the position: P = a * V0 + b * V1 + (1-a-b) * V2
/// So (1-a-b) is the weight for the third vertex
template<typename T>
struct TriPointf {
    T a{0};  ///< Weight of first vertex (V0)
    T b{0};  ///< Weight of second vertex (V1)
    
    constexpr TriPointf() noexcept = default;
    constexpr TriPointf(T a_, T b_) noexcept : a(a_), b(b_) {}
    
    /// Weight of third vertex (V2) = 1 - a - b
    [[nodiscard]] constexpr T c() const noexcept { return T(1) - a - b; }
    
    /// Get weight by index (0, 1, or 2)
    [[nodiscard]] constexpr T operator[](int i) const noexcept {
        switch(i) {
            case 0: return a;
            case 1: return b;
            case 2: return c();
            default: return T(0);
        }
    }
    
    /// Check if point is inside triangle (all weights >= 0)
    [[nodiscard]] constexpr bool isValid(T eps = T(1e-6)) const noexcept {
        return a >= -eps && b >= -eps && c() >= -eps;
    }
    
    /// Check if point is on triangle boundary (one weight is ~0)
    [[nodiscard]] constexpr bool onBoundary(T eps = T(1e-6)) const noexcept {
        return isValid(eps) && (a <= eps || b <= eps || c() <= eps);
    }
    
    /// Check if point is at a vertex (two weights are ~0)
    [[nodiscard]] constexpr bool atVertex(T eps = T(1e-6)) const noexcept {
        int zeroCount = (a <= eps ? 1 : 0) + (b <= eps ? 1 : 0) + (c() <= eps ? 1 : 0);
        return zeroCount >= 2;
    }
    
    /// Check if point is on an edge (one weight is ~0, two are positive)
    [[nodiscard]] constexpr bool onEdge(T eps = T(1e-6)) const noexcept {
        return onBoundary(eps) && !atVertex(eps);
    }
    
    /// Check if point is strictly inside triangle (all weights > eps)
    [[nodiscard]] constexpr bool strictlyInside(T eps = T(1e-6)) const noexcept {
        return a > eps && b > eps && c() > eps;
    }
    
    /// Get index of dominant weight (0, 1, or 2)
    [[nodiscard]] constexpr int dominantVertex() const noexcept {
        if (a >= b && a >= c()) return 0;
        if (b >= a && b >= c()) return 1;
        return 2;
    }
    
    /// Get edge index if on boundary (0=edge01, 1=edge12, 2=edge20), -1 otherwise
    [[nodiscard]] constexpr int edgeIndex(T eps = T(1e-6)) const noexcept {
        if (c() <= eps && a > eps && b > eps) return 0;  // c~0: on edge V0-V1
        if (a <= eps && b > eps && c() > eps) return 1;  // a~0: on edge V1-V2
        if (b <= eps && a > eps && c() > eps) return 2;  // b~0: on edge V2-V0
        return -1;
    }
    
    /// Clamp to valid barycentric range [0, 1]
    [[nodiscard]] constexpr TriPointf clamped() const noexcept {
        T na = std::max(T(0), std::min(T(1), a));
        T nb = std::max(T(0), std::min(T(1) - na, b));
        return {na, nb};
    }
    
    /// Linear interpolation between two TriPoints
    [[nodiscard]] friend constexpr TriPointf lerp(const TriPointf& p0, const TriPointf& p1, T t) noexcept {
        return {p0.a + t * (p1.a - p0.a), p0.b + t * (p1.b - p0.b)};
    }
    
    /// Create from three weights (will normalize if needed)
    [[nodiscard]] static constexpr TriPointf fromWeights(T w0, T w1, T w2) noexcept {
        T sum = w0 + w1 + w2;
        if (sum == T(0)) return {T(1)/T(3), T(1)/T(3)};
        return {w0 / sum, w1 / sum};
    }
    
    /// At first vertex (1, 0, 0)
    [[nodiscard]] static constexpr TriPointf vertex0() noexcept { return {T(1), T(0)}; }
    /// At second vertex (0, 1, 0)  
    [[nodiscard]] static constexpr TriPointf vertex1() noexcept { return {T(0), T(1)}; }
    /// At third vertex (0, 0, 1)
    [[nodiscard]] static constexpr TriPointf vertex2() noexcept { return {T(0), T(0)}; }
    /// At center of triangle (1/3, 1/3, 1/3)
    [[nodiscard]] static constexpr TriPointf centroid() noexcept { 
        return {T(1)/T(3), T(1)/T(3)}; 
    }
    
    /// Middle of edge 01
    [[nodiscard]] static constexpr TriPointf midEdge01() noexcept { return {T(0.5), T(0.5)}; }
    /// Middle of edge 12
    [[nodiscard]] static constexpr TriPointf midEdge12() noexcept { return {T(0), T(0.5)}; }
    /// Middle of edge 20
    [[nodiscard]] static constexpr TriPointf midEdge20() noexcept { return {T(0.5), T(0)}; }
    
    bool operator==(const TriPointf& other) const noexcept = default;
};

using TriPointf32 = TriPointf<float>;
using TriPointf64 = TriPointf<double>;

/// A point on a specific face of a mesh, identified by face ID and barycentric coordinates
template<typename T>
struct MeshTriPoint {
    FaceId face;           ///< Face this point belongs to
    TriPointf<T> bary;     ///< Barycentric coordinates within the face
    
    constexpr MeshTriPoint() noexcept = default;
    constexpr MeshTriPoint(FaceId f, const TriPointf<T>& b) noexcept 
        : face(f), bary(b) {}
    constexpr MeshTriPoint(FaceId f, T a, T b) noexcept 
        : face(f), bary(a, b) {}
    
    /// Check if this is a valid point (has a valid face)
    [[nodiscard]] constexpr bool valid() const noexcept { return face.valid(); }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return valid(); }
    
    /// Interpolate a vertex attribute using barycentric coordinates
    /// T0, T1, T2 are the attribute values at the three vertices
    template<typename V>
    [[nodiscard]] constexpr V interpolate(const V& v0, const V& v1, const V& v2) const noexcept {
        return v0 * bary.a + v1 * bary.b + v2 * bary.c();
    }
    
    /// Interpolate 3D position from triangle vertices
    [[nodiscard]] Vector3<T> interpolate(
        const Vector3<T>& p0, 
        const Vector3<T>& p1, 
        const Vector3<T>& p2
    ) const noexcept {
        return p0 * bary.a + p1 * bary.b + p2 * bary.c();
    }
    
    /// Static creation helpers
    [[nodiscard]] static constexpr MeshTriPoint atVertex(FaceId f, int vertIndex) noexcept {
        switch(vertIndex) {
            case 0: return {f, TriPointf<T>::vertex0()};
            case 1: return {f, TriPointf<T>::vertex1()};
            case 2: return {f, TriPointf<T>::vertex2()};
            default: return {};
        }
    }
    
    [[nodiscard]] static constexpr MeshTriPoint atCentroid(FaceId f) noexcept {
        return {f, TriPointf<T>::centroid()};
    }
    
    bool operator==(const MeshTriPoint& other) const noexcept = default;
};

using MeshTriPointf = MeshTriPoint<float>;
using MeshTriPointd = MeshTriPoint<double>;

/// Compute barycentric coordinates for a point in a triangle
/// Returns empty optional if triangle is degenerate
template<typename T>
[[nodiscard]] std::optional<TriPointf<T>> computeBarycentric(
    const Vector3<T>& p,
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2,
    T degenerateThreshold = T(1e-12)
) noexcept {
    // Edge vectors from v0
    Vector3<T> e1 = v1 - v0;
    Vector3<T> e2 = v2 - v0;
    Vector3<T> pv = p - v0;
    
    // Compute dot products
    T d11 = dot(e1, e1);
    T d12 = dot(e1, e2);
    T d22 = dot(e2, e2);
    T d1p = dot(e1, pv);
    T d2p = dot(e2, pv);
    
    // Compute barycentric coordinates using Cramer's rule
    T denom = d11 * d22 - d12 * d12;
    if (std::abs(denom) < degenerateThreshold) {
        return std::nullopt;  // Degenerate triangle
    }
    
    T invDenom = T(1) / denom;
    T b = (d22 * d1p - d12 * d2p) * invDenom;  // Weight for v1
    T c = (d11 * d2p - d12 * d1p) * invDenom;  // Weight for v2
    T a = T(1) - b - c;  // Weight for v0
    
    return TriPointf<T>{a, b};
}

/// Compute barycentric coordinates for a 2D point in a 2D triangle
template<typename T>
[[nodiscard]] std::optional<TriPointf<T>> computeBarycentric2D(
    const Vector2<T>& p,
    const Vector2<T>& v0,
    const Vector2<T>& v1,
    const Vector2<T>& v2,
    T degenerateThreshold = T(1e-12)
) noexcept {
    // Signed area approach
    T denom = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
    if (std::abs(denom) < degenerateThreshold) {
        return std::nullopt;
    }
    
    T a = ((v1.y - v2.y) * (p.x - v2.x) + (v2.x - v1.x) * (p.y - v2.y)) / denom;
    T b = ((v2.y - v0.y) * (p.x - v2.x) + (v0.x - v2.x) * (p.y - v2.y)) / denom;
    
    return TriPointf<T>{a, b};
}

/// Project a point onto a triangle and compute barycentric coordinates
/// Returns the closest point on the triangle (clamped to edges/vertices if necessary)
template<typename T>
[[nodiscard]] MeshTriPoint<T> projectOntoTriangle(
    const Vector3<T>& p,
    FaceId face,
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) noexcept {
    // First compute unclamped barycentric coordinates
    auto baryOpt = computeBarycentric(p, v0, v1, v2);
    if (!baryOpt) {
        // Degenerate triangle - return centroid as fallback
        return MeshTriPoint<T>::atCentroid(face);
    }
    
    TriPointf<T> bary = *baryOpt;
    
    // If inside triangle, return as-is
    if (bary.isValid()) {
        return {face, bary};
    }
    
    // Otherwise, project onto closest edge or vertex
    // This involves clamping and re-projecting
    auto clamp01 = [](T x) { return std::max(T(0), std::min(T(1), x)); };
    
    // Check each edge and find closest point
    T bestDistSq = std::numeric_limits<T>::max();
    TriPointf<T> bestBary;
    
    // Edge 0-1 (c = 0)
    {
        T t = clamp01(bary.a / (bary.a + bary.b + T(1e-10)));
        TriPointf<T> edgeBary{t, T(1) - t};
        Vector3<T> pt = v0 * edgeBary.a + v1 * edgeBary.b;
        T distSq = (p - pt).lengthSq();
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestBary = edgeBary;
        }
    }
    
    // Edge 1-2 (a = 0)
    {
        T t = clamp01(bary.b / (bary.b + bary.c() + T(1e-10)));
        TriPointf<T> edgeBary{T(0), t};
        Vector3<T> pt = v1 * edgeBary.b + v2 * edgeBary.c();
        T distSq = (p - pt).lengthSq();
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestBary = edgeBary;
        }
    }
    
    // Edge 2-0 (b = 0)
    {
        T t = clamp01(bary.a / (bary.a + bary.c() + T(1e-10)));
        TriPointf<T> edgeBary{t, T(0)};
        Vector3<T> pt = v0 * edgeBary.a + v2 * edgeBary.c();
        T distSq = (p - pt).lengthSq();
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestBary = edgeBary;
        }
    }
    
    return {face, bestBary};
}

/// Compute distance from point to triangle
template<typename T>
[[nodiscard]] T distanceToTriangle(
    const Vector3<T>& p,
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) noexcept {
    auto proj = projectOntoTriangle(p, FaceId{0}, v0, v1, v2);
    Vector3<T> closest = proj.interpolate(v0, v1, v2);
    return (p - closest).length();
}

/// Compute squared distance from point to triangle (faster, no sqrt)
template<typename T>
[[nodiscard]] T distanceSquaredToTriangle(
    const Vector3<T>& p,
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) noexcept {
    auto proj = projectOntoTriangle(p, FaceId{0}, v0, v1, v2);
    Vector3<T> closest = proj.interpolate(v0, v1, v2);
    return (p - closest).lengthSq();
}

} // namespace meshlib
