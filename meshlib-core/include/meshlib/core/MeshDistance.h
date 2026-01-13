// MeshDistance.h - Distance queries for meshes
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/Vector.h"
#include "meshlib/core/Box.h"
#include "meshlib/core/AABBTree.h"
#include "meshlib/core/MeshTriPoint.h"
#include "meshlib/core/MeshProject.h"
#include "meshlib/core/MeshNormals.h"
#include <vector>
#include <cmath>
#include <optional>
#include <functional>

namespace meshlib {

// Forward declarations
class MeshTopology;

/// Result of a signed distance query
template<typename T>
struct SignedDistanceResult {
    T distance{std::numeric_limits<T>::max()};  ///< Signed distance (+ outside, - inside)
    MeshTriPoint<T> closestPoint;                ///< Closest point on mesh
    Vector3<T> closestPosition;                  ///< 3D position of closest point
    
    [[nodiscard]] bool valid() const noexcept { return closestPoint.valid(); }
    [[nodiscard]] explicit operator bool() const noexcept { return valid(); }
    
    /// Get unsigned distance
    [[nodiscard]] T unsignedDistance() const noexcept { return std::abs(distance); }
    
    /// Check if query point is inside the mesh
    [[nodiscard]] bool isInside() const noexcept { return distance < T(0); }
    
    /// Check if query point is outside the mesh  
    [[nodiscard]] bool isOutside() const noexcept { return distance > T(0); }
};

using SignedDistanceResultf = SignedDistanceResult<float>;
using SignedDistanceResultd = SignedDistanceResult<double>;

/// Method for determining inside/outside (sign of distance)
enum class SignDeterminationMethod {
    PseudoNormal,    ///< Use face normal at closest point (fast, less accurate at edges)
    WindingNumber,   ///< Use generalized winding number (slower, more robust)
    RayCasting       ///< Count ray intersections (moderate speed, robust for watertight meshes)
};

/// Settings for signed distance computation
template<typename T>
struct SignedDistanceSettings {
    /// Method for determining sign
    SignDeterminationMethod signMethod{SignDeterminationMethod::PseudoNormal};
    
    /// Maximum distance to search (for performance)
    T maxDistance{std::numeric_limits<T>::max()};
    
    /// Face mask (only consider these faces)
    const FaceBitSet* faceMask{nullptr};
    
    /// Pre-computed face normals (for PseudoNormal method)
    const FaceMap<Vector3<T>>* faceNormals{nullptr};
    
    /// Pre-computed vertex normals (for smooth interpolation)
    const VertMap<Vector3<T>>* vertexNormals{nullptr};
    
    /// AABB tree for acceleration
    const AABBTree<T>* tree{nullptr};
};

using SignedDistanceSettingsf = SignedDistanceSettings<float>;
using SignedDistanceSettingsd = SignedDistanceSettings<double>;

/// Compute unsigned distance from point to mesh
template<typename T>
[[nodiscard]] T unsignedDistanceToMesh(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree
) {
    auto proj = projectPointToMesh(query, points, topology, tree);
    return proj.valid() ? proj.distance() : std::numeric_limits<T>::max();
}

/// Compute signed distance using pseudo-normal method
/// Uses face normal at closest point to determine sign
template<typename T>
[[nodiscard]] SignedDistanceResult<T> signedDistancePseudoNormal(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree,
    const FaceMap<Vector3<T>>& faceNormals
) {
    SignedDistanceResult<T> result;
    
    auto proj = projectPointToMesh(query, points, topology, tree);
    if (!proj.valid()) {
        return result;
    }
    
    result.closestPoint = proj.triPoint;
    result.closestPosition = proj.point;
    
    // Get normal at closest point
    Vector3<T> normal = faceNormals[proj.triPoint.face];
    
    // If closest point is on an edge or vertex, average adjacent normals
    if (proj.triPoint.bary.onBoundary()) {
        // Simplified - just use face normal for now
        // A more sophisticated approach would interpolate or average
    }
    
    // Determine sign: positive if query is on the same side as normal
    Vector3<T> toQuery = query - result.closestPosition;
    T sign = dot(toQuery, normal) >= T(0) ? T(1) : T(-1);
    
    result.distance = sign * proj.distance();
    return result;
}

/// Compute generalized winding number at a point
/// Returns ~1 if inside, ~0 if outside, intermediate values near surface
template<typename T>
[[nodiscard]] T computeWindingNumber(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    T windingNumber{0};
    const T fourPi = T(4) * T(3.14159265358979323846);
    
    size_t numFaces = topology.faceCount();
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        if (!verts[0].valid()) continue;
        
        // Vectors from query to triangle vertices
        Vector3<T> a = points[verts[0].get()] - query;
        Vector3<T> b = points[verts[1].get()] - query;
        Vector3<T> c = points[verts[2].get()] - query;
        
        T la = a.length();
        T lb = b.length();
        T lc = c.length();
        
        // Avoid division by zero
        if (la < T(1e-10) || lb < T(1e-10) || lc < T(1e-10)) {
            // Query is very close to vertex - likely inside
            return T(0.5);
        }
        
        // Solid angle of triangle as seen from query point
        T numerator = dot(a, cross(b, c));
        T denominator = la * lb * lc + 
                        dot(a, b) * lc +
                        dot(b, c) * la +
                        dot(c, a) * lb;
        
        T solidAngle = T(2) * std::atan2(numerator, denominator);
        windingNumber += solidAngle;
    }
    
    return windingNumber / fourPi;
}

/// Compute signed distance using winding number method
template<typename T>
[[nodiscard]] SignedDistanceResult<T> signedDistanceWindingNumber(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree,
    T windingThreshold = T(0.5)
) {
    SignedDistanceResult<T> result;
    
    auto proj = projectPointToMesh(query, points, topology, tree);
    if (!proj.valid()) {
        return result;
    }
    
    result.closestPoint = proj.triPoint;
    result.closestPosition = proj.point;
    
    // Compute winding number to determine sign
    T winding = computeWindingNumber(query, points, topology);
    T sign = winding > windingThreshold ? T(-1) : T(1);
    
    result.distance = sign * proj.distance();
    return result;
}

/// Ray-triangle intersection test for sign determination
template<typename T>
[[nodiscard]] bool rayIntersectsTriangle(
    const Vector3<T>& origin,
    const Vector3<T>& direction,
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2,
    T& outT
) {
    const T eps = T(1e-10);
    
    Vector3<T> e1 = v1 - v0;
    Vector3<T> e2 = v2 - v0;
    Vector3<T> h = cross(direction, e2);
    T a = dot(e1, h);
    
    if (std::abs(a) < eps) {
        return false;  // Ray parallel to triangle
    }
    
    T f = T(1) / a;
    Vector3<T> s = origin - v0;
    T u = f * dot(s, h);
    
    if (u < T(0) || u > T(1)) {
        return false;
    }
    
    Vector3<T> q = cross(s, e1);
    T v = f * dot(direction, q);
    
    if (v < T(0) || u + v > T(1)) {
        return false;
    }
    
    outT = f * dot(e2, q);
    return outT > eps;  // Intersection in front of ray
}

/// Count ray intersections with mesh (for inside/outside test)
template<typename T>
[[nodiscard]] int countRayIntersections(
    const Vector3<T>& origin,
    const Vector3<T>& direction,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    int count = 0;
    
    size_t numFaces = topology.faceCount();
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        if (!verts[0].valid()) continue;
        
        T t;
        if (rayIntersectsTriangle(origin, direction,
                points[verts[0].get()],
                points[verts[1].get()],
                points[verts[2].get()], t)) {
            count++;
        }
    }
    
    return count;
}

/// Compute signed distance using ray casting method
template<typename T>
[[nodiscard]] SignedDistanceResult<T> signedDistanceRayCasting(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree
) {
    SignedDistanceResult<T> result;
    
    auto proj = projectPointToMesh(query, points, topology, tree);
    if (!proj.valid()) {
        return result;
    }
    
    result.closestPoint = proj.triPoint;
    result.closestPosition = proj.point;
    
    // Cast ray in arbitrary direction (using +X)
    Vector3<T> rayDir{T(1), T(0), T(0)};
    int intersections = countRayIntersections(query, rayDir, points, topology);
    
    // Odd number of intersections = inside
    T sign = (intersections % 2 == 1) ? T(-1) : T(1);
    
    result.distance = sign * proj.distance();
    return result;
}

/// Main signed distance function with configurable method
template<typename T>
[[nodiscard]] SignedDistanceResult<T> computeSignedDistance(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const SignedDistanceSettings<T>& settings = {}
) {
    // Build AABB tree if not provided
    AABBTree<T> localTree;
    const AABBTree<T>* tree = settings.tree;
    if (!tree) {
        localTree = AABBTree<T>::build(points, topology);
        tree = &localTree;
    }
    
    switch (settings.signMethod) {
        case SignDeterminationMethod::PseudoNormal: {
            // Compute face normals if not provided
            FaceMap<Vector3<T>> localNormals;
            const FaceMap<Vector3<T>>* normals = settings.faceNormals;
            if (!normals) {
                localNormals = computeFaceNormals(points, topology);
                normals = &localNormals;
            }
            return signedDistancePseudoNormal(query, points, topology, *tree, *normals);
        }
        
        case SignDeterminationMethod::WindingNumber:
            return signedDistanceWindingNumber(query, points, topology, *tree);
        
        case SignDeterminationMethod::RayCasting:
            return signedDistanceRayCasting(query, points, topology, *tree);
    }
    
    return {};
}

/// Batch compute signed distances for multiple query points
template<typename T>
[[nodiscard]] std::vector<SignedDistanceResult<T>> computeSignedDistances(
    const std::vector<Vector3<T>>& queries,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const SignedDistanceSettings<T>& settings = {}
) {
    // Build shared resources
    AABBTree<T> tree = settings.tree ? *settings.tree : AABBTree<T>::build(points, topology);
    FaceMap<Vector3<T>> faceNormals;
    if (settings.signMethod == SignDeterminationMethod::PseudoNormal && !settings.faceNormals) {
        faceNormals = computeFaceNormals(points, topology);
    }
    
    SignedDistanceSettings<T> localSettings = settings;
    localSettings.tree = &tree;
    if (settings.signMethod == SignDeterminationMethod::PseudoNormal && !settings.faceNormals) {
        localSettings.faceNormals = &faceNormals;
    }
    
    std::vector<SignedDistanceResult<T>> results;
    results.reserve(queries.size());
    
    for (const auto& query : queries) {
        results.push_back(computeSignedDistance(query, points, topology, localSettings));
    }
    
    return results;
}

/// Check if a point is inside a closed mesh
template<typename T>
[[nodiscard]] bool isPointInsideMesh(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    SignDeterminationMethod method = SignDeterminationMethod::RayCasting
) {
    SignedDistanceSettings<T> settings;
    settings.signMethod = method;
    auto result = computeSignedDistance(query, points, topology, settings);
    return result.isInside();
}

/// Compute Hausdorff distance between two meshes
/// Returns the maximum of minimum distances from points of mesh1 to mesh2
template<typename T>
[[nodiscard]] T computeHausdorffDistance(
    const std::vector<Vector3<T>>& points1,
    const MeshTopology& topology1,
    const std::vector<Vector3<T>>& points2,
    const MeshTopology& topology2
) {
    // Build AABB tree for mesh2
    AABBTree<T> tree2 = AABBTree<T>::build(points2, topology2);
    
    T maxDist{0};
    
    // Check all vertices of mesh1
    for (const auto& p : points1) {
        auto proj = projectPointToMesh(p, points2, topology2, tree2);
        if (proj.valid()) {
            maxDist = std::max(maxDist, proj.distance());
        }
    }
    
    return maxDist;
}

/// Compute symmetric Hausdorff distance (max of both directions)
template<typename T>
[[nodiscard]] T computeSymmetricHausdorffDistance(
    const std::vector<Vector3<T>>& points1,
    const MeshTopology& topology1,
    const std::vector<Vector3<T>>& points2,
    const MeshTopology& topology2
) {
    T d12 = computeHausdorffDistance(points1, topology1, points2, topology2);
    T d21 = computeHausdorffDistance(points2, topology2, points1, topology1);
    return std::max(d12, d21);
}

/// Compute mean distance between mesh vertices and another mesh
template<typename T>
[[nodiscard]] T computeMeanDistance(
    const std::vector<Vector3<T>>& queryPoints,
    const std::vector<Vector3<T>>& meshPoints,
    const MeshTopology& meshTopology,
    const AABBTree<T>& tree
) {
    T totalDist{0};
    int count{0};
    
    for (const auto& p : queryPoints) {
        auto proj = projectPointToMesh(p, meshPoints, meshTopology, tree);
        if (proj.valid()) {
            totalDist += proj.distance();
            count++;
        }
    }
    
    return count > 0 ? totalDist / T(count) : T(0);
}

/// Compute RMS (root mean square) distance
template<typename T>
[[nodiscard]] T computeRMSDistance(
    const std::vector<Vector3<T>>& queryPoints,
    const std::vector<Vector3<T>>& meshPoints,
    const MeshTopology& meshTopology,
    const AABBTree<T>& tree
) {
    T sumSqDist{0};
    int count{0};
    
    for (const auto& p : queryPoints) {
        auto proj = projectPointToMesh(p, meshPoints, meshTopology, tree);
        if (proj.valid()) {
            sumSqDist += proj.distanceSquared;
            count++;
        }
    }
    
    return count > 0 ? std::sqrt(sumSqDist / T(count)) : T(0);
}

/// Find all points within a given distance from mesh surface
template<typename T>
[[nodiscard]] std::vector<size_t> findPointsNearMesh(
    const std::vector<Vector3<T>>& queryPoints,
    const std::vector<Vector3<T>>& meshPoints,
    const MeshTopology& meshTopology,
    const AABBTree<T>& tree,
    T maxDistance
) {
    std::vector<size_t> nearPoints;
    T maxDistSq = maxDistance * maxDistance;
    
    for (size_t i = 0; i < queryPoints.size(); ++i) {
        ProjectionSettings<T> settings;
        settings.maxDistance = maxDistance;
        auto proj = projectPointToMesh(queryPoints[i], meshPoints, meshTopology, tree, settings);
        if (proj.valid() && proj.distanceSquared <= maxDistSq) {
            nearPoints.push_back(i);
        }
    }
    
    return nearPoints;
}

} // namespace meshlib
