// MeshProject.h - Point-to-mesh projection algorithms
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/Vector.h"
#include "meshlib/core/MeshTriPoint.h"
#include "meshlib/core/MeshEdgePoint.h"
#include "meshlib/core/Box.h"
#include "meshlib/core/AABBTree.h"
#include <vector>
#include <limits>
#include <optional>
#include <functional>

namespace meshlib {

// Forward declarations
class MeshTopology;

/// Result of projecting a point onto a mesh
template<typename T>
struct MeshProjectionResult {
    MeshTriPoint<T> triPoint;     ///< Location on the mesh surface
    Vector3<T> point;              ///< 3D coordinates of projected point
    T distanceSquared{std::numeric_limits<T>::max()};  ///< Squared distance from query to projection
    
    [[nodiscard]] bool valid() const noexcept { return triPoint.valid(); }
    [[nodiscard]] explicit operator bool() const noexcept { return valid(); }
    
    [[nodiscard]] T distance() const noexcept { return std::sqrt(distanceSquared); }
    
    /// Compare by distance (for priority queue)
    bool operator<(const MeshProjectionResult& other) const noexcept {
        return distanceSquared < other.distanceSquared;
    }
    bool operator>(const MeshProjectionResult& other) const noexcept {
        return distanceSquared > other.distanceSquared;
    }
};

using MeshProjectionResultf = MeshProjectionResult<float>;
using MeshProjectionResultd = MeshProjectionResult<double>;

/// Result of projecting a point onto a mesh edge
template<typename T>
struct EdgeProjectionResult {
    MeshEdgePoint<T> edgePoint;
    Vector3<T> point;
    T distanceSquared{std::numeric_limits<T>::max()};
    
    [[nodiscard]] bool valid() const noexcept { return edgePoint.valid(); }
    [[nodiscard]] explicit operator bool() const noexcept { return valid(); }
    [[nodiscard]] T distance() const noexcept { return std::sqrt(distanceSquared); }
};

using EdgeProjectionResultf = EdgeProjectionResult<float>;
using EdgeProjectionResultd = EdgeProjectionResult<double>;

/// Settings for mesh projection
template<typename T>
struct ProjectionSettings {
    /// Maximum distance to search (optimization)
    T maxDistance{std::numeric_limits<T>::max()};
    
    /// Minimum distance threshold (skip closer projections)
    T minDistance{0};
    
    /// Face mask - only project onto faces in this set (optional)
    const FaceBitSet* faceMask{nullptr};
    
    /// Whether to project to boundary edges only
    bool boundaryOnly{false};
    
    /// Pre-computed AABB tree (optional, avoids rebuilding)
    const AABBTree<T>* tree{nullptr};
};

using ProjectionSettingsf = ProjectionSettings<float>;
using ProjectionSettingsd = ProjectionSettings<double>;

/// Callback for finding faces near a point
using FaceCallback = std::function<bool(FaceId)>;  // Return false to stop iteration

/// Project a point onto a single triangle
/// @param query The point to project
/// @param face The face ID
/// @param v0, v1, v2 The triangle vertices
/// @return Projection result with closest point and distance
template<typename T>
[[nodiscard]] MeshProjectionResult<T> projectPointToTriangle(
    const Vector3<T>& query,
    FaceId face,
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) noexcept {
    MeshProjectionResult<T> result;
    result.triPoint = projectOntoTriangle(query, face, v0, v1, v2);
    result.point = result.triPoint.interpolate(v0, v1, v2);
    result.distanceSquared = (query - result.point).lengthSq();
    return result;
}

/// Project a point onto a mesh using brute force (no spatial indexing)
/// @param query The point to project
/// @param points Vertex positions
/// @param topology Mesh topology
/// @param settings Projection settings
/// @return Closest projection result
template<typename T>
[[nodiscard]] MeshProjectionResult<T> projectPointToMeshBruteForce(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const ProjectionSettings<T>& settings = {}
) {
    MeshProjectionResult<T> best;
    T maxDistSq = settings.maxDistance * settings.maxDistance;
    T minDistSq = settings.minDistance * settings.minDistance;
    
    size_t numFaces = topology.faceCount();
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        
        // Skip masked faces
        if (settings.faceMask && !settings.faceMask->test(face)) {
            continue;
        }
        
        auto verts = topology.getTriVerts(face);
        if (!verts[0].valid()) continue;
        
        const Vector3<T>& v0 = points[verts[0].get()];
        const Vector3<T>& v1 = points[verts[1].get()];
        const Vector3<T>& v2 = points[verts[2].get()];
        
        auto proj = projectPointToTriangle(query, face, v0, v1, v2);
        
        // Check distance constraints
        if (proj.distanceSquared < minDistSq || proj.distanceSquared > maxDistSq) {
            continue;
        }
        
        if (proj.distanceSquared < best.distanceSquared) {
            best = proj;
            maxDistSq = proj.distanceSquared;  // Tighten search bound
        }
    }
    
    return best;
}

/// Parameters for AABB tree-based projection
template<typename T>
struct AABBProjectionParams {
    Vector3<T> query;
    const std::vector<Vector3<T>>* points{nullptr};
    const MeshTopology* topology{nullptr};
    const FaceBitSet* faceMask{nullptr};
    T maxDistSq{std::numeric_limits<T>::max()};
    T minDistSq{0};
};

/// Internal: Project point using AABB tree node
template<typename T>
void projectPointRecursive(
    const AABBTree<T>& tree,
    size_t nodeIndex,
    AABBProjectionParams<T>& params,
    MeshProjectionResult<T>& best
) {
    const auto& node = tree.nodes()[nodeIndex];
    
    // Check if bounding box is too far
    T boxDistSq = node.box.squaredDistanceFrom(params.query);
    if (boxDistSq > params.maxDistSq || boxDistSq > best.distanceSquared) {
        return;  // Prune this branch
    }
    
    if (node.isLeaf()) {
        // Leaf node - check faces
        for (FaceId face : node.faces) {
            if (params.faceMask && !params.faceMask->test(face)) {
                continue;
            }
            
            auto verts = params.topology->getTriVerts(face);
            if (!verts[0].valid()) continue;
            
            const Vector3<T>& v0 = (*params.points)[verts[0].get()];
            const Vector3<T>& v1 = (*params.points)[verts[1].get()];
            const Vector3<T>& v2 = (*params.points)[verts[2].get()];
            
            auto proj = projectPointToTriangle(params.query, face, v0, v1, v2);
            
            if (proj.distanceSquared < params.minDistSq) continue;
            
            if (proj.distanceSquared < best.distanceSquared) {
                best = proj;
            }
        }
    } else {
        // Internal node - recurse into children
        // Visit closer child first for better pruning
        size_t leftChild = node.leftChild;
        size_t rightChild = node.rightChild;
        
        T leftDistSq = tree.nodes()[leftChild].box.squaredDistanceFrom(params.query);
        T rightDistSq = tree.nodes()[rightChild].box.squaredDistanceFrom(params.query);
        
        if (leftDistSq <= rightDistSq) {
            projectPointRecursive(tree, leftChild, params, best);
            projectPointRecursive(tree, rightChild, params, best);
        } else {
            projectPointRecursive(tree, rightChild, params, best);
            projectPointRecursive(tree, leftChild, params, best);
        }
    }
}

/// Project a point onto a mesh using AABB tree acceleration
/// @param query The point to project
/// @param points Vertex positions  
/// @param topology Mesh topology
/// @param tree AABB tree for spatial acceleration
/// @param settings Projection settings
/// @return Closest projection result
template<typename T>
[[nodiscard]] MeshProjectionResult<T> projectPointToMesh(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree,
    const ProjectionSettings<T>& settings = {}
) {
    if (tree.nodes().empty()) {
        return projectPointToMeshBruteForce(query, points, topology, settings);
    }
    
    AABBProjectionParams<T> params;
    params.query = query;
    params.points = &points;
    params.topology = &topology;
    params.faceMask = settings.faceMask;
    params.maxDistSq = settings.maxDistance * settings.maxDistance;
    params.minDistSq = settings.minDistance * settings.minDistance;
    
    MeshProjectionResult<T> best;
    projectPointRecursive(tree, 0, params, best);
    
    return best;
}

/// Project multiple points onto a mesh (batch operation)
/// @param queries Points to project
/// @param points Vertex positions
/// @param topology Mesh topology
/// @param tree AABB tree for spatial acceleration  
/// @param settings Projection settings (applied to all queries)
/// @return Vector of projection results
template<typename T>
[[nodiscard]] std::vector<MeshProjectionResult<T>> projectPointsToMesh(
    const std::vector<Vector3<T>>& queries,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree,
    const ProjectionSettings<T>& settings = {}
) {
    std::vector<MeshProjectionResult<T>> results;
    results.reserve(queries.size());
    
    for (const auto& query : queries) {
        results.push_back(projectPointToMesh(query, points, topology, tree, settings));
    }
    
    return results;
}

/// Find all faces within a given distance from a point
/// @param query Center point
/// @param maxDistance Maximum distance
/// @param points Vertex positions
/// @param topology Mesh topology
/// @param tree AABB tree
/// @return Vector of (FaceId, distance) pairs, sorted by distance
template<typename T>
[[nodiscard]] std::vector<std::pair<FaceId, T>> findFacesNearPoint(
    const Vector3<T>& query,
    T maxDistance,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree
) {
    std::vector<std::pair<FaceId, T>> result;
    T maxDistSq = maxDistance * maxDistance;
    
    // Recursive helper using lambda
    std::function<void(size_t)> findNear = [&](size_t nodeIndex) {
        const auto& node = tree.nodes()[nodeIndex];
        
        // Check bounding box
        T boxDistSq = node.box.squaredDistanceFrom(query);
        if (boxDistSq > maxDistSq) return;
        
        if (node.isLeaf()) {
            for (FaceId face : node.faces) {
                auto verts = topology.getTriVerts(face);
                if (!verts[0].valid()) continue;
                
                const Vector3<T>& v0 = points[verts[0].get()];
                const Vector3<T>& v1 = points[verts[1].get()];
                const Vector3<T>& v2 = points[verts[2].get()];
                
                T distSq = distanceSquaredToTriangle(query, v0, v1, v2);
                if (distSq <= maxDistSq) {
                    result.emplace_back(face, std::sqrt(distSq));
                }
            }
        } else {
            findNear(node.leftChild);
            findNear(node.rightChild);
        }
    };
    
    if (!tree.nodes().empty()) {
        findNear(0);
    }
    
    // Sort by distance
    std::sort(result.begin(), result.end(), 
        [](const auto& a, const auto& b) { return a.second < b.second; });
    
    return result;
}

/// Project a point onto mesh boundary edges only
template<typename T>
[[nodiscard]] EdgeProjectionResult<T> projectPointToBoundary(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    EdgeProjectionResult<T> best;
    
    // Iterate over boundary edges
    size_t numEdges = topology.edgeCount();
    for (size_t i = 0; i < numEdges; ++i) {
        EdgeId edge{static_cast<int>(i)};
        
        // Check if this is a boundary edge (no face on sym side)
        if (topology.left(edge).valid() && topology.left(edge.sym()).valid()) {
            continue;  // Not a boundary edge
        }
        
        VertId vOrg = topology.org(edge);
        VertId vDest = topology.dest(edge);
        
        if (!vOrg.valid() || !vDest.valid()) continue;
        
        const Vector3<T>& pOrg = points[vOrg.get()];
        const Vector3<T>& pDest = points[vDest.get()];
        
        auto proj = projectOntoEdge(query, edge, pOrg, pDest);
        Vector3<T> closest = proj.interpolate(pOrg, pDest);
        T distSq = (query - closest).lengthSq();
        
        if (distSq < best.distanceSquared) {
            best.edgePoint = proj;
            best.point = closest;
            best.distanceSquared = distSq;
        }
    }
    
    return best;
}

/// Compute signed distance from point to mesh
/// Positive = outside, Negative = inside
/// @param query Point to test
/// @param points Vertex positions
/// @param topology Mesh topology  
/// @param tree AABB tree
/// @param faceNormals Pre-computed face normals
/// @return Signed distance (approximate sign based on closest face normal)
template<typename T>
[[nodiscard]] T signedDistanceToMesh(
    const Vector3<T>& query,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const AABBTree<T>& tree,
    const std::vector<Vector3<T>>& faceNormals
) {
    auto proj = projectPointToMesh(query, points, topology, tree);
    if (!proj.valid()) {
        return std::numeric_limits<T>::max();
    }
    
    // Determine sign using face normal
    const Vector3<T>& normal = faceNormals[proj.triPoint.face.get()];
    Vector3<T> toQuery = query - proj.point;
    
    T sign = dot(toQuery, normal) >= T(0) ? T(1) : T(-1);
    return sign * proj.distance();
}

} // namespace meshlib
