// MeshBoundary.h - Boundary detection and hole enumeration
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/BitSet.h"
#include "meshlib/core/IdVector.h"
#include "meshlib/core/Vector.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace meshlib {

// Forward declarations
class MeshTopology;

/// Information about a single boundary loop (hole)
template<typename T>
struct BoundaryLoop {
    std::vector<EdgeId> edges;       ///< Ordered boundary edges forming the loop
    std::vector<VertId> vertices;    ///< Ordered vertices on the boundary
    T perimeter{0};                  ///< Total length of the boundary
    Vector3<T> centroid;             ///< Centroid of boundary vertices
    Vector3<T> normal;               ///< Average normal direction of the hole
    T area{0};                       ///< Approximate area enclosed by the loop
    
    /// Number of edges/vertices in the loop
    [[nodiscard]] size_t size() const noexcept { return edges.size(); }
    
    /// Check if loop is valid
    [[nodiscard]] bool valid() const noexcept { return !edges.empty(); }
    [[nodiscard]] explicit operator bool() const noexcept { return valid(); }
    
    /// Get a representative edge of this loop
    [[nodiscard]] EdgeId representativeEdge() const noexcept {
        return edges.empty() ? EdgeId{} : edges[0];
    }
};

using BoundaryLoopf = BoundaryLoop<float>;
using BoundaryLoopd = BoundaryLoop<double>;

/// Result of boundary analysis
template<typename T>
struct BoundaryAnalysis {
    std::vector<BoundaryLoop<T>> loops;  ///< All boundary loops (holes)
    EdgeBitSet boundaryEdges;            ///< All boundary edges
    VertBitSet boundaryVertices;         ///< All boundary vertices
    size_t totalBoundaryEdges{0};        ///< Total count of boundary edges
    T totalPerimeter{0};                 ///< Sum of all loop perimeters
    
    /// Number of holes in the mesh
    [[nodiscard]] size_t numHoles() const noexcept { return loops.size(); }
    
    /// Check if mesh is closed (no boundary)
    [[nodiscard]] bool isClosed() const noexcept { return loops.empty(); }
    
    /// Get the largest hole by perimeter
    [[nodiscard]] size_t largestHole() const {
        if (loops.empty()) return ~size_t(0);
        return std::max_element(loops.begin(), loops.end(),
            [](const auto& a, const auto& b) { return a.perimeter < b.perimeter; }
        ) - loops.begin();
    }
    
    /// Get the smallest hole by perimeter
    [[nodiscard]] size_t smallestHole() const {
        if (loops.empty()) return ~size_t(0);
        return std::min_element(loops.begin(), loops.end(),
            [](const auto& a, const auto& b) { return a.perimeter < b.perimeter; }
        ) - loops.begin();
    }
    
    /// Get the largest hole by edge count
    [[nodiscard]] size_t largestHoleByEdges() const {
        if (loops.empty()) return ~size_t(0);
        return std::max_element(loops.begin(), loops.end(),
            [](const auto& a, const auto& b) { return a.size() < b.size(); }
        ) - loops.begin();
    }
};

using BoundaryAnalysisf = BoundaryAnalysis<float>;
using BoundaryAnalysisd = BoundaryAnalysis<double>;

/// Check if an edge is a boundary edge (has no face on one side)
[[nodiscard]] inline bool isBoundaryEdge(const MeshTopology& topology, EdgeId edge) {
    return !topology.left(edge).valid() || !topology.left(edge.sym()).valid();
}

/// Check if an edge is specifically a hole edge (no left face)
[[nodiscard]] inline bool isHoleEdge(const MeshTopology& topology, EdgeId edge) {
    return !topology.left(edge).valid();
}

/// Check if a vertex is on the boundary
[[nodiscard]] inline bool isBoundaryVertex(const MeshTopology& topology, VertId v) {
    EdgeId startEdge = topology.edgeWithOrg(v);
    if (!startEdge.valid()) return false;
    
    EdgeId edge = startEdge;
    do {
        if (!topology.left(edge).valid()) {
            return true;  // Found a boundary edge
        }
        edge = topology.next(edge.sym());
    } while (edge.valid() && edge != startEdge);
    
    return false;
}

/// Find all boundary edges in the mesh
[[nodiscard]] inline EdgeBitSet findBoundaryEdges(const MeshTopology& topology) {
    EdgeBitSet boundary;
    boundary.resize(topology.edgeCount());
    
    for (size_t i = 0; i < topology.edgeCount(); ++i) {
        EdgeId edge{static_cast<int>(i)};
        if (isHoleEdge(topology, edge)) {
            boundary.set(edge);
        }
    }
    
    return boundary;
}

/// Find all boundary vertices in the mesh
[[nodiscard]] inline VertBitSet findBoundaryVertices(const MeshTopology& topology) {
    VertBitSet boundary;
    boundary.resize(topology.vertCount());
    
    for (size_t i = 0; i < topology.edgeCount(); ++i) {
        EdgeId edge{static_cast<int>(i)};
        if (isHoleEdge(topology, edge)) {
            VertId v = topology.org(edge);
            if (v.valid()) {
                boundary.set(v);
            }
        }
    }
    
    return boundary;
}

/// Trace a single boundary loop starting from a given boundary edge
/// @param startEdge Must be a hole edge (no left face)
/// @return Ordered list of edges forming the loop
[[nodiscard]] inline std::vector<EdgeId> traceBoundaryLoop(
    const MeshTopology& topology,
    EdgeId startEdge
) {
    std::vector<EdgeId> loop;
    
    if (!startEdge.valid() || topology.left(startEdge).valid()) {
        return loop;  // Not a valid hole edge
    }
    
    EdgeId edge = startEdge;
    do {
        loop.push_back(edge);
        
        // Move to next boundary edge:
        // Go to destination, then find next edge without left face
        edge = topology.next(edge);
        while (edge.valid() && topology.left(edge).valid()) {
            edge = topology.next(edge.sym());
        }
        
        if (!edge.valid()) break;  // Topology error
        
    } while (edge != startEdge && loop.size() < topology.edgeCount());
    
    return loop;
}

/// Get vertices from a boundary loop
[[nodiscard]] inline std::vector<VertId> getBoundaryLoopVertices(
    const MeshTopology& topology,
    const std::vector<EdgeId>& loopEdges
) {
    std::vector<VertId> vertices;
    vertices.reserve(loopEdges.size());
    
    for (EdgeId edge : loopEdges) {
        VertId v = topology.org(edge);
        if (v.valid()) {
            vertices.push_back(v);
        }
    }
    
    return vertices;
}

/// Compute perimeter of a boundary loop
template<typename T>
[[nodiscard]] T computeLoopPerimeter(
    const std::vector<VertId>& vertices,
    const std::vector<Vector3<T>>& points
) {
    T perimeter{0};
    size_t n = vertices.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        if (vertices[i].valid() && vertices[j].valid()) {
            perimeter += (points[vertices[j].get()] - points[vertices[i].get()]).length();
        }
    }
    
    return perimeter;
}

/// Compute centroid of a boundary loop
template<typename T>
[[nodiscard]] Vector3<T> computeLoopCentroid(
    const std::vector<VertId>& vertices,
    const std::vector<Vector3<T>>& points
) {
    Vector3<T> centroid{0, 0, 0};
    int count = 0;
    
    for (VertId v : vertices) {
        if (v.valid()) {
            centroid = centroid + points[v.get()];
            count++;
        }
    }
    
    if (count > 0) {
        centroid = centroid / T(count);
    }
    
    return centroid;
}

/// Compute approximate normal of a boundary loop using Newell's method
template<typename T>
[[nodiscard]] Vector3<T> computeLoopNormal(
    const std::vector<VertId>& vertices,
    const std::vector<Vector3<T>>& points
) {
    Vector3<T> normal{0, 0, 0};
    size_t n = vertices.size();
    
    if (n < 3) return Vector3<T>{0, 0, 1};
    
    // Newell's method for polygon normal
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        if (!vertices[i].valid() || !vertices[j].valid()) continue;
        
        const Vector3<T>& pi = points[vertices[i].get()];
        const Vector3<T>& pj = points[vertices[j].get()];
        
        normal.x += (pi.y - pj.y) * (pi.z + pj.z);
        normal.y += (pi.z - pj.z) * (pi.x + pj.x);
        normal.z += (pi.x - pj.x) * (pi.y + pj.y);
    }
    
    T len = normal.length();
    if (len > T(1e-10)) {
        normal = normal / len;
    } else {
        normal = Vector3<T>{0, 0, 1};
    }
    
    return normal;
}

/// Compute approximate area enclosed by a boundary loop (projected to average plane)
template<typename T>
[[nodiscard]] T computeLoopArea(
    const std::vector<VertId>& vertices,
    const std::vector<Vector3<T>>& points,
    const Vector3<T>& centroid,
    const Vector3<T>& normal
) {
    T area{0};
    size_t n = vertices.size();
    
    if (n < 3) return T(0);
    
    // Sum triangular areas from centroid
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        if (!vertices[i].valid() || !vertices[j].valid()) continue;
        
        Vector3<T> v1 = points[vertices[i].get()] - centroid;
        Vector3<T> v2 = points[vertices[j].get()] - centroid;
        
        // Triangle area = 0.5 * |cross product|
        // Project onto plane normal for signed area
        area += dot(cross(v1, v2), normal) * T(0.5);
    }
    
    return std::abs(area);
}

/// Find all boundary loops (holes) in the mesh
template<typename T>
[[nodiscard]] BoundaryAnalysis<T> analyzeBoundary(
    const MeshTopology& topology,
    const std::vector<Vector3<T>>& points
) {
    BoundaryAnalysis<T> result;
    
    // Find all boundary edges
    result.boundaryEdges = findBoundaryEdges(topology);
    result.boundaryVertices = findBoundaryVertices(topology);
    
    // Track which edges have been processed
    EdgeBitSet processed;
    processed.resize(topology.edgeCount());
    
    // Find all loops
    for (size_t i = 0; i < topology.edgeCount(); ++i) {
        EdgeId edge{static_cast<int>(i)};
        
        // Skip if not a hole edge or already processed
        if (!isHoleEdge(topology, edge) || processed.test(edge)) {
            continue;
        }
        
        // Trace this loop
        auto loopEdges = traceBoundaryLoop(topology, edge);
        if (loopEdges.empty()) continue;
        
        // Mark edges as processed
        for (EdgeId e : loopEdges) {
            processed.set(e);
        }
        
        // Build BoundaryLoop
        BoundaryLoop<T> loop;
        loop.edges = std::move(loopEdges);
        loop.vertices = getBoundaryLoopVertices(topology, loop.edges);
        loop.perimeter = computeLoopPerimeter(loop.vertices, points);
        loop.centroid = computeLoopCentroid(loop.vertices, points);
        loop.normal = computeLoopNormal(loop.vertices, points);
        loop.area = computeLoopArea(loop.vertices, points, loop.centroid, loop.normal);
        
        result.totalPerimeter += loop.perimeter;
        result.loops.push_back(std::move(loop));
    }
    
    result.totalBoundaryEdges = result.boundaryEdges.count();
    
    return result;
}

/// Check if the mesh is closed (has no boundary)
[[nodiscard]] inline bool isMeshClosed(const MeshTopology& topology) {
    for (size_t i = 0; i < topology.edgeCount(); ++i) {
        if (isHoleEdge(topology, EdgeId{static_cast<int>(i)})) {
            return false;
        }
    }
    return true;
}

/// Count the number of boundary loops (holes)
[[nodiscard]] inline size_t countBoundaryLoops(const MeshTopology& topology) {
    EdgeBitSet processed;
    processed.resize(topology.edgeCount());
    size_t count = 0;
    
    for (size_t i = 0; i < topology.edgeCount(); ++i) {
        EdgeId edge{static_cast<int>(i)};
        
        if (!isHoleEdge(topology, edge) || processed.test(edge)) {
            continue;
        }
        
        // Trace and mark this loop
        auto loop = traceBoundaryLoop(topology, edge);
        for (EdgeId e : loop) {
            processed.set(e);
        }
        
        if (!loop.empty()) {
            count++;
        }
    }
    
    return count;
}

/// Find the boundary edge starting from a given vertex (if any)
[[nodiscard]] inline EdgeId findBoundaryEdgeFromVertex(
    const MeshTopology& topology,
    VertId v
) {
    EdgeId startEdge = topology.edgeWithOrg(v);
    if (!startEdge.valid()) return EdgeId{};
    
    EdgeId edge = startEdge;
    do {
        if (!topology.left(edge).valid()) {
            return edge;
        }
        edge = topology.next(edge.sym());
    } while (edge.valid() && edge != startEdge);
    
    return EdgeId{};  // Not a boundary vertex
}

/// Get edges incident to a boundary vertex, ordered around the vertex
[[nodiscard]] inline std::vector<EdgeId> getBoundaryVertexEdges(
    const MeshTopology& topology,
    VertId v
) {
    std::vector<EdgeId> edges;
    
    EdgeId startEdge = topology.edgeWithOrg(v);
    if (!startEdge.valid()) return edges;
    
    EdgeId edge = startEdge;
    do {
        edges.push_back(edge);
        edge = topology.next(edge.sym());
    } while (edge.valid() && edge != startEdge);
    
    return edges;
}

/// Check if a vertex is present on a boundary multiple times (complicated hole)
[[nodiscard]] inline bool isVertexRepeatedOnBoundary(
    const MeshTopology& topology,
    VertId v
) {
    int boundaryCount = 0;
    
    EdgeId startEdge = topology.edgeWithOrg(v);
    if (!startEdge.valid()) return false;
    
    EdgeId edge = startEdge;
    do {
        if (!topology.left(edge).valid()) {
            boundaryCount++;
            if (boundaryCount > 1) return true;
        }
        edge = topology.next(edge.sym());
    } while (edge.valid() && edge != startEdge);
    
    return false;
}

/// Find all vertices that appear multiple times on boundaries
[[nodiscard]] inline VertBitSet findRepeatedBoundaryVertices(const MeshTopology& topology) {
    VertBitSet repeated;
    repeated.resize(topology.vertCount());
    
    VertBitSet boundaryVerts = findBoundaryVertices(topology);
    
    for (size_t i = 0; i < topology.vertCount(); ++i) {
        VertId v{static_cast<int>(i)};
        if (boundaryVerts.test(v) && isVertexRepeatedOnBoundary(topology, v)) {
            repeated.set(v);
        }
    }
    
    return repeated;
}

/// Compute the Euler characteristic: V - E + F = 2 - 2g - b
/// where g = genus, b = number of boundary components
[[nodiscard]] inline int computeEulerCharacteristic(const MeshTopology& topology) {
    int V = static_cast<int>(topology.vertCount());
    int E = static_cast<int>(topology.edgeCount() / 2);  // Undirected edges
    int F = static_cast<int>(topology.faceCount());
    
    return V - E + F;
}

/// Estimate the genus of the mesh (assuming manifold)
/// genus = (2 - V + E - F - b) / 2 where b = number of boundary loops
[[nodiscard]] inline int estimateGenus(const MeshTopology& topology) {
    int euler = computeEulerCharacteristic(topology);
    int b = static_cast<int>(countBoundaryLoops(topology));
    
    // For orientable surface: χ = 2 - 2g - b
    // So: g = (2 - χ - b) / 2
    int genus = (2 - euler - b) / 2;
    return std::max(0, genus);
}

} // namespace meshlib
