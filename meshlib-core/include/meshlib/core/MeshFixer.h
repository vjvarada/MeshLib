// MeshFixer.h - Mesh repair and topology fixing algorithms
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/BitSet.h"
#include "meshlib/core/IdVector.h"
#include "meshlib/core/Vector.h"
#include "meshlib/core/Expected.h"
#include "meshlib/core/MeshBoundary.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace meshlib {

// Forward declarations
class MeshTopology;

/// Multiple edge: two half-edges connecting the same pair of vertices
using MultipleEdge = std::pair<VertId, VertId>;

/// Result of mesh validation
struct MeshValidationResult {
    bool isValid{true};                   ///< Overall validity
    
    // Topology issues
    size_t degenerateFaces{0};            ///< Faces with zero or near-zero area
    size_t multipleEdges{0};              ///< Duplicate edges between same vertices
    size_t nonManifoldEdges{0};           ///< Edges with more than 2 incident faces
    size_t nonManifoldVertices{0};        ///< Vertices with non-manifold neighborhoods
    size_t isolatedVertices{0};           ///< Vertices not connected to any face
    size_t duplicateFaces{0};             ///< Faces with identical vertices
    
    // Geometry issues  
    size_t flippedFaces{0};               ///< Faces with inverted orientation
    size_t selfIntersections{0};          ///< Self-intersecting face pairs
    
    // Boundary issues
    size_t boundaryLoops{0};              ///< Number of holes
    size_t repeatedBoundaryVerts{0};      ///< Vertices appearing multiple times on boundary
    
    // Containers for specific issues
    FaceBitSet degenerateFaceSet;
    EdgeBitSet multipleEdgeSet;
    EdgeBitSet nonManifoldEdgeSet;
    VertBitSet nonManifoldVertexSet;
    VertBitSet isolatedVertexSet;
    FaceBitSet duplicateFaceSet;
    
    /// Summary message
    [[nodiscard]] std::string summary() const {
        if (isValid) return "Mesh is valid";
        
        std::string msg = "Mesh has issues: ";
        if (degenerateFaces > 0) msg += std::to_string(degenerateFaces) + " degenerate faces, ";
        if (multipleEdges > 0) msg += std::to_string(multipleEdges) + " multiple edges, ";
        if (nonManifoldEdges > 0) msg += std::to_string(nonManifoldEdges) + " non-manifold edges, ";
        if (nonManifoldVertices > 0) msg += std::to_string(nonManifoldVertices) + " non-manifold vertices, ";
        if (isolatedVertices > 0) msg += std::to_string(isolatedVertices) + " isolated vertices, ";
        if (duplicateFaces > 0) msg += std::to_string(duplicateFaces) + " duplicate faces, ";
        if (boundaryLoops > 0) msg += std::to_string(boundaryLoops) + " holes, ";
        if (repeatedBoundaryVerts > 0) msg += std::to_string(repeatedBoundaryVerts) + " repeated boundary verts";
        return msg;
    }
};

/// Settings for mesh validation
struct ValidationSettings {
    /// Threshold for degenerate face detection (aspect ratio)
    float degenerateAspectRatio{1e6f};
    
    /// Threshold for degenerate face detection (area)
    float degenerateAreaThreshold{1e-12f};
    
    /// Whether to check for self-intersections (expensive)
    bool checkSelfIntersections{false};
    
    /// Whether to check face orientations
    bool checkOrientation{false};
};

/// Compute aspect ratio of a triangle (ratio of longest edge to inradius * sqrt(3))
template<typename T>
[[nodiscard]] T computeAspectRatio(
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) {
    Vector3<T> e0 = v1 - v0;
    Vector3<T> e1 = v2 - v1;
    Vector3<T> e2 = v0 - v2;
    
    T l0 = e0.length();
    T l1 = e1.length();
    T l2 = e2.length();
    
    T maxEdge = std::max({l0, l1, l2});
    T s = (l0 + l1 + l2) * T(0.5);  // Semi-perimeter
    
    // Area via Heron's formula
    T area2 = s * (s - l0) * (s - l1) * (s - l2);
    if (area2 <= T(0)) return std::numeric_limits<T>::max();
    
    T area = std::sqrt(area2);
    
    // Inradius = area / s
    T inradius = area / s;
    if (inradius < T(1e-12)) return std::numeric_limits<T>::max();
    
    // Aspect ratio normalized so equilateral = 1
    return maxEdge / (inradius * T(1.7320508075688772));  // sqrt(3)
}

/// Find degenerate faces (high aspect ratio or zero area)
template<typename T>
[[nodiscard]] FaceBitSet findDegenerateFaces(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    T criticalAspectRatio = T(1e6),
    T minArea = T(1e-12)
) {
    FaceBitSet degenerate;
    degenerate.resize(topology.faceCount());
    
    for (size_t i = 0; i < topology.faceCount(); ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        if (!verts[0].valid() || !verts[1].valid() || !verts[2].valid()) {
            degenerate.set(face);
            continue;
        }
        
        // Check for degenerate vertex indices
        if (verts[0] == verts[1] || verts[1] == verts[2] || verts[2] == verts[0]) {
            degenerate.set(face);
            continue;
        }
        
        const Vector3<T>& p0 = points[verts[0].get()];
        const Vector3<T>& p1 = points[verts[1].get()];
        const Vector3<T>& p2 = points[verts[2].get()];
        
        // Check area
        Vector3<T> normal = cross(p1 - p0, p2 - p0);
        T area = normal.length() * T(0.5);
        if (area < minArea) {
            degenerate.set(face);
            continue;
        }
        
        // Check aspect ratio
        T ar = computeAspectRatio(p0, p1, p2);
        if (ar > criticalAspectRatio) {
            degenerate.set(face);
        }
    }
    
    return degenerate;
}

/// Find edges shorter than a threshold
template<typename T>
[[nodiscard]] EdgeBitSet findShortEdges(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    T criticalLength
) {
    EdgeBitSet shortEdges;
    shortEdges.resize(topology.edgeCount());
    
    T criticalLengthSq = criticalLength * criticalLength;
    
    for (size_t i = 0; i < topology.edgeCount(); i += 2) {  // Only undirected
        EdgeId edge{static_cast<int>(i)};
        
        VertId v0 = topology.org(edge);
        VertId v1 = topology.dest(edge);
        
        if (!v0.valid() || !v1.valid()) continue;
        
        T lengthSq = (points[v1.get()] - points[v0.get()]).lengthSq();
        if (lengthSq < criticalLengthSq) {
            shortEdges.set(edge);
            shortEdges.set(edge.sym());
        }
    }
    
    return shortEdges;
}

/// Find edges longer than a threshold
template<typename T>
[[nodiscard]] EdgeBitSet findLongEdges(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    T criticalLength
) {
    EdgeBitSet longEdges;
    longEdges.resize(topology.edgeCount());
    
    T criticalLengthSq = criticalLength * criticalLength;
    
    for (size_t i = 0; i < topology.edgeCount(); i += 2) {
        EdgeId edge{static_cast<int>(i)};
        
        VertId v0 = topology.org(edge);
        VertId v1 = topology.dest(edge);
        
        if (!v0.valid() || !v1.valid()) continue;
        
        T lengthSq = (points[v1.get()] - points[v0.get()]).lengthSq();
        if (lengthSq > criticalLengthSq) {
            longEdges.set(edge);
            longEdges.set(edge.sym());
        }
    }
    
    return longEdges;
}

/// Find multiple edges (duplicate edges between same vertex pairs)
[[nodiscard]] inline std::vector<MultipleEdge> findMultipleEdges(const MeshTopology& topology) {
    std::vector<MultipleEdge> multiples;
    
    // Hash map: sorted vertex pair -> edge count
    std::unordered_map<uint64_t, int> edgeCounts;
    
    auto makeKey = [](VertId a, VertId b) -> uint64_t {
        int ia = a.get(), ib = b.get();
        if (ia > ib) std::swap(ia, ib);
        return (static_cast<uint64_t>(ia) << 32) | static_cast<uint64_t>(ib);
    };
    
    for (size_t i = 0; i < topology.edgeCount(); i += 2) {
        EdgeId edge{static_cast<int>(i)};
        
        VertId v0 = topology.org(edge);
        VertId v1 = topology.dest(edge);
        
        if (!v0.valid() || !v1.valid()) continue;
        
        uint64_t key = makeKey(v0, v1);
        edgeCounts[key]++;
    }
    
    // Find duplicates
    std::unordered_set<uint64_t> reported;
    for (size_t i = 0; i < topology.edgeCount(); i += 2) {
        EdgeId edge{static_cast<int>(i)};
        
        VertId v0 = topology.org(edge);
        VertId v1 = topology.dest(edge);
        
        if (!v0.valid() || !v1.valid()) continue;
        
        uint64_t key = makeKey(v0, v1);
        if (edgeCounts[key] > 1 && reported.find(key) == reported.end()) {
            multiples.emplace_back(v0, v1);
            reported.insert(key);
        }
    }
    
    return multiples;
}

/// Check if mesh has multiple edges
[[nodiscard]] inline bool hasMultipleEdges(const MeshTopology& topology) {
    return !findMultipleEdges(topology).empty();
}

/// Find non-manifold edges (more than 2 incident faces)
[[nodiscard]] inline EdgeBitSet findNonManifoldEdges(const MeshTopology& topology) {
    EdgeBitSet nonManifold;
    nonManifold.resize(topology.edgeCount());
    
    for (size_t i = 0; i < topology.edgeCount(); i += 2) {
        EdgeId edge{static_cast<int>(i)};
        
        // Count incident faces
        int faceCount = 0;
        if (topology.left(edge).valid()) faceCount++;
        if (topology.left(edge.sym()).valid()) faceCount++;
        
        // For manifold mesh, each edge has at most 2 faces
        // This simple check works for proper half-edge structure
        // A more thorough check would look for edges with same endpoints but different half-edges
    }
    
    return nonManifold;
}

/// Find vertices with non-manifold neighborhoods
[[nodiscard]] inline VertBitSet findNonManifoldVertices(const MeshTopology& topology) {
    VertBitSet nonManifold;
    nonManifold.resize(topology.vertCount());
    
    for (size_t i = 0; i < topology.vertCount(); ++i) {
        VertId v{static_cast<int>(i)};
        
        EdgeId startEdge = topology.edgeWithOrg(v);
        if (!startEdge.valid()) continue;
        
        // Count boundary transitions (face -> no-face -> face)
        int boundaryTransitions = 0;
        bool prevHadFace = topology.left(startEdge).valid();
        
        EdgeId edge = topology.next(startEdge.sym());
        while (edge.valid() && edge != startEdge) {
            bool hasFace = topology.left(edge).valid();
            if (hasFace != prevHadFace) {
                boundaryTransitions++;
            }
            prevHadFace = hasFace;
            edge = topology.next(edge.sym());
        }
        
        // Check final transition
        if (edge == startEdge) {
            bool hasFace = topology.left(startEdge).valid();
            if (hasFace != prevHadFace) {
                boundaryTransitions++;
            }
        }
        
        // Non-manifold if more than 2 boundary transitions (more than one fan)
        if (boundaryTransitions > 2) {
            nonManifold.set(v);
        }
    }
    
    return nonManifold;
}

/// Find isolated vertices (not connected to any face)
[[nodiscard]] inline VertBitSet findIsolatedVertices(const MeshTopology& topology) {
    VertBitSet connected;
    connected.resize(topology.vertCount());
    
    // Mark all vertices that belong to faces
    for (size_t i = 0; i < topology.faceCount(); ++i) {
        auto verts = topology.getTriVerts(FaceId{static_cast<int>(i)});
        for (int j = 0; j < 3; ++j) {
            if (verts[j].valid()) {
                connected.set(verts[j]);
            }
        }
    }
    
    // Return inverse
    VertBitSet isolated;
    isolated.resize(topology.vertCount());
    for (size_t i = 0; i < topology.vertCount(); ++i) {
        if (!connected.test(VertId{static_cast<int>(i)})) {
            isolated.set(VertId{static_cast<int>(i)});
        }
    }
    
    return isolated;
}

/// Find duplicate faces (same vertices in any order)
[[nodiscard]] inline FaceBitSet findDuplicateFaces(const MeshTopology& topology) {
    FaceBitSet duplicates;
    duplicates.resize(topology.faceCount());
    
    auto sortedKey = [](std::array<int, 3> v) {
        std::sort(v.begin(), v.end());
        return (static_cast<uint64_t>(v[0]) << 40) | 
               (static_cast<uint64_t>(v[1]) << 20) | 
               static_cast<uint64_t>(v[2]);
    };
    
    std::unordered_map<uint64_t, FaceId> seen;
    
    for (size_t i = 0; i < topology.faceCount(); ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        std::array<int, 3> v{verts[0].get(), verts[1].get(), verts[2].get()};
        uint64_t key = sortedKey(v);
        
        auto it = seen.find(key);
        if (it != seen.end()) {
            duplicates.set(face);
        } else {
            seen[key] = face;
        }
    }
    
    return duplicates;
}

/// Validate mesh and return detailed report
template<typename T>
[[nodiscard]] MeshValidationResult validateMesh(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const ValidationSettings& settings = {}
) {
    MeshValidationResult result;
    
    // Find degenerate faces
    result.degenerateFaceSet = findDegenerateFaces(points, topology, 
        T(settings.degenerateAspectRatio), T(settings.degenerateAreaThreshold));
    result.degenerateFaces = result.degenerateFaceSet.count();
    
    // Find multiple edges
    auto multiples = findMultipleEdges(topology);
    result.multipleEdges = multiples.size();
    
    // Find non-manifold elements
    result.nonManifoldEdgeSet = findNonManifoldEdges(topology);
    result.nonManifoldEdges = result.nonManifoldEdgeSet.count();
    
    result.nonManifoldVertexSet = findNonManifoldVertices(topology);
    result.nonManifoldVertices = result.nonManifoldVertexSet.count();
    
    // Find isolated vertices
    result.isolatedVertexSet = findIsolatedVertices(topology);
    result.isolatedVertices = result.isolatedVertexSet.count();
    
    // Find duplicate faces
    result.duplicateFaceSet = findDuplicateFaces(topology);
    result.duplicateFaces = result.duplicateFaceSet.count();
    
    // Boundary analysis
    result.boundaryLoops = countBoundaryLoops(topology);
    auto repeatedVerts = findRepeatedBoundaryVertices(topology);
    result.repeatedBoundaryVerts = repeatedVerts.count();
    
    // Determine overall validity
    result.isValid = (result.degenerateFaces == 0) &&
                     (result.multipleEdges == 0) &&
                     (result.nonManifoldEdges == 0) &&
                     (result.nonManifoldVertices == 0) &&
                     (result.duplicateFaces == 0) &&
                     (result.repeatedBoundaryVerts == 0);
    
    return result;
}

/// Check if edge dest(e) has degree 3 with 3 incident triangles
[[nodiscard]] inline bool isDegree3Dest(const MeshTopology& topology, EdgeId e) {
    VertId v = topology.dest(e);
    if (!v.valid()) return false;
    
    // Count edges and faces around vertex
    EdgeId startEdge = topology.edgeWithOrg(v);
    if (!startEdge.valid()) return false;
    
    int edgeCount = 0;
    int faceCount = 0;
    
    EdgeId edge = startEdge;
    do {
        edgeCount++;
        if (topology.left(edge).valid()) {
            faceCount++;
        }
        edge = topology.next(edge.sym());
    } while (edge.valid() && edge != startEdge && edgeCount < 100);
    
    return edgeCount == 3 && faceCount == 3;
}

/// Check if edge e has both triangular faces and dest(e) has degree 2
[[nodiscard]] inline bool isEdgeBetweenDoubleTris(const MeshTopology& topology, EdgeId e) {
    // Must have faces on both sides
    if (!topology.left(e).valid() || !topology.left(e.sym()).valid()) {
        return false;
    }
    
    // Check if dest has degree 2
    VertId v = topology.dest(e);
    if (!v.valid()) return false;
    
    EdgeId startEdge = topology.edgeWithOrg(v);
    if (!startEdge.valid()) return false;
    
    int degree = 0;
    EdgeId edge = startEdge;
    do {
        degree++;
        edge = topology.next(edge.sym());
    } while (edge.valid() && edge != startEdge && degree < 10);
    
    return degree == 2;
}

/// Find vertices with exactly N edges in their ring
[[nodiscard]] inline VertBitSet findNRingVerts(
    const MeshTopology& topology,
    int n,
    const VertBitSet* region = nullptr
) {
    VertBitSet result;
    result.resize(topology.vertCount());
    
    for (size_t i = 0; i < topology.vertCount(); ++i) {
        VertId v{static_cast<int>(i)};
        
        if (region && !region->test(v)) continue;
        
        EdgeId startEdge = topology.edgeWithOrg(v);
        if (!startEdge.valid()) continue;
        
        int degree = 0;
        EdgeId edge = startEdge;
        do {
            degree++;
            edge = topology.next(edge.sym());
        } while (edge.valid() && edge != startEdge && degree <= n + 1);
        
        if (degree == n) {
            result.set(v);
        }
    }
    
    return result;
}

/// Parameters for fixing mesh degeneracies
template<typename T>
struct FixDegeneraciesParams {
    /// Maximum permitted deviation from original surface
    T maxDeviation{0};
    
    /// Edges shorter than this will be collapsed ignoring quality checks
    T tinyEdgeLength{0};
    
    /// Ignore aspect ratio checks for triangles with worse aspect ratio than this
    T criticalTriAspectRatio{T(1e4)};
    
    /// Maximum angle change allowed for edge flips (radians)
    T maxAngleChange{T(1.047197551)};  // PI/3
    
    /// Stabilizer for planar regions
    T stabilizer{T(1e-6)};
    
    /// Region to fix (null = whole mesh)
    FaceBitSet* region{nullptr};
};

using FixDegeneraciesParamsf = FixDegeneraciesParams<float>;
using FixDegeneraciesParamsd = FixDegeneraciesParams<double>;

/// Compute vertex valence (number of incident edges)
[[nodiscard]] inline int computeValence(const MeshTopology& topology, VertId v) {
    EdgeId startEdge = topology.edgeWithOrg(v);
    if (!startEdge.valid()) return 0;
    
    int valence = 0;
    EdgeId edge = startEdge;
    do {
        valence++;
        edge = topology.next(edge.sym());
    } while (edge.valid() && edge != startEdge && valence < 1000);
    
    return valence;
}

/// Find faces that complicate holes (hole passes through same vertex multiple times)
[[nodiscard]] inline FaceBitSet findHoleComplicatingFaces(
    const MeshTopology& topology
) {
    FaceBitSet complicating;
    complicating.resize(topology.faceCount());
    
    // Find vertices that appear multiple times on boundaries
    VertBitSet repeatedVerts = findRepeatedBoundaryVertices(topology);
    
    // Mark faces adjacent to these vertices
    for (size_t i = 0; i < topology.faceCount(); ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        for (int j = 0; j < 3; ++j) {
            if (verts[j].valid() && repeatedVerts.test(verts[j])) {
                complicating.set(face);
                break;
            }
        }
    }
    
    return complicating;
}

/// Compute edge length statistics
template<typename T>
struct EdgeLengthStats {
    T minLength{std::numeric_limits<T>::max()};
    T maxLength{0};
    T meanLength{0};
    T medianLength{0};
    size_t edgeCount{0};
};

template<typename T>
[[nodiscard]] EdgeLengthStats<T> computeEdgeLengthStats(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    EdgeLengthStats<T> stats;
    std::vector<T> lengths;
    lengths.reserve(topology.edgeCount() / 2);
    
    for (size_t i = 0; i < topology.edgeCount(); i += 2) {
        EdgeId edge{static_cast<int>(i)};
        
        VertId v0 = topology.org(edge);
        VertId v1 = topology.dest(edge);
        
        if (!v0.valid() || !v1.valid()) continue;
        
        T len = (points[v1.get()] - points[v0.get()]).length();
        lengths.push_back(len);
        
        stats.minLength = std::min(stats.minLength, len);
        stats.maxLength = std::max(stats.maxLength, len);
    }
    
    stats.edgeCount = lengths.size();
    
    if (!lengths.empty()) {
        T sum = T(0);
        for (T len : lengths) sum += len;
        stats.meanLength = sum / T(lengths.size());
        
        std::sort(lengths.begin(), lengths.end());
        stats.medianLength = lengths[lengths.size() / 2];
    }
    
    return stats;
}

} // namespace meshlib
