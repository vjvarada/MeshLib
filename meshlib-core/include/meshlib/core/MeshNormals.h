// MeshNormals.h - Face and vertex normal computation
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/Vector.h"
#include "meshlib/core/BitSet.h"
#include "meshlib/core/IdVector.h"
#include <vector>
#include <cmath>

namespace meshlib {

// Forward declarations
class MeshTopology;

/// Compute the normal of a triangle from its vertices
/// Normal direction follows right-hand rule (CCW winding)
template<typename T>
[[nodiscard]] Vector3<T> computeTriangleNormal(
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) noexcept {
    Vector3<T> e1 = v1 - v0;
    Vector3<T> e2 = v2 - v0;
    return cross(e1, e2).normalized();
}

/// Compute the area-weighted normal of a triangle (not normalized)
/// The magnitude is twice the triangle area
template<typename T>
[[nodiscard]] Vector3<T> computeTriangleNormalWeighted(
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) noexcept {
    Vector3<T> e1 = v1 - v0;
    Vector3<T> e2 = v2 - v0;
    return cross(e1, e2);
}

/// Compute the area of a triangle
template<typename T>
[[nodiscard]] T computeTriangleArea(
    const Vector3<T>& v0,
    const Vector3<T>& v1,
    const Vector3<T>& v2
) noexcept {
    return computeTriangleNormalWeighted(v0, v1, v2).length() * T(0.5);
}

/// Compute face normals for all faces in a mesh
/// @param points Vertex positions
/// @param topology Mesh topology
/// @return Vector of face normals (indexed by FaceId)
template<typename T>
[[nodiscard]] FaceMap<Vector3<T>> computeFaceNormals(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    size_t numFaces = topology.faceCount();
    FaceMap<Vector3<T>> normals;
    normals.resize(numFaces);
    
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        if (verts[0].valid() && verts[1].valid() && verts[2].valid()) {
            normals[face] = computeTriangleNormal(
                points[verts[0].get()],
                points[verts[1].get()],
                points[verts[2].get()]
            );
        } else {
            normals[face] = Vector3<T>{0, 0, 1};  // Default for degenerate
        }
    }
    
    return normals;
}

/// Compute face areas for all faces in a mesh
/// @param points Vertex positions
/// @param topology Mesh topology
/// @return Vector of face areas (indexed by FaceId)
template<typename T>
[[nodiscard]] FaceMap<T> computeFaceAreas(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    size_t numFaces = topology.faceCount();
    FaceMap<T> areas;
    areas.resize(numFaces);
    
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        if (verts[0].valid() && verts[1].valid() && verts[2].valid()) {
            areas[face] = computeTriangleArea(
                points[verts[0].get()],
                points[verts[1].get()],
                points[verts[2].get()]
            );
        } else {
            areas[face] = T(0);
        }
    }
    
    return areas;
}

/// Weighting scheme for vertex normal computation
enum class NormalWeighting {
    Uniform,    ///< Equal weight for all adjacent faces
    Area,       ///< Weight by face area (default)
    Angle       ///< Weight by incident angle at vertex
};

/// Compute vertex normals by averaging adjacent face normals
/// @param points Vertex positions
/// @param topology Mesh topology
/// @param weighting How to weight face normals
/// @return Vector of vertex normals (indexed by VertId)
template<typename T>
[[nodiscard]] VertMap<Vector3<T>> computeVertexNormals(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    NormalWeighting weighting = NormalWeighting::Area
) {
    size_t numVerts = topology.vertCount();
    size_t numFaces = topology.faceCount();
    
    VertMap<Vector3<T>> normals;
    normals.resize(numVerts, Vector3<T>{0, 0, 0});
    
    // Accumulate weighted normals from each face
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        if (!verts[0].valid() || !verts[1].valid() || !verts[2].valid()) {
            continue;
        }
        
        const Vector3<T>& p0 = points[verts[0].get()];
        const Vector3<T>& p1 = points[verts[1].get()];
        const Vector3<T>& p2 = points[verts[2].get()];
        
        Vector3<T> e01 = p1 - p0;
        Vector3<T> e02 = p2 - p0;
        Vector3<T> e12 = p2 - p1;
        
        Vector3<T> faceNormal = cross(e01, e02);  // Not normalized - includes area weight
        
        switch (weighting) {
            case NormalWeighting::Uniform: {
                faceNormal = faceNormal.normalized();
                normals[verts[0]] = normals[verts[0]] + faceNormal;
                normals[verts[1]] = normals[verts[1]] + faceNormal;
                normals[verts[2]] = normals[verts[2]] + faceNormal;
                break;
            }
            
            case NormalWeighting::Area: {
                // faceNormal already has area weighting (magnitude = 2*area)
                normals[verts[0]] = normals[verts[0]] + faceNormal;
                normals[verts[1]] = normals[verts[1]] + faceNormal;
                normals[verts[2]] = normals[verts[2]] + faceNormal;
                break;
            }
            
            case NormalWeighting::Angle: {
                // Weight by angle at each vertex
                T len01 = e01.length();
                T len02 = e02.length();
                T len12 = e12.length();
                
                // Avoid division by zero
                T eps = T(1e-10);
                len01 = std::max(len01, eps);
                len02 = std::max(len02, eps);
                len12 = std::max(len12, eps);
                
                // Angle at v0
                T cos0 = dot(e01, e02) / (len01 * len02);
                cos0 = std::max(T(-1), std::min(T(1), cos0));
                T angle0 = std::acos(cos0);
                
                // Angle at v1
                Vector3<T> e10 = -e01;
                T cos1 = dot(e10, e12) / (len01 * len12);
                cos1 = std::max(T(-1), std::min(T(1), cos1));
                T angle1 = std::acos(cos1);
                
                // Angle at v2
                T angle2 = T(3.14159265358979323846) - angle0 - angle1;
                
                Vector3<T> unitNormal = faceNormal.normalized();
                normals[verts[0]] = normals[verts[0]] + unitNormal * angle0;
                normals[verts[1]] = normals[verts[1]] + unitNormal * angle1;
                normals[verts[2]] = normals[verts[2]] + unitNormal * angle2;
                break;
            }
        }
    }
    
    // Normalize all vertex normals
    for (size_t i = 0; i < numVerts; ++i) {
        Vector3<T>& n = normals[VertId{static_cast<int>(i)}];
        T len = n.length();
        if (len > T(1e-10)) {
            n = n / len;
        } else {
            n = Vector3<T>{0, 0, 1};  // Default for isolated vertices
        }
    }
    
    return normals;
}

/// Compute pseudo-normals for vertices on boundary edges
/// These point outward from the mesh surface along the boundary
template<typename T>
[[nodiscard]] VertMap<Vector3<T>> computeBoundaryVertexNormals(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const VertMap<Vector3<T>>& surfaceNormals
) {
    size_t numVerts = topology.vertCount();
    VertMap<Vector3<T>> boundaryNormals;
    boundaryNormals.resize(numVerts, Vector3<T>{0, 0, 0});
    
    // Find boundary edges and compute tangent-normal product
    size_t numEdges = topology.edgeCount();
    for (size_t i = 0; i < numEdges; ++i) {
        EdgeId edge{static_cast<int>(i)};
        
        // Check if boundary edge
        if (topology.left(edge).valid() && topology.left(edge.sym()).valid()) {
            continue;  // Not boundary
        }
        
        VertId v0 = topology.org(edge);
        VertId v1 = topology.dest(edge);
        
        if (!v0.valid() || !v1.valid()) continue;
        
        const Vector3<T>& p0 = points[v0.get()];
        const Vector3<T>& p1 = points[v1.get()];
        
        Vector3<T> edgeDir = (p1 - p0).normalized();
        
        // Get average surface normal at this edge
        Vector3<T> avgNormal = (surfaceNormals[v0] + surfaceNormals[v1]).normalized();
        
        // Boundary normal is perpendicular to edge and surface normal
        Vector3<T> boundaryDir = cross(avgNormal, edgeDir).normalized();
        
        // Ensure it points outward (away from mesh interior)
        FaceId face = topology.left(edge);
        if (!face.valid()) face = topology.left(edge.sym());
        if (face.valid()) {
            auto verts = topology.getTriVerts(face);
            // Find the third vertex of the face
            VertId v2;
            for (int j = 0; j < 3; ++j) {
                if (verts[j] != v0 && verts[j] != v1) {
                    v2 = verts[j];
                    break;
                }
            }
            if (v2.valid()) {
                Vector3<T> toCenter = (points[v2.get()] - (p0 + p1) * T(0.5)).normalized();
                if (dot(boundaryDir, toCenter) > 0) {
                    boundaryDir = -boundaryDir;  // Flip to point outward
                }
            }
        }
        
        boundaryNormals[v0] = boundaryNormals[v0] + boundaryDir;
        boundaryNormals[v1] = boundaryNormals[v1] + boundaryDir;
    }
    
    // Normalize
    for (size_t i = 0; i < numVerts; ++i) {
        Vector3<T>& n = boundaryNormals[VertId{static_cast<int>(i)}];
        T len = n.length();
        if (len > T(1e-10)) {
            n = n / len;
        }
        // Keep as zero for non-boundary vertices
    }
    
    return boundaryNormals;
}

/// Smooth normals by averaging with neighbors
/// @param normals Input normals
/// @param topology Mesh topology
/// @param iterations Number of smoothing iterations
/// @return Smoothed normals
template<typename T>
[[nodiscard]] VertMap<Vector3<T>> smoothVertexNormals(
    const VertMap<Vector3<T>>& normals,
    const MeshTopology& topology,
    int iterations = 1
) {
    VertMap<Vector3<T>> result = normals;
    VertMap<Vector3<T>> temp;
    temp.resize(normals.size());
    
    size_t numVerts = topology.vertCount();
    size_t numFaces = topology.faceCount();
    
    for (int iter = 0; iter < iterations; ++iter) {
        // Reset temp
        for (size_t i = 0; i < numVerts; ++i) {
            temp[VertId{static_cast<int>(i)}] = Vector3<T>{0, 0, 0};
        }
        
        // Accumulate neighbor normals
        VertMap<int> counts;
        counts.resize(numVerts, 0);
        
        for (size_t i = 0; i < numFaces; ++i) {
            auto verts = topology.getTriVerts(FaceId{static_cast<int>(i)});
            for (int v = 0; v < 3; ++v) {
                if (verts[v].valid()) {
                    // Add normals from all vertices in this face
                    for (int w = 0; w < 3; ++w) {
                        if (verts[w].valid()) {
                            temp[verts[v]] = temp[verts[v]] + result[verts[w]];
                            counts[verts[v]]++;
                        }
                    }
                }
            }
        }
        
        // Normalize
        for (size_t i = 0; i < numVerts; ++i) {
            VertId v{static_cast<int>(i)};
            if (counts[v] > 0) {
                result[v] = temp[v].normalized();
            }
        }
    }
    
    return result;
}

/// Compute edge normals (average of adjacent face normals)
template<typename T>
[[nodiscard]] EdgeMap<Vector3<T>> computeEdgeNormals(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const FaceMap<Vector3<T>>& faceNormals
) {
    size_t numEdges = topology.edgeCount();
    EdgeMap<Vector3<T>> edgeNormals;
    edgeNormals.resize(numEdges);
    
    for (size_t i = 0; i < numEdges; i += 2) {  // Process undirected edges
        EdgeId edge{static_cast<int>(i)};
        
        FaceId f1 = topology.left(edge);
        FaceId f2 = topology.left(edge.sym());
        
        Vector3<T> n{0, 0, 0};
        int count = 0;
        
        if (f1.valid()) {
            n = n + faceNormals[f1];
            count++;
        }
        if (f2.valid()) {
            n = n + faceNormals[f2];
            count++;
        }
        
        if (count > 0) {
            n = n.normalized();
        } else {
            // Isolated edge - use perpendicular to edge direction
            VertId v0 = topology.org(edge);
            VertId v1 = topology.dest(edge);
            if (v0.valid() && v1.valid()) {
                Vector3<T> edgeDir = (points[v1.get()] - points[v0.get()]).normalized();
                // Find a perpendicular direction
                if (std::abs(edgeDir.x) < T(0.9)) {
                    n = cross(edgeDir, Vector3<T>{1, 0, 0}).normalized();
                } else {
                    n = cross(edgeDir, Vector3<T>{0, 1, 0}).normalized();
                }
            }
        }
        
        edgeNormals[edge] = n;
        edgeNormals[edge.sym()] = n;
    }
    
    return edgeNormals;
}

/// Compute dihedral angle between adjacent faces along an edge
/// Returns angle in radians [0, pi]
template<typename T>
[[nodiscard]] T computeDihedralAngle(
    EdgeId edge,
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const FaceMap<Vector3<T>>& faceNormals
) {
    FaceId f1 = topology.left(edge);
    FaceId f2 = topology.left(edge.sym());
    
    if (!f1.valid() || !f2.valid()) {
        return T(0);  // Boundary edge
    }
    
    const Vector3<T>& n1 = faceNormals[f1];
    const Vector3<T>& n2 = faceNormals[f2];
    
    T cosAngle = dot(n1, n2);
    cosAngle = std::max(T(-1), std::min(T(1), cosAngle));
    
    return std::acos(cosAngle);
}

/// Compute all dihedral angles for edges
template<typename T>
[[nodiscard]] EdgeMap<T> computeDihedralAngles(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const FaceMap<Vector3<T>>& faceNormals
) {
    size_t numEdges = topology.edgeCount();
    EdgeMap<T> angles;
    angles.resize(numEdges);
    
    for (size_t i = 0; i < numEdges; i += 2) {
        EdgeId edge{static_cast<int>(i)};
        T angle = computeDihedralAngle(edge, points, topology, faceNormals);
        angles[edge] = angle;
        angles[edge.sym()] = angle;
    }
    
    return angles;
}

/// Find sharp edges (dihedral angle above threshold)
template<typename T>
[[nodiscard]] EdgeBitSet findSharpEdges(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    T thresholdRadians = T(0.5236)  // Default ~30 degrees
) {
    auto faceNormals = computeFaceNormals(points, topology);
    auto angles = computeDihedralAngles(points, topology, faceNormals);
    
    size_t numEdges = topology.edgeCount();
    EdgeBitSet sharp;
    sharp.resize(numEdges);
    
    for (size_t i = 0; i < numEdges; i += 2) {
        EdgeId edge{static_cast<int>(i)};
        if (angles[edge] > thresholdRadians) {
            sharp.set(edge);
            sharp.set(edge.sym());
        }
    }
    
    return sharp;
}

/// Compute vertex normals respecting sharp edges
/// Sharp edges break the normal averaging
template<typename T>
[[nodiscard]] VertMap<Vector3<T>> computeVertexNormalsWithCreases(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology,
    const EdgeBitSet& sharpEdges,
    NormalWeighting weighting = NormalWeighting::Area
) {
    // First compute regular normals
    auto normals = computeVertexNormals(points, topology, weighting);
    
    // For vertices adjacent to sharp edges, recompute by grouping faces
    // This is a simplified version - full crease handling would need
    // per-corner normals, not per-vertex
    
    size_t numEdges = topology.edgeCount();
    VertBitSet sharpVerts;
    sharpVerts.resize(topology.vertCount());
    
    for (size_t i = 0; i < numEdges; ++i) {
        EdgeId edge{static_cast<int>(i)};
        if (sharpEdges.test(edge)) {
            VertId v0 = topology.org(edge);
            VertId v1 = topology.dest(edge);
            if (v0.valid()) sharpVerts.set(v0);
            if (v1.valid()) sharpVerts.set(v1);
        }
    }
    
    // Recompute normals for sharp vertices using only adjacent non-sharp faces
    // (Simplified - a proper implementation would need corner normals)
    
    return normals;
}

/// Compute total surface area of mesh
template<typename T>
[[nodiscard]] T computeTotalArea(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    auto areas = computeFaceAreas(points, topology);
    T total{0};
    for (size_t i = 0; i < areas.size(); ++i) {
        total += areas[FaceId{static_cast<int>(i)}];
    }
    return total;
}

/// Compute centroid of mesh (area-weighted average of face centroids)
template<typename T>
[[nodiscard]] Vector3<T> computeCentroid(
    const std::vector<Vector3<T>>& points,
    const MeshTopology& topology
) {
    Vector3<T> centroid{0, 0, 0};
    T totalArea{0};
    
    size_t numFaces = topology.faceCount();
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        if (!verts[0].valid()) continue;
        
        const Vector3<T>& p0 = points[verts[0].get()];
        const Vector3<T>& p1 = points[verts[1].get()];
        const Vector3<T>& p2 = points[verts[2].get()];
        
        T area = computeTriangleArea(p0, p1, p2);
        Vector3<T> faceCentroid = (p0 + p1 + p2) / T(3);
        
        centroid = centroid + faceCentroid * area;
        totalArea += area;
    }
    
    if (totalArea > T(1e-10)) {
        centroid = centroid / totalArea;
    }
    
    return centroid;
}

} // namespace meshlib
