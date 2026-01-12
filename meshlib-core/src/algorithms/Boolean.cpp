/**
 * @file Boolean.cpp
 * @brief Implementation of mesh boolean (CSG) operations
 * 
 * This is a simplified implementation for demonstration purposes.
 * Production use cases should consider more robust algorithms like:
 * - Cork (https://github.com/gilbo/cork)
 * - libigl's boolean operations
 * - CGAL's Nef polyhedra
 */

#include "meshlib/algorithms/Boolean.h"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

namespace meshlib {

namespace {

// Compute signed distance from point to plane
float pointPlaneDistance(const Vector3f& point, const Vector3f& planePoint, const Vector3f& planeNormal) {
    return (point - planePoint).dot(planeNormal);
}

// Check if point is inside mesh (using ray casting)
bool checkPointInsideMesh(const Vector3f& point, const Mesh& mesh) {
    const auto& vertices = mesh.vertices();
    const auto& triangles = mesh.triangles();
    
    // Cast ray in +X direction and count intersections
    int intersections = 0;
    
    Vector3f rayDir(1.0f, 0.0f, 0.0f);
    
    for (const auto& tri : triangles) {
        const Vector3f& v0 = vertices[tri[0]];
        const Vector3f& v1 = vertices[tri[1]];
        const Vector3f& v2 = vertices[tri[2]];
        
        // Möller–Trumbore intersection algorithm
        Vector3f edge1 = v1 - v0;
        Vector3f edge2 = v2 - v0;
        Vector3f h = rayDir.cross(edge2);
        float a = edge1.dot(h);
        
        if (std::abs(a) < 1e-10f) continue;
        
        float f = 1.0f / a;
        Vector3f s = point - v0;
        float u = f * s.dot(h);
        
        if (u < 0.0f || u > 1.0f) continue;
        
        Vector3f q = s.cross(edge1);
        float v = f * rayDir.dot(q);
        
        if (v < 0.0f || u + v > 1.0f) continue;
        
        float t = f * edge2.dot(q);
        
        if (t > 1e-6f) {
            intersections++;
        }
    }
    
    return (intersections % 2) == 1;
}

// Classify vertices of mesh A with respect to mesh B
std::vector<bool> classifyVertices(const Mesh& meshA, const Mesh& meshB) {
    std::vector<bool> inside;
    inside.reserve(meshA.vertexCount());
    
    for (const auto& v : meshA.vertices()) {
        inside.push_back(checkPointInsideMesh(v, meshB));
    }
    
    return inside;
}

} // anonymous namespace

BooleanResult boolean(const Mesh& meshA, const Mesh& meshB, BooleanOperation op, const BooleanParams& params) {
    BooleanResult result;
    
    if (meshA.empty() || meshB.empty()) {
        result.status.code = ErrorCode::EmptyMesh;
        result.status.message = "Cannot perform boolean on empty mesh";
        return result;
    }
    
    // Classify vertices
    auto aInsideB = classifyVertices(meshA, meshB);
    auto bInsideA = classifyVertices(meshB, meshA);
    
    std::vector<Vector3f> resultVertices;
    std::vector<std::array<int, 3>> resultTriangles;
    
    const auto& verticesA = meshA.vertices();
    const auto& trianglesA = meshA.triangles();
    const auto& verticesB = meshB.vertices();
    const auto& trianglesB = meshB.triangles();
    
    // Map from original vertex indices to result indices
    std::unordered_map<int, int> vertexMapA;
    std::unordered_map<int, int> vertexMapB;
    
    auto getOrAddVertexA = [&](int idx) -> int {
        auto it = vertexMapA.find(idx);
        if (it != vertexMapA.end()) return it->second;
        int newIdx = static_cast<int>(resultVertices.size());
        resultVertices.push_back(verticesA[idx]);
        vertexMapA[idx] = newIdx;
        return newIdx;
    };
    
    auto getOrAddVertexB = [&](int idx) -> int {
        auto it = vertexMapB.find(idx);
        if (it != vertexMapB.end()) return it->second;
        int newIdx = static_cast<int>(resultVertices.size());
        resultVertices.push_back(verticesB[idx]);
        vertexMapB[idx] = newIdx;
        return newIdx;
    };
    
    // Process triangles based on operation type
    switch (op) {
        case BooleanOperation::Union: {
            // Keep triangles from A that are outside B
            for (const auto& tri : trianglesA) {
                bool allOutside = !aInsideB[tri[0]] && !aInsideB[tri[1]] && !aInsideB[tri[2]];
                if (allOutside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexA(tri[0]),
                        getOrAddVertexA(tri[1]),
                        getOrAddVertexA(tri[2])
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            
            // Keep triangles from B that are outside A
            for (const auto& tri : trianglesB) {
                bool allOutside = !bInsideA[tri[0]] && !bInsideA[tri[1]] && !bInsideA[tri[2]];
                if (allOutside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexB(tri[0]),
                        getOrAddVertexB(tri[1]),
                        getOrAddVertexB(tri[2])
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            break;
        }
        
        case BooleanOperation::Intersection: {
            // Keep triangles from A that are inside B
            for (const auto& tri : trianglesA) {
                bool allInside = aInsideB[tri[0]] && aInsideB[tri[1]] && aInsideB[tri[2]];
                if (allInside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexA(tri[0]),
                        getOrAddVertexA(tri[1]),
                        getOrAddVertexA(tri[2])
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            
            // Keep triangles from B that are inside A
            for (const auto& tri : trianglesB) {
                bool allInside = bInsideA[tri[0]] && bInsideA[tri[1]] && bInsideA[tri[2]];
                if (allInside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexB(tri[0]),
                        getOrAddVertexB(tri[1]),
                        getOrAddVertexB(tri[2])
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            break;
        }
        
        case BooleanOperation::Difference: {
            // Keep triangles from A that are outside B
            for (const auto& tri : trianglesA) {
                bool allOutside = !aInsideB[tri[0]] && !aInsideB[tri[1]] && !aInsideB[tri[2]];
                if (allOutside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexA(tri[0]),
                        getOrAddVertexA(tri[1]),
                        getOrAddVertexA(tri[2])
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            
            // Keep triangles from B that are inside A (with flipped normals)
            for (const auto& tri : trianglesB) {
                bool allInside = bInsideA[tri[0]] && bInsideA[tri[1]] && bInsideA[tri[2]];
                if (allInside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexB(tri[0]),
                        getOrAddVertexB(tri[2]),  // Flipped
                        getOrAddVertexB(tri[1])   // Flipped
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            break;
        }
        
        case BooleanOperation::SymDifference: {
            // Union of (A - B) and (B - A)
            
            // A - B: Keep triangles from A outside B
            for (const auto& tri : trianglesA) {
                bool allOutside = !aInsideB[tri[0]] && !aInsideB[tri[1]] && !aInsideB[tri[2]];
                if (allOutside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexA(tri[0]),
                        getOrAddVertexA(tri[1]),
                        getOrAddVertexA(tri[2])
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            
            // B - A: Keep triangles from B outside A
            for (const auto& tri : trianglesB) {
                bool allOutside = !bInsideA[tri[0]] && !bInsideA[tri[1]] && !bInsideA[tri[2]];
                if (allOutside) {
                    std::array<int, 3> newTri = {
                        getOrAddVertexB(tri[0]),
                        getOrAddVertexB(tri[1]),
                        getOrAddVertexB(tri[2])
                    };
                    resultTriangles.push_back(newTri);
                }
            }
            break;
        }
    }
    
    result.mesh = Mesh::fromTriangles(resultVertices, resultTriangles);
    result.status.code = ErrorCode::Success;
    return result;
}

BooleanResult booleanUnion(const Mesh& meshA, const Mesh& meshB, const BooleanParams& params) {
    return boolean(meshA, meshB, BooleanOperation::Union, params);
}

BooleanResult booleanIntersection(const Mesh& meshA, const Mesh& meshB, const BooleanParams& params) {
    return boolean(meshA, meshB, BooleanOperation::Intersection, params);
}

BooleanResult booleanDifference(const Mesh& meshA, const Mesh& meshB, const BooleanParams& params) {
    return boolean(meshA, meshB, BooleanOperation::Difference, params);
}

} // namespace meshlib
