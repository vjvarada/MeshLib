/**
 * @file Subdivide.cpp
 * @brief Implementation of mesh subdivision algorithms
 */

#include "meshlib/algorithms/Subdivide.h"
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace meshlib {

// Type alias for triangles
using TriangleType = std::array<int, 3>;

namespace {

// Edge key for caching midpoints
struct EdgeKey {
    int v0, v1;
    
    EdgeKey(int a, int b) : v0(std::min(a, b)), v1(std::max(a, b)) {}
    
    bool operator==(const EdgeKey& other) const {
        return v0 == other.v0 && v1 == other.v1;
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& e) const {
        return std::hash<uint64_t>()(
            (static_cast<uint64_t>(e.v0) << 32) | static_cast<uint64_t>(e.v1)
        );
    }
};

// Get or create edge midpoint
int getOrCreateMidpoint(
    int v0, int v1,
    std::vector<Vector3f>& vertices,
    std::unordered_map<EdgeKey, int, EdgeKeyHash>& midpointCache) {
    
    EdgeKey key(v0, v1);
    auto it = midpointCache.find(key);
    
    if (it != midpointCache.end()) {
        return it->second;
    }
    
    // Create new midpoint
    Vector3f mid = (vertices[v0] + vertices[v1]) * 0.5f;
    int newIdx = static_cast<int>(vertices.size());
    vertices.push_back(mid);
    midpointCache[key] = newIdx;
    
    return newIdx;
}

// Single iteration of midpoint subdivision
std::pair<std::vector<Vector3f>, std::vector<TriangleType>> 
subdivideMidpointOnce(
    const std::vector<Vector3f>& vertices,
    const std::vector<TriangleType>& triangles) {
    
    std::vector<Vector3f> newVertices = vertices;
    std::vector<TriangleType> newTriangles;
    std::unordered_map<EdgeKey, int, EdgeKeyHash> midpointCache;
    
    newTriangles.reserve(triangles.size() * 4);
    
    for (const auto& tri : triangles) {
        int v0 = tri[0];
        int v1 = tri[1];
        int v2 = tri[2];
        
        int m01 = getOrCreateMidpoint(v0, v1, newVertices, midpointCache);
        int m12 = getOrCreateMidpoint(v1, v2, newVertices, midpointCache);
        int m20 = getOrCreateMidpoint(v2, v0, newVertices, midpointCache);
        
        // Create 4 new triangles
        newTriangles.push_back({v0, m01, m20});
        newTriangles.push_back({m01, v1, m12});
        newTriangles.push_back({m20, m12, v2});
        newTriangles.push_back({m01, m12, m20});
    }
    
    return {std::move(newVertices), std::move(newTriangles)};
}

// Build vertex adjacency
std::vector<std::vector<int>> buildVertexAdjacency(
    int vertexCount,
    const std::vector<TriangleType>& triangles) {
    
    std::vector<std::vector<int>> adj(vertexCount);
    
    for (const auto& tri : triangles) {
        for (int i = 0; i < 3; ++i) {
            int v0 = tri[i];
            int v1 = tri[(i + 1) % 3];
            
            if (std::find(adj[v0].begin(), adj[v0].end(), v1) == adj[v0].end()) {
                adj[v0].push_back(v1);
            }
            if (std::find(adj[v1].begin(), adj[v1].end(), v0) == adj[v1].end()) {
                adj[v1].push_back(v0);
            }
        }
    }
    
    return adj;
}

// Single iteration of Loop subdivision
std::pair<std::vector<Vector3f>, std::vector<TriangleType>>
subdivideLoopOnce(
    const std::vector<Vector3f>& vertices,
    const std::vector<TriangleType>& triangles) {
    
    // First, do midpoint subdivision
    auto [newVertices, newTriangles] = subdivideMidpointOnce(vertices, triangles);
    
    // Then apply Loop subdivision weights to original vertices
    auto adj = buildVertexAdjacency(static_cast<int>(vertices.size()), triangles);
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        int n = static_cast<int>(adj[i].size());
        if (n < 3) continue;
        
        // Loop subdivision weight
        float beta;
        if (n == 3) {
            beta = 3.0f / 16.0f;
        } else {
            beta = 3.0f / (8.0f * n);
        }
        
        Vector3f neighborSum = Vector3f::zero();
        for (int neighbor : adj[i]) {
            neighborSum = neighborSum + vertices[neighbor];
        }
        
        newVertices[i] = vertices[i] * (1.0f - n * beta) + neighborSum * beta;
    }
    
    return {std::move(newVertices), std::move(newTriangles)};
}

} // anonymous namespace

SubdivideResult subdivide(const Mesh& mesh, const SubdivideParams& params) {
    SubdivideResult result;
    
    if (mesh.empty()) {
        result.status.code = ErrorCode::EmptyMesh;
        result.status.message = "Cannot subdivide empty mesh";
        return result;
    }
    
    if (params.iterations <= 0 && params.maxEdgeLength <= 0) {
        result.mesh = mesh;
        result.status.code = ErrorCode::Success;
        result.actualIterations = 0;
        return result;
    }
    
    std::vector<Vector3f> vertices = mesh.vertices();
    std::vector<TriangleType> triangles = mesh.triangles();
    int iterCount = 0;
    
    // Iterate
    for (int iter = 0; iter < params.iterations; ++iter) {
        if (params.maxTriangles > 0 && triangles.size() * 4 > params.maxTriangles) {
            break;
        }
        
        switch (params.scheme) {
            case SubdivisionScheme::Loop:
            case SubdivisionScheme::Butterfly: {
                auto [newVerts, newTris] = subdivideLoopOnce(vertices, triangles);
                vertices = std::move(newVerts);
                triangles = std::move(newTris);
                break;
            }
            
            case SubdivisionScheme::MidPoint:
            case SubdivisionScheme::CatmullClark:
            case SubdivisionScheme::Sqrt3:
            default: {
                auto [newVerts, newTris] = subdivideMidpointOnce(vertices, triangles);
                vertices = std::move(newVerts);
                triangles = std::move(newTris);
                break;
            }
        }
        
        iterCount++;
        
        // Progress callback
        if (params.progressCallback) {
            float progress = static_cast<float>(iter + 1) / params.iterations;
            if (!params.progressCallback(progress)) {
                result.status.code = ErrorCode::OperationFailed;
                result.status.message = "Operation cancelled";
                return result;
            }
        }
    }
    
    result.mesh = Mesh::fromTriangles(vertices, triangles);
    result.status.code = ErrorCode::Success;
    result.actualIterations = iterCount;
    
    return result;
}

Mesh subdivideLoop(const Mesh& mesh, int iterations) {
    SubdivideParams params;
    params.iterations = iterations;
    params.scheme = SubdivisionScheme::Loop;
    
    auto result = subdivide(mesh, params);
    return result.mesh;
}

Mesh subdivideMidpoint(const Mesh& mesh, int iterations) {
    SubdivideParams params;
    params.iterations = iterations;
    params.scheme = SubdivisionScheme::MidPoint;
    
    auto result = subdivide(mesh, params);
    return result.mesh;
}

Mesh subdivideAdaptive(const Mesh& mesh, float maxEdgeLength) {
    if (mesh.empty() || maxEdgeLength <= 0) {
        return mesh;
    }
    
    std::vector<Vector3f> vertices = mesh.vertices();
    std::vector<TriangleType> triangles = mesh.triangles();
    
    const int maxIterations = 10;
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        std::vector<TriangleType> newTriangles;
        std::unordered_map<EdgeKey, int, EdgeKeyHash> midpointCache;
        bool anySubdivided = false;
        
        for (const auto& tri : triangles) {
            int v0 = tri[0];
            int v1 = tri[1];
            int v2 = tri[2];
            
            float e01 = (vertices[v1] - vertices[v0]).length();
            float e12 = (vertices[v2] - vertices[v1]).length();
            float e20 = (vertices[v0] - vertices[v2]).length();
            
            bool split01 = e01 > maxEdgeLength;
            bool split12 = e12 > maxEdgeLength;
            bool split20 = e20 > maxEdgeLength;
            
            if (!split01 && !split12 && !split20) {
                newTriangles.push_back(tri);
                continue;
            }
            
            anySubdivided = true;
            
            // Full subdivision if all edges need splitting
            if (split01 && split12 && split20) {
                int m01 = getOrCreateMidpoint(v0, v1, vertices, midpointCache);
                int m12 = getOrCreateMidpoint(v1, v2, vertices, midpointCache);
                int m20 = getOrCreateMidpoint(v2, v0, vertices, midpointCache);
                
                newTriangles.push_back({v0, m01, m20});
                newTriangles.push_back({m01, v1, m12});
                newTriangles.push_back({m20, m12, v2});
                newTriangles.push_back({m01, m12, m20});
            } else {
                // Partial subdivision for 1-2 long edges
                int m01 = split01 ? getOrCreateMidpoint(v0, v1, vertices, midpointCache) : -1;
                int m12 = split12 ? getOrCreateMidpoint(v1, v2, vertices, midpointCache) : -1;
                int m20 = split20 ? getOrCreateMidpoint(v2, v0, vertices, midpointCache) : -1;
                
                // Create appropriate triangles based on which edges were split
                // This is simplified - just do full subdivision for now
                if (m01 >= 0) {
                    newTriangles.push_back({v0, m01, v2});
                    newTriangles.push_back({m01, v1, v2});
                } else if (m12 >= 0) {
                    newTriangles.push_back({v0, v1, m12});
                    newTriangles.push_back({v0, m12, v2});
                } else if (m20 >= 0) {
                    newTriangles.push_back({v0, v1, m20});
                    newTriangles.push_back({m20, v1, v2});
                } else {
                    newTriangles.push_back(tri);
                }
            }
        }
        
        if (!anySubdivided) {
            break;
        }
        
        triangles = std::move(newTriangles);
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh subdivideSelected(const Mesh& mesh, const std::vector<int>& faceIndices, int iterations) {
    if (mesh.empty() || faceIndices.empty() || iterations <= 0) {
        return mesh;
    }
    
    // Mark selected faces
    std::vector<bool> selected(mesh.triangleCount(), false);
    for (int idx : faceIndices) {
        if (idx >= 0 && idx < static_cast<int>(mesh.triangleCount())) {
            selected[idx] = true;
        }
    }
    
    std::vector<Vector3f> vertices = mesh.vertices();
    std::vector<TriangleType> triangles = mesh.triangles();
    
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<TriangleType> newTriangles;
        std::vector<bool> newSelected;
        std::unordered_map<EdgeKey, int, EdgeKeyHash> midpointCache;
        
        for (size_t i = 0; i < triangles.size(); ++i) {
            const auto& tri = triangles[i];
            
            if (!selected[i]) {
                newTriangles.push_back(tri);
                newSelected.push_back(false);
                continue;
            }
            
            int v0 = tri[0];
            int v1 = tri[1];
            int v2 = tri[2];
            
            int m01 = getOrCreateMidpoint(v0, v1, vertices, midpointCache);
            int m12 = getOrCreateMidpoint(v1, v2, vertices, midpointCache);
            int m20 = getOrCreateMidpoint(v2, v0, vertices, midpointCache);
            
            newTriangles.push_back({v0, m01, m20});
            newTriangles.push_back({m01, v1, m12});
            newTriangles.push_back({m20, m12, v2});
            newTriangles.push_back({m01, m12, m20});
            
            // All new triangles are selected
            newSelected.push_back(true);
            newSelected.push_back(true);
            newSelected.push_back(true);
            newSelected.push_back(true);
        }
        
        triangles = std::move(newTriangles);
        selected = std::move(newSelected);
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

} // namespace meshlib
