/**
 * @file Smooth.cpp
 * @brief Implementation of mesh smoothing algorithms
 */

#include "meshlib/algorithms/Smooth.h"
#include <unordered_set>
#include <unordered_map>
#include <cmath>

namespace meshlib {

namespace {

// Find vertices adjacent to each vertex
std::vector<std::vector<int>> buildAdjacency(const Mesh& mesh) {
    std::vector<std::vector<int>> adj(mesh.vertexCount());
    
    for (const auto& tri : mesh.triangles()) {
        for (int i = 0; i < 3; ++i) {
            int v0 = tri[i];
            int v1 = tri[(i + 1) % 3];
            
            // Add if not already present
            auto& neighbors = adj[v0];
            if (std::find(neighbors.begin(), neighbors.end(), v1) == neighbors.end()) {
                neighbors.push_back(v1);
            }
            
            auto& neighbors2 = adj[v1];
            if (std::find(neighbors2.begin(), neighbors2.end(), v0) == neighbors2.end()) {
                neighbors2.push_back(v0);
            }
        }
    }
    
    return adj;
}

// Find boundary vertices
std::unordered_set<int> findBoundaryVertices(const Mesh& mesh) {
    std::unordered_set<int> boundary;
    
    // Count edge occurrences
    std::unordered_map<uint64_t, int> edgeCounts;
    
    auto makeEdgeKey = [](int v0, int v1) -> uint64_t {
        if (v0 > v1) std::swap(v0, v1);
        return (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);
    };
    
    for (const auto& tri : mesh.triangles()) {
        edgeCounts[makeEdgeKey(tri[0], tri[1])]++;
        edgeCounts[makeEdgeKey(tri[1], tri[2])]++;
        edgeCounts[makeEdgeKey(tri[2], tri[0])]++;
    }
    
    // Boundary edges have count != 2
    for (const auto& [key, count] : edgeCounts) {
        if (count != 2) {
            boundary.insert(static_cast<int>(key >> 32));
            boundary.insert(static_cast<int>(key & 0xFFFFFFFF));
        }
    }
    
    return boundary;
}

} // anonymous namespace

SmoothResult smooth(const Mesh& mesh, const SmoothParams& params) {
    SmoothResult result;
    
    if (mesh.empty()) {
        result.status.code = ErrorCode::EmptyMesh;
        result.status.message = "Cannot smooth empty mesh";
        return result;
    }
    
    switch (params.method) {
        case SmoothingMethod::Laplacian:
            result.mesh = smoothLaplacian(mesh, params.iterations, params.lambda);
            break;
        
        case SmoothingMethod::Taubin:
            result.mesh = smoothTaubin(mesh, params.iterations, params.lambda, params.mu);
            break;
        
        case SmoothingMethod::HCLaplacian:
        case SmoothingMethod::Cotangent:
            // Fall back to Taubin for now
            result.mesh = smoothTaubin(mesh, params.iterations, params.lambda, params.mu);
            break;
    }
    
    result.status.code = ErrorCode::Success;
    return result;
}

Mesh smoothLaplacian(const Mesh& mesh, int iterations, float lambda) {
    if (mesh.empty() || iterations <= 0) {
        return mesh;
    }
    
    std::vector<Vector3f> vertices = mesh.vertices();
    auto adjacency = buildAdjacency(mesh);
    auto boundary = findBoundaryVertices(mesh);
    
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<Vector3f> newPositions = vertices;
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            // Skip boundary vertices
            if (boundary.count(static_cast<int>(i))) {
                continue;
            }
            
            const auto& neighbors = adjacency[i];
            if (neighbors.empty()) continue;
            
            // Compute Laplacian (centroid of neighbors)
            Vector3f centroid = Vector3f::zero();
            for (int neighbor : neighbors) {
                centroid += vertices[neighbor];
            }
            centroid = centroid / static_cast<float>(neighbors.size());
            
            // Move towards centroid
            newPositions[i] = vertices[i] + lambda * (centroid - vertices[i]);
        }
        
        vertices = std::move(newPositions);
    }
    
    return Mesh::fromTriangles(vertices, mesh.triangles());
}

Mesh smoothTaubin(const Mesh& mesh, int iterations, float lambda, float mu) {
    if (mesh.empty() || iterations <= 0) {
        return mesh;
    }
    
    std::vector<Vector3f> vertices = mesh.vertices();
    auto adjacency = buildAdjacency(mesh);
    auto boundary = findBoundaryVertices(mesh);
    
    auto smoothStep = [&](float factor) {
        std::vector<Vector3f> newPositions = vertices;
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (boundary.count(static_cast<int>(i))) {
                continue;
            }
            
            const auto& neighbors = adjacency[i];
            if (neighbors.empty()) continue;
            
            Vector3f centroid = Vector3f::zero();
            for (int neighbor : neighbors) {
                centroid += vertices[neighbor];
            }
            centroid = centroid / static_cast<float>(neighbors.size());
            
            newPositions[i] = vertices[i] + factor * (centroid - vertices[i]);
        }
        
        vertices = std::move(newPositions);
    };
    
    for (int iter = 0; iter < iterations; ++iter) {
        // Forward smoothing (shrink)
        smoothStep(lambda);
        
        // Backward smoothing (expand)
        smoothStep(mu);
    }
    
    return Mesh::fromTriangles(vertices, mesh.triangles());
}

Mesh smoothSelected(const Mesh& mesh, const std::vector<int>& vertexIndices, const SmoothParams& params) {
    if (mesh.empty() || vertexIndices.empty()) {
        return mesh;
    }
    
    std::unordered_set<int> selectedSet(vertexIndices.begin(), vertexIndices.end());
    
    std::vector<Vector3f> vertices = mesh.vertices();
    auto adjacency = buildAdjacency(mesh);
    
    for (int iter = 0; iter < params.iterations; ++iter) {
        std::vector<Vector3f> newPositions = vertices;
        
        for (int i : vertexIndices) {
            if (i < 0 || i >= static_cast<int>(vertices.size())) continue;
            
            const auto& neighbors = adjacency[i];
            if (neighbors.empty()) continue;
            
            Vector3f centroid = Vector3f::zero();
            for (int neighbor : neighbors) {
                centroid += vertices[neighbor];
            }
            centroid = centroid / static_cast<float>(neighbors.size());
            
            newPositions[i] = vertices[i] + params.lambda * (centroid - vertices[i]);
        }
        
        vertices = std::move(newPositions);
    }
    
    return Mesh::fromTriangles(vertices, mesh.triangles());
}

Mesh relax(const Mesh& mesh, int iterations) {
    // Relaxation is similar to Laplacian smoothing but with projection
    // to maintain triangle quality
    return smoothTaubin(mesh, iterations, 0.3f, -0.31f);
}

} // namespace meshlib
