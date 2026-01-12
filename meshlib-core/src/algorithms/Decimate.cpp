/**
 * @file Decimate.cpp
 * @brief Implementation of mesh decimation algorithms
 */

#include "meshlib/algorithms/Decimate.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace meshlib {

namespace {

// Quadric error metric for a plane
struct Quadric {
    float a, b, c, d;     // Plane equation coefficients
    float area;           // Triangle area
    
    // The quadric matrix elements (symmetric 4x4)
    float q[10];  // Stores upper triangle
    
    Quadric() {
        std::fill(std::begin(q), std::end(q), 0.0f);
        area = 0;
    }
    
    void addPlane(const Vector3f& n, float d, float weight = 1.0f) {
        // Q = n * n^T + d * n + d^2
        q[0] += weight * n.x * n.x;
        q[1] += weight * n.x * n.y;
        q[2] += weight * n.x * n.z;
        q[3] += weight * n.x * d;
        q[4] += weight * n.y * n.y;
        q[5] += weight * n.y * n.z;
        q[6] += weight * n.y * d;
        q[7] += weight * n.z * n.z;
        q[8] += weight * n.z * d;
        q[9] += weight * d * d;
        area += weight;
    }
    
    Quadric operator+(const Quadric& other) const {
        Quadric result;
        for (int i = 0; i < 10; ++i) {
            result.q[i] = q[i] + other.q[i];
        }
        result.area = area + other.area;
        return result;
    }
    
    float evaluate(const Vector3f& v) const {
        // v^T * Q * v
        return q[0] * v.x * v.x + 2 * q[1] * v.x * v.y + 2 * q[2] * v.x * v.z + 2 * q[3] * v.x
             + q[4] * v.y * v.y + 2 * q[5] * v.y * v.z + 2 * q[6] * v.y
             + q[7] * v.z * v.z + 2 * q[8] * v.z
             + q[9];
    }
};

struct EdgeCollapse {
    int v0, v1;          // Vertices to collapse
    Vector3f newPos;     // Optimal position
    float error;         // Error if collapsed
    
    bool operator>(const EdgeCollapse& other) const {
        return error > other.error;
    }
};

} // anonymous namespace

DecimateResult decimate(const Mesh& mesh, const DecimateParams& params) {
    DecimateResult result;
    
    if (mesh.empty()) {
        result.status.code = ErrorCode::EmptyMesh;
        result.status.message = "Cannot decimate empty mesh";
        return result;
    }
    
    // Copy mesh data
    std::vector<Vector3f> vertices = mesh.vertices();
    std::vector<std::array<int, 3>> triangles = mesh.triangles();
    
    // Compute target triangle count
    size_t targetCount = params.targetTriangleCount;
    if (targetCount == 0) {
        targetCount = static_cast<size_t>(mesh.triangleCount() * params.targetRatio);
    }
    targetCount = std::max(targetCount, size_t(4)); // Minimum 4 triangles
    
    // Initialize quadrics for each vertex
    std::vector<Quadric> quadrics(vertices.size());
    
    for (const auto& tri : triangles) {
        const Vector3f& v0 = vertices[tri[0]];
        const Vector3f& v1 = vertices[tri[1]];
        const Vector3f& v2 = vertices[tri[2]];
        
        Vector3f edge1 = v1 - v0;
        Vector3f edge2 = v2 - v0;
        Vector3f normal = cross(edge1, edge2);
        float area = normal.length() * 0.5f;
        
        if (area > 1e-10f) {
            normal = normal.normalized();
            float d = -dot(normal, v0);
            
            quadrics[tri[0]].addPlane(normal, d, area);
            quadrics[tri[1]].addPlane(normal, d, area);
            quadrics[tri[2]].addPlane(normal, d, area);
        }
    }
    
    // Build edge list
    std::unordered_map<uint64_t, std::pair<int, int>> edges;
    
    auto makeEdgeKey = [](int v0, int v1) -> uint64_t {
        if (v0 > v1) std::swap(v0, v1);
        return (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);
    };
    
    for (const auto& tri : triangles) {
        edges[makeEdgeKey(tri[0], tri[1])] = {tri[0], tri[1]};
        edges[makeEdgeKey(tri[1], tri[2])] = {tri[1], tri[2]};
        edges[makeEdgeKey(tri[2], tri[0])] = {tri[2], tri[0]};
    }
    
    // Build priority queue of edge collapses
    std::priority_queue<EdgeCollapse, std::vector<EdgeCollapse>, std::greater<EdgeCollapse>> pq;
    
    for (const auto& [key, edge] : edges) {
        int v0 = edge.first;
        int v1 = edge.second;
        
        Quadric combined = quadrics[v0] + quadrics[v1];
        
        // Find optimal position (use midpoint for simplicity)
        Vector3f midpoint = (vertices[v0] + vertices[v1]) * 0.5f;
        float error = combined.evaluate(midpoint);
        
        pq.push({v0, v1, midpoint, error});
    }
    
    // Track which vertices are still valid
    std::vector<int> vertexMap(vertices.size());
    std::iota(vertexMap.begin(), vertexMap.end(), 0);
    
    auto findRoot = [&vertexMap](int v) -> int {
        while (vertexMap[v] != v) {
            vertexMap[v] = vertexMap[vertexMap[v]]; // Path compression
            v = vertexMap[v];
        }
        return v;
    };
    
    // Perform collapses
    size_t removedTriangles = 0;
    float maxError = 0;
    
    while (triangles.size() - removedTriangles > targetCount && !pq.empty()) {
        EdgeCollapse collapse = pq.top();
        pq.pop();
        
        // Check if edge is still valid
        int root0 = findRoot(collapse.v0);
        int root1 = findRoot(collapse.v1);
        
        if (root0 == root1) continue; // Already collapsed
        
        if (collapse.error > params.maxError) break;
        
        // Perform collapse
        vertices[root0] = collapse.newPos;
        vertexMap[root1] = root0;
        quadrics[root0] = quadrics[root0] + quadrics[root1];
        
        maxError = std::max(maxError, collapse.error);
        
        // Mark affected triangles as degenerate
        for (auto& tri : triangles) {
            for (int& idx : tri) {
                idx = findRoot(idx);
            }
        }
    }
    
    // Remove degenerate triangles
    std::vector<std::array<int, 3>> newTriangles;
    for (const auto& tri : triangles) {
        if (tri[0] != tri[1] && tri[1] != tri[2] && tri[2] != tri[0]) {
            newTriangles.push_back(tri);
        }
    }
    
    // Create new vertex list (only used vertices)
    std::vector<bool> usedVerts(vertices.size(), false);
    for (const auto& tri : newTriangles) {
        usedVerts[tri[0]] = true;
        usedVerts[tri[1]] = true;
        usedVerts[tri[2]] = true;
    }
    
    std::vector<int> newIndices(vertices.size(), -1);
    std::vector<Vector3f> newVertices;
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        if (usedVerts[i]) {
            newIndices[i] = static_cast<int>(newVertices.size());
            newVertices.push_back(vertices[i]);
        }
    }
    
    for (auto& tri : newTriangles) {
        tri[0] = newIndices[tri[0]];
        tri[1] = newIndices[tri[1]];
        tri[2] = newIndices[tri[2]];
    }
    
    result.mesh = Mesh::fromTriangles(newVertices, newTriangles);
    result.status.code = ErrorCode::Success;
    result.removedTriangles = mesh.triangleCount() - newTriangles.size();
    result.removedVertices = mesh.vertexCount() - newVertices.size();
    result.maxErrorAchieved = maxError;
    
    return result;
}

Mesh decimateToCount(const Mesh& mesh, size_t targetTriangles) {
    DecimateParams params;
    params.targetTriangleCount = targetTriangles;
    auto result = decimate(mesh, params);
    return result.ok() ? result.mesh : mesh;
}

Mesh decimateByRatio(const Mesh& mesh, float ratio) {
    DecimateParams params;
    params.targetRatio = ratio;
    auto result = decimate(mesh, params);
    return result.ok() ? result.mesh : mesh;
}

Mesh decimateToError(const Mesh& mesh, float maxError) {
    DecimateParams params;
    params.maxError = maxError;
    params.targetRatio = 0.0f; // Go as low as possible
    params.targetTriangleCount = 4;
    auto result = decimate(mesh, params);
    return result.ok() ? result.mesh : mesh;
}

std::vector<Mesh> generateLODs(const Mesh& mesh, int levels, float baseRatio) {
    std::vector<Mesh> lods;
    lods.push_back(mesh); // LOD 0 is original
    
    float ratio = baseRatio;
    for (int i = 1; i < levels; ++i) {
        auto simplified = decimateByRatio(mesh, ratio);
        lods.push_back(simplified);
        ratio *= 0.5f;
    }
    
    return lods;
}

} // namespace meshlib
