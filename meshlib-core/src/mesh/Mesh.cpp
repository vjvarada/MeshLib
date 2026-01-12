/**
 * @file Mesh.cpp
 * @brief Implementation of the Mesh class
 */

#include "meshlib/mesh/Mesh.h"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <numeric>

namespace meshlib {

// ==================== Construction ====================

Mesh::Mesh(const Mesh& other) 
    : vertices_(other.vertices_)
    , triangles_(other.triangles_)
    , uvCoords_(other.uvCoords_)
{}

Mesh& Mesh::operator=(const Mesh& other) {
    if (this != &other) {
        vertices_ = other.vertices_;
        triangles_ = other.triangles_;
        uvCoords_ = other.uvCoords_;
        invalidateCache();
    }
    return *this;
}

Mesh::~Mesh() = default;

Mesh Mesh::fromTriangles(
    const std::vector<Vector3f>& vertices,
    const std::vector<std::array<int, 3>>& triangles) 
{
    Mesh mesh;
    mesh.vertices_ = vertices;
    mesh.triangles_ = triangles;
    return mesh;
}

Mesh Mesh::fromTriangleVertices(const std::vector<Triangle3f>& triangleVertices) {
    Mesh mesh;
    
    if (triangleVertices.empty()) {
        return mesh;
    }
    
    std::unordered_map<size_t, int> vertexMap;
    
    auto hashVertex = [](const Vector3f& v) -> size_t {
        size_t h = 0;
        auto hashFloat = [](float f) -> size_t {
            return std::hash<float>{}(f);
        };
        h ^= hashFloat(v.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= hashFloat(v.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= hashFloat(v.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    };
    
    mesh.triangles_.reserve(triangleVertices.size());
    mesh.vertices_.reserve(triangleVertices.size() * 3);
    
    for (const auto& tri : triangleVertices) {
        std::array<int, 3> indices;
        
        for (int i = 0; i < 3; ++i) {
            size_t hash = hashVertex(tri[i]);
            auto it = vertexMap.find(hash);
            
            if (it != vertexMap.end() && mesh.vertices_[it->second] == tri[i]) {
                indices[i] = it->second;
                continue;
            }
            
            int idx = static_cast<int>(mesh.vertices_.size());
            mesh.vertices_.push_back(tri[i]);
            vertexMap[hash] = idx;
            indices[i] = idx;
        }
        
        mesh.triangles_.push_back(indices);
    }
    
    return mesh;
}

bool Mesh::valid() const {
    if (vertices_.empty() || triangles_.empty()) {
        return false;
    }
    
    int numVerts = static_cast<int>(vertices_.size());
    
    for (const auto& tri : triangles_) {
        for (int idx : tri) {
            if (idx < 0 || idx >= numVerts) {
                return false;
            }
        }
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0]) {
            return false;
        }
    }
    
    return true;
}

const std::vector<Vector3f>& Mesh::vertexNormals() const {
    if (!cachedVertexNormals_.has_value()) {
        computeVertexNormals();
    }
    return *cachedVertexNormals_;
}

const std::vector<Vector3f>& Mesh::faceNormals() const {
    if (!cachedFaceNormals_.has_value()) {
        computeFaceNormals();
    }
    return *cachedFaceNormals_;
}

void Mesh::computeVertexNormals() const {
    cachedVertexNormals_ = std::vector<Vector3f>(vertices_.size(), Vector3f::zero());
    auto& normals = *cachedVertexNormals_;
    
    for (const auto& tri : triangles_) {
        const Vector3f& v0 = vertices_[tri[0]];
        const Vector3f& v1 = vertices_[tri[1]];
        const Vector3f& v2 = vertices_[tri[2]];
        
        Vector3f edge1 = v1 - v0;
        Vector3f edge2 = v2 - v0;
        Vector3f faceNormal = cross(edge1, edge2);
        
        normals[tri[0]] += faceNormal;
        normals[tri[1]] += faceNormal;
        normals[tri[2]] += faceNormal;
    }
    
    for (auto& n : normals) {
        float len = n.length();
        if (len > 1e-10f) {
            n = n / len;
        }
    }
}

void Mesh::computeFaceNormals() const {
    cachedFaceNormals_ = std::vector<Vector3f>(triangles_.size());
    auto& normals = *cachedFaceNormals_;
    
    for (size_t i = 0; i < triangles_.size(); ++i) {
        normals[i] = faceNormal(static_cast<int>(i));
    }
}

void Mesh::recomputeNormals() {
    cachedVertexNormals_.reset();
    cachedFaceNormals_.reset();
    computeVertexNormals();
    computeFaceNormals();
}

Triangle3f Mesh::triangleVertices(int faceIndex) const {
    const auto& tri = triangles_[faceIndex];
    return {vertices_[tri[0]], vertices_[tri[1]], vertices_[tri[2]]};
}

Box3f Mesh::boundingBox() const {
    if (cachedBoundingBox_.has_value()) {
        return *cachedBoundingBox_;
    }
    
    Box3f box;
    for (const auto& v : vertices_) {
        box.include(v);
    }
    
    cachedBoundingBox_ = box;
    return box;
}

Vector3f Mesh::centroid() const {
    if (vertices_.empty()) {
        return Vector3f::zero();
    }
    
    Vector3f sum = Vector3f::zero();
    for (const auto& v : vertices_) {
        sum += v;
    }
    
    return sum / static_cast<float>(vertices_.size());
}

double Mesh::area() const {
    double totalArea = 0.0;
    for (size_t i = 0; i < triangles_.size(); ++i) {
        totalArea += faceArea(static_cast<int>(i));
    }
    return totalArea;
}

double Mesh::volume() const {
    double vol = 0.0;
    
    for (const auto& tri : triangles_) {
        const Vector3f& v0 = vertices_[tri[0]];
        const Vector3f& v1 = vertices_[tri[1]];
        const Vector3f& v2 = vertices_[tri[2]];
        
        vol += static_cast<double>(v0.dot(cross(v1, v2))) / 6.0;
    }
    
    return std::abs(vol);
}

Vector3f Mesh::faceNormal(int faceIndex) const {
    const auto& tri = triangles_[faceIndex];
    const Vector3f& v0 = vertices_[tri[0]];
    const Vector3f& v1 = vertices_[tri[1]];
    const Vector3f& v2 = vertices_[tri[2]];
    
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    
    return cross(edge1, edge2).normalized();
}

float Mesh::faceArea(int faceIndex) const {
    const auto& tri = triangles_[faceIndex];
    const Vector3f& v0 = vertices_[tri[0]];
    const Vector3f& v1 = vertices_[tri[1]];
    const Vector3f& v2 = vertices_[tri[2]];
    
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    
    return 0.5f * cross(edge1, edge2).length();
}

Vector3f Mesh::faceCentroid(int faceIndex) const {
    const auto& tri = triangles_[faceIndex];
    return (vertices_[tri[0]] + vertices_[tri[1]] + vertices_[tri[2]]) / 3.0f;
}

bool Mesh::isClosed() const {
    return boundaryEdges().empty();
}

std::vector<std::pair<int, int>> Mesh::boundaryEdges() const {
    std::unordered_map<uint64_t, int> edgeCounts;
    
    auto makeEdgeKey = [](int v0, int v1) -> uint64_t {
        if (v0 > v1) std::swap(v0, v1);
        return (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);
    };
    
    for (const auto& tri : triangles_) {
        edgeCounts[makeEdgeKey(tri[0], tri[1])]++;
        edgeCounts[makeEdgeKey(tri[1], tri[2])]++;
        edgeCounts[makeEdgeKey(tri[2], tri[0])]++;
    }
    
    std::vector<std::pair<int, int>> boundary;
    for (const auto& [key, count] : edgeCounts) {
        if (count != 2) {
            int v0 = static_cast<int>(key >> 32);
            int v1 = static_cast<int>(key & 0xFFFFFFFF);
            boundary.push_back({v0, v1});
        }
    }
    
    return boundary;
}

bool Mesh::isManifold() const {
    std::unordered_map<uint64_t, int> edgeCounts;
    
    auto makeEdgeKey = [](int v0, int v1) -> uint64_t {
        if (v0 > v1) std::swap(v0, v1);
        return (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);
    };
    
    for (const auto& tri : triangles_) {
        edgeCounts[makeEdgeKey(tri[0], tri[1])]++;
        edgeCounts[makeEdgeKey(tri[1], tri[2])]++;
        edgeCounts[makeEdgeKey(tri[2], tri[0])]++;
    }
    
    for (const auto& [key, count] : edgeCounts) {
        if (count > 2) {
            return false;
        }
    }
    
    return true;
}

int Mesh::boundaryLoopCount() const {
    auto boundary = boundaryEdges();
    if (boundary.empty()) {
        return 0;
    }
    
    std::unordered_map<int, std::vector<int>> adj;
    for (const auto& [v0, v1] : boundary) {
        adj[v0].push_back(v1);
        adj[v1].push_back(v0);
    }
    
    std::unordered_set<int> visited;
    int loops = 0;
    
    for (const auto& [start, _] : adj) {
        if (visited.count(start)) continue;
        
        std::vector<int> queue = {start};
        while (!queue.empty()) {
            int v = queue.back();
            queue.pop_back();
            
            if (visited.count(v)) continue;
            visited.insert(v);
            
            for (int neighbor : adj[v]) {
                if (!visited.count(neighbor)) {
                    queue.push_back(neighbor);
                }
            }
        }
        
        ++loops;
    }
    
    return loops;
}

std::vector<int> Mesh::adjacentTriangles(int vertexIndex) const {
    std::vector<int> result;
    
    for (size_t i = 0; i < triangles_.size(); ++i) {
        const auto& tri = triangles_[i];
        if (tri[0] == vertexIndex || tri[1] == vertexIndex || tri[2] == vertexIndex) {
            result.push_back(static_cast<int>(i));
        }
    }
    
    return result;
}

std::vector<int> Mesh::adjacentVertices(int vertexIndex) const {
    std::unordered_set<int> neighbors;
    
    for (const auto& tri : triangles_) {
        for (int i = 0; i < 3; ++i) {
            if (tri[i] == vertexIndex) {
                neighbors.insert(tri[(i + 1) % 3]);
                neighbors.insert(tri[(i + 2) % 3]);
            }
        }
    }
    
    return std::vector<int>(neighbors.begin(), neighbors.end());
}

void Mesh::transform(const AffineTransform3f& xf) {
    for (auto& v : vertices_) {
        v = xf(v);
    }
    invalidateCache();
}

void Mesh::translate(const Vector3f& offset) {
    for (auto& v : vertices_) {
        v += offset;
    }
    invalidateCache();
}

void Mesh::scale(float factor) {
    for (auto& v : vertices_) {
        v *= factor;
    }
    invalidateCache();
}

void Mesh::scale(const Vector3f& factors) {
    for (auto& v : vertices_) {
        v.x *= factors.x;
        v.y *= factors.y;
        v.z *= factors.z;
    }
    invalidateCache();
}

void Mesh::flipFaces() {
    for (auto& tri : triangles_) {
        std::swap(tri[1], tri[2]);
    }
    invalidateCache();
}

int Mesh::mergeCloseVertices(float tolerance) {
    if (vertices_.empty()) return 0;
    
    float tolSq = tolerance * tolerance;
    std::vector<int> mapping(vertices_.size());
    std::iota(mapping.begin(), mapping.end(), 0);
    
    int merged = 0;
    
    for (size_t i = 0; i < vertices_.size(); ++i) {
        if (mapping[i] != static_cast<int>(i)) continue;
        
        for (size_t j = i + 1; j < vertices_.size(); ++j) {
            if (mapping[j] != static_cast<int>(j)) continue;
            
            if (distanceSq(vertices_[i], vertices_[j]) < tolSq) {
                mapping[j] = static_cast<int>(i);
                ++merged;
            }
        }
    }
    
    if (merged == 0) return 0;
    
    for (auto& tri : triangles_) {
        tri[0] = mapping[tri[0]];
        tri[1] = mapping[tri[1]];
        tri[2] = mapping[tri[2]];
    }
    
    removeUnusedVertices();
    invalidateCache();
    
    return merged;
}

int Mesh::removeDegenerateTriangles(float minArea) {
    auto it = std::remove_if(triangles_.begin(), triangles_.end(),
        [this, minArea](const std::array<int, 3>& tri) {
            if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0]) {
                return true;
            }
            
            const Vector3f& v0 = vertices_[tri[0]];
            const Vector3f& v1 = vertices_[tri[1]];
            const Vector3f& v2 = vertices_[tri[2]];
            
            float area = 0.5f * cross(v1 - v0, v2 - v0).length();
            return area < minArea;
        });
    
    int removed = static_cast<int>(std::distance(it, triangles_.end()));
    triangles_.erase(it, triangles_.end());
    
    if (removed > 0) {
        invalidateCache();
    }
    
    return removed;
}

int Mesh::removeUnusedVertices() {
    if (vertices_.empty()) return 0;
    
    std::vector<bool> used(vertices_.size(), false);
    for (const auto& tri : triangles_) {
        used[tri[0]] = true;
        used[tri[1]] = true;
        used[tri[2]] = true;
    }
    
    std::vector<int> mapping(vertices_.size(), -1);
    std::vector<Vector3f> newVertices;
    
    for (size_t i = 0; i < vertices_.size(); ++i) {
        if (used[i]) {
            mapping[i] = static_cast<int>(newVertices.size());
            newVertices.push_back(vertices_[i]);
        }
    }
    
    int removed = static_cast<int>(vertices_.size() - newVertices.size());
    
    if (removed > 0) {
        for (auto& tri : triangles_) {
            tri[0] = mapping[tri[0]];
            tri[1] = mapping[tri[1]];
            tri[2] = mapping[tri[2]];
        }
        
        vertices_ = std::move(newVertices);
        invalidateCache();
    }
    
    return removed;
}

void Mesh::clear() {
    vertices_.clear();
    triangles_.clear();
    uvCoords_.clear();
    invalidateCache();
}

void Mesh::invalidateCache() {
    cachedVertexNormals_.reset();
    cachedFaceNormals_.reset();
    cachedBoundingBox_.reset();
}

} // namespace meshlib
