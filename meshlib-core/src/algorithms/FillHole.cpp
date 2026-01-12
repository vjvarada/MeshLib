/**
 * @file FillHole.cpp
 * @brief Implementation of mesh hole detection and filling algorithms
 */

#include "meshlib/algorithms/FillHole.h"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

namespace meshlib {

// Type alias for triangles
using TriangleType = std::array<int, 3>;

namespace {

// Build adjacency for boundary detection
std::unordered_map<int, std::vector<int>> buildBoundaryGraph(const Mesh& mesh) {
    const auto& triangles = mesh.triangles();
    
    // Count directed edges
    std::unordered_map<uint64_t, int> edgeCount;
    
    auto makeKey = [](int from, int to) -> uint64_t {
        return (static_cast<uint64_t>(static_cast<uint32_t>(from)) << 32) | 
               static_cast<uint64_t>(static_cast<uint32_t>(to));
    };
    
    for (const auto& tri : triangles) {
        for (int i = 0; i < 3; ++i) {
            int from = tri[i];
            int to = tri[(i + 1) % 3];
            edgeCount[makeKey(from, to)]++;
        }
    }
    
    // Find boundary edges (edges without reverse)
    std::unordered_map<int, std::vector<int>> boundaryNext;
    
    for (const auto& [key, count] : edgeCount) {
        int from = static_cast<int>(key >> 32);
        int to = static_cast<int>(key & 0xFFFFFFFF);
        
        uint64_t reverseKey = makeKey(to, from);
        if (edgeCount.find(reverseKey) == edgeCount.end()) {
            boundaryNext[to].push_back(from);
        }
    }
    
    return boundaryNext;
}

// Trace a single boundary loop
std::vector<int> traceBoundaryLoop(
    int startVertex,
    const std::unordered_map<int, std::vector<int>>& boundaryNext,
    std::unordered_set<int>& visited) {
    
    std::vector<int> loop;
    int current = startVertex;
    
    while (true) {
        if (visited.count(current) && !loop.empty()) {
            auto it = std::find(loop.begin(), loop.end(), current);
            if (it != loop.end()) {
                return std::vector<int>(it, loop.end());
            }
            break;
        }
        
        loop.push_back(current);
        visited.insert(current);
        
        auto nextIt = boundaryNext.find(current);
        if (nextIt == boundaryNext.end() || nextIt->second.empty()) {
            break;
        }
        
        int next = -1;
        for (int candidate : nextIt->second) {
            if (!visited.count(candidate) || candidate == startVertex) {
                next = candidate;
                break;
            }
        }
        
        if (next == -1) {
            next = nextIt->second[0];
        }
        
        current = next;
    }
    
    return {};
}

// Simple fan triangulation
std::vector<TriangleType> fanTriangulate(const std::vector<int>& boundary) {
    std::vector<TriangleType> triangles;
    
    if (boundary.size() < 3) return triangles;
    
    int v0 = boundary[0];
    for (size_t i = 1; i < boundary.size() - 1; ++i) {
        triangles.push_back({v0, boundary[i], boundary[i + 1]});
    }
    
    return triangles;
}

// Ear clipping triangulation
std::vector<TriangleType> earClipTriangulate(
    const std::vector<Vector3f>& vertices,
    const std::vector<int>& boundary) {
    
    std::vector<TriangleType> triangles;
    
    if (boundary.size() < 3) return triangles;
    if (boundary.size() == 3) {
        triangles.push_back({boundary[0], boundary[1], boundary[2]});
        return triangles;
    }
    
    // Compute hole normal
    Vector3f centroid = Vector3f::zero();
    for (int vi : boundary) {
        centroid = centroid + vertices[vi];
    }
    centroid = centroid * (1.0f / boundary.size());
    
    Vector3f normal = Vector3f::zero();
    for (size_t i = 0; i < boundary.size(); ++i) {
        int v0 = boundary[i];
        int v1 = boundary[(i + 1) % boundary.size()];
        Vector3f e0 = vertices[v0] - centroid;
        Vector3f e1 = vertices[v1] - centroid;
        normal = normal + e0.cross(e1);
    }
    
    float len = normal.length();
    if (len > 1e-10f) {
        normal = normal * (1.0f / len);
    } else {
        normal = Vector3f(0, 0, 1);
    }
    
    // Copy boundary for modification
    std::vector<int> remaining = boundary;
    
    // Ear clipping
    while (remaining.size() > 3) {
        bool earFound = false;
        
        for (size_t i = 0; i < remaining.size(); ++i) {
            size_t prev = (i + remaining.size() - 1) % remaining.size();
            size_t next = (i + 1) % remaining.size();
            
            int vp = remaining[prev];
            int vi = remaining[i];
            int vn = remaining[next];
            
            Vector3f e0 = vertices[vi] - vertices[vp];
            Vector3f e1 = vertices[vn] - vertices[vi];
            Vector3f cross = e0.cross(e1);
            
            // Check if ear (convex)
            if (cross.dot(normal) < 0) continue;
            
            // Check no vertices inside
            bool isEar = true;
            for (size_t j = 0; j < remaining.size(); ++j) {
                if (j == prev || j == i || j == next) continue;
                
                // Simple inside test - skip for now
            }
            
            if (isEar) {
                triangles.push_back({vp, vi, vn});
                remaining.erase(remaining.begin() + static_cast<long>(i));
                earFound = true;
                break;
            }
        }
        
        if (!earFound) {
            // Fall back to fan triangulation
            auto fanTris = fanTriangulate(remaining);
            triangles.insert(triangles.end(), fanTris.begin(), fanTris.end());
            break;
        }
    }
    
    if (remaining.size() == 3) {
        triangles.push_back({remaining[0], remaining[1], remaining[2]});
    }
    
    return triangles;
}

} // anonymous namespace

std::vector<HoleInfo> findHoles(const Mesh& mesh) {
    std::vector<HoleInfo> holes;
    
    if (mesh.empty()) {
        return holes;
    }
    
    auto boundaryNext = buildBoundaryGraph(mesh);
    const auto& vertices = mesh.vertices();
    std::unordered_set<int> visited;
    
    for (const auto& [startVertex, _] : boundaryNext) {
        if (visited.count(startVertex)) continue;
        
        auto loop = traceBoundaryLoop(startVertex, boundaryNext, visited);
        
        if (loop.size() >= 3) {
            HoleInfo hole;
            hole.boundaryVertices = std::move(loop);
            
            // Compute properties
            hole.perimeter = 0.0f;
            hole.centroid = Vector3f::zero();
            
            for (size_t i = 0; i < hole.boundaryVertices.size(); ++i) {
                int v0 = hole.boundaryVertices[i];
                int v1 = hole.boundaryVertices[(i + 1) % hole.boundaryVertices.size()];
                hole.perimeter += (vertices[v1] - vertices[v0]).length();
                hole.centroid = hole.centroid + vertices[v0];
            }
            hole.centroid = hole.centroid * (1.0f / hole.boundaryVertices.size());
            
            // Estimate area
            hole.area = 0.0f;
            for (size_t i = 0; i < hole.boundaryVertices.size(); ++i) {
                int v0 = hole.boundaryVertices[i];
                int v1 = hole.boundaryVertices[(i + 1) % hole.boundaryVertices.size()];
                Vector3f e0 = vertices[v0] - hole.centroid;
                Vector3f e1 = vertices[v1] - hole.centroid;
                hole.area += e0.cross(e1).length() * 0.5f;
            }
            
            // Estimate normal
            hole.normal = Vector3f::zero();
            for (size_t i = 0; i < hole.boundaryVertices.size(); ++i) {
                int v0 = hole.boundaryVertices[i];
                int v1 = hole.boundaryVertices[(i + 1) % hole.boundaryVertices.size()];
                Vector3f e0 = vertices[v0] - hole.centroid;
                Vector3f e1 = vertices[v1] - hole.centroid;
                hole.normal = hole.normal + e0.cross(e1);
            }
            float len = hole.normal.length();
            if (len > 1e-10f) {
                hole.normal = hole.normal * (1.0f / len);
            }
            
            holes.push_back(std::move(hole));
        }
    }
    
    return holes;
}

int countHoles(const Mesh& mesh) {
    return static_cast<int>(findHoles(mesh).size());
}

bool isClosed(const Mesh& mesh) {
    return countHoles(mesh) == 0;
}

FillHoleResult fillHoles(const Mesh& mesh, const FillHoleParams& params) {
    FillHoleResult result;
    
    if (mesh.empty()) {
        result.status.code = ErrorCode::EmptyMesh;
        result.status.message = "Cannot fill holes in empty mesh";
        return result;
    }
    
    auto holes = findHoles(mesh);
    
    if (holes.empty()) {
        result.mesh = mesh;
        result.status.code = ErrorCode::Success;
        result.holesFilled = 0;
        result.trianglesAdded = 0;
        return result;
    }
    
    std::vector<Vector3f> vertices = mesh.vertices();
    std::vector<TriangleType> triangles = mesh.triangles();
    int trianglesAdded = 0;
    
    for (size_t i = 0; i < holes.size(); ++i) {
        const auto& hole = holes[i];
        
        // Check size limits
        if (params.maxPerimeter > 0 && hole.perimeter > params.maxPerimeter) {
            continue;
        }
        if (params.maxBoundaryEdges > 0 && 
            static_cast<int>(hole.boundaryVertices.size()) > params.maxBoundaryEdges) {
            continue;
        }
        
        // Triangulate
        std::vector<TriangleType> fillTris;
        
        if (params.method == FillHoleMethod::Planar) {
            fillTris = fanTriangulate(hole.boundaryVertices);
        } else {
            fillTris = earClipTriangulate(vertices, hole.boundaryVertices);
        }
        
        triangles.insert(triangles.end(), fillTris.begin(), fillTris.end());
        trianglesAdded += static_cast<int>(fillTris.size());
        
        // Progress callback
        if (params.progressCallback) {
            float progress = static_cast<float>(i + 1) / holes.size();
            if (!params.progressCallback(progress)) {
                result.status.code = ErrorCode::OperationFailed;
                result.status.message = "Operation cancelled";
                return result;
            }
        }
    }
    
    result.mesh = Mesh::fromTriangles(vertices, triangles);
    result.status.code = ErrorCode::Success;
    result.holesFilled = static_cast<int>(holes.size());
    result.trianglesAdded = trianglesAdded;
    
    return result;
}

FillHoleResult fillHole(const Mesh& mesh, int holeIndex, const FillHoleParams& params) {
    FillHoleResult result;
    
    auto holes = findHoles(mesh);
    
    if (holeIndex < 0 || holeIndex >= static_cast<int>(holes.size())) {
        result.status.code = ErrorCode::InvalidInput;
        result.status.message = "Invalid hole index";
        return result;
    }
    
    return fillHoleByBoundary(mesh, holes[holeIndex].boundaryVertices, params);
}

FillHoleResult fillHoleByBoundary(
    const Mesh& mesh, 
    const std::vector<int>& boundaryVertices,
    const FillHoleParams& params) {
    
    FillHoleResult result;
    
    if (boundaryVertices.size() < 3) {
        result.status.code = ErrorCode::InvalidInput;
        result.status.message = "Boundary must have at least 3 vertices";
        return result;
    }
    
    std::vector<Vector3f> vertices = mesh.vertices();
    std::vector<TriangleType> triangles = mesh.triangles();
    
    std::vector<TriangleType> fillTris;
    
    if (params.method == FillHoleMethod::Planar) {
        fillTris = fanTriangulate(boundaryVertices);
    } else {
        fillTris = earClipTriangulate(vertices, boundaryVertices);
    }
    
    triangles.insert(triangles.end(), fillTris.begin(), fillTris.end());
    
    result.mesh = Mesh::fromTriangles(vertices, triangles);
    result.status.code = ErrorCode::Success;
    result.holesFilled = 1;
    result.trianglesAdded = static_cast<int>(fillTris.size());
    
    return result;
}

Mesh fillHolesPlanar(const Mesh& mesh) {
    FillHoleParams params;
    params.method = FillHoleMethod::Planar;
    
    auto result = fillHoles(mesh, params);
    return result.mesh;
}

FillHoleResult fillLargestHole(const Mesh& mesh, const FillHoleParams& params) {
    auto holes = findHoles(mesh);
    
    if (holes.empty()) {
        FillHoleResult result;
        result.mesh = mesh;
        result.status.code = ErrorCode::Success;
        result.holesFilled = 0;
        return result;
    }
    
    // Find largest hole by area
    int largestIdx = 0;
    float largestArea = holes[0].area;
    
    for (size_t i = 1; i < holes.size(); ++i) {
        if (holes[i].area > largestArea) {
            largestArea = holes[i].area;
            largestIdx = static_cast<int>(i);
        }
    }
    
    return fillHole(mesh, largestIdx, params);
}

} // namespace meshlib
