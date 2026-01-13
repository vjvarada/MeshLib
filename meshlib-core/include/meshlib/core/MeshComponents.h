// MeshComponents.h - Connected component analysis for meshes
// Part of meshlib-core: Industrial-strength mesh processing
// SPDX-License-Identifier: MIT

#pragma once

#include "meshlib/core/Id.h"
#include "meshlib/core/BitSet.h"
#include "meshlib/core/IdVector.h"
#include "meshlib/core/UnionFind.h"
#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>
#include <queue>

namespace meshlib {

// Forward declarations
class MeshTopology;

/// Information about a single connected component
struct ComponentInfo {
    size_t faceCount{0};       ///< Number of faces in component
    size_t vertexCount{0};     ///< Number of vertices in component
    size_t edgeCount{0};       ///< Number of edges in component
    size_t boundaryLoops{0};   ///< Number of boundary loops
    FaceId representativeFace; ///< One face from this component
    
    /// Check if component is valid (has at least one face)
    [[nodiscard]] bool valid() const noexcept { return faceCount > 0; }
    [[nodiscard]] explicit operator bool() const noexcept { return valid(); }
};

/// Result of component analysis
struct ComponentsResult {
    /// Number of connected components
    size_t numComponents{0};
    
    /// Per-face component ID (0 to numComponents-1)
    FaceMap<size_t> faceComponent;
    
    /// Per-vertex component ID (0 to numComponents-1, ~0 for isolated)
    VertMap<size_t> vertexComponent;
    
    /// Information about each component
    std::vector<ComponentInfo> components;
    
    /// Faces in each component (optional, computed if requested)
    std::vector<FaceBitSet> componentFaces;
    
    /// Check if analysis was successful
    [[nodiscard]] bool valid() const noexcept { return numComponents > 0; }
    
    /// Get the component containing a face
    [[nodiscard]] size_t getComponent(FaceId f) const {
        return faceComponent.empty() ? ~size_t(0) : faceComponent[f];
    }
    
    /// Get the largest component by face count
    [[nodiscard]] size_t largestComponent() const {
        if (components.empty()) return ~size_t(0);
        return std::max_element(components.begin(), components.end(),
            [](const auto& a, const auto& b) { return a.faceCount < b.faceCount; }
        ) - components.begin();
    }
    
    /// Get the smallest component by face count
    [[nodiscard]] size_t smallestComponent() const {
        if (components.empty()) return ~size_t(0);
        return std::min_element(components.begin(), components.end(),
            [](const auto& a, const auto& b) { return a.faceCount < b.faceCount; }
        ) - components.begin();
    }
    
    /// Get faces of a specific component as a BitSet
    [[nodiscard]] FaceBitSet getFaces(size_t componentId) const {
        if (!componentFaces.empty() && componentId < componentFaces.size()) {
            return componentFaces[componentId];
        }
        // Build on demand
        FaceBitSet result;
        result.resize(faceComponent.size());
        for (size_t i = 0; i < faceComponent.size(); ++i) {
            if (faceComponent[FaceId{static_cast<int>(i)}] == componentId) {
                result.set(FaceId{static_cast<int>(i)});
            }
        }
        return result;
    }
};

/// Settings for component analysis
struct ComponentSettings {
    /// Whether to compute per-vertex component assignments
    bool computeVertexComponents{true};
    
    /// Whether to store FaceBitSet for each component
    bool storeComponentFaces{false};
    
    /// Whether to count boundary loops per component
    bool countBoundaryLoops{false};
    
    /// Optional mask - only analyze faces in this set
    const FaceBitSet* faceMask{nullptr};
    
    /// Minimum component size (faces) to include in results
    size_t minFaceCount{0};
};

/// Compute connected components of a mesh using UnionFind
/// @param topology Mesh topology
/// @param settings Analysis settings
/// @return Component analysis results
[[nodiscard]] inline ComponentsResult computeComponents(
    const MeshTopology& topology,
    const ComponentSettings& settings = {}
) {
    ComponentsResult result;
    
    size_t numFaces = topology.faceCount();
    if (numFaces == 0) {
        return result;
    }
    
    // Create union-find structure for faces
    UnionFind<FaceId> uf(numFaces);
    
    // Union faces that share edges
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        
        // Skip masked faces
        if (settings.faceMask && !settings.faceMask->test(face)) {
            continue;
        }
        
        // Get the three edges of this face
        EdgeId edge = topology.faceHalfEdge(face);
        if (!edge.valid()) continue;
        
        for (int e = 0; e < 3; ++e) {
            // Get adjacent face via symmetric edge
            EdgeId symEdge = edge.sym();
            FaceId adjFace = topology.left(symEdge);
            
            if (adjFace.valid()) {
                // Skip if adjacent is masked
                if (!settings.faceMask || settings.faceMask->test(adjFace)) {
                    uf.unite(face, adjFace);
                }
            }
            
            edge = topology.next(edge);
        }
    }
    
    // Collect component representatives and count
    std::vector<FaceId> roots;
    std::unordered_map<int, size_t> rootToIndex;
    
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        if (settings.faceMask && !settings.faceMask->test(face)) {
            continue;
        }
        
        FaceId root = uf.find(face);
        if (rootToIndex.find(root.get()) == rootToIndex.end()) {
            rootToIndex[root.get()] = roots.size();
            roots.push_back(root);
        }
    }
    
    result.numComponents = roots.size();
    
    // Assign component indices to faces
    result.faceComponent.resize(numFaces, ~size_t(0));
    result.components.resize(result.numComponents);
    
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        if (settings.faceMask && !settings.faceMask->test(face)) {
            continue;
        }
        
        FaceId root = uf.find(face);
        size_t compIdx = rootToIndex[root.get()];
        result.faceComponent[face] = compIdx;
        result.components[compIdx].faceCount++;
        
        if (!result.components[compIdx].representativeFace.valid()) {
            result.components[compIdx].representativeFace = face;
        }
    }
    
    // Optionally store face bitsets
    if (settings.storeComponentFaces) {
        result.componentFaces.resize(result.numComponents);
        for (auto& bs : result.componentFaces) {
            bs.resize(numFaces);
        }
        
        for (size_t i = 0; i < numFaces; ++i) {
            FaceId face{static_cast<int>(i)};
            size_t compIdx = result.faceComponent[face];
            if (compIdx != ~size_t(0)) {
                result.componentFaces[compIdx].set(face);
            }
        }
    }
    
    // Compute vertex components if requested
    if (settings.computeVertexComponents) {
        size_t numVerts = topology.vertCount();
        result.vertexComponent.resize(numVerts, ~size_t(0));
        
        for (size_t i = 0; i < numFaces; ++i) {
            FaceId face{static_cast<int>(i)};
            size_t compIdx = result.faceComponent[face];
            if (compIdx == ~size_t(0)) continue;
            
            auto verts = topology.getTriVerts(face);
            for (int v = 0; v < 3; ++v) {
                if (verts[v].valid()) {
                    result.vertexComponent[verts[v]] = compIdx;
                    result.components[compIdx].vertexCount++;
                }
            }
        }
        
        // Remove duplicate vertex counts (vertices appear in multiple faces)
        for (auto& comp : result.components) {
            comp.vertexCount = 0;
        }
        for (size_t i = 0; i < numVerts; ++i) {
            size_t compIdx = result.vertexComponent[VertId{static_cast<int>(i)}];
            if (compIdx != ~size_t(0)) {
                result.components[compIdx].vertexCount++;
            }
        }
    }
    
    // Count boundary loops if requested
    if (settings.countBoundaryLoops) {
        // For each component, count boundary loops using Euler characteristic
        // V - E + F = 2 - 2g - b where g=genus, b=boundary loops
        // For a mesh with triangular faces: 3F = 2E_interior + E_boundary
        // This is a simplified version - just count boundary edges
        
        for (size_t c = 0; c < result.numComponents; ++c) {
            EdgeBitSet visited;
            visited.resize(topology.edgeCount());
            size_t boundaryCount = 0;
            
            // Find boundary edges in this component
            for (size_t i = 0; i < numFaces; ++i) {
                FaceId face{static_cast<int>(i)};
                if (result.faceComponent[face] != c) continue;
                
                EdgeId edge = topology.faceHalfEdge(face);
                for (int e = 0; e < 3 && edge.valid(); ++e) {
                    EdgeId sym = edge.sym();
                    if (!topology.left(sym).valid()) {
                        // This is a boundary edge
                        if (!visited.test(edge.undirected())) {
                            visited.set(edge.undirected());
                            boundaryCount++;
                        }
                    }
                    edge = topology.next(edge);
                }
            }
            
            // Count boundary loops by tracing
            visited.clear();
            visited.resize(topology.edgeCount());
            
            for (size_t i = 0; i < numFaces; ++i) {
                FaceId face{static_cast<int>(i)};
                if (result.faceComponent[face] != c) continue;
                
                EdgeId edge = topology.faceHalfEdge(face);
                for (int e = 0; e < 3 && edge.valid(); ++e) {
                    EdgeId sym = edge.sym();
                    if (!topology.left(sym).valid() && !visited.test(edge.undirected())) {
                        // Start tracing a new boundary loop
                        result.components[c].boundaryLoops++;
                        
                        EdgeId curr = edge;
                        while (curr.valid() && !visited.test(curr.undirected())) {
                            visited.set(curr.undirected());
                            // Move to next boundary edge
                            curr = topology.next(curr);
                            while (curr.valid() && topology.left(curr.sym()).valid()) {
                                curr = topology.next(curr.sym());
                            }
                            if (curr == edge) break;  // Completed loop
                        }
                    }
                    edge = topology.next(edge);
                }
            }
        }
    }
    
    // Filter by minimum face count
    if (settings.minFaceCount > 0) {
        std::vector<size_t> remap(result.numComponents, ~size_t(0));
        size_t newCount = 0;
        
        std::vector<ComponentInfo> newComponents;
        for (size_t c = 0; c < result.numComponents; ++c) {
            if (result.components[c].faceCount >= settings.minFaceCount) {
                remap[c] = newCount++;
                newComponents.push_back(result.components[c]);
            }
        }
        
        // Remap face and vertex components
        for (auto& fc : result.faceComponent) {
            if (fc != ~size_t(0)) {
                fc = remap[fc];
            }
        }
        for (auto& vc : result.vertexComponent) {
            if (vc != ~size_t(0)) {
                vc = remap[vc];
            }
        }
        
        result.numComponents = newCount;
        result.components = std::move(newComponents);
    }
    
    return result;
}

/// Get the largest connected component as a face set
[[nodiscard]] inline FaceBitSet getLargestComponent(
    const MeshTopology& topology
) {
    auto components = computeComponents(topology);
    if (components.numComponents == 0) {
        return {};
    }
    return components.getFaces(components.largestComponent());
}

/// Get all small components (face count < threshold) as a combined face set
[[nodiscard]] inline FaceBitSet getSmallComponents(
    const MeshTopology& topology,
    size_t maxFaceCount
) {
    auto components = computeComponents(topology);
    FaceBitSet result;
    result.resize(topology.faceCount());
    
    for (size_t c = 0; c < components.numComponents; ++c) {
        if (components.components[c].faceCount < maxFaceCount) {
            FaceBitSet compFaces = components.getFaces(c);
            for (size_t i = 0; i < compFaces.size(); ++i) {
                if (compFaces.test(FaceId{static_cast<int>(i)})) {
                    result.set(FaceId{static_cast<int>(i)});
                }
            }
        }
    }
    
    return result;
}

/// Check if mesh is a single connected component
[[nodiscard]] inline bool isSingleComponent(const MeshTopology& topology) {
    auto components = computeComponents(topology);
    return components.numComponents == 1;
}

/// Count connected components
[[nodiscard]] inline size_t countComponents(const MeshTopology& topology) {
    auto components = computeComponents(topology);
    return components.numComponents;
}

/// Separate mesh into individual components
/// Returns a vector of FaceBitSets, one per component, sorted by size (largest first)
[[nodiscard]] inline std::vector<FaceBitSet> separateComponents(
    const MeshTopology& topology
) {
    ComponentSettings settings;
    settings.storeComponentFaces = true;
    auto components = computeComponents(topology, settings);
    
    // Sort by face count (largest first)
    std::vector<std::pair<size_t, size_t>> indexed;
    for (size_t c = 0; c < components.numComponents; ++c) {
        indexed.emplace_back(components.components[c].faceCount, c);
    }
    std::sort(indexed.begin(), indexed.end(), std::greater<>());
    
    std::vector<FaceBitSet> result;
    result.reserve(components.numComponents);
    for (const auto& [count, idx] : indexed) {
        result.push_back(std::move(components.componentFaces[idx]));
    }
    
    return result;
}

/// Compute vertex connectivity components (vertices connected by edges)
/// This is different from face components - considers only edge connectivity
[[nodiscard]] inline VertMap<size_t> computeVertexComponents(
    const MeshTopology& topology
) {
    size_t numVerts = topology.vertCount();
    UnionFind<VertId> uf(numVerts);
    
    // Union vertices connected by edges
    size_t numEdges = topology.edgeCount();
    for (size_t i = 0; i < numEdges; i += 2) {  // Only even edges (undirected)
        EdgeId edge{static_cast<int>(i)};
        VertId v0 = topology.org(edge);
        VertId v1 = topology.dest(edge);
        if (v0.valid() && v1.valid()) {
            uf.unite(v0, v1);
        }
    }
    
    // Build component map
    std::unordered_map<int, size_t> rootToIndex;
    VertMap<size_t> result;
    result.resize(numVerts);
    
    for (size_t i = 0; i < numVerts; ++i) {
        VertId v{static_cast<int>(i)};
        VertId root = uf.find(v);
        if (rootToIndex.find(root.get()) == rootToIndex.end()) {
            rootToIndex[root.get()] = rootToIndex.size();
        }
        result[v] = rootToIndex[root.get()];
    }
    
    return result;
}

/// Find isolated vertices (vertices not connected to any face)
[[nodiscard]] inline VertBitSet findIsolatedVertices(
    const MeshTopology& topology
) {
    size_t numVerts = topology.vertCount();
    VertBitSet connected;
    connected.resize(numVerts);
    
    // Mark vertices that belong to faces
    size_t numFaces = topology.faceCount();
    for (size_t i = 0; i < numFaces; ++i) {
        auto verts = topology.getTriVerts(FaceId{static_cast<int>(i)});
        for (int v = 0; v < 3; ++v) {
            if (verts[v].valid()) {
                connected.set(verts[v]);
            }
        }
    }
    
    // Return inverse
    VertBitSet isolated;
    isolated.resize(numVerts);
    for (size_t i = 0; i < numVerts; ++i) {
        if (!connected.test(VertId{static_cast<int>(i)})) {
            isolated.set(VertId{static_cast<int>(i)});
        }
    }
    
    return isolated;
}

/// Find duplicate faces (faces with same vertices in any order)
[[nodiscard]] inline FaceBitSet findDuplicateFaces(
    const MeshTopology& topology
) {
    size_t numFaces = topology.faceCount();
    FaceBitSet duplicates;
    duplicates.resize(numFaces);
    
    // Hash function for sorted vertex triple
    auto hashTriple = [](const std::array<int, 3>& v) {
        return std::hash<int>{}(v[0]) ^ 
               (std::hash<int>{}(v[1]) << 11) ^ 
               (std::hash<int>{}(v[2]) << 22);
    };
    
    std::unordered_map<size_t, FaceId> seen;
    
    for (size_t i = 0; i < numFaces; ++i) {
        FaceId face{static_cast<int>(i)};
        auto verts = topology.getTriVerts(face);
        
        std::array<int, 3> sorted{verts[0].get(), verts[1].get(), verts[2].get()};
        std::sort(sorted.begin(), sorted.end());
        
        size_t hash = hashTriple(sorted);
        
        if (seen.find(hash) != seen.end()) {
            // Verify it's actually a duplicate (not just hash collision)
            auto otherVerts = topology.getTriVerts(seen[hash]);
            std::array<int, 3> otherSorted{otherVerts[0].get(), otherVerts[1].get(), otherVerts[2].get()};
            std::sort(otherSorted.begin(), otherSorted.end());
            
            if (sorted == otherSorted) {
                duplicates.set(face);
            }
        } else {
            seen[hash] = face;
        }
    }
    
    return duplicates;
}

} // namespace meshlib
