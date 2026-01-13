#pragma once

/**
 * @file MeshTopology.h
 * @brief Half-edge mesh topology data structure
 * 
 * Industrial-strength port of MeshLib's MRMeshTopology.h providing:
 * - Half-edge connectivity (next, prev, org, dest, left, right)
 * - Efficient edge/vertex/face queries
 * - Boundary detection
 * - Topology modification operations
 * 
 * This is the foundation for all advanced mesh operations.
 */

#include "meshlib/config.h"
#include "../core/Id.h"
#include "../core/BitSet.h"
#include "../core/IdVector.h"
#include "../core/Vector.h"
#include <vector>
#include <functional>
#include <array>

namespace meshlib {

// Forward declarations
struct MeshTriPoint;

/// Edge path = sequence of edges
using EdgePath = std::vector<EdgeId>;

/// Edge loop = closed sequence of edges
using EdgeLoop = std::vector<EdgeId>;

/// Multiple edge loops
using EdgeLoops = std::vector<EdgeLoop>;

/// Triangulation = collection of triangles as vertex id triples
using Triangulation = std::vector<ThreeVertIds>;

/**
 * @brief Half-edge mesh topology
 * 
 * Stores the connectivity of a mesh using a half-edge data structure.
 * Each edge is represented as two directed half-edges (sym() of each other).
 * 
 * Key relationships:
 * - next(e): next half-edge around the origin vertex (counter-clockwise)
 * - prev(e): previous half-edge around the origin vertex (clockwise)
 * - org(e): origin vertex of the half-edge
 * - dest(e): destination vertex (= org(sym(e)))
 * - left(e): face to the left of the half-edge
 * - right(e): face to the right (= left(sym(e)))
 */
class MESHLIB_API MeshTopology {
public:
    // ==================== Edge Creation ====================
    
    /// Creates a new lone edge not connected to anything
    EdgeId makeEdge();
    
    /// Check if edge is lone (not connected to anything)
    [[nodiscard]] bool isLoneEdge(EdgeId e) const;
    
    /// Returns last non-lone undirected edge id
    [[nodiscard]] UndirectedEdgeId lastNotLoneUndirectedEdge() const;
    
    /// Returns last non-lone edge id
    [[nodiscard]] EdgeId lastNotLoneEdge() const {
        auto ue = lastNotLoneUndirectedEdge();
        return ue.valid() ? EdgeId(ue) + 1 : EdgeId{};
    }
    
    /// Remove lone edges from the set
    void excludeLoneEdges(UndirectedEdgeBitSet& edges) const;
    
    // ==================== Size Queries ====================
    
    /// Number of half-edge records (including lone ones)
    [[nodiscard]] size_t edgeSize() const { return edges_.size(); }
    
    /// Capacity of edge storage
    [[nodiscard]] size_t edgeCapacity() const { return edges_.capacity(); }
    
    /// Number of undirected edges (half-edges / 2)
    [[nodiscard]] size_t undirectedEdgeSize() const { return edges_.size() >> 1; }
    [[nodiscard]] size_t undirectedEdgeCapacity() const { return edges_.capacity() >> 1; }
    
    /// Count non-lone undirected edges
    [[nodiscard]] size_t computeNotLoneUndirectedEdges() const;
    
    /// Find all non-lone undirected edges
    [[nodiscard]] UndirectedEdgeBitSet findNotLoneUndirectedEdges() const;
    
    /// Reserve edge storage
    void edgeReserve(size_t newCapacity) { edges_.reserve(newCapacity); }
    
    /// Check if edge exists and is valid
    [[nodiscard]] bool hasEdge(EdgeId e) const {
        return e.valid() && static_cast<size_t>(e) < edgeSize() && !isLoneEdge(e);
    }
    
    // ==================== Basic Topology Access ====================
    
    /// Next half-edge around origin (counter-clockwise)
    [[nodiscard]] EdgeId next(EdgeId e) const {
        assert(e.valid() && static_cast<size_t>(e) < edges_.size());
        return edges_[e].next;
    }
    
    /// Previous half-edge around origin (clockwise)
    [[nodiscard]] EdgeId prev(EdgeId e) const {
        assert(e.valid() && static_cast<size_t>(e) < edges_.size());
        return edges_[e].prev;
    }
    
    /// Origin vertex of half-edge
    [[nodiscard]] VertId org(EdgeId e) const {
        assert(e.valid() && static_cast<size_t>(e) < edges_.size());
        return edges_[e].org;
    }
    
    /// Destination vertex of half-edge
    [[nodiscard]] VertId dest(EdgeId e) const {
        return org(e.sym());
    }
    
    /// Face to the left of half-edge
    [[nodiscard]] FaceId left(EdgeId e) const {
        assert(e.valid() && static_cast<size_t>(e) < edges_.size());
        return edges_[e].left;
    }
    
    /// Face to the right of half-edge
    [[nodiscard]] FaceId right(EdgeId e) const {
        return left(e.sym());
    }
    
    // ==================== Topology Modification ====================
    
    /// Set origin of the entire origin ring
    void setOrg(EdgeId e, VertId v);
    
    /// Set left face of the entire left ring
    void setLeft(EdgeId e, FaceId f);
    
    /// Splice two half-edges (combines or splits rings)
    void splice(EdgeId a, EdgeId b);
    
    // ==================== Ring Queries ====================
    
    /// Check if two edges share the same origin ring
    [[nodiscard]] bool fromSameOriginRing(EdgeId a, EdgeId b) const;
    
    /// Check if two edges share the same left face ring
    [[nodiscard]] bool fromSameLeftRing(EdgeId a, EdgeId b) const;
    
    /// Number of edges around origin vertex
    [[nodiscard]] int getOrgDegree(EdgeId e) const;
    
    /// Number of edges around vertex
    [[nodiscard]] int getVertDegree(VertId v) const {
        return getOrgDegree(edgeWithOrg(v));
    }
    
    /// Number of edges around left face
    [[nodiscard]] int getLeftDegree(EdgeId e) const;
    
    /// Number of edges around face
    [[nodiscard]] int getFaceDegree(FaceId f) const {
        return getLeftDegree(edgeWithLeft(f));
    }
    
    /// Check if left face is a triangle
    [[nodiscard]] bool isLeftTri(EdgeId e) const;
    
    // ==================== Triangle Access ====================
    
    /// Get vertices of triangular face (counter-clockwise)
    void getTriVerts(FaceId f, VertId& v0, VertId& v1, VertId& v2) const {
        getLeftTriVerts(edgeWithLeft(f), v0, v1, v2);
    }
    
    void getTriVerts(FaceId f, ThreeVertIds& v) const {
        getLeftTriVerts(edgeWithLeft(f), v);
    }
    
    [[nodiscard]] ThreeVertIds getTriVerts(FaceId f) const {
        return getLeftTriVerts(edgeWithLeft(f));
    }
    
    /// Get vertices of left triangle (v0 = org(e), v1 = dest(e))
    void getLeftTriVerts(EdgeId e, VertId& v0, VertId& v1, VertId& v2) const;
    
    void getLeftTriVerts(EdgeId e, ThreeVertIds& v) const {
        getLeftTriVerts(e, v[0], v[1], v[2]);
    }
    
    [[nodiscard]] ThreeVertIds getLeftTriVerts(EdgeId e) const {
        ThreeVertIds v;
        getLeftTriVerts(e, v[0], v[1], v[2]);
        return v;
    }
    
    /// Get all valid triangles as vertex id triples
    [[nodiscard]] std::vector<ThreeVertIds> getAllTriVerts() const;
    
    /// Get triangulation (face-indexed)
    [[nodiscard]] Triangulation getTriangulation() const;
    
    /// Get edges of left triangle
    void getLeftTriEdges(EdgeId e0, EdgeId& e1, EdgeId& e2) const;
    
    // ==================== Vertex Management ====================
    
    /// Get edge-per-vertex array
    [[nodiscard]] const IdVector<EdgeId, VertId>& edgePerVertex() const {
        return edgePerVertex_;
    }
    
    /// Get an edge with given origin vertex
    [[nodiscard]] EdgeId edgeWithOrg(VertId v) const {
        return v.valid() && static_cast<size_t>(v) < edgePerVertex_.size() 
            ? edgePerVertex_[v] : EdgeId{};
    }
    
    /// Check if vertex exists
    [[nodiscard]] bool hasVert(VertId v) const {
        return validVerts_.test(v);
    }
    
    /// Number of valid vertices
    [[nodiscard]] int numValidVerts() const { return numValidVerts_; }
    
    /// Last valid vertex id
    [[nodiscard]] VertId lastValidVert() const;
    
    /// Add new vertex id (not connected to any edge yet)
    [[nodiscard]] VertId addVertId() {
        edgePerVertex_.emplace_back();
        validVerts_.push_back(false);
        return edgePerVertex_.backId();
    }
    
    /// Resize vertex storage
    void vertResize(size_t newSize);
    void vertResizeWithReserve(size_t newSize);
    void vertReserve(size_t newCapacity) {
        edgePerVertex_.reserve(newCapacity);
        validVerts_.reserve(newCapacity);
    }
    
    [[nodiscard]] size_t vertSize() const { return edgePerVertex_.size(); }
    [[nodiscard]] size_t vertCapacity() const { return edgePerVertex_.capacity(); }
    
    /// Get set of valid vertices
    [[nodiscard]] const VertBitSet& getValidVerts() const { return validVerts_; }
    
    /// Get vertices in region (or all valid if region is null)
    [[nodiscard]] const VertBitSet& getVertIds(const VertBitSet* region) const {
        return region ? *region : validVerts_;
    }
    
    // ==================== Face Management ====================
    
    /// Get edge-per-face array
    [[nodiscard]] const IdVector<EdgeId, FaceId>& edgePerFace() const {
        return edgePerFace_;
    }
    
    /// Get an edge with given left face
    [[nodiscard]] EdgeId edgeWithLeft(FaceId f) const {
        return f.valid() && static_cast<size_t>(f) < edgePerFace_.size()
            ? edgePerFace_[f] : EdgeId{};
    }
    
    /// Check if face exists
    [[nodiscard]] bool hasFace(FaceId f) const {
        return validFaces_.test(f);
    }
    
    /// Number of valid faces
    [[nodiscard]] int numValidFaces() const { return numValidFaces_; }
    
    /// Last valid face id
    [[nodiscard]] FaceId lastValidFace() const;
    
    /// Add new face id (not connected to any edge yet)
    [[nodiscard]] FaceId addFaceId() {
        edgePerFace_.emplace_back();
        validFaces_.push_back(false);
        return edgePerFace_.backId();
    }
    
    /// Delete face and optionally its unshared edges/vertices
    void deleteFace(FaceId f, const UndirectedEdgeBitSet* keepEdges = nullptr);
    
    /// Delete multiple faces
    void deleteFaces(const FaceBitSet& fs, const UndirectedEdgeBitSet* keepEdges = nullptr);
    
    /// Resize face storage
    void faceResize(size_t newSize);
    void faceResizeWithReserve(size_t newSize);
    void faceReserve(size_t newCapacity) {
        edgePerFace_.reserve(newCapacity);
        validFaces_.reserve(newCapacity);
    }
    
    [[nodiscard]] size_t faceSize() const { return edgePerFace_.size(); }
    [[nodiscard]] size_t faceCapacity() const { return edgePerFace_.capacity(); }
    
    /// Get set of valid faces
    [[nodiscard]] const FaceBitSet& getValidFaces() const { return validFaces_; }
    
    /// Get faces in region (or all valid if region is null)
    [[nodiscard]] const FaceBitSet& getFaceIds(const FaceBitSet* region) const {
        return region ? *region : validFaces_;
    }
    
    // ==================== Boundary Queries ====================
    
    /// Check if edge is on boundary (no face on one side)
    [[nodiscard]] bool isBdEdge(EdgeId e, const FaceBitSet* region = nullptr) const;
    
    /// Check if left face is on boundary
    [[nodiscard]] bool isLeftBdFace(EdgeId e, const FaceBitSet* region = nullptr) const;
    
    /// Find boundary edge starting from e with same left face
    [[nodiscard]] EdgeId bdEdgeSameLeft(EdgeId e, const FaceBitSet* region = nullptr) const;
    
    /// Find boundary edge starting from e with same origin
    [[nodiscard]] EdgeId bdEdgeSameOrigin(EdgeId e, const FaceBitSet* region = nullptr) const;
    
    /// Check if vertex is on boundary
    [[nodiscard]] bool isBdVertex(VertId v, const FaceBitSet* region = nullptr) const {
        return bdEdgeWithOrigin(v, region).valid();
    }
    
    /// Get boundary edge with given origin
    [[nodiscard]] EdgeId bdEdgeWithOrigin(VertId v, const FaceBitSet* region = nullptr) const {
        return bdEdgeSameOrigin(edgeWithOrg(v), region);
    }
    
    /// Find all boundary vertices
    [[nodiscard]] VertBitSet findBdVerts(const FaceBitSet* region = nullptr) const;
    
    /// Find all boundary faces
    [[nodiscard]] FaceBitSet findBdFaces(const FaceBitSet* region = nullptr) const;
    
    /// Find all boundary edges (left edge has no face)
    [[nodiscard]] EdgeBitSet findLeftBdEdges(const FaceBitSet* region = nullptr) const;
    
    // ==================== Hole Detection ====================
    
    /// Check if mesh (or region) is closed (no boundary)
    [[nodiscard]] bool isClosed(const FaceBitSet* region = nullptr) const;
    
    /// Find representative edges for each hole
    [[nodiscard]] std::vector<EdgeId> findHoleRepresentiveEdges(const FaceBitSet* region = nullptr) const;
    
    /// Count number of holes
    [[nodiscard]] int findNumHoles(EdgeBitSet* holeRepresentativeEdges = nullptr) const;
    
    // ==================== Edge Loops ====================
    
    /// Get edge loop around left face
    [[nodiscard]] EdgeLoop getLeftRing(EdgeId e) const;
    
    /// Get edge loops for multiple edges
    [[nodiscard]] std::vector<EdgeLoop> getLeftRings(const std::vector<EdgeId>& es) const;
    
    // ==================== Edge Finding ====================
    
    /// Find edge from o to d, or invalid if not found
    [[nodiscard]] EdgeId findEdge(VertId o, VertId d) const;
    
    /// Find shared edge between two faces
    [[nodiscard]] EdgeId sharedEdge(FaceId l, FaceId r) const;
    
    /// Find shared vertex between two edges
    [[nodiscard]] EdgeId sharedVertInOrg(EdgeId a, EdgeId b) const;
    
    /// Find shared face between two edges
    [[nodiscard]] FaceId sharedFace(EdgeId a, EdgeId b) const;
    
    // ==================== Utility ====================
    
    /// Heap memory usage
    [[nodiscard]] size_t heapBytes() const;
    
    /// Shrink all storage
    void shrinkToFit();
    
    /// Pack the topology removing gaps
    void pack();
    
private:
    /// Half-edge record
    struct HalfEdge {
        EdgeId next;  ///< Next half-edge around origin
        EdgeId prev;  ///< Previous half-edge around origin
        VertId org;   ///< Origin vertex
        FaceId left;  ///< Face to the left
    };
    
    std::vector<HalfEdge> edges_;
    IdVector<EdgeId, VertId> edgePerVertex_;
    IdVector<EdgeId, FaceId> edgePerFace_;
    
    VertBitSet validVerts_;
    FaceBitSet validFaces_;
    
    int numValidVerts_ = 0;
    int numValidFaces_ = 0;
};

// ==================== Iteration Helpers ====================

/**
 * @brief Iterate over all edges around a vertex origin
 * 
 * Usage:
 *   for (EdgeId e : orgRing(topology, startEdge)) { ... }
 */
class OrgRingIterator {
public:
    class Iterator {
    public:
        Iterator(const MeshTopology* t, EdgeId e, bool end) 
            : topology_(t), edge_(e), start_(e), ended_(end) {}
        
        EdgeId operator*() const { return edge_; }
        
        Iterator& operator++() {
            edge_ = topology_->next(edge_);
            if (edge_ == start_) ended_ = true;
            return *this;
        }
        
        bool operator!=(const Iterator& other) const {
            return ended_ != other.ended_;
        }
        
    private:
        const MeshTopology* topology_;
        EdgeId edge_;
        EdgeId start_;
        bool ended_;
    };
    
    OrgRingIterator(const MeshTopology& t, EdgeId e) : topology_(&t), edge_(e) {}
    
    Iterator begin() const { return Iterator(topology_, edge_, !edge_.valid()); }
    Iterator end() const { return Iterator(topology_, edge_, true); }
    
private:
    const MeshTopology* topology_;
    EdgeId edge_;
};

inline OrgRingIterator orgRing(const MeshTopology& t, EdgeId e) {
    return OrgRingIterator(t, e);
}

/**
 * @brief Iterate over all edges around a left face
 */
class LeftRingIterator {
public:
    class Iterator {
    public:
        Iterator(const MeshTopology* t, EdgeId e, bool end) 
            : topology_(t), edge_(e), start_(e), ended_(end) {}
        
        EdgeId operator*() const { return edge_; }
        
        Iterator& operator++() {
            edge_ = topology_->next(edge_.sym()).sym();
            if (edge_ == start_) ended_ = true;
            return *this;
        }
        
        bool operator!=(const Iterator& other) const {
            return ended_ != other.ended_;
        }
        
    private:
        const MeshTopology* topology_;
        EdgeId edge_;
        EdgeId start_;
        bool ended_;
    };
    
    LeftRingIterator(const MeshTopology& t, EdgeId e) : topology_(&t), edge_(e) {}
    
    Iterator begin() const { return Iterator(topology_, edge_, !edge_.valid()); }
    Iterator end() const { return Iterator(topology_, edge_, true); }
    
private:
    const MeshTopology* topology_;
    EdgeId edge_;
};

inline LeftRingIterator leftRing(const MeshTopology& t, EdgeId e) {
    return LeftRingIterator(t, e);
}

} // namespace meshlib
