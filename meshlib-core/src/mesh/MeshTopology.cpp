/**
 * @file MeshTopology.cpp
 * @brief Implementation of half-edge mesh topology
 */

#include "meshlib/mesh/MeshTopology.h"
#include <algorithm>
#include <unordered_set>

namespace meshlib {

// ==================== Edge Creation ====================

EdgeId MeshTopology::makeEdge() {
    EdgeId e0{static_cast<int>(edges_.size())};
    EdgeId e1 = e0 + 1;
    
    // Create two half-edges pointing to each other
    edges_.push_back({e0, e0, VertId{}, FaceId{}});  // e0: prev=next=self (lone)
    edges_.push_back({e1, e1, VertId{}, FaceId{}});  // e1: prev=next=self (lone)
    
    return e0;
}

bool MeshTopology::isLoneEdge(EdgeId e) const {
    if (!e.valid() || static_cast<size_t>(e) >= edges_.size()) return true;
    // An edge is lone if next points to itself
    return edges_[e].next == e;
}

UndirectedEdgeId MeshTopology::lastNotLoneUndirectedEdge() const {
    for (size_t i = undirectedEdgeSize(); i > 0; --i) {
        UndirectedEdgeId ue{static_cast<int>(i - 1)};
        EdgeId e{ue};
        if (!isLoneEdge(e)) {
            return ue;
        }
    }
    return UndirectedEdgeId{};
}

void MeshTopology::excludeLoneEdges(UndirectedEdgeBitSet& edges) const {
    for (auto ue = edges.find_first(); ue.valid(); ue = edges.find_next(ue)) {
        EdgeId e{ue};
        if (isLoneEdge(e)) {
            edges.reset(ue);
        }
    }
}

size_t MeshTopology::computeNotLoneUndirectedEdges() const {
    size_t count = 0;
    for (size_t i = 0; i < undirectedEdgeSize(); ++i) {
        EdgeId e{static_cast<int>(i * 2)};
        if (!isLoneEdge(e)) {
            ++count;
        }
    }
    return count;
}

UndirectedEdgeBitSet MeshTopology::findNotLoneUndirectedEdges() const {
    UndirectedEdgeBitSet result(undirectedEdgeSize());
    for (size_t i = 0; i < undirectedEdgeSize(); ++i) {
        EdgeId e{static_cast<int>(i * 2)};
        if (!isLoneEdge(e)) {
            result.set(UndirectedEdgeId{static_cast<int>(i)});
        }
    }
    return result;
}

// ==================== Topology Modification ====================

void MeshTopology::setOrg(EdgeId e, VertId v) {
    if (!e.valid()) return;
    
    // Set org for all edges in the origin ring
    EdgeId curr = e;
    do {
        edges_[curr].org = v;
        curr = next(curr);
    } while (curr != e);
    
    // Update edgePerVertex
    if (v.valid()) {
        if (static_cast<size_t>(v) >= edgePerVertex_.size()) {
            edgePerVertex_.resizeWithReserve(static_cast<size_t>(v) + 1);
            validVerts_.resizeWithReserve(static_cast<size_t>(v) + 1);
        }
        
        if (!edgePerVertex_[v].valid()) {
            edgePerVertex_[v] = e;
            if (!validVerts_.test(v)) {
                validVerts_.set(v);
                ++numValidVerts_;
            }
        }
    }
}

void MeshTopology::setLeft(EdgeId e, FaceId f) {
    if (!e.valid()) return;
    
    // Set left for all edges in the left ring
    EdgeId curr = e;
    do {
        edges_[curr].left = f;
        // Move to next edge around the face (following the boundary)
        curr = next(curr.sym()).sym();
    } while (curr != e);
    
    // Update edgePerFace
    if (f.valid()) {
        if (static_cast<size_t>(f) >= edgePerFace_.size()) {
            edgePerFace_.resizeWithReserve(static_cast<size_t>(f) + 1);
            validFaces_.resizeWithReserve(static_cast<size_t>(f) + 1);
        }
        
        if (!edgePerFace_[f].valid()) {
            edgePerFace_[f] = e;
            if (!validFaces_.test(f)) {
                validFaces_.set(f);
                ++numValidFaces_;
            }
        }
    }
}

void MeshTopology::splice(EdgeId a, EdgeId b) {
    if (!a.valid() || !b.valid() || a == b) return;
    
    // Get the edges before a and b in their respective origin rings
    EdgeId aPrev = prev(a);
    EdgeId bPrev = prev(b);
    
    // Swap the next pointers
    edges_[aPrev].next = b;
    edges_[bPrev].next = a;
    
    // Update prev pointers
    edges_[a].prev = bPrev;
    edges_[b].prev = aPrev;
}

// ==================== Ring Queries ====================

bool MeshTopology::fromSameOriginRing(EdgeId a, EdgeId b) const {
    if (!a.valid() || !b.valid()) return false;
    
    EdgeId curr = a;
    do {
        if (curr == b) return true;
        curr = next(curr);
    } while (curr != a);
    
    return false;
}

bool MeshTopology::fromSameLeftRing(EdgeId a, EdgeId b) const {
    if (!a.valid() || !b.valid()) return false;
    
    EdgeId curr = a;
    do {
        if (curr == b) return true;
        curr = next(curr.sym()).sym();
    } while (curr != a);
    
    return false;
}

int MeshTopology::getOrgDegree(EdgeId e) const {
    if (!e.valid()) return 0;
    
    int degree = 0;
    EdgeId curr = e;
    do {
        ++degree;
        curr = next(curr);
    } while (curr != e);
    
    return degree;
}

int MeshTopology::getLeftDegree(EdgeId e) const {
    if (!e.valid()) return 0;
    
    int degree = 0;
    EdgeId curr = e;
    do {
        ++degree;
        curr = next(curr.sym()).sym();
    } while (curr != e);
    
    return degree;
}

bool MeshTopology::isLeftTri(EdgeId e) const {
    return getLeftDegree(e) == 3;
}

// ==================== Triangle Access ====================

void MeshTopology::getLeftTriVerts(EdgeId e, VertId& v0, VertId& v1, VertId& v2) const {
    v0 = org(e);
    v1 = dest(e);
    EdgeId e2 = next(e.sym()).sym();
    v2 = dest(e2);
}

std::vector<ThreeVertIds> MeshTopology::getAllTriVerts() const {
    std::vector<ThreeVertIds> result;
    result.reserve(numValidFaces_);
    
    for (FaceId f{0}; static_cast<size_t>(f) < faceSize(); ++f) {
        if (hasFace(f)) {
            result.push_back(getTriVerts(f));
        }
    }
    
    return result;
}

Triangulation MeshTopology::getTriangulation() const {
    Triangulation result(faceSize());
    
    for (FaceId f{0}; static_cast<size_t>(f) < faceSize(); ++f) {
        if (hasFace(f)) {
            result[f] = getTriVerts(f);
        }
    }
    
    return result;
}

void MeshTopology::getLeftTriEdges(EdgeId e0, EdgeId& e1, EdgeId& e2) const {
    e1 = next(e0.sym()).sym();
    e2 = next(e1.sym()).sym();
}

// ==================== Vertex Management ====================

VertId MeshTopology::lastValidVert() const {
    return validVerts_.find_last();
}

void MeshTopology::vertResize(size_t newSize) {
    edgePerVertex_.resize(newSize);
    validVerts_.resize(newSize);
}

void MeshTopology::vertResizeWithReserve(size_t newSize) {
    edgePerVertex_.resizeWithReserve(newSize);
    validVerts_.resizeWithReserve(newSize);
}

// ==================== Face Management ====================

FaceId MeshTopology::lastValidFace() const {
    return validFaces_.find_last();
}

void MeshTopology::deleteFace(FaceId f, const UndirectedEdgeBitSet* keepEdges) {
    if (!hasFace(f)) return;
    
    EdgeId e = edgeWithLeft(f);
    if (!e.valid()) return;
    
    // Clear face from all edges in the ring
    EdgeId curr = e;
    do {
        edges_[curr].left = FaceId{};
        curr = next(curr.sym()).sym();
    } while (curr != e);
    
    // Mark face as invalid
    validFaces_.reset(f);
    edgePerFace_[f] = EdgeId{};
    --numValidFaces_;
}

void MeshTopology::deleteFaces(const FaceBitSet& fs, const UndirectedEdgeBitSet* keepEdges) {
    for (FaceId f = fs.find_first(); f.valid(); f = fs.find_next(f)) {
        deleteFace(f, keepEdges);
    }
}

void MeshTopology::faceResize(size_t newSize) {
    edgePerFace_.resize(newSize);
    validFaces_.resize(newSize);
}

void MeshTopology::faceResizeWithReserve(size_t newSize) {
    edgePerFace_.resizeWithReserve(newSize);
    validFaces_.resizeWithReserve(newSize);
}

// ==================== Boundary Queries ====================

bool MeshTopology::isBdEdge(EdgeId e, const FaceBitSet* region) const {
    if (!e.valid()) return false;
    
    FaceId lf = left(e);
    FaceId rf = right(e);
    
    if (region) {
        bool leftIn = contains(region, lf);
        bool rightIn = contains(region, rf);
        return leftIn != rightIn;  // Boundary if exactly one side is in region
    } else {
        return !lf.valid() || !rf.valid();  // Boundary if either side has no face
    }
}

bool MeshTopology::isLeftBdFace(EdgeId e, const FaceBitSet* region) const {
    return contains(region, left(e)) && bdEdgeSameLeft(e, region).valid();
}

EdgeId MeshTopology::bdEdgeSameLeft(EdgeId e, const FaceBitSet* region) const {
    if (!e.valid()) return EdgeId{};
    
    EdgeId curr = e;
    do {
        if (isBdEdge(curr, region)) {
            return curr;
        }
        curr = next(curr.sym()).sym();
    } while (curr != e);
    
    return EdgeId{};
}

EdgeId MeshTopology::bdEdgeSameOrigin(EdgeId e, const FaceBitSet* region) const {
    if (!e.valid()) return EdgeId{};
    
    EdgeId curr = e;
    do {
        if (isBdEdge(curr, region)) {
            return curr;
        }
        curr = next(curr);
    } while (curr != e);
    
    return EdgeId{};
}

VertBitSet MeshTopology::findBdVerts(const FaceBitSet* region) const {
    VertBitSet result(vertSize());
    
    for (VertId v{0}; static_cast<size_t>(v) < vertSize(); ++v) {
        if (hasVert(v) && isBdVertex(v, region)) {
            result.set(v);
        }
    }
    
    return result;
}

FaceBitSet MeshTopology::findBdFaces(const FaceBitSet* region) const {
    FaceBitSet result(faceSize());
    
    for (FaceId f{0}; static_cast<size_t>(f) < faceSize(); ++f) {
        if (hasFace(f)) {
            EdgeId e = edgeWithLeft(f);
            if (isLeftBdFace(e, region)) {
                result.set(f);
            }
        }
    }
    
    return result;
}

EdgeBitSet MeshTopology::findLeftBdEdges(const FaceBitSet* region) const {
    EdgeBitSet result(edgeSize());
    
    for (size_t i = 0; i < edgeSize(); ++i) {
        EdgeId e{static_cast<int>(i)};
        if (!isLoneEdge(e) && !left(e).valid()) {
            if (!region || contains(region, right(e))) {
                result.set(e);
            }
        }
    }
    
    return result;
}

// ==================== Hole Detection ====================

bool MeshTopology::isClosed(const FaceBitSet* region) const {
    for (size_t i = 0; i < edgeSize(); ++i) {
        EdgeId e{static_cast<int>(i)};
        if (!isLoneEdge(e) && isBdEdge(e, region)) {
            return false;
        }
    }
    return true;
}

std::vector<EdgeId> MeshTopology::findHoleRepresentiveEdges(const FaceBitSet* region) const {
    std::vector<EdgeId> result;
    EdgeBitSet visited(edgeSize());
    
    for (size_t i = 0; i < edgeSize(); ++i) {
        EdgeId e{static_cast<int>(i)};
        if (isLoneEdge(e) || visited.test(e)) continue;
        
        // Check if this is a boundary edge (no left face)
        if (!left(e).valid() && (region == nullptr || contains(region, right(e)))) {
            // Found a hole, mark all edges in this hole as visited
            EdgeId curr = e;
            EdgeId minEdge = e;
            do {
                visited.set(curr);
                if (curr < minEdge) minEdge = curr;
                
                // Move to next boundary edge
                EdgeId next_e = curr.sym();
                while (left(next_e).valid()) {
                    next_e = next(next_e);
                    if (next_e == curr.sym()) break;
                }
                curr = next_e;
            } while (curr != e && curr.valid());
            
            result.push_back(minEdge);
        }
    }
    
    return result;
}

int MeshTopology::findNumHoles(EdgeBitSet* holeRepresentativeEdges) const {
    auto holes = findHoleRepresentiveEdges();
    
    if (holeRepresentativeEdges) {
        holeRepresentativeEdges->resize(edgeSize());
        for (EdgeId e : holes) {
            holeRepresentativeEdges->set(e);
        }
    }
    
    return static_cast<int>(holes.size());
}

// ==================== Edge Loops ====================

EdgeLoop MeshTopology::getLeftRing(EdgeId e) const {
    EdgeLoop result;
    if (!e.valid()) return result;
    
    EdgeId curr = e;
    do {
        result.push_back(curr);
        curr = next(curr.sym()).sym();
    } while (curr != e);
    
    return result;
}

std::vector<EdgeLoop> MeshTopology::getLeftRings(const std::vector<EdgeId>& es) const {
    std::vector<EdgeLoop> result;
    FaceBitSet visitedFaces(faceSize());
    
    for (EdgeId e : es) {
        FaceId f = left(e);
        if (f.valid() && !visitedFaces.test(f)) {
            visitedFaces.set(f);
            result.push_back(getLeftRing(e));
        }
    }
    
    return result;
}

// ==================== Edge Finding ====================

EdgeId MeshTopology::findEdge(VertId o, VertId d) const {
    EdgeId e = edgeWithOrg(o);
    if (!e.valid()) return EdgeId{};
    
    EdgeId curr = e;
    do {
        if (dest(curr) == d) {
            return curr;
        }
        curr = next(curr);
    } while (curr != e);
    
    return EdgeId{};
}

EdgeId MeshTopology::sharedEdge(FaceId l, FaceId r) const {
    if (!l.valid() || !r.valid()) return EdgeId{};
    
    EdgeId e = edgeWithLeft(l);
    if (!e.valid()) return EdgeId{};
    
    EdgeId curr = e;
    do {
        if (right(curr) == r) {
            return curr;
        }
        curr = next(curr.sym()).sym();
    } while (curr != e);
    
    return EdgeId{};
}

EdgeId MeshTopology::sharedVertInOrg(EdgeId a, EdgeId b) const {
    if (!a.valid() || !b.valid()) return EdgeId{};
    
    // Check if they share origin
    if (fromSameOriginRing(a, b)) {
        return a;
    }
    
    // Check if dest(a) == org(b)
    if (fromSameOriginRing(a.sym(), b)) {
        return a.sym();
    }
    
    // Check if org(a) == dest(b)
    if (fromSameOriginRing(a, b.sym())) {
        return a;
    }
    
    // Check if dest(a) == dest(b)
    if (fromSameOriginRing(a.sym(), b.sym())) {
        return a.sym();
    }
    
    return EdgeId{};
}

FaceId MeshTopology::sharedFace(EdgeId a, EdgeId b) const {
    if (!a.valid() || !b.valid()) return FaceId{};
    
    // Check if left(a) contains b
    FaceId lf = left(a);
    if (lf.valid() && fromSameLeftRing(a, b)) {
        return lf;
    }
    
    // Check if right(a) contains b
    FaceId rf = right(a);
    if (rf.valid() && fromSameLeftRing(a.sym(), b)) {
        return rf;
    }
    
    return FaceId{};
}

// ==================== Utility ====================

size_t MeshTopology::heapBytes() const {
    return edges_.capacity() * sizeof(HalfEdge) +
           edgePerVertex_.heapBytes() +
           edgePerFace_.heapBytes() +
           validVerts_.heapBytes() +
           validFaces_.heapBytes();
}

void MeshTopology::shrinkToFit() {
    edges_.shrink_to_fit();
    edgePerVertex_.shrink_to_fit();
    edgePerFace_.shrink_to_fit();
    validVerts_.shrink_to_fit();
    validFaces_.shrink_to_fit();
}

void MeshTopology::pack() {
    // TODO: Implement packing to remove gaps in indices
    // This is complex and requires remapping all references
}

} // namespace meshlib
