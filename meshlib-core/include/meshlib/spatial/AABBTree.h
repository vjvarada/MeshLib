#pragma once

/**
 * @file AABBTree.h
 * @brief Axis-Aligned Bounding Box Tree for spatial queries
 * 
 * Industrial-strength port of MeshLib's MRAABBTree.h providing O(log n)
 * spatial queries for meshes, point clouds, and polylines.
 * 
 * Key features:
 * - Fast ray-mesh intersection
 * - Fast closest point queries
 * - Fast collision detection
 * - Efficient memory layout
 */

#include "meshlib/config.h"
#include "AABBTreeNode.h"
#include "../core/IdVector.h"
#include "../core/BitSet.h"
#include "../core/Vector.h"
#include "../core/Box.h"
#include <vector>
#include <functional>

namespace meshlib {

// Forward declarations
class Mesh;
class PointCloud;

// ==================== Base AABB Tree ====================

/**
 * @brief Base class for AABB trees
 * 
 * @tparam T Tree traits (defines leaf type and bounding box type)
 */
template <typename T>
class AABBTreeBase {
public:
    using Traits = T;
    using Node = AABBTreeNode<T>;
    using NodeVec = IdVector<Node, NodeId>;
    using LeafTag = typename T::LeafTag;
    using LeafId = typename T::LeafId;
    using LeafBitSet = TaggedBitSet<LeafTag>;
    using BoxT = typename T::BoxT;
    
    /// Access all nodes
    [[nodiscard]] const NodeVec& nodes() const { return nodes_; }
    
    /// Access specific node
    [[nodiscard]] const Node& operator[](NodeId nid) const { return nodes_[nid]; }
    
    /// Root node id (always 0)
    [[nodiscard]] static NodeId rootNodeId() { return NodeId{0}; }
    
    /// Get bounding box of entire tree
    [[nodiscard]] BoxT getBoundingBox() const {
        return nodes_.empty() ? BoxT{} : nodes_[rootNodeId()].box;
    }
    
    /// Heap memory usage
    [[nodiscard]] size_t heapBytes() const { return nodes_.heapBytes(); }
    
    /// Number of leaves
    [[nodiscard]] size_t numLeaves() const {
        return nodes_.empty() ? 0 : (nodes_.size() + 1) / 2;
    }
    
    /// Check if tree is empty
    [[nodiscard]] bool empty() const { return nodes_.empty(); }
    
    /// Get subtrees for parallel processing
    [[nodiscard]] std::vector<NodeId> getSubtrees(int minNum) const;
    
    /// Get all leaves in a subtree
    [[nodiscard]] LeafBitSet getSubtreeLeaves(NodeId subtreeRoot) const;
    
protected:
    NodeVec nodes_;
};

// ==================== Mesh AABB Tree ====================

/**
 * @brief AABB tree for mesh faces
 * 
 * Provides O(log n) queries for:
 * - Ray-mesh intersection
 * - Closest point on mesh
 * - Triangle-triangle collision
 */
class MESHLIB_API AABBTree : public AABBTreeBase<FaceTreeTraits3> {
public:
    /// Build tree from mesh
    explicit AABBTree(const Mesh& mesh);
    
    /// Default constructor (empty tree)
    AABBTree() = default;
    
    // ==================== Ray Queries ====================
    
    struct RayIntersection {
        FaceId faceId;          ///< Hit face
        float t = -1.0f;        ///< Distance along ray (negative = no hit)
        Vector3f point;         ///< Intersection point
        Vector2f barycentric;   ///< Barycentric coordinates in triangle
        
        [[nodiscard]] bool valid() const { return t >= 0.0f; }
    };
    
    /// Find first ray-mesh intersection
    [[nodiscard]] RayIntersection findRayIntersection(
        const Vector3f& origin,
        const Vector3f& direction,
        float maxDistance = std::numeric_limits<float>::max()) const;
    
    /// Check if ray hits mesh (faster than finding exact intersection)
    [[nodiscard]] bool rayHits(
        const Vector3f& origin,
        const Vector3f& direction,
        float maxDistance = std::numeric_limits<float>::max()) const;
    
    // ==================== Closest Point Queries ====================
    
    struct ClosestPoint {
        FaceId faceId;          ///< Closest face
        Vector3f point;         ///< Closest point on mesh
        float distSq = -1.0f;   ///< Squared distance (negative = not found)
        Vector2f barycentric;   ///< Barycentric coordinates
        
        [[nodiscard]] bool valid() const { return distSq >= 0.0f; }
        [[nodiscard]] float distance() const { return distSq >= 0 ? std::sqrt(distSq) : -1.0f; }
    };
    
    /// Find closest point on mesh to query point
    [[nodiscard]] ClosestPoint findClosestPoint(
        const Vector3f& point,
        float maxDistSq = std::numeric_limits<float>::max()) const;
    
    // ==================== Collision Queries ====================
    
    /// Find all triangles intersecting a box
    [[nodiscard]] FaceBitSet findTrianglesInBox(const Box3f& box) const;
    
    /// Find all triangles within distance of a point
    [[nodiscard]] FaceBitSet findTrianglesInBall(
        const Vector3f& center, float radius) const;
    
private:
    const Mesh* mesh_ = nullptr;
    
    void buildTree(const Mesh& mesh);
    
    void buildSubtree(
        NodeId nodeId,
        std::vector<FaceId>& faces,
        size_t begin, size_t end,
        const std::vector<Box3f>& boxes,
        const std::vector<Vector3f>& centers);
};

// ==================== Point Cloud AABB Tree ====================

/**
 * @brief AABB tree for point clouds
 * 
 * Specialized for point-based queries where leaves are individual points.
 */
class MESHLIB_API AABBTreePoints {
public:
    using BoxT = Box3f;
    
    /// Build tree from point cloud
    explicit AABBTreePoints(const PointCloud& points);
    
    /// Build tree from point positions
    explicit AABBTreePoints(const std::vector<Vector3f>& points);
    
    /// Default constructor (empty tree)
    AABBTreePoints() = default;
    
    /// Node structure for points
    struct Node {
        Box3f box;
        NodeId l, r;
        
        [[nodiscard]] bool leaf() const { return !r.valid(); }
        [[nodiscard]] VertId pointId() const { 
            assert(leaf()); 
            return VertId{static_cast<int>(l)}; 
        }
    };
    
    /// Access nodes
    [[nodiscard]] const IdVector<Node, NodeId>& nodes() const { return nodes_; }
    [[nodiscard]] const Node& operator[](NodeId nid) const { return nodes_[nid]; }
    
    [[nodiscard]] static NodeId rootNodeId() { return NodeId{0}; }
    [[nodiscard]] Box3f getBoundingBox() const {
        return nodes_.empty() ? Box3f{} : nodes_[rootNodeId()].box;
    }
    
    [[nodiscard]] bool empty() const { return nodes_.empty(); }
    [[nodiscard]] size_t numPoints() const { return numPoints_; }
    
    // ==================== Queries ====================
    
    struct ClosestPoint {
        VertId pointId;
        float distSq = -1.0f;
        
        [[nodiscard]] bool valid() const { return distSq >= 0.0f; }
    };
    
    /// Find closest point to query
    [[nodiscard]] ClosestPoint findClosestPoint(
        const Vector3f& query,
        float maxDistSq = std::numeric_limits<float>::max()) const;
    
    /// Find K nearest neighbors
    [[nodiscard]] std::vector<VertId> findKNearest(
        const Vector3f& query, int k) const;
    
    /// Find all points within radius
    [[nodiscard]] VertBitSet findPointsInBall(
        const Vector3f& center, float radius) const;
    
    /// Find all points in box
    [[nodiscard]] VertBitSet findPointsInBox(const Box3f& box) const;
    
private:
    IdVector<Node, NodeId> nodes_;
    const std::vector<Vector3f>* points_ = nullptr;
    std::vector<Vector3f> ownedPoints_;  // If we own the points
    size_t numPoints_ = 0;
    
    void buildTree(const std::vector<Vector3f>& points);
    
    void buildSubtree(
        NodeId nodeId,
        std::vector<VertId>& pointIds,
        size_t begin, size_t end,
        const std::vector<Vector3f>& points);
};

// ==================== Template Implementation ====================

template <typename T>
std::vector<NodeId> AABBTreeBase<T>::getSubtrees(int minNum) const {
    std::vector<NodeId> result;
    if (nodes_.empty()) return result;
    
    std::vector<NodeId> stack;
    stack.push_back(rootNodeId());
    
    while (!stack.empty() && static_cast<int>(result.size() + stack.size()) < minNum) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        if (node.leaf()) {
            result.push_back(nid);
        } else {
            stack.push_back(node.l);
            stack.push_back(node.r);
        }
    }
    
    // Add remaining stack items to result
    result.insert(result.end(), stack.begin(), stack.end());
    
    return result;
}

template <typename T>
typename AABBTreeBase<T>::LeafBitSet AABBTreeBase<T>::getSubtreeLeaves(NodeId subtreeRoot) const {
    LeafBitSet result;
    if (nodes_.empty()) return result;
    
    std::vector<NodeId> stack;
    stack.push_back(subtreeRoot);
    
    while (!stack.empty()) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        if (node.leaf()) {
            result.autoResizeSet(node.leafId());
        } else {
            stack.push_back(node.l);
            stack.push_back(node.r);
        }
    }
    
    return result;
}

} // namespace meshlib
