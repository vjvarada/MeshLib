#pragma once

/**
 * @file AABBTreeNode.h
 * @brief AABB Tree node structure and traits
 * 
 * Industrial-strength port of MeshLib's MRAABBTreeNode.h
 */

#include "meshlib/config.h"
#include "../core/Id.h"
#include "../core/Box.h"
#include "../core/Vector.h"
#include "../core/IdVector.h"

namespace meshlib {

// ==================== Tree Traits ====================

/**
 * @brief Traits for AABB tree specializations
 * 
 * @tparam L Leaf tag type (FaceTag for meshes, UndirectedEdgeTag for polylines)
 * @tparam B Bounding box type
 */
template <typename L, typename B>
struct AABBTreeTraits {
    using LeafTag = L;
    using LeafId = Id<L>;
    using BoxT = B;
};

/// Traits for mesh face tree (3D)
using FaceTreeTraits3 = AABBTreeTraits<FaceTag, Box3f>;

/// Traits for polyline tree (3D)
using LineTreeTraits3 = AABBTreeTraits<UndirectedEdgeTag, Box3f>;

// ==================== Tree Node ====================

/**
 * @brief Single node in the AABB tree
 * 
 * Can be either:
 * - Internal node: has two children (l, r both valid)
 * - Leaf node: stores a LeafId (l is the leaf, r is invalid)
 */
template <typename T>
struct AABBTreeNode {
    using LeafId = typename T::LeafId;
    using BoxT = typename T::BoxT;
    
    BoxT box;      ///< Bounding box of this subtree
    NodeId l, r;   ///< Left and right children (for leaf: l is LeafId, r is invalid)
    
    /// Returns true if this is a leaf node
    [[nodiscard]] bool leaf() const { return !r.valid(); }
    
    /// Returns the leaf id (for leaf nodes only)
    [[nodiscard]] LeafId leafId() const { 
        assert(leaf()); 
        return LeafId{static_cast<int>(l)}; 
    }
    
    /// Set this node as a leaf with given id
    void setLeafId(LeafId id) { 
        l = NodeId{static_cast<int>(id)}; 
        r = NodeId{}; 
    }
};

/// Node vector type
template <typename T>
using AABBTreeNodeVec = IdVector<AABBTreeNode<T>, NodeId>;

// ==================== Node Pair ====================

/// Pair of node ids for tree traversal
struct NodeNode {
    NodeId aNode;
    NodeId bNode;
};

} // namespace meshlib
