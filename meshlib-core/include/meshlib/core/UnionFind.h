#pragma once

/**
 * @file UnionFind.h
 * @brief Union-Find (Disjoint Set Union) data structure
 * 
 * Industrial-strength port of MeshLib's MRUnionFind.h for efficient
 * connected component analysis.
 */

#include "meshlib/config.h"
#include "Id.h"
#include <vector>
#include <numeric>
#include <algorithm>

namespace meshlib {

/**
 * @brief Union-Find data structure with path compression and union by rank
 * 
 * Efficiently maintains a collection of disjoint sets with near O(1)
 * operations for union and find. Essential for mesh component analysis.
 * 
 * @tparam I Index type (can be int, VertId, FaceId, etc.)
 */
template <typename I = int>
class UnionFind {
public:
    // ==================== Construction ====================
    
    /// Create empty union-find
    UnionFind() = default;
    
    /// Create union-find with n elements, each in its own set
    explicit UnionFind(size_t n) : parent_(n), rank_(n, 0) {
        reset(n);
    }
    
    /// Reset to n elements, each in its own set
    void reset(size_t n) {
        parent_.resize(n);
        rank_.resize(n, 0);
        std::iota(parent_.begin(), parent_.end(), I{0});
        std::fill(rank_.begin(), rank_.end(), 0);
    }
    
    /// Clear all elements
    void clear() {
        parent_.clear();
        rank_.clear();
    }
    
    // ==================== Core Operations ====================
    
    /// Find the root (representative) of the set containing x
    /// Uses path compression for efficiency
    I find(I x) {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]);  // Path compression
        }
        return parent_[x];
    }
    
    /// Find root without path compression (const version)
    I findConst(I x) const {
        while (parent_[x] != x) {
            x = parent_[x];
        }
        return x;
    }
    
    /// Unite the sets containing x and y
    /// Returns the new root of the combined set
    I unite(I x, I y) {
        I rootX = find(x);
        I rootY = find(y);
        
        if (rootX == rootY) return rootX;
        
        // Union by rank
        if (rank_[rootX] < rank_[rootY]) {
            parent_[rootX] = rootY;
            return rootY;
        } else if (rank_[rootX] > rank_[rootY]) {
            parent_[rootY] = rootX;
            return rootX;
        } else {
            parent_[rootY] = rootX;
            ++rank_[rootX];
            return rootX;
        }
    }
    
    /// Check if x and y are in the same set
    bool connected(I x, I y) {
        return find(x) == find(y);
    }
    
    /// Check if x and y are in the same set (const version)
    bool connectedConst(I x, I y) const {
        return findConst(x) == findConst(y);
    }
    
    // ==================== Accessors ====================
    
    /// Number of elements
    size_t size() const { return parent_.size(); }
    
    /// Check if empty
    bool empty() const { return parent_.empty(); }
    
    /// Get parent array (for iteration)
    const std::vector<I>& parents() const { return parent_; }
    
    // ==================== Analysis ====================
    
    /// Count number of distinct sets (components)
    size_t countSets() const {
        size_t count = 0;
        for (size_t i = 0; i < parent_.size(); ++i) {
            if (parent_[i] == I(i)) ++count;
        }
        return count;
    }
    
    /// Get all root elements
    std::vector<I> getRoots() const {
        std::vector<I> roots;
        for (size_t i = 0; i < parent_.size(); ++i) {
            if (parent_[i] == I(i)) {
                roots.push_back(I(i));
            }
        }
        return roots;
    }
    
    /// Get size of set containing x
    size_t setSize(I x) {
        I root = find(x);
        size_t count = 0;
        for (size_t i = 0; i < parent_.size(); ++i) {
            if (find(I(i)) == root) ++count;
        }
        return count;
    }
    
    /// Get all elements in the same set as x
    std::vector<I> getSet(I x) {
        I root = find(x);
        std::vector<I> result;
        for (size_t i = 0; i < parent_.size(); ++i) {
            if (find(I(i)) == root) {
                result.push_back(I(i));
            }
        }
        return result;
    }
    
    /// Get sizes of all sets, indexed by root
    std::vector<size_t> getAllSetSizes() {
        std::vector<size_t> sizes(parent_.size(), 0);
        for (size_t i = 0; i < parent_.size(); ++i) {
            ++sizes[find(I(i))];
        }
        return sizes;
    }
    
    /// Get largest set
    std::vector<I> getLargestSet() {
        auto sizes = getAllSetSizes();
        I largestRoot = I(std::max_element(sizes.begin(), sizes.end()) - sizes.begin());
        return getSet(largestRoot);
    }
    
private:
    std::vector<I> parent_;  ///< Parent pointers (root has parent = self)
    std::vector<int> rank_;  ///< Rank for union by rank
};

// ==================== Type Aliases ====================

/// Union-find for integers
using UnionFindInt = UnionFind<int>;

/// Union-find for vertex IDs
using UnionFindVert = UnionFind<VertId>;

/// Union-find for face IDs  
using UnionFindFace = UnionFind<FaceId>;

/// Union-find for edge IDs
using UnionFindEdge = UnionFind<EdgeId>;

// ==================== Specialized Functions ====================

/**
 * @brief Build union-find from edge connectivity
 * 
 * Given a list of edges (pairs of vertex indices), build a union-find
 * structure representing connected components.
 * 
 * @param numVerts Number of vertices
 * @param edges Array of edge pairs
 * @param numEdges Number of edges
 * @return Union-find structure with connected components
 */
inline UnionFind<int> buildUnionFindFromEdges(
    size_t numVerts,
    const std::pair<int, int>* edges,
    size_t numEdges) {
    
    UnionFind<int> uf(numVerts);
    for (size_t i = 0; i < numEdges; ++i) {
        uf.unite(edges[i].first, edges[i].second);
    }
    return uf;
}

} // namespace meshlib
