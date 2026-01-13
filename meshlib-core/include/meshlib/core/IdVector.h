#pragma once

/**
 * @file IdVector.h
 * @brief Id-indexed vector container
 * 
 * Industrial-strength port of MeshLib's MRVector.h providing type-safe
 * vector containers indexed by Id types.
 */

#include "meshlib/config.h"
#include "Id.h"
#include "Vector.h"
#include <vector>
#include <cassert>
#include <initializer_list>

namespace meshlib {

/**
 * @brief std::vector<T>-like container with type-safe Id indexing
 * 
 * Requires specific indexing type to prevent accidentally using
 * a VertId to access a FaceId-indexed vector.
 * 
 * @tparam T Type of stored elements
 * @tparam I Index type (should be an Id type like VertId, FaceId, etc.)
 */
template <typename T, typename I>
class IdVector {
public:
    using value_type = T;
    using reference = T&;
    using const_reference = const T&;
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    using IndexType = I;
    
    // ==================== Construction ====================
    
    /// Creates empty vector
    IdVector() = default;
    
    /// Creates vector with given size and default value
    explicit IdVector(size_t size) : vec_(size) {}
    
    /// Creates vector with given size and fill value
    explicit IdVector(size_t size, const T& val) : vec_(size, val) {}
    
    /// Move from std::vector
    IdVector(std::vector<T>&& vec) : vec_(std::move(vec)) {}
    
    /// Construct from iterator range
    template <class InputIt>
    IdVector(InputIt first, InputIt last) : vec_(first, last) {}
    
    /// Construct from initializer list
    IdVector(std::initializer_list<T> init) : vec_(init) {}
    
    // ==================== Comparison ====================
    
    bool operator==(const IdVector& b) const { return vec_ == b.vec_; }
    bool operator!=(const IdVector& b) const { return vec_ != b.vec_; }
    
    // ==================== Size Management ====================
    
    void clear() { vec_.clear(); }
    
    [[nodiscard]] bool empty() const { return vec_.empty(); }
    [[nodiscard]] size_t size() const { return vec_.size(); }
    [[nodiscard]] size_t capacity() const { return vec_.capacity(); }
    
    void resize(size_t newSize) { vec_.resize(newSize); }
    void resize(size_t newSize, const T& t) { vec_.resize(newSize, t); }
    void reserve(size_t capacity) { vec_.reserve(capacity); }
    void shrink_to_fit() { vec_.shrink_to_fit(); }
    
    /// Resize with exponential growth strategy
    void resizeWithReserve(size_t newSize) {
        auto reserved = vec_.capacity();
        if (reserved > 0 && newSize > reserved) {
            while (newSize > reserved) {
                reserved <<= 1;
            }
            vec_.reserve(reserved);
        }
        vec_.resize(newSize);
    }
    
    void resizeWithReserve(size_t newSize, const T& value) {
        auto reserved = vec_.capacity();
        if (reserved > 0 && newSize > reserved) {
            while (newSize > reserved) {
                reserved <<= 1;
            }
            vec_.reserve(reserved);
        }
        vec_.resize(newSize, value);
    }
    
    // ==================== Element Access ====================
    
    /// Access element by Id (const)
    [[nodiscard]] const_reference operator[](I i) const {
        assert(i.valid() && static_cast<size_t>(i) < vec_.size());
        return vec_[static_cast<size_t>(i)];
    }
    
    /// Access element by Id (mutable)
    [[nodiscard]] reference operator[](I i) {
        assert(i.valid() && static_cast<size_t>(i) < vec_.size());
        return vec_[static_cast<size_t>(i)];
    }
    
    /// Auto-resizing access
    [[nodiscard]] reference autoResizeAt(I i) {
        if (static_cast<size_t>(i) + 1 > size()) {
            resizeWithReserve(static_cast<size_t>(i) + 1);
        }
        return vec_[static_cast<size_t>(i)];
    }
    
    /// Set element with auto-resize
    void autoResizeSet(I i, T val) {
        assert(i.valid());
        size_t pos = static_cast<size_t>(i);
        if (pos >= size()) {
            resizeWithReserve(pos + 1, val);
            return;
        }
        vec_[pos] = std::move(val);
    }
    
    [[nodiscard]] const_reference front() const { return vec_.front(); }
    [[nodiscard]] reference front() { return vec_.front(); }
    [[nodiscard]] const_reference back() const { return vec_.back(); }
    [[nodiscard]] reference back() { return vec_.back(); }
    
    [[nodiscard]] T* data() { return vec_.data(); }
    [[nodiscard]] const T* data() const { return vec_.data(); }
    
    // ==================== Modification ====================
    
    void push_back(const T& t) { vec_.push_back(t); }
    void push_back(T&& t) { vec_.push_back(std::move(t)); }
    void pop_back() { vec_.pop_back(); }
    
    template <typename... Args>
    T& emplace_back(Args&&... args) {
        return vec_.emplace_back(std::forward<Args>(args)...);
    }
    
    void swap(IdVector& b) { vec_.swap(b.vec_); }
    
    // ==================== Id-based Interface ====================
    
    /// First valid Id
    [[nodiscard]] I beginId() const { return I{0}; }
    
    /// Id of last element
    [[nodiscard]] I backId() const { 
        assert(!vec_.empty()); 
        return I{static_cast<int>(vec_.size() - 1)}; 
    }
    
    /// One past last valid Id
    [[nodiscard]] I endId() const { return I{static_cast<int>(vec_.size())}; }
    
    // ==================== Iterators ====================
    
    [[nodiscard]] iterator begin() { return vec_.begin(); }
    [[nodiscard]] const_iterator begin() const { return vec_.begin(); }
    [[nodiscard]] iterator end() { return vec_.end(); }
    [[nodiscard]] const_iterator end() const { return vec_.end(); }
    [[nodiscard]] const_iterator cbegin() const { return vec_.cbegin(); }
    [[nodiscard]] const_iterator cend() const { return vec_.cend(); }
    
    // ==================== Utility ====================
    
    /// Returns heap memory usage in bytes
    [[nodiscard]] size_t heapBytes() const { return capacity() * sizeof(T); }
    
    /// Direct access to underlying vector (use with caution)
    std::vector<T>& vec() { return vec_; }
    const std::vector<T>& vec() const { return vec_; }
    
private:
    std::vector<T> vec_;
};

// Free function iterators (for range-based for)
template <typename T, typename I>
auto begin(IdVector<T, I>& a) { return a.begin(); }

template <typename T, typename I>
auto begin(const IdVector<T, I>& a) { return a.begin(); }

template <typename T, typename I>
auto end(IdVector<T, I>& a) { return a.end(); }

template <typename T, typename I>
auto end(const IdVector<T, I>& a) { return a.end(); }

// ==================== Utility Functions ====================

/// Get value at id, or default if id is invalid or out of range
template <typename T, typename I>
[[nodiscard]] T getAt(const IdVector<T, I>& a, I id, T def = {}) {
    return (id.valid() && static_cast<size_t>(id) < a.size()) ? a[id] : def;
}

/// Get value from std::vector at index, or default if out of range
template <typename T>
[[nodiscard]] T getAt(const std::vector<T>& a, size_t id, T def = {}) {
    return (id < a.size()) ? a[id] : def;
}

// ==================== Type Aliases ====================

// Vertex-indexed vectors
template <typename T>
using VertVector = IdVector<T, VertId>;

// Face-indexed vectors
template <typename T>
using FaceVector = IdVector<T, FaceId>;

// Edge-indexed vectors
template <typename T>
using EdgeVector = IdVector<T, EdgeId>;

// Undirected edge-indexed vectors
template <typename T>
using UndirectedEdgeVector = IdVector<T, UndirectedEdgeId>;

// Node-indexed vectors (for AABB trees)
template <typename T>
using NodeVector = IdVector<T, NodeId>;

// Common concrete types
using VertCoords = VertVector<Vector3f>;
using VertNormals = VertVector<Vector3f>;
using FaceNormals = FaceVector<Vector3f>;

} // namespace meshlib
