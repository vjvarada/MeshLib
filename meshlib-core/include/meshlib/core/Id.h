#pragma once

/**
 * @file Id.h
 * @brief Type-safe identifier types for mesh elements
 * 
 * Industrial-strength port of MeshLib's MRId.h providing type-safe identifiers
 * that prevent mixing faces, edges, and vertices at compile time.
 * 
 * Key features:
 * - Type-safe: Can't mix VertId with FaceId
 * - Half-edge support: EdgeId has sym(), undirected() methods
 * - Invalid state: Default-constructed IDs are invalid (-1)
 */

#include "meshlib/config.h"
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <functional>

namespace meshlib {

// ==================== Tag Types ====================
// These are empty types used to distinguish different ID types

class EdgeTag;
class UndirectedEdgeTag;
class FaceTag;
class VertTag;
class NodeTag;

// ==================== Primary Id Template ====================

/**
 * @brief Type-safe identifier for mesh elements
 * 
 * Prevents mixing of different element types at compile time.
 * Invalid IDs have value -1.
 * 
 * @tparam T Tag type (EdgeTag, FaceTag, VertTag, etc.)
 */
template <typename T>
class Id {
public:
    using ValueType = int;
    
    /// Creates invalid ID (-1)
    constexpr Id() noexcept : id_(-1) {}
    
    /// Creates ID with given value
    template <typename U, std::enable_if_t<std::is_integral_v<U>, int> = 0>
    explicit constexpr Id(U i) noexcept : id_(static_cast<ValueType>(i)) {}
    
    /// Implicit conversion to underlying value
    constexpr operator ValueType() const { return id_; }
    
    /// Check if ID is valid (>= 0)
    constexpr bool valid() const { return id_ >= 0; }
    
    /// Explicit bool conversion
    explicit constexpr operator bool() const { return id_ >= 0; }
    
    /// Get reference to underlying value
    constexpr ValueType& get() noexcept { return id_; }
    constexpr const ValueType& get() const noexcept { return id_; }
    
    // Comparison operators
    constexpr bool operator==(Id b) const { return id_ == b.id_; }
    constexpr bool operator!=(Id b) const { return id_ != b.id_; }
    constexpr bool operator<(Id b) const { return id_ < b.id_; }
    constexpr bool operator<=(Id b) const { return id_ <= b.id_; }
    constexpr bool operator>(Id b) const { return id_ > b.id_; }
    constexpr bool operator>=(Id b) const { return id_ >= b.id_; }
    
    // Prevent comparison with different Id types
    template <typename U>
    bool operator==(Id<U> b) const = delete;
    template <typename U>
    bool operator!=(Id<U> b) const = delete;
    template <typename U>
    bool operator<(Id<U> b) const = delete;
    
    // Increment/decrement
    constexpr Id& operator--() { --id_; return *this; }
    constexpr Id& operator++() { ++id_; return *this; }
    constexpr Id operator--(int) { auto res = *this; --id_; return res; }
    constexpr Id operator++(int) { auto res = *this; ++id_; return res; }
    
    // Arithmetic
    constexpr Id& operator-=(ValueType a) { id_ -= a; return *this; }
    constexpr Id& operator+=(ValueType a) { id_ += a; return *this; }
    
    constexpr Id operator+(ValueType a) const { return Id(id_ + a); }
    constexpr Id operator-(ValueType a) const { return Id(id_ - a); }
    friend constexpr ValueType operator-(Id a, Id b) { return a.id_ - b.id_; }
    
private:
    ValueType id_;
};

// ==================== UndirectedEdgeId ====================

// Forward declaration
template <> class Id<UndirectedEdgeTag>;
using UndirectedEdgeId = Id<UndirectedEdgeTag>;

/**
 * @brief Specialization for UndirectedEdgeId
 * 
 * Represents an undirected edge (pair of half-edges).
 */
template <>
class Id<UndirectedEdgeTag> {
public:
    using ValueType = int;
    
    constexpr Id() noexcept : id_(-1) {}
    
    template <typename U, std::enable_if_t<std::is_integral_v<U>, int> = 0>
    explicit constexpr Id(U i) noexcept : id_(static_cast<ValueType>(i)) {}
    
    constexpr operator ValueType() const { return id_; }
    constexpr bool valid() const { return id_ >= 0; }
    explicit constexpr operator bool() const { return id_ >= 0; }
    constexpr ValueType& get() noexcept { return id_; }
    constexpr const ValueType& get() const noexcept { return id_; }
    
    // Comparison operators
    constexpr bool operator==(Id b) const { return id_ == b.id_; }
    constexpr bool operator!=(Id b) const { return id_ != b.id_; }
    constexpr bool operator<(Id b) const { return id_ < b.id_; }
    constexpr bool operator<=(Id b) const { return id_ <= b.id_; }
    constexpr bool operator>(Id b) const { return id_ > b.id_; }
    constexpr bool operator>=(Id b) const { return id_ >= b.id_; }
    
    template <typename U>
    bool operator==(Id<U> b) const = delete;
    template <typename U>
    bool operator!=(Id<U> b) const = delete;
    template <typename U>
    bool operator<(Id<U> b) const = delete;
    
    constexpr Id& operator--() { --id_; return *this; }
    constexpr Id& operator++() { ++id_; return *this; }
    constexpr Id operator--(int) { auto res = *this; --id_; return res; }
    constexpr Id operator++(int) { auto res = *this; ++id_; return res; }
    
    constexpr Id& operator-=(ValueType a) { id_ -= a; return *this; }
    constexpr Id& operator+=(ValueType a) { id_ += a; return *this; }
    
    constexpr Id operator+(ValueType a) const { return Id(id_ + a); }
    constexpr Id operator-(ValueType a) const { return Id(id_ - a); }
    friend constexpr ValueType operator-(Id a, Id b) { return a.id_ - b.id_; }
    
private:
    ValueType id_;
};

// ==================== EdgeId (Half-Edge) ====================

/**
 * @brief Specialization for EdgeId with half-edge semantics
 * 
 * EdgeId stores directed half-edges. Each undirected edge has two EdgeIds:
 * - Even EdgeIds (0, 2, 4, ...) and their sym() counterparts (1, 3, 5, ...)
 * 
 * Key methods:
 * - sym(): Get the opposite direction half-edge
 * - undirected(): Get the UndirectedEdgeId
 * - even()/odd(): Check if this is the even or odd half-edge
 */
template <>
class Id<EdgeTag> {
public:
    using ValueType = int;
    
    constexpr Id() noexcept : id_(-1) {}
    
    /// Construct from UndirectedEdgeId (creates the even half-edge)
    constexpr Id(UndirectedEdgeId u) noexcept 
        : id_(u.valid() ? (static_cast<ValueType>(u) << 1) : -1) {}
    
    template <typename U, std::enable_if_t<std::is_integral_v<U>, int> = 0>
    explicit constexpr Id(U i) noexcept : id_(static_cast<ValueType>(i)) {}
    
    constexpr operator ValueType() const { return id_; }
    constexpr bool valid() const { return id_ >= 0; }
    explicit constexpr operator bool() const { return id_ >= 0; }
    constexpr ValueType& get() noexcept { return id_; }
    constexpr const ValueType& get() const noexcept { return id_; }
    
    /// Returns the symmetric (opposite direction) half-edge
    constexpr Id sym() const { assert(valid()); return Id(id_ ^ 1); }
    
    /// Returns true if this is an even half-edge (id % 2 == 0)
    constexpr bool even() const { assert(valid()); return (id_ & 1) == 0; }
    
    /// Returns true if this is an odd half-edge (id % 2 == 1)
    constexpr bool odd() const { assert(valid()); return (id_ & 1) == 1; }
    
    /// Returns the undirected edge identifier
    constexpr UndirectedEdgeId undirected() const { 
        assert(valid()); 
        return UndirectedEdgeId(id_ >> 1); 
    }
    
    /// Implicit conversion to UndirectedEdgeId
    constexpr operator UndirectedEdgeId() const { return undirected(); }
    
    // Comparison operators
    constexpr bool operator==(Id b) const { return id_ == b.id_; }
    constexpr bool operator!=(Id b) const { return id_ != b.id_; }
    constexpr bool operator<(Id b) const { return id_ < b.id_; }
    constexpr bool operator<=(Id b) const { return id_ <= b.id_; }
    constexpr bool operator>(Id b) const { return id_ > b.id_; }
    constexpr bool operator>=(Id b) const { return id_ >= b.id_; }
    
    template <typename U>
    bool operator==(Id<U> b) const = delete;
    template <typename U>
    bool operator!=(Id<U> b) const = delete;
    template <typename U>
    bool operator<(Id<U> b) const = delete;
    
    constexpr Id& operator--() { --id_; return *this; }
    constexpr Id& operator++() { ++id_; return *this; }
    constexpr Id operator--(int) { auto res = *this; --id_; return res; }
    constexpr Id operator++(int) { auto res = *this; ++id_; return res; }
    
    constexpr Id& operator-=(ValueType a) { id_ -= a; return *this; }
    constexpr Id& operator+=(ValueType a) { id_ += a; return *this; }
    
    constexpr Id operator+(ValueType a) const { return Id(id_ + a); }
    constexpr Id operator-(ValueType a) const { return Id(id_ - a); }
    friend constexpr ValueType operator-(Id a, Id b) { return a.id_ - b.id_; }
    
private:
    ValueType id_;
};

// ==================== Type Aliases ====================

using EdgeId = Id<EdgeTag>;
using FaceId = Id<FaceTag>;
using VertId = Id<VertTag>;
using VertexId = VertId;  // Backwards compatibility alias
using NodeId = Id<NodeTag>;

// ==================== ThreeVertIds ====================

/// Three vertex ids representing a triangle
using ThreeVertIds = std::array<VertId, 3>;

// ==================== Hash Support ====================

template <typename T>
struct IdHash {
    size_t operator()(Id<T> id) const noexcept {
        return std::hash<typename Id<T>::ValueType>{}(id.get());
    }
};

} // namespace meshlib

// Standard library hash specializations
namespace std {

template <typename T>
struct hash<meshlib::Id<T>> {
    size_t operator()(meshlib::Id<T> id) const noexcept {
        return std::hash<typename meshlib::Id<T>::ValueType>{}(id.get());
    }
};

} // namespace std
