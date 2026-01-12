#pragma once

/**
 * @file Id.h
 * @brief Type-safe ID class for mesh elements
 * 
 * Provides type-safe indexing for vertices, faces, and edges.
 */

#include <cstdint>
#include <functional>
#include <limits>

namespace meshlib {

/**
 * @brief Type-safe ID for mesh elements
 * @tparam Tag A tag type to distinguish different ID types
 * 
 * This class provides type-safe indexing to prevent mixing up
 * vertex, face, and edge indices.
 */
template <typename Tag>
class Id {
public:
    using ValueType = int32_t;
    static constexpr ValueType Invalid = -1;
    
    /// Default constructor creates an invalid ID
    constexpr Id() noexcept : value_(Invalid) {}
    
    /// Construct from integer value
    explicit constexpr Id(ValueType value) noexcept : value_(value) {}
    
    /// Check if the ID is valid (non-negative)
    constexpr bool valid() const noexcept { return value_ >= 0; }
    
    /// Explicit conversion to bool (true if valid)
    explicit constexpr operator bool() const noexcept { return valid(); }
    
    /// Get the underlying value
    constexpr ValueType get() const noexcept { return value_; }
    
    /// Implicit conversion to ValueType
    constexpr operator ValueType() const noexcept { return value_; }
    
    /// Get mutable reference to underlying value
    constexpr ValueType& ref() noexcept { return value_; }
    
    // Comparison operators
    constexpr bool operator==(Id other) const noexcept { return value_ == other.value_; }
    constexpr bool operator!=(Id other) const noexcept { return value_ != other.value_; }
    constexpr bool operator<(Id other) const noexcept { return value_ < other.value_; }
    constexpr bool operator<=(Id other) const noexcept { return value_ <= other.value_; }
    constexpr bool operator>(Id other) const noexcept { return value_ > other.value_; }
    constexpr bool operator>=(Id other) const noexcept { return value_ >= other.value_; }
    
    // Arithmetic operators
    constexpr Id& operator++() noexcept { ++value_; return *this; }
    constexpr Id operator++(int) noexcept { Id tmp(*this); ++value_; return tmp; }
    constexpr Id& operator--() noexcept { --value_; return *this; }
    constexpr Id operator--(int) noexcept { Id tmp(*this); --value_; return tmp; }
    
    constexpr Id& operator+=(ValueType n) noexcept { value_ += n; return *this; }
    constexpr Id& operator-=(ValueType n) noexcept { value_ -= n; return *this; }
    
    friend constexpr Id operator+(Id id, ValueType n) noexcept { return Id(id.value_ + n); }
    friend constexpr Id operator-(Id id, ValueType n) noexcept { return Id(id.value_ - n); }
    friend constexpr ValueType operator-(Id a, Id b) noexcept { return a.value_ - b.value_; }
    
private:
    ValueType value_;
};

// User-defined literals for convenient ID creation
inline constexpr Id<struct VertexTag> operator""_v(unsigned long long i) noexcept {
    return Id<struct VertexTag>(static_cast<int32_t>(i));
}

inline constexpr Id<struct FaceTag> operator""_f(unsigned long long i) noexcept {
    return Id<struct FaceTag>(static_cast<int32_t>(i));
}

inline constexpr Id<struct EdgeTag> operator""_e(unsigned long long i) noexcept {
    return Id<struct EdgeTag>(static_cast<int32_t>(i));
}

} // namespace meshlib

// Hash support for std::unordered_map/set
namespace std {
template <typename Tag>
struct hash<meshlib::Id<Tag>> {
    size_t operator()(meshlib::Id<Tag> id) const noexcept {
        return std::hash<typename meshlib::Id<Tag>::ValueType>()(id.get());
    }
};
} // namespace std
