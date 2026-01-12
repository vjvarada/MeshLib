#pragma once

/**
 * @file Vector.h
 * @brief 2D and 3D vector types
 */

#include <cmath>
#include <algorithm>
#include <type_traits>
#include <ostream>

namespace meshlib {

/**
 * @brief 2D vector template
 */
template <typename T>
struct Vector2 {
    T x, y;
    
    constexpr Vector2() noexcept : x(T(0)), y(T(0)) {}
    constexpr Vector2(T x, T y) noexcept : x(x), y(y) {}
    
    template <typename U>
    explicit constexpr Vector2(const Vector2<U>& v) noexcept 
        : x(static_cast<T>(v.x)), y(static_cast<T>(v.y)) {}
    
    // Element access
    constexpr T& operator[](int i) noexcept { return i == 0 ? x : y; }
    constexpr const T& operator[](int i) const noexcept { return i == 0 ? x : y; }
    
    // Arithmetic operations
    constexpr Vector2 operator+(const Vector2& v) const noexcept { return {x + v.x, y + v.y}; }
    constexpr Vector2 operator-(const Vector2& v) const noexcept { return {x - v.x, y - v.y}; }
    constexpr Vector2 operator*(T s) const noexcept { return {x * s, y * s}; }
    constexpr Vector2 operator/(T s) const noexcept { return {x / s, y / s}; }
    constexpr Vector2 operator-() const noexcept { return {-x, -y}; }
    
    constexpr Vector2& operator+=(const Vector2& v) noexcept { x += v.x; y += v.y; return *this; }
    constexpr Vector2& operator-=(const Vector2& v) noexcept { x -= v.x; y -= v.y; return *this; }
    constexpr Vector2& operator*=(T s) noexcept { x *= s; y *= s; return *this; }
    constexpr Vector2& operator/=(T s) noexcept { x /= s; y /= s; return *this; }
    
    // Comparison
    constexpr bool operator==(const Vector2& v) const noexcept { return x == v.x && y == v.y; }
    constexpr bool operator!=(const Vector2& v) const noexcept { return !(*this == v); }
    
    // Vector operations
    constexpr T dot(const Vector2& v) const noexcept { return x * v.x + y * v.y; }
    constexpr T lengthSq() const noexcept { return dot(*this); }
    T length() const noexcept { return std::sqrt(lengthSq()); }
    
    Vector2 normalized() const noexcept {
        T len = length();
        return len > T(0) ? *this / len : Vector2();
    }
    
    // Static constructors
    static constexpr Vector2 zero() noexcept { return {T(0), T(0)}; }
    static constexpr Vector2 one() noexcept { return {T(1), T(1)}; }
    static constexpr Vector2 unitX() noexcept { return {T(1), T(0)}; }
    static constexpr Vector2 unitY() noexcept { return {T(0), T(1)}; }
};

/**
 * @brief 3D vector template
 */
template <typename T>
struct Vector3 {
    T x, y, z;
    
    constexpr Vector3() noexcept : x(T(0)), y(T(0)), z(T(0)) {}
    constexpr Vector3(T x, T y, T z) noexcept : x(x), y(y), z(z) {}
    constexpr Vector3(const Vector2<T>& v, T z = T(0)) noexcept : x(v.x), y(v.y), z(z) {}
    
    template <typename U>
    explicit constexpr Vector3(const Vector3<U>& v) noexcept 
        : x(static_cast<T>(v.x)), y(static_cast<T>(v.y)), z(static_cast<T>(v.z)) {}
    
    // Element access
    constexpr T& operator[](int i) noexcept { return (&x)[i]; }
    constexpr const T& operator[](int i) const noexcept { return (&x)[i]; }
    
    // Arithmetic operations
    constexpr Vector3 operator+(const Vector3& v) const noexcept { return {x + v.x, y + v.y, z + v.z}; }
    constexpr Vector3 operator-(const Vector3& v) const noexcept { return {x - v.x, y - v.y, z - v.z}; }
    constexpr Vector3 operator*(T s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vector3 operator/(T s) const noexcept { return {x / s, y / s, z / s}; }
    constexpr Vector3 operator-() const noexcept { return {-x, -y, -z}; }
    
    // Component-wise multiplication
    constexpr Vector3 operator*(const Vector3& v) const noexcept { return {x * v.x, y * v.y, z * v.z}; }
    
    constexpr Vector3& operator+=(const Vector3& v) noexcept { x += v.x; y += v.y; z += v.z; return *this; }
    constexpr Vector3& operator-=(const Vector3& v) noexcept { x -= v.x; y -= v.y; z -= v.z; return *this; }
    constexpr Vector3& operator*=(T s) noexcept { x *= s; y *= s; z *= s; return *this; }
    constexpr Vector3& operator/=(T s) noexcept { x /= s; y /= s; z /= s; return *this; }
    
    // Comparison
    constexpr bool operator==(const Vector3& v) const noexcept { return x == v.x && y == v.y && z == v.z; }
    constexpr bool operator!=(const Vector3& v) const noexcept { return !(*this == v); }
    
    // Vector operations
    constexpr T dot(const Vector3& v) const noexcept { return x * v.x + y * v.y + z * v.z; }
    constexpr T lengthSq() const noexcept { return dot(*this); }
    T length() const noexcept { return std::sqrt(lengthSq()); }
    
    constexpr Vector3 cross(const Vector3& v) const noexcept {
        return {
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        };
    }
    
    Vector3 normalized() const noexcept {
        T len = length();
        return len > T(0) ? *this / len : Vector3();
    }
    
    // Check for NaN/Inf
    bool isFinite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
    
    // Static constructors
    static constexpr Vector3 zero() noexcept { return {T(0), T(0), T(0)}; }
    static constexpr Vector3 one() noexcept { return {T(1), T(1), T(1)}; }
    static constexpr Vector3 unitX() noexcept { return {T(1), T(0), T(0)}; }
    static constexpr Vector3 unitY() noexcept { return {T(0), T(1), T(0)}; }
    static constexpr Vector3 unitZ() noexcept { return {T(0), T(0), T(1)}; }
};

// Scalar multiplication from left
template <typename T>
constexpr Vector2<T> operator*(T s, const Vector2<T>& v) noexcept { return v * s; }

template <typename T>
constexpr Vector3<T> operator*(T s, const Vector3<T>& v) noexcept { return v * s; }

// Free functions
template <typename T>
constexpr T dot(const Vector3<T>& a, const Vector3<T>& b) noexcept { return a.dot(b); }

template <typename T>
constexpr Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b) noexcept { return a.cross(b); }

template <typename T>
T distance(const Vector3<T>& a, const Vector3<T>& b) noexcept { return (b - a).length(); }

template <typename T>
constexpr T distanceSq(const Vector3<T>& a, const Vector3<T>& b) noexcept { return (b - a).lengthSq(); }

// Linear interpolation
template <typename T>
constexpr Vector3<T> lerp(const Vector3<T>& a, const Vector3<T>& b, T t) noexcept {
    return a + (b - a) * t;
}

// Stream output
template <typename T>
std::ostream& operator<<(std::ostream& os, const Vector2<T>& v) {
    return os << "(" << v.x << ", " << v.y << ")";
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

// Type aliases
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;
using Vector2i = Vector2<int>;

using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;
using Vector3i = Vector3<int>;

} // namespace meshlib
