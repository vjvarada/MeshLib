#pragma once

/**
 * @file Quaternion.h
 * @brief Quaternion representation for 3D rotations
 * 
 * Industrial-strength port of MeshLib's MRQuaternion.h providing
 * quaternion operations for rotation representation and interpolation.
 */

#include "meshlib/config.h"
#include "Vector.h"
#include "Matrix.h"
#include <cmath>

namespace meshlib {

/**
 * @brief Quaternion for representing 3D rotations
 * 
 * Quaternion q = a + b*i + c*j + d*k, stored as (a, b, c, d) = (w, x, y, z)
 * For unit quaternions representing rotations:
 * - w = cos(angle/2)
 * - (x, y, z) = sin(angle/2) * axis
 * 
 * @tparam T Scalar type (float or double)
 */
template <typename T>
struct Quaternion {
    T a = 1;  ///< Scalar part (w), default = 1 for identity
    T b = 0;  ///< i component (x)
    T c = 0;  ///< j component (y)
    T d = 0;  ///< k component (z)
    
    // ==================== Constructors ====================
    
    /// Default constructor - identity quaternion
    constexpr Quaternion() noexcept = default;
    
    /// Construct from components
    constexpr Quaternion(T a_, T b_, T c_, T d_) noexcept 
        : a(a_), b(b_), c(c_), d(d_) {}
    
    /// Construct from scalar and vector parts
    constexpr Quaternion(T scalar, const Vector3<T>& vec) noexcept
        : a(scalar), b(vec.x), c(vec.y), d(vec.z) {}
    
    /// Construct rotation quaternion from axis and angle (radians)
    static Quaternion fromAxisAngle(const Vector3<T>& axis, T angle) noexcept {
        T halfAngle = angle * T(0.5);
        T s = std::sin(halfAngle);
        Vector3<T> n = axis.normalized();
        return Quaternion(std::cos(halfAngle), n.x * s, n.y * s, n.z * s);
    }
    
    /// Construct rotation quaternion from rotation matrix
    static Quaternion fromMatrix(const Matrix3<T>& m) noexcept {
        Quaternion q;
        T trace = m(0,0) + m(1,1) + m(2,2);
        
        if (trace > 0) {
            T s = std::sqrt(trace + 1) * 2;
            q.a = s * T(0.25);
            q.b = (m(2,1) - m(1,2)) / s;
            q.c = (m(0,2) - m(2,0)) / s;
            q.d = (m(1,0) - m(0,1)) / s;
        } else if (m(0,0) > m(1,1) && m(0,0) > m(2,2)) {
            T s = std::sqrt(1 + m(0,0) - m(1,1) - m(2,2)) * 2;
            q.a = (m(2,1) - m(1,2)) / s;
            q.b = s * T(0.25);
            q.c = (m(0,1) + m(1,0)) / s;
            q.d = (m(0,2) + m(2,0)) / s;
        } else if (m(1,1) > m(2,2)) {
            T s = std::sqrt(1 + m(1,1) - m(0,0) - m(2,2)) * 2;
            q.a = (m(0,2) - m(2,0)) / s;
            q.b = (m(0,1) + m(1,0)) / s;
            q.c = s * T(0.25);
            q.d = (m(1,2) + m(2,1)) / s;
        } else {
            T s = std::sqrt(1 + m(2,2) - m(0,0) - m(1,1)) * 2;
            q.a = (m(1,0) - m(0,1)) / s;
            q.b = (m(0,2) + m(2,0)) / s;
            q.c = (m(1,2) + m(2,1)) / s;
            q.d = s * T(0.25);
        }
        return q.normalized();
    }
    
    /// Create rotation that rotates 'from' vector to 'to' vector
    static Quaternion rotation(const Vector3<T>& from, const Vector3<T>& to) noexcept {
        Vector3<T> f = from.normalized();
        Vector3<T> t = to.normalized();
        T d = dot(f, t);
        
        if (d >= T(1) - T(1e-6)) {
            // Vectors are parallel
            return Quaternion();
        }
        
        if (d <= T(-1) + T(1e-6)) {
            // Vectors are opposite - find perpendicular axis
            Vector3<T> axis = cross(Vector3<T>::unitX(), f);
            if (axis.lengthSq() < T(1e-6)) {
                axis = cross(Vector3<T>::unitY(), f);
            }
            return fromAxisAngle(axis.normalized(), T(3.14159265358979323846));
        }
        
        Vector3<T> axis = cross(f, t);
        T s = std::sqrt((1 + d) * 2);
        return Quaternion(s * T(0.5), axis.x / s, axis.y / s, axis.z / s).normalized();
    }
    
    // ==================== Accessors ====================
    
    /// Get scalar part (w)
    constexpr T scalar() const noexcept { return a; }
    
    /// Get vector part (x, y, z)
    constexpr Vector3<T> vec() const noexcept { return {b, c, d}; }
    
    /// Get rotation axis (for unit quaternions)
    Vector3<T> axis() const noexcept {
        T s = std::sqrt(1 - a * a);
        if (s < T(1e-10)) return Vector3<T>::unitZ();
        return Vector3<T>(b / s, c / s, d / s);
    }
    
    /// Get rotation angle in radians (for unit quaternions)
    T angle() const noexcept {
        return 2 * std::acos(std::clamp(a, T(-1), T(1)));
    }
    
    // ==================== Operations ====================
    
    /// Squared norm
    constexpr T normSq() const noexcept {
        return a * a + b * b + c * c + d * d;
    }
    
    /// Norm (magnitude)
    T norm() const noexcept {
        return std::sqrt(normSq());
    }
    
    /// Normalize to unit quaternion
    Quaternion normalized() const noexcept {
        T n = norm();
        if (n < T(1e-10)) return Quaternion();
        return Quaternion(a / n, b / n, c / n, d / n);
    }
    
    /// Conjugate (inverse for unit quaternions)
    constexpr Quaternion conjugate() const noexcept {
        return Quaternion(a, -b, -c, -d);
    }
    
    /// Inverse
    Quaternion inverse() const noexcept {
        T n2 = normSq();
        if (n2 < T(1e-10)) return Quaternion();
        return Quaternion(a / n2, -b / n2, -c / n2, -d / n2);
    }
    
    /// Convert to 3x3 rotation matrix
    Matrix3<T> toMatrix() const noexcept {
        T xx = b * b, yy = c * c, zz = d * d;
        T xy = b * c, xz = b * d, yz = c * d;
        T wx = a * b, wy = a * c, wz = a * d;
        
        return Matrix3<T>{
            1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy),
                2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx),
                2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)
        };
    }
    
    /// Rotate a vector by this quaternion
    Vector3<T> operator()(const Vector3<T>& v) const noexcept {
        // Optimized rotation: v' = q * v * q^-1
        Vector3<T> qv(b, c, d);
        Vector3<T> uv = cross(qv, v);
        Vector3<T> uuv = cross(qv, uv);
        return v + (uv * a + uuv) * T(2);
    }
    
    // ==================== Arithmetic ====================
    
    /// Quaternion multiplication (composition of rotations)
    constexpr Quaternion operator*(const Quaternion& q) const noexcept {
        return Quaternion(
            a * q.a - b * q.b - c * q.c - d * q.d,
            a * q.b + b * q.a + c * q.d - d * q.c,
            a * q.c - b * q.d + c * q.a + d * q.b,
            a * q.d + b * q.c - c * q.b + d * q.a
        );
    }
    
    /// Quaternion addition
    constexpr Quaternion operator+(const Quaternion& q) const noexcept {
        return Quaternion(a + q.a, b + q.b, c + q.c, d + q.d);
    }
    
    /// Quaternion subtraction
    constexpr Quaternion operator-(const Quaternion& q) const noexcept {
        return Quaternion(a - q.a, b - q.b, c - q.c, d - q.d);
    }
    
    /// Scalar multiplication
    constexpr Quaternion operator*(T s) const noexcept {
        return Quaternion(a * s, b * s, c * s, d * s);
    }
    
    /// Scalar division
    constexpr Quaternion operator/(T s) const noexcept {
        return Quaternion(a / s, b / s, c / s, d / s);
    }
    
    /// Negation
    constexpr Quaternion operator-() const noexcept {
        return Quaternion(-a, -b, -c, -d);
    }
    
    // ==================== Comparison ====================
    
    constexpr bool operator==(const Quaternion& q) const noexcept {
        return a == q.a && b == q.b && c == q.c && d == q.d;
    }
    
    constexpr bool operator!=(const Quaternion& q) const noexcept {
        return !(*this == q);
    }
    
    // ==================== Interpolation ====================
    
    /// Spherical linear interpolation (SLERP)
    static Quaternion slerp(const Quaternion& q0, const Quaternion& q1, T t) noexcept {
        T cosTheta = q0.a * q1.a + q0.b * q1.b + q0.c * q1.c + q0.d * q1.d;
        
        // If q0 and q1 are close, use linear interpolation
        if (cosTheta > T(0.9995)) {
            return (q0 * (1 - t) + q1 * t).normalized();
        }
        
        // Handle negative dot product (take shorter path)
        Quaternion q1_ = q1;
        if (cosTheta < 0) {
            q1_ = -q1;
            cosTheta = -cosTheta;
        }
        
        cosTheta = std::clamp(cosTheta, T(-1), T(1));
        T theta = std::acos(cosTheta);
        T sinTheta = std::sin(theta);
        
        T w0 = std::sin((1 - t) * theta) / sinTheta;
        T w1 = std::sin(t * theta) / sinTheta;
        
        return q0 * w0 + q1_ * w1;
    }
    
    /// Linear interpolation (LERP), then normalize
    static Quaternion nlerp(const Quaternion& q0, const Quaternion& q1, T t) noexcept {
        T cosTheta = q0.a * q1.a + q0.b * q1.b + q0.c * q1.c + q0.d * q1.d;
        Quaternion q1_ = cosTheta < 0 ? -q1 : q1;
        return (q0 * (1 - t) + q1_ * t).normalized();
    }
    
    // ==================== Static Constructors ====================
    
    /// Identity quaternion
    static constexpr Quaternion identity() noexcept {
        return Quaternion(1, 0, 0, 0);
    }
};

// ==================== Free Functions ====================

/// Scalar multiplication (scalar * quaternion)
template <typename T>
constexpr Quaternion<T> operator*(T s, const Quaternion<T>& q) noexcept {
    return q * s;
}

/// Dot product of quaternions
template <typename T>
constexpr T dot(const Quaternion<T>& a, const Quaternion<T>& b) noexcept {
    return a.a * b.a + a.b * b.b + a.c * b.c + a.d * b.d;
}

// ==================== Type Aliases ====================

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;

} // namespace meshlib
