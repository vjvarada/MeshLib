#pragma once

/**
 * @file AffineXf.h
 * @brief Affine transformation for 3D space
 * 
 * Industrial-strength port of MeshLib's MRAffineXf3.h providing
 * affine transformations combining rotation, scale, and translation.
 */

#include "meshlib/config.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include <cmath>

namespace meshlib {

/**
 * @brief 3D Affine transformation
 * 
 * Represents transformation: p' = A * p + b
 * where A is a 3x3 matrix (rotation/scale/shear) and b is translation.
 * 
 * @tparam T Scalar type (float or double)
 */
template <typename T>
struct AffineXf3 {
    Matrix3<T> A;    ///< Linear part (rotation, scale, shear)
    Vector3<T> b;    ///< Translation part
    
    // ==================== Constructors ====================
    
    /// Default constructor - identity transform
    constexpr AffineXf3() noexcept : A(Matrix3<T>::identity()), b() {}
    
    /// Construct from linear and translation parts
    constexpr AffineXf3(const Matrix3<T>& linear, const Vector3<T>& translation = Vector3<T>()) noexcept
        : A(linear), b(translation) {}
    
    /// Construct from quaternion rotation
    explicit AffineXf3(const Quaternion<T>& q) noexcept
        : A(q.toMatrix()), b() {}
    
    /// Construct from quaternion rotation and translation
    AffineXf3(const Quaternion<T>& q, const Vector3<T>& translation) noexcept
        : A(q.toMatrix()), b(translation) {}
    
    // ==================== Static Constructors ====================
    
    /// Identity transformation
    static constexpr AffineXf3 identity() noexcept {
        return AffineXf3();
    }
    
    /// Pure translation
    static constexpr AffineXf3 translation(const Vector3<T>& t) noexcept {
        return AffineXf3(Matrix3<T>::identity(), t);
    }
    
    /// Pure rotation around X axis
    static AffineXf3 rotationX(T angle) noexcept {
        return AffineXf3(Matrix3<T>::rotationX(angle));
    }
    
    /// Pure rotation around Y axis
    static AffineXf3 rotationY(T angle) noexcept {
        return AffineXf3(Matrix3<T>::rotationY(angle));
    }
    
    /// Pure rotation around Z axis
    static AffineXf3 rotationZ(T angle) noexcept {
        return AffineXf3(Matrix3<T>::rotationZ(angle));
    }
    
    /// Rotation around arbitrary axis through origin
    static AffineXf3 rotation(const Vector3<T>& axis, T angle) noexcept {
        return AffineXf3(Quaternion<T>::fromAxisAngle(axis, angle));
    }
    
    /// Rotation around arbitrary axis through a point
    static AffineXf3 rotation(const Vector3<T>& axis, T angle, const Vector3<T>& center) noexcept {
        AffineXf3 xf = rotation(axis, angle);
        xf.b = center - xf.A * center;
        return xf;
    }
    
    /// Rotation from one vector to another
    static AffineXf3 rotation(const Vector3<T>& from, const Vector3<T>& to) noexcept {
        return AffineXf3(Quaternion<T>::rotation(from, to));
    }
    
    /// Uniform scale around origin
    static constexpr AffineXf3 scale(T s) noexcept {
        return AffineXf3(Matrix3<T>::scale(s));
    }
    
    /// Non-uniform scale around origin
    static constexpr AffineXf3 scale(T sx, T sy, T sz) noexcept {
        return AffineXf3(Matrix3<T>::scale(sx, sy, sz));
    }
    
    /// Non-uniform scale around origin
    static constexpr AffineXf3 scale(const Vector3<T>& s) noexcept {
        return AffineXf3(Matrix3<T>::scale(s.x, s.y, s.z));
    }
    
    /// Uniform scale around a center point
    static constexpr AffineXf3 scale(T s, const Vector3<T>& center) noexcept {
        return AffineXf3(Matrix3<T>::scale(s), center * (1 - s));
    }
    
    /// Create transform that maps from -> to for corresponding point pairs
    /// Uses linear least squares for best fit when points don't perfectly match
    static AffineXf3 xfFromPoints(
        const Vector3<T>* from, const Vector3<T>* to, size_t count) noexcept;
    
    // ==================== Transform Operations ====================
    
    /// Transform a point: A * p + b
    constexpr Vector3<T> operator()(const Vector3<T>& p) const noexcept {
        return A * p + b;
    }
    
    /// Transform a direction (no translation): A * d
    constexpr Vector3<T> transformDirection(const Vector3<T>& d) const noexcept {
        return A * d;
    }
    
    /// Transform a normal (uses inverse transpose)
    Vector3<T> transformNormal(const Vector3<T>& n) const noexcept {
        // For orthogonal transforms, A^(-T) = A
        // For general transforms, we need the inverse transpose
        return (A.inverse().transposed() * n).normalized();
    }
    
    // ==================== Composition ====================
    
    /// Compose transformations: (this * other)(p) = this(other(p))
    constexpr AffineXf3 operator*(const AffineXf3& other) const noexcept {
        return AffineXf3(A * other.A, A * other.b + b);
    }
    
    /// Compound assignment
    AffineXf3& operator*=(const AffineXf3& other) noexcept {
        b = A * other.b + b;
        A = A * other.A;
        return *this;
    }
    
    // ==================== Inverse ====================
    
    /// Compute inverse transformation
    AffineXf3 inverse() const noexcept {
        Matrix3<T> Ainv = A.inverse();
        return AffineXf3(Ainv, -(Ainv * b));
    }
    
    // ==================== Comparison ====================
    
    constexpr bool operator==(const AffineXf3& other) const noexcept {
        return A == other.A && b == other.b;
    }
    
    constexpr bool operator!=(const AffineXf3& other) const noexcept {
        return !(*this == other);
    }
    
    // ==================== Decomposition ====================
    
    /// Extract translation component
    constexpr Vector3<T> translation() const noexcept {
        return b;
    }
    
    /// Extract rotation as quaternion (assumes A is pure rotation)
    Quaternion<T> rotation() const noexcept {
        return Quaternion<T>::fromMatrix(A);
    }
    
    /// Extract uniform scale factor (returns 1 if not uniform scale)
    T uniformScale() const noexcept {
        // Take geometric mean of column lengths
        T sx = Vector3<T>(A(0,0), A(1,0), A(2,0)).length();
        T sy = Vector3<T>(A(0,1), A(1,1), A(2,1)).length();
        T sz = Vector3<T>(A(0,2), A(1,2), A(2,2)).length();
        return std::cbrt(sx * sy * sz);
    }
    
    /// Check if this is a rigid transform (rotation + translation only)
    bool isRigid(T tolerance = T(1e-6)) const noexcept {
        // Check if A is orthogonal with determinant +1
        Matrix3<T> AAt = A * A.transposed();
        Matrix3<T> I = Matrix3<T>::identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (std::abs(AAt(i, j) - I(i, j)) > tolerance)
                    return false;
            }
        }
        return std::abs(A.determinant() - 1) < tolerance;
    }
    
    // ==================== Interpolation ====================
    
    /// Linear interpolation between transformations
    static AffineXf3 lerp(const AffineXf3& a, const AffineXf3& b, T t) noexcept {
        // For rigid transforms, use SLERP for rotation
        // For general transforms, linear interpolation
        Matrix3<T> A_interp;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                A_interp(i, j) = a.A(i, j) * (1 - t) + b.A(i, j) * t;
            }
        }
        Vector3<T> b_interp = a.b * (1 - t) + b.b * t;
        return AffineXf3(A_interp, b_interp);
    }
    
    /// Spherical interpolation for rigid transforms
    static AffineXf3 slerp(const AffineXf3& a, const AffineXf3& b, T t) noexcept {
        Quaternion<T> qa = a.rotation();
        Quaternion<T> qb = b.rotation();
        Quaternion<T> q = Quaternion<T>::slerp(qa, qb, t);
        Vector3<T> translation = a.b * (1 - t) + b.b * t;
        return AffineXf3(q, translation);
    }
};

// ==================== 2D Affine Transformation ====================

/**
 * @brief 2D Affine transformation
 */
template <typename T>
struct AffineXf2 {
    T m00 = 1, m01 = 0;
    T m10 = 0, m11 = 1;
    T tx = 0, ty = 0;
    
    /// Default constructor - identity
    constexpr AffineXf2() noexcept = default;
    
    /// Transform a point
    constexpr Vector2<T> operator()(const Vector2<T>& p) const noexcept {
        return Vector2<T>(
            m00 * p.x + m01 * p.y + tx,
            m10 * p.x + m11 * p.y + ty
        );
    }
    
    /// Identity transformation
    static constexpr AffineXf2 identity() noexcept {
        return AffineXf2();
    }
    
    /// Pure translation
    static constexpr AffineXf2 translation(const Vector2<T>& t) noexcept {
        AffineXf2 xf;
        xf.tx = t.x;
        xf.ty = t.y;
        return xf;
    }
    
    /// Rotation around origin
    static AffineXf2 rotation(T angle) noexcept {
        T c = std::cos(angle);
        T s = std::sin(angle);
        AffineXf2 xf;
        xf.m00 = c; xf.m01 = -s;
        xf.m10 = s; xf.m11 = c;
        return xf;
    }
    
    /// Uniform scale
    static constexpr AffineXf2 scale(T s) noexcept {
        AffineXf2 xf;
        xf.m00 = s;
        xf.m11 = s;
        return xf;
    }
};

// ==================== Type Aliases ====================

using AffineXf3f = AffineXf3<float>;
using AffineXf3d = AffineXf3<double>;
using AffineXf2f = AffineXf2<float>;
using AffineXf2d = AffineXf2<double>;

// Backwards compatibility
using AffineTransform3f = AffineXf3f;
using AffineTransform3d = AffineXf3d;

} // namespace meshlib
