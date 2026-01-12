#pragma once

/**
 * @file Matrix.h
 * @brief 3x3 and 4x4 matrix types
 */

#include "Vector.h"
#include <array>
#include <cmath>

namespace meshlib {

/**
 * @brief 3x3 matrix template (column-major storage)
 */
template <typename T>
struct Matrix3 {
    Vector3<T> cols[3];  // Column vectors
    
    /// Default constructor - identity matrix
    constexpr Matrix3() noexcept 
        : cols{{T(1), T(0), T(0)}, {T(0), T(1), T(0)}, {T(0), T(0), T(1)}} {}
    
    /// Construct from column vectors
    constexpr Matrix3(const Vector3<T>& c0, const Vector3<T>& c1, const Vector3<T>& c2) noexcept
        : cols{c0, c1, c2} {}
    
    /// Construct from individual elements (row-major input for convenience)
    constexpr Matrix3(T m00, T m01, T m02,
                      T m10, T m11, T m12,
                      T m20, T m21, T m22) noexcept
        : cols{{m00, m10, m20}, {m01, m11, m21}, {m02, m12, m22}} {}
    
    // Column access
    constexpr Vector3<T>& operator[](int i) noexcept { return cols[i]; }
    constexpr const Vector3<T>& operator[](int i) const noexcept { return cols[i]; }
    
    // Element access (row, column)
    constexpr T& operator()(int row, int col) noexcept { return cols[col][row]; }
    constexpr const T& operator()(int row, int col) const noexcept { return cols[col][row]; }
    
    /// Get row vector
    constexpr Vector3<T> row(int i) const noexcept {
        return {cols[0][i], cols[1][i], cols[2][i]};
    }
    
    /// Matrix-vector multiplication
    constexpr Vector3<T> operator*(const Vector3<T>& v) const noexcept {
        return cols[0] * v.x + cols[1] * v.y + cols[2] * v.z;
    }
    
    /// Matrix-matrix multiplication
    constexpr Matrix3 operator*(const Matrix3& m) const noexcept {
        return {
            (*this) * m.cols[0],
            (*this) * m.cols[1],
            (*this) * m.cols[2]
        };
    }
    
    /// Transpose
    constexpr Matrix3 transposed() const noexcept {
        return {
            cols[0].x, cols[0].y, cols[0].z,
            cols[1].x, cols[1].y, cols[1].z,
            cols[2].x, cols[2].y, cols[2].z
        };
    }
    
    /// Determinant
    constexpr T determinant() const noexcept {
        return cols[0].x * (cols[1].y * cols[2].z - cols[2].y * cols[1].z)
             - cols[1].x * (cols[0].y * cols[2].z - cols[2].y * cols[0].z)
             + cols[2].x * (cols[0].y * cols[1].z - cols[1].y * cols[0].z);
    }
    
    /// Inverse (returns identity if singular)
    Matrix3 inverse() const noexcept {
        T det = determinant();
        if (std::abs(det) < T(1e-10)) {
            return identity();
        }
        
        T invDet = T(1) / det;
        Matrix3 result;
        
        result(0, 0) = (cols[1].y * cols[2].z - cols[2].y * cols[1].z) * invDet;
        result(0, 1) = (cols[2].x * cols[1].z - cols[1].x * cols[2].z) * invDet;
        result(0, 2) = (cols[1].x * cols[2].y - cols[2].x * cols[1].y) * invDet;
        
        result(1, 0) = (cols[2].y * cols[0].z - cols[0].y * cols[2].z) * invDet;
        result(1, 1) = (cols[0].x * cols[2].z - cols[2].x * cols[0].z) * invDet;
        result(1, 2) = (cols[2].x * cols[0].y - cols[0].x * cols[2].y) * invDet;
        
        result(2, 0) = (cols[0].y * cols[1].z - cols[1].y * cols[0].z) * invDet;
        result(2, 1) = (cols[1].x * cols[0].z - cols[0].x * cols[1].z) * invDet;
        result(2, 2) = (cols[0].x * cols[1].y - cols[1].x * cols[0].y) * invDet;
        
        return result;
    }
    
    // Static constructors
    static constexpr Matrix3 identity() noexcept {
        return {};
    }
    
    static constexpr Matrix3 zero() noexcept {
        return {
            {T(0), T(0), T(0)},
            {T(0), T(0), T(0)},
            {T(0), T(0), T(0)}
        };
    }
    
    static constexpr Matrix3 scale(T s) noexcept {
        return {
            s, T(0), T(0),
            T(0), s, T(0),
            T(0), T(0), s
        };
    }
    
    static constexpr Matrix3 scale(const Vector3<T>& s) noexcept {
        return {
            s.x, T(0), T(0),
            T(0), s.y, T(0),
            T(0), T(0), s.z
        };
    }
    
    /// Rotation around X axis
    static Matrix3 rotationX(T angle) noexcept {
        T c = std::cos(angle);
        T s = std::sin(angle);
        return {
            T(1), T(0), T(0),
            T(0), c, -s,
            T(0), s, c
        };
    }
    
    /// Rotation around Y axis
    static Matrix3 rotationY(T angle) noexcept {
        T c = std::cos(angle);
        T s = std::sin(angle);
        return {
            c, T(0), s,
            T(0), T(1), T(0),
            -s, T(0), c
        };
    }
    
    /// Rotation around Z axis
    static Matrix3 rotationZ(T angle) noexcept {
        T c = std::cos(angle);
        T s = std::sin(angle);
        return {
            c, -s, T(0),
            s, c, T(0),
            T(0), T(0), T(1)
        };
    }
    
    /// Rotation around arbitrary axis (axis must be normalized)
    static Matrix3 rotation(const Vector3<T>& axis, T angle) noexcept {
        T c = std::cos(angle);
        T s = std::sin(angle);
        T t = T(1) - c;
        
        T x = axis.x, y = axis.y, z = axis.z;
        
        return {
            t*x*x + c,     t*x*y - s*z,   t*x*z + s*y,
            t*x*y + s*z,   t*y*y + c,     t*y*z - s*x,
            t*x*z - s*y,   t*y*z + s*x,   t*z*z + c
        };
    }
};

/**
 * @brief Affine transformation (3x3 rotation/scale + translation)
 */
template <typename T>
struct AffineTransform3 {
    Matrix3<T> linear;    // Linear part (rotation/scale)
    Vector3<T> translation;  // Translation
    
    /// Default constructor - identity transform
    constexpr AffineTransform3() noexcept : linear(), translation() {}
    
    /// Construct from linear and translation parts
    constexpr AffineTransform3(const Matrix3<T>& lin, const Vector3<T>& trans) noexcept
        : linear(lin), translation(trans) {}
    
    /// Transform a point
    constexpr Vector3<T> operator()(const Vector3<T>& p) const noexcept {
        return linear * p + translation;
    }
    
    /// Transform a direction (ignores translation)
    constexpr Vector3<T> transformDirection(const Vector3<T>& d) const noexcept {
        return linear * d;
    }
    
    /// Compose transformations: this * other
    constexpr AffineTransform3 operator*(const AffineTransform3& other) const noexcept {
        return {
            linear * other.linear,
            linear * other.translation + translation
        };
    }
    
    /// Inverse transform
    AffineTransform3 inverse() const noexcept {
        Matrix3<T> invLinear = linear.inverse();
        return {invLinear, invLinear * (-translation)};
    }
    
    // Static constructors
    static constexpr AffineTransform3 identity() noexcept {
        return {};
    }
    
    static constexpr AffineTransform3 fromTranslation(const Vector3<T>& t) noexcept {
        return {Matrix3<T>::identity(), t};
    }
    
    static AffineTransform3 fromRotation(const Matrix3<T>& r) noexcept {
        return {r, Vector3<T>::zero()};
    }
    
    static constexpr AffineTransform3 fromScale(T s) noexcept {
        return {Matrix3<T>::scale(s), Vector3<T>::zero()};
    }
};

// Type aliases
using Matrix3f = Matrix3<float>;
using Matrix3d = Matrix3<double>;

using AffineTransform3f = AffineTransform3<float>;
using AffineTransform3d = AffineTransform3<double>;

} // namespace meshlib
