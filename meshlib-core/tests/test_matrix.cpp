/**
 * @file test_matrix.cpp
 * @brief Unit tests for Matrix3 and AffineTransform3 classes
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <gtest/gtest.h>
#include "meshlib/core/Matrix.h"
#include "meshlib/core/Vector.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace meshlib;

class Matrix3Test : public ::testing::Test {
protected:
    Matrix3f identity = Matrix3f::identity();
    Matrix3f m1;
    
    void SetUp() override {
        // Set up a test matrix
        m1(0, 0) = 1.0f; m1(0, 1) = 2.0f; m1(0, 2) = 3.0f;
        m1(1, 0) = 4.0f; m1(1, 1) = 5.0f; m1(1, 2) = 6.0f;
        m1(2, 0) = 7.0f; m1(2, 1) = 8.0f; m1(2, 2) = 9.0f;
    }
};

TEST_F(Matrix3Test, Identity) {
    EXPECT_FLOAT_EQ(identity(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(identity(1, 1), 1.0f);
    EXPECT_FLOAT_EQ(identity(2, 2), 1.0f);
    EXPECT_FLOAT_EQ(identity(0, 1), 0.0f);
    EXPECT_FLOAT_EQ(identity(1, 0), 0.0f);
}

TEST_F(Matrix3Test, Zero) {
    Matrix3f zero = Matrix3f::zero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_FLOAT_EQ(zero(i, j), 0.0f);
        }
    }
}

TEST_F(Matrix3Test, IndexOperator) {
    EXPECT_FLOAT_EQ(m1(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m1(1, 2), 6.0f);
    EXPECT_FLOAT_EQ(m1(2, 0), 7.0f);
}

TEST_F(Matrix3Test, VectorMultiplication) {
    Vector3f v{1.0f, 0.0f, 0.0f};
    auto result = identity * v;
    
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
    
    result = m1 * v;
    EXPECT_FLOAT_EQ(result.x, 1.0f);  // First column
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 7.0f);
}

TEST_F(Matrix3Test, MatrixMultiplication) {
    auto result = identity * m1;
    
    // Multiplying by identity should give same matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_FLOAT_EQ(result(i, j), m1(i, j));
        }
    }
}

TEST_F(Matrix3Test, Transpose) {
    auto t = m1.transposed();
    
    EXPECT_FLOAT_EQ(t(0, 1), m1(1, 0));
    EXPECT_FLOAT_EQ(t(0, 2), m1(2, 0));
    EXPECT_FLOAT_EQ(t(1, 0), m1(0, 1));
    EXPECT_FLOAT_EQ(t(1, 2), m1(2, 1));
}

TEST_F(Matrix3Test, RotationX) {
    float angle = static_cast<float>(M_PI / 2);  // 90 degrees
    auto r = Matrix3f::rotationX(angle);
    
    // Rotating unit Y should give unit Z
    Vector3f y = Vector3f::unitY();
    Vector3f result = r * y;
    
    EXPECT_NEAR(result.x, 0.0f, 1e-6f);
    EXPECT_NEAR(result.y, 0.0f, 1e-6f);
    EXPECT_NEAR(result.z, 1.0f, 1e-6f);
}

TEST_F(Matrix3Test, RotationY) {
    float angle = static_cast<float>(M_PI / 2);  // 90 degrees
    auto r = Matrix3f::rotationY(angle);
    
    // Rotating unit Z should give unit X
    Vector3f z = Vector3f::unitZ();
    Vector3f result = r * z;
    
    EXPECT_NEAR(result.x, 1.0f, 1e-6f);
    EXPECT_NEAR(result.y, 0.0f, 1e-6f);
    EXPECT_NEAR(result.z, 0.0f, 1e-6f);
}

TEST_F(Matrix3Test, RotationZ) {
    float angle = static_cast<float>(M_PI / 2);  // 90 degrees
    auto r = Matrix3f::rotationZ(angle);
    
    // Rotating unit X should give unit Y
    Vector3f x = Vector3f::unitX();
    Vector3f result = r * x;
    
    EXPECT_NEAR(result.x, 0.0f, 1e-6f);
    EXPECT_NEAR(result.y, 1.0f, 1e-6f);
    EXPECT_NEAR(result.z, 0.0f, 1e-6f);
}

TEST_F(Matrix3Test, ScaleUniform) {
    auto s = Matrix3f::scale(2.0f);
    Vector3f v{1.0f, 1.0f, 1.0f};
    auto result = s * v;
    
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 2.0f);
}

TEST_F(Matrix3Test, ScaleNonUniform) {
    // Matrix3f::scale expects a Vector3f for non-uniform scaling
    auto s = Matrix3f::scale(Vector3f{2.0f, 3.0f, 4.0f});
    Vector3f v{1.0f, 1.0f, 1.0f};
    auto result = s * v;
    
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 4.0f);
}

TEST_F(Matrix3Test, Determinant) {
    // Identity matrix determinant should be 1
    EXPECT_FLOAT_EQ(identity.determinant(), 1.0f);
    
    // Singular matrix (m1) has determinant 0
    // det([1,2,3],[4,5,6],[7,8,9]) = 0
    EXPECT_NEAR(m1.determinant(), 0.0f, 1e-5f);
}

TEST_F(Matrix3Test, Inverse) {
    // Create an invertible matrix
    Matrix3f invertible(
        1.0f, 0.0f, 0.0f,
        0.0f, 2.0f, 0.0f,
        0.0f, 0.0f, 3.0f
    );
    
    auto inv = invertible.inverse();
    
    // Check that M * M^-1 = I
    auto product = invertible * inv;
    EXPECT_NEAR(product(0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(product(1, 1), 1.0f, 1e-5f);
    EXPECT_NEAR(product(2, 2), 1.0f, 1e-5f);
    EXPECT_NEAR(product(0, 1), 0.0f, 1e-5f);
}

// AffineTransform3 Tests
class AffineTransformTest : public ::testing::Test {
protected:
    AffineTransform3f identity;  // Default constructor creates identity
};

TEST_F(AffineTransformTest, Identity) {
    Vector3f v{1.0f, 2.0f, 3.0f};
    auto result = identity(v);  // operator() transforms a point
    
    EXPECT_FLOAT_EQ(result.x, v.x);
    EXPECT_FLOAT_EQ(result.y, v.y);
    EXPECT_FLOAT_EQ(result.z, v.z);
}

TEST_F(AffineTransformTest, Translation) {
    // Use fromTranslation static method
    auto t = AffineTransform3f::fromTranslation({1.0f, 2.0f, 3.0f});
    Vector3f v{0.0f, 0.0f, 0.0f};
    auto result = t(v);  // operator() transforms a point
    
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(AffineTransformTest, TranslationMember) {
    // Create transform and set translation directly
    AffineTransform3f t;
    t.translation = {1.0f, 2.0f, 3.0f};
    
    Vector3f v{0.0f, 0.0f, 0.0f};
    auto result = t(v);
    
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(AffineTransformTest, RotationViaLinear) {
    float angle = static_cast<float>(M_PI / 2);  // 90 degrees
    // Create rotation using Matrix3f::rotationX and fromRotation
    auto rotMatrix = Matrix3f::rotationX(angle);
    auto r = AffineTransform3f::fromRotation(rotMatrix);
    
    Vector3f y = Vector3f::unitY();
    auto result = r(y);
    
    EXPECT_NEAR(result.x, 0.0f, 1e-6f);
    EXPECT_NEAR(result.y, 0.0f, 1e-6f);
    EXPECT_NEAR(result.z, 1.0f, 1e-6f);
}

TEST_F(AffineTransformTest, ScaleUniform) {
    auto s = AffineTransform3f::fromScale(2.0f);
    Vector3f v{1.0f, 2.0f, 3.0f};
    auto result = s(v);
    
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
}

TEST_F(AffineTransformTest, ScaleNonUniform) {
    // For non-uniform scale, set the linear part directly
    AffineTransform3f s;
    s.linear = Matrix3f::scale(Vector3f{2.0f, 3.0f, 4.0f});
    
    Vector3f v{1.0f, 1.0f, 1.0f};
    auto result = s(v);
    
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 4.0f);
}

TEST_F(AffineTransformTest, TransformDirection) {
    auto t = AffineTransform3f::fromTranslation({10.0f, 20.0f, 30.0f});
    Vector3f dir{1.0f, 0.0f, 0.0f};
    auto result = t.transformDirection(dir);
    
    // Direction should not be affected by translation
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(AffineTransformTest, Composition) {
    auto t = AffineTransform3f::fromTranslation({1.0f, 0.0f, 0.0f});
    auto s = AffineTransform3f::fromScale(2.0f);
    
    // First scale, then translate
    auto composed = t * s;
    Vector3f v{1.0f, 0.0f, 0.0f};
    auto result = composed(v);
    
    // Scale to (2, 0, 0), then translate to (3, 0, 0)
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST_F(AffineTransformTest, Inverse) {
    auto t = AffineTransform3f::fromTranslation({1.0f, 2.0f, 3.0f});
    auto inv = t.inverse();
    
    Vector3f v{0.0f, 0.0f, 0.0f};
    auto transformed = t(v);
    auto backToOriginal = inv(transformed);
    
    EXPECT_NEAR(backToOriginal.x, v.x, 1e-5f);
    EXPECT_NEAR(backToOriginal.y, v.y, 1e-5f);
    EXPECT_NEAR(backToOriginal.z, v.z, 1e-5f);
}
