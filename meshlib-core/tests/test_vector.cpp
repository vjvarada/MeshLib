/**
 * @file test_vector.cpp
 * @brief Unit tests for Vector2 and Vector3 classes
 */

#include <gtest/gtest.h>
#include "meshlib/core/Vector.h"
#include <cmath>

using namespace meshlib;

class Vector2Test : public ::testing::Test {
protected:
    Vector2f v1{3.0f, 4.0f};
    Vector2f v2{1.0f, 2.0f};
};

TEST_F(Vector2Test, DefaultConstructor) {
    Vector2f v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
}

TEST_F(Vector2Test, ParameterizedConstructor) {
    EXPECT_FLOAT_EQ(v1.x, 3.0f);
    EXPECT_FLOAT_EQ(v1.y, 4.0f);
}

TEST_F(Vector2Test, IndexOperator) {
    EXPECT_FLOAT_EQ(v1[0], 3.0f);
    EXPECT_FLOAT_EQ(v1[1], 4.0f);
    
    v1[0] = 5.0f;
    EXPECT_FLOAT_EQ(v1[0], 5.0f);
}

TEST_F(Vector2Test, Addition) {
    auto result = v1 + v2;
    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
}

TEST_F(Vector2Test, Subtraction) {
    auto result = v1 - v2;
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
}

TEST_F(Vector2Test, ScalarMultiplication) {
    auto result = v1 * 2.0f;
    EXPECT_FLOAT_EQ(result.x, 6.0f);
    EXPECT_FLOAT_EQ(result.y, 8.0f);
}

TEST_F(Vector2Test, ScalarDivision) {
    auto result = v1 / 2.0f;
    EXPECT_FLOAT_EQ(result.x, 1.5f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
}

TEST_F(Vector2Test, DotProduct) {
    float dot = v1.dot(v2);
    EXPECT_FLOAT_EQ(dot, 11.0f);  // 3*1 + 4*2 = 11
}

TEST_F(Vector2Test, Length) {
    EXPECT_FLOAT_EQ(v1.length(), 5.0f);  // sqrt(9 + 16) = 5
}

TEST_F(Vector2Test, LengthSquared) {
    EXPECT_FLOAT_EQ(v1.lengthSq(), 25.0f);  // 9 + 16 = 25
}

TEST_F(Vector2Test, Normalized) {
    auto norm = v1.normalized();
    EXPECT_FLOAT_EQ(norm.x, 0.6f);
    EXPECT_FLOAT_EQ(norm.y, 0.8f);
    EXPECT_NEAR(norm.length(), 1.0f, 1e-6f);
}

TEST_F(Vector2Test, Zero) {
    auto zero = Vector2f::zero();
    EXPECT_FLOAT_EQ(zero.x, 0.0f);
    EXPECT_FLOAT_EQ(zero.y, 0.0f);
}

TEST_F(Vector2Test, UnitX) {
    auto unitX = Vector2f::unitX();
    EXPECT_FLOAT_EQ(unitX.x, 1.0f);
    EXPECT_FLOAT_EQ(unitX.y, 0.0f);
}

// Vector3 Tests
class Vector3Test : public ::testing::Test {
protected:
    Vector3f v1{1.0f, 2.0f, 3.0f};
    Vector3f v2{4.0f, 5.0f, 6.0f};
};

TEST_F(Vector3Test, DefaultConstructor) {
    Vector3f v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST_F(Vector3Test, ParameterizedConstructor) {
    EXPECT_FLOAT_EQ(v1.x, 1.0f);
    EXPECT_FLOAT_EQ(v1.y, 2.0f);
    EXPECT_FLOAT_EQ(v1.z, 3.0f);
}

TEST_F(Vector3Test, Addition) {
    auto result = v1 + v2;
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 7.0f);
    EXPECT_FLOAT_EQ(result.z, 9.0f);
}

TEST_F(Vector3Test, Subtraction) {
    auto result = v2 - v1;
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(Vector3Test, DotProduct) {
    float dot = v1.dot(v2);
    EXPECT_FLOAT_EQ(dot, 32.0f);  // 1*4 + 2*5 + 3*6 = 32
}

TEST_F(Vector3Test, CrossProduct) {
    auto cross = v1.cross(v2);
    // (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
    EXPECT_FLOAT_EQ(cross.x, -3.0f);
    EXPECT_FLOAT_EQ(cross.y, 6.0f);
    EXPECT_FLOAT_EQ(cross.z, -3.0f);
}

TEST_F(Vector3Test, Length) {
    EXPECT_FLOAT_EQ(v1.length(), std::sqrt(14.0f));  // sqrt(1 + 4 + 9)
}

TEST_F(Vector3Test, Normalized) {
    auto norm = v1.normalized();
    EXPECT_NEAR(norm.length(), 1.0f, 1e-6f);
}

TEST_F(Vector3Test, Negation) {
    auto neg = -v1;
    EXPECT_FLOAT_EQ(neg.x, -1.0f);
    EXPECT_FLOAT_EQ(neg.y, -2.0f);
    EXPECT_FLOAT_EQ(neg.z, -3.0f);
}

TEST_F(Vector3Test, UnitVectors) {
    auto unitX = Vector3f::unitX();
    auto unitY = Vector3f::unitY();
    auto unitZ = Vector3f::unitZ();
    
    EXPECT_FLOAT_EQ(unitX.x, 1.0f);
    EXPECT_FLOAT_EQ(unitY.y, 1.0f);
    EXPECT_FLOAT_EQ(unitZ.z, 1.0f);
    
    // Cross products of unit vectors
    auto cross = unitX.cross(unitY);
    EXPECT_FLOAT_EQ(cross.x, unitZ.x);
    EXPECT_FLOAT_EQ(cross.y, unitZ.y);
    EXPECT_FLOAT_EQ(cross.z, unitZ.z);
}

TEST_F(Vector3Test, Equality) {
    Vector3f v3{1.0f, 2.0f, 3.0f};
    EXPECT_EQ(v1, v3);
    EXPECT_NE(v1, v2);
}

TEST_F(Vector3Test, CompoundAssignment) {
    Vector3f v = v1;
    v += v2;
    EXPECT_FLOAT_EQ(v.x, 5.0f);
    EXPECT_FLOAT_EQ(v.y, 7.0f);
    EXPECT_FLOAT_EQ(v.z, 9.0f);
    
    v = v1;
    v *= 2.0f;
    EXPECT_FLOAT_EQ(v.x, 2.0f);
    EXPECT_FLOAT_EQ(v.y, 4.0f);
    EXPECT_FLOAT_EQ(v.z, 6.0f);
}
