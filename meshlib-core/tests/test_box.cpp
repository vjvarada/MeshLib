/**
 * @file test_box.cpp
 * @brief Unit tests for Box3 class (axis-aligned bounding box)
 */

#include <gtest/gtest.h>
#include "meshlib/core/Box.h"
#include "meshlib/core/Vector.h"
#include <cmath>

using namespace meshlib;

class Box3Test : public ::testing::Test {
protected:
    Box3f box1{{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};
    Box3f box2{{0.5f, 0.5f, 0.5f}, {1.5f, 1.5f, 1.5f}};
};

TEST_F(Box3Test, DefaultConstructor) {
    Box3f box;
    EXPECT_FALSE(box.valid());
}

TEST_F(Box3Test, ParameterizedConstructor) {
    EXPECT_FLOAT_EQ(box1.min.x, 0.0f);
    EXPECT_FLOAT_EQ(box1.max.x, 1.0f);
    EXPECT_FLOAT_EQ(box1.min.y, 0.0f);
    EXPECT_FLOAT_EQ(box1.max.y, 1.0f);
}

TEST_F(Box3Test, Valid) {
    EXPECT_TRUE(box1.valid());
    
    Box3f invalid{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}};  // min.x > max.x
    EXPECT_FALSE(invalid.valid());
}

TEST_F(Box3Test, Empty) {
    Box3f emptyBox;
    EXPECT_TRUE(emptyBox.empty());
    EXPECT_FALSE(box1.empty());
}

TEST_F(Box3Test, Center) {
    auto center = box1.center();
    EXPECT_FLOAT_EQ(center.x, 0.5f);
    EXPECT_FLOAT_EQ(center.y, 0.5f);
    EXPECT_FLOAT_EQ(center.z, 0.5f);
}

TEST_F(Box3Test, Size) {
    auto size = box1.size();
    EXPECT_FLOAT_EQ(size.x, 1.0f);
    EXPECT_FLOAT_EQ(size.y, 1.0f);
    EXPECT_FLOAT_EQ(size.z, 1.0f);
}

TEST_F(Box3Test, Diagonal) {
    float diag = box1.diagonal();
    EXPECT_NEAR(diag, std::sqrt(3.0f), 1e-6f);
}

TEST_F(Box3Test, Volume) {
    float vol = box1.volume();
    EXPECT_FLOAT_EQ(vol, 1.0f);
}

TEST_F(Box3Test, SurfaceArea) {
    float area = box1.surfaceArea();
    EXPECT_FLOAT_EQ(area, 6.0f);
}

TEST_F(Box3Test, ContainsPoint) {
    EXPECT_TRUE(box1.contains({0.5f, 0.5f, 0.5f}));
    EXPECT_TRUE(box1.contains({0.0f, 0.0f, 0.0f}));
    EXPECT_TRUE(box1.contains({1.0f, 1.0f, 1.0f}));
    EXPECT_FALSE(box1.contains({1.5f, 0.5f, 0.5f}));
    EXPECT_FALSE(box1.contains({-0.1f, 0.5f, 0.5f}));
}

TEST_F(Box3Test, Intersects) {
    EXPECT_TRUE(box1.intersects(box2));
    
    Box3f distant{{5.0f, 5.0f, 5.0f}, {6.0f, 6.0f, 6.0f}};
    EXPECT_FALSE(box1.intersects(distant));
}

TEST_F(Box3Test, Intersection) {
    auto inter = box1.intersection(box2);
    EXPECT_TRUE(inter.valid());
    EXPECT_FLOAT_EQ(inter.min.x, 0.5f);
    EXPECT_FLOAT_EQ(inter.max.x, 1.0f);
}

TEST_F(Box3Test, IncludePoint) {
    Box3f box = box1;
    box.include({2.0f, 2.0f, 2.0f});
    
    EXPECT_FLOAT_EQ(box.max.x, 2.0f);
    EXPECT_FLOAT_EQ(box.max.y, 2.0f);
    EXPECT_FLOAT_EQ(box.max.z, 2.0f);
}

TEST_F(Box3Test, IncludeBox) {
    Box3f box = box1;
    Box3f other{{-1.0f, -1.0f, -1.0f}, {2.0f, 2.0f, 2.0f}};
    box.include(other);
    
    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
    EXPECT_FLOAT_EQ(box.max.x, 2.0f);
}

TEST_F(Box3Test, Expanded) {
    // expanded takes a float margin, not a Vector3f
    auto expanded = box1.expanded(0.5f);
    
    EXPECT_FLOAT_EQ(expanded.min.x, -0.5f);
    EXPECT_FLOAT_EQ(expanded.max.x, 1.5f);
    EXPECT_FLOAT_EQ(expanded.min.y, -0.5f);
    EXPECT_FLOAT_EQ(expanded.max.y, 1.5f);
    EXPECT_FLOAT_EQ(expanded.min.z, -0.5f);
    EXPECT_FLOAT_EQ(expanded.max.z, 1.5f);
}

TEST_F(Box3Test, Corner) {
    auto c0 = box1.corner(0);  // (min, min, min)
    EXPECT_FLOAT_EQ(c0.x, 0.0f);
    EXPECT_FLOAT_EQ(c0.y, 0.0f);
    EXPECT_FLOAT_EQ(c0.z, 0.0f);
    
    auto c7 = box1.corner(7);  // (max, max, max)
    EXPECT_FLOAT_EQ(c7.x, 1.0f);
    EXPECT_FLOAT_EQ(c7.y, 1.0f);
    EXPECT_FLOAT_EQ(c7.z, 1.0f);
}

TEST_F(Box3Test, Equality) {
    Box3f box3{{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};
    EXPECT_EQ(box1, box3);
    EXPECT_NE(box1, box2);
}

TEST_F(Box3Test, InvalidIntersection) {
    Box3f distant{{10.0f, 10.0f, 10.0f}, {11.0f, 11.0f, 11.0f}};
    auto inter = box1.intersection(distant);
    EXPECT_FALSE(inter.valid());
}

TEST_F(Box3Test, FromCenterHalfSize) {
    auto box = Box3f::fromCenterHalfSize({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
    EXPECT_FLOAT_EQ(box.max.x, 1.0f);
}

TEST_F(Box3Test, FromCenterSize) {
    auto box = Box3f::fromCenterSize({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
    EXPECT_FLOAT_EQ(box.max.x, 1.0f);
}

TEST_F(Box3Test, FromPoint) {
    auto box = Box3f::fromPoint({1.0f, 2.0f, 3.0f});
    EXPECT_FLOAT_EQ(box.min.x, 1.0f);
    EXPECT_FLOAT_EQ(box.max.x, 1.0f);
    EXPECT_FLOAT_EQ(box.min.y, 2.0f);
    EXPECT_FLOAT_EQ(box.max.y, 2.0f);
}

TEST_F(Box3Test, FromPoints) {
    std::vector<Vector3f> points = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 2.0f, 3.0f},
        {-1.0f, -2.0f, -3.0f}
    };
    auto box = Box3f::fromPoints(points.begin(), points.end());
    
    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
    EXPECT_FLOAT_EQ(box.max.x, 1.0f);
    EXPECT_FLOAT_EQ(box.min.y, -2.0f);
    EXPECT_FLOAT_EQ(box.max.y, 2.0f);
    EXPECT_FLOAT_EQ(box.min.z, -3.0f);
    EXPECT_FLOAT_EQ(box.max.z, 3.0f);
}
