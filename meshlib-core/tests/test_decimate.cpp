/**
 * @file test_decimate.cpp
 * @brief Unit tests for mesh decimation algorithm
 */

#include <gtest/gtest.h>
#include "meshlib/algorithms/Decimate.h"
#include "meshlib/mesh/Primitives.h"

using namespace meshlib;

class DecimateTest : public ::testing::Test {
protected:
    Mesh sphere;
    Mesh box;
    
    void SetUp() override {
        sphere = createSphere(1.0f, 32, 16);
        box = createBox({1.0f, 1.0f, 1.0f});
    }
};

TEST_F(DecimateTest, DecimateBasic) {
    int originalCount = sphere.triangleCount();
    
    DecimateParams params;
    params.targetTriangleCount = originalCount / 2;
    
    auto result = decimate(sphere, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_FALSE(result.mesh.empty());
    EXPECT_LE(result.mesh.triangleCount(), params.targetTriangleCount);
}

TEST_F(DecimateTest, DecimateToCount) {
    int targetCount = 100;
    
    auto result = decimateToCount(sphere, targetCount);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_LE(result.mesh.triangleCount(), targetCount);
}

TEST_F(DecimateTest, DecimateByRatio) {
    int originalCount = sphere.triangleCount();
    
    auto result = decimateByRatio(sphere, 0.5f);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_LE(result.mesh.triangleCount(), static_cast<int>(originalCount * 0.55f));  // Some tolerance
}

TEST_F(DecimateTest, DecimatePreservesTopology) {
    auto result = decimateToCount(sphere, 50);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    
    // Decimated mesh should still be valid
    auto bbox = result.mesh.boundingBox();
    EXPECT_TRUE(bbox.valid());
    
    // Should still have positive area
    EXPECT_GT(result.mesh.area(), 0.0f);
}

TEST_F(DecimateTest, DecimateEmptyMesh) {
    Mesh empty;
    auto result = decimateToCount(empty, 10);
    
    EXPECT_NE(result.status.code, ErrorCode::Success);
}

TEST_F(DecimateTest, DecimateNoReductionNeeded) {
    // Target more triangles than exist
    int originalCount = box.triangleCount();
    
    auto result = decimateToCount(box, originalCount + 100);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_EQ(result.mesh.triangleCount(), originalCount);  // No change
}

TEST_F(DecimateTest, DecimateWithParams) {
    DecimateParams params;
    params.targetTriangleCount = 50;
    params.maxError = 0.1f;
    params.preserveBoundary = true;
    
    auto result = decimate(sphere, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
}

TEST_F(DecimateTest, GenerateLODs) {
    std::vector<int> lodCounts = {200, 100, 50, 25};
    
    auto lods = generateLODs(sphere, lodCounts);
    
    EXPECT_EQ(lods.size(), lodCounts.size());
    
    // Each LOD should have fewer triangles than the previous
    for (size_t i = 0; i < lods.size() - 1; ++i) {
        EXPECT_GE(lods[i].triangleCount(), lods[i + 1].triangleCount());
    }
    
    // Last LOD should have approximately the requested count
    EXPECT_LE(lods.back().triangleCount(), lodCounts.back() + 10);  // Some tolerance
}

TEST_F(DecimateTest, DecimatePreservesBounds) {
    auto originalBbox = sphere.boundingBox();
    
    auto result = decimateByRatio(sphere, 0.3f);
    
    auto decimatedBbox = result.mesh.boundingBox();
    
    // Bounding box should be similar (within tolerance)
    EXPECT_NEAR(decimatedBbox.center().x, originalBbox.center().x, 0.1f);
    EXPECT_NEAR(decimatedBbox.center().y, originalBbox.center().y, 0.1f);
    EXPECT_NEAR(decimatedBbox.center().z, originalBbox.center().z, 0.1f);
}

TEST_F(DecimateTest, DecimateProgressCallback) {
    bool callbackCalled = false;
    
    DecimateParams params;
    params.targetTriangleCount = 50;
    params.callback = [&callbackCalled](float progress) {
        callbackCalled = true;
        return true;  // Continue
    };
    
    decimate(sphere, params);
    
    EXPECT_TRUE(callbackCalled);
}

TEST_F(DecimateTest, DecimateCancellation) {
    DecimateParams params;
    params.targetTriangleCount = 10;
    params.callback = [](float progress) {
        return false;  // Cancel immediately
    };
    
    auto result = decimate(sphere, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Cancelled);
}

TEST_F(DecimateTest, DecimateToError) {
    auto result = decimateToError(sphere, 0.05f);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_FALSE(result.mesh.empty());
    
    // Should have reduced the triangle count
    EXPECT_LT(result.mesh.triangleCount(), sphere.triangleCount());
}
