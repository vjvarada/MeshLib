/**
 * @file test_subdivide.cpp
 * @brief Unit tests for mesh subdivision algorithms
 */

#include <gtest/gtest.h>
#include "meshlib/algorithms/Subdivide.h"
#include "meshlib/mesh/Primitives.h"

using namespace meshlib;

class SubdivideTest : public ::testing::Test {
protected:
    Mesh box;
    Mesh triangle;
    
    void SetUp() override {
        box = createBox({1.0f, 1.0f, 1.0f});
        
        // Create a single triangle
        std::vector<Vector3f> vertices = {
            {0.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
            {0.5f, 1.0f, 0.0f}
        };
        std::vector<Triangle> triangles = {{0, 1, 2}};
        triangle = Mesh::fromTriangles(vertices, triangles);
    }
};

TEST_F(SubdivideTest, SubdivideBasic) {
    SubdivideParams params;
    params.iterations = 1;
    
    auto result = subdivide(box, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_FALSE(result.mesh.empty());
    EXPECT_GT(result.mesh.triangleCount(), box.triangleCount());
}

TEST_F(SubdivideTest, SubdivideLoop) {
    auto subdivided = subdivideLoop(triangle);
    
    // One triangle becomes 4 triangles
    EXPECT_EQ(subdivided.triangleCount(), 4);
}

TEST_F(SubdivideTest, SubdivideMidpoint) {
    auto subdivided = subdivideMidpoint(triangle);
    
    // One triangle becomes 4 triangles
    EXPECT_EQ(subdivided.triangleCount(), 4);
}

TEST_F(SubdivideTest, SubdivideMultipleIterations) {
    SubdivideParams params;
    params.iterations = 2;
    params.scheme = SubdivisionScheme::Loop;
    
    auto result = subdivide(triangle, params);
    
    // After 2 iterations: 1 -> 4 -> 16 triangles
    EXPECT_EQ(result.mesh.triangleCount(), 16);
}

TEST_F(SubdivideTest, SubdividePreservesBounds) {
    auto originalBbox = box.boundingBox();
    
    SubdivideParams params;
    params.iterations = 2;
    
    auto result = subdivide(box, params);
    auto subdividedBbox = result.mesh.boundingBox();
    
    // Bounding box should be preserved (midpoint subdivision doesn't move vertices)
    EXPECT_NEAR(subdividedBbox.min.x, originalBbox.min.x, 1e-5f);
    EXPECT_NEAR(subdividedBbox.max.x, originalBbox.max.x, 1e-5f);
}

TEST_F(SubdivideTest, SubdivideEmptyMesh) {
    Mesh empty;
    
    SubdivideParams params;
    params.iterations = 1;
    
    auto result = subdivide(empty, params);
    
    EXPECT_NE(result.status.code, ErrorCode::Success);
}

TEST_F(SubdivideTest, SubdivideZeroIterations) {
    SubdivideParams params;
    params.iterations = 0;
    
    auto result = subdivide(box, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_EQ(result.mesh.triangleCount(), box.triangleCount());  // Unchanged
}

TEST_F(SubdivideTest, SubdivideAdaptive) {
    // Create a mesh with varying edge lengths
    auto coarse = createBox({10.0f, 10.0f, 10.0f});
    
    auto subdivided = subdivideAdaptive(coarse, 5.0f, 3);
    
    EXPECT_GT(subdivided.triangleCount(), coarse.triangleCount());
    
    // Check that no edge is longer than maxEdgeLength * 2 (accounting for iterations)
    const auto& vertices = subdivided.vertices();
    const auto& triangles = subdivided.triangles();
    
    for (const auto& tri : triangles) {
        for (int i = 0; i < 3; ++i) {
            int v0 = tri[i];
            int v1 = tri[(i + 1) % 3];
            float edgeLen = (vertices[v1] - vertices[v0]).length();
            // After adaptive subdivision, edges should be smaller
            EXPECT_LT(edgeLen, 10.0f);  // Original box diagonal was ~17
        }
    }
}

TEST_F(SubdivideTest, SubdivideIncreasesVertexCount) {
    int originalVertices = box.vertexCount();
    
    auto subdivided = subdivideLoop(box);
    
    EXPECT_GT(subdivided.vertexCount(), originalVertices);
}

TEST_F(SubdivideTest, SubdivideProgressCallback) {
    bool callbackCalled = false;
    
    SubdivideParams params;
    params.iterations = 2;
    params.callback = [&callbackCalled](float progress) {
        callbackCalled = true;
        return true;  // Continue
    };
    
    subdivide(box, params);
    
    EXPECT_TRUE(callbackCalled);
}

TEST_F(SubdivideTest, SubdivideCancellation) {
    SubdivideParams params;
    params.iterations = 5;
    params.callback = [](float progress) {
        return false;  // Cancel immediately
    };
    
    auto result = subdivide(box, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Cancelled);
}

TEST_F(SubdivideTest, SubdivideSchemes) {
    SubdivideParams params;
    params.iterations = 1;
    
    // Test Loop scheme
    params.scheme = SubdivisionScheme::Loop;
    auto loop = subdivide(box, params);
    EXPECT_EQ(loop.status.code, ErrorCode::Success);
    
    // Test Midpoint scheme
    params.scheme = SubdivisionScheme::Midpoint;
    auto midpoint = subdivide(box, params);
    EXPECT_EQ(midpoint.status.code, ErrorCode::Success);
    
    // Both should produce same number of triangles
    EXPECT_EQ(loop.mesh.triangleCount(), midpoint.mesh.triangleCount());
}

TEST_F(SubdivideTest, SubdivideTriangle) {
    // Subdivide a single triangle of the box
    auto subdivided = subdivideTriangle(box, 0, 2);
    
    // Original box has 12 triangles
    // After subdividing one triangle twice: -1 + 16 = +15 triangles
    // Total: 12 + 15 = 27 (approximately, depends on implementation)
    EXPECT_GT(subdivided.triangleCount(), box.triangleCount());
}
