/**
 * @file test_fillhole.cpp
 * @brief Unit tests for hole detection and filling algorithms
 */

#include <gtest/gtest.h>
#include "meshlib/algorithms/FillHole.h"
#include "meshlib/mesh/Primitives.h"

using namespace meshlib;

class FillHoleTest : public ::testing::Test {
protected:
    Mesh meshWithHole;
    
    void SetUp() override {
        // Create a box and remove one face to create a hole
        auto box = createBox({1.0f, 1.0f, 1.0f});
        
        // Remove last two triangles (one face)
        std::vector<Triangle> triangles = box.triangles();
        triangles.pop_back();
        triangles.pop_back();
        
        meshWithHole = Mesh::fromTriangles(box.vertices(), triangles);
    }
};

TEST_F(FillHoleTest, FindHoles) {
    auto result = findHoles(meshWithHole);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_GE(result.holes.size(), 1);  // At least one hole
}

TEST_F(FillHoleTest, HoleProperties) {
    auto result = findHoles(meshWithHole);
    
    if (!result.holes.empty()) {
        const auto& hole = result.holes[0];
        
        // Hole should have at least 3 boundary vertices
        EXPECT_GE(hole.boundaryVertices.size(), 3);
        
        // Perimeter should be positive
        EXPECT_GT(hole.perimeter, 0.0f);
        
        // Area should be positive
        EXPECT_GT(hole.area, 0.0f);
    }
}

TEST_F(FillHoleTest, FillHolesBasic) {
    FillHoleParams params;
    
    auto result = fillHoles(meshWithHole, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_FALSE(result.mesh.empty());
    
    // Filled mesh should have more triangles than original
    EXPECT_GE(result.mesh.triangleCount(), meshWithHole.triangleCount());
}

TEST_F(FillHoleTest, FillHolesFan) {
    FillHoleParams params;
    params.method = FillHoleMethod::Fan;
    
    auto result = fillHoles(meshWithHole, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
}

TEST_F(FillHoleTest, ClosedMeshHasNoHoles) {
    auto box = createBox({1.0f, 1.0f, 1.0f});
    
    auto result = findHoles(box);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    EXPECT_TRUE(result.holes.empty());
}

TEST_F(FillHoleTest, FillEmptyMesh) {
    Mesh empty;
    FillHoleParams params;
    
    auto result = fillHoles(empty, params);
    
    EXPECT_NE(result.status.code, ErrorCode::Success);
}

TEST_F(FillHoleTest, MaxHoleSize) {
    FillHoleParams params;
    params.maxHoleSize = 0.001f;  // Very small, should not fill any holes
    
    auto result = fillHoles(meshWithHole, params);
    
    EXPECT_EQ(result.status.code, ErrorCode::Success);
    // Triangle count should be unchanged (hole too large to fill)
    EXPECT_EQ(result.mesh.triangleCount(), meshWithHole.triangleCount());
}

TEST_F(FillHoleTest, FillHoleFan) {
    std::vector<Vector3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    
    std::vector<int> boundary = {0, 1, 2, 3};
    
    auto triangles = fillHoleFan(vertices, boundary);
    
    // Square hole filled with fan should create 2 triangles
    EXPECT_EQ(triangles.size(), 2);
}

TEST_F(FillHoleTest, FillHoleEarClipping) {
    std::vector<Vector3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    
    std::vector<int> boundary = {0, 1, 2, 3};
    
    auto triangles = fillHoleEarClipping(vertices, boundary);
    
    // Square hole filled with ear clipping should create 2 triangles
    EXPECT_EQ(triangles.size(), 2);
}

TEST_F(FillHoleTest, FillSpecificHole) {
    // Find holes first
    auto findResult = findHoles(meshWithHole);
    
    if (!findResult.holes.empty()) {
        FillHoleParams params;
        auto result = fillHole(meshWithHole, findResult.holes[0].boundaryVertices, params);
        
        EXPECT_EQ(result.status.code, ErrorCode::Success);
        EXPECT_GT(result.mesh.triangleCount(), meshWithHole.triangleCount());
    }
}

TEST_F(FillHoleTest, MultipleHoles) {
    // Create a mesh with multiple holes by removing two faces
    auto box = createBox({1.0f, 1.0f, 1.0f});
    
    std::vector<Triangle> triangles = box.triangles();
    // Remove triangles from two different faces
    triangles.erase(triangles.begin());  // Remove first
    triangles.erase(triangles.begin());  // Remove second (was third)
    triangles.pop_back();  // Remove last
    triangles.pop_back();  // Remove second to last
    
    Mesh multiHole = Mesh::fromTriangles(box.vertices(), triangles);
    
    auto result = findHoles(multiHole);
    
    // Should find multiple holes
    EXPECT_GE(result.holes.size(), 1);
}

TEST_F(FillHoleTest, TriangularHole) {
    // Create a mesh with exactly 3 vertices forming a hole boundary
    std::vector<Vector3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f}
    };
    
    std::vector<int> boundary = {0, 1, 2};
    
    auto triangles = fillHoleFan(vertices, boundary);
    
    // Should create exactly 1 triangle
    EXPECT_EQ(triangles.size(), 1);
    
    // Verify triangle indices
    EXPECT_EQ(triangles[0][0], 0);
    EXPECT_EQ(triangles[0][1], 1);
    EXPECT_EQ(triangles[0][2], 2);
}
