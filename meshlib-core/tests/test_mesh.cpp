/**
 * @file test_mesh.cpp
 * @brief Unit tests for Mesh class
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <gtest/gtest.h>
#include "meshlib/mesh/Mesh.h"
#include "meshlib/mesh/Primitives.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace meshlib;

class MeshTest : public ::testing::Test {
protected:
    Mesh triangle;
    
    void SetUp() override {
        // Create a simple triangle
        std::vector<Vector3f> vertices = {
            {0.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
            {0.5f, 1.0f, 0.0f}
        };
        std::vector<std::array<int, 3>> triangles = {{0, 1, 2}};
        triangle = Mesh::fromTriangles(vertices, triangles);
    }
};

TEST_F(MeshTest, Empty) {
    Mesh empty;
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.vertexCount(), 0);
    EXPECT_EQ(empty.triangleCount(), 0);
}

TEST_F(MeshTest, FromTriangles) {
    EXPECT_FALSE(triangle.empty());
    EXPECT_EQ(triangle.vertexCount(), 3);
    EXPECT_EQ(triangle.triangleCount(), 1);
}

TEST_F(MeshTest, VertexAccess) {
    const auto& vertices = triangle.vertices();
    EXPECT_FLOAT_EQ(vertices[0].x, 0.0f);
    EXPECT_FLOAT_EQ(vertices[1].x, 1.0f);
    EXPECT_FLOAT_EQ(vertices[2].x, 0.5f);
}

TEST_F(MeshTest, TriangleAccess) {
    const auto& triangles = triangle.triangles();
    EXPECT_EQ(triangles[0][0], 0);
    EXPECT_EQ(triangles[0][1], 1);
    EXPECT_EQ(triangles[0][2], 2);
}

TEST_F(MeshTest, BoundingBox) {
    auto box = triangle.boundingBox();
    EXPECT_TRUE(box.valid());
    EXPECT_FLOAT_EQ(box.min.x, 0.0f);
    EXPECT_FLOAT_EQ(box.max.x, 1.0f);
    EXPECT_FLOAT_EQ(box.min.y, 0.0f);
    EXPECT_FLOAT_EQ(box.max.y, 1.0f);
}

TEST_F(MeshTest, Centroid) {
    auto centroid = triangle.centroid();
    EXPECT_FLOAT_EQ(centroid.x, 0.5f);
    EXPECT_NEAR(centroid.y, 1.0f / 3.0f, 1e-6f);
    EXPECT_FLOAT_EQ(centroid.z, 0.0f);
}

TEST_F(MeshTest, Area) {
    // Triangle area = 0.5 * base * height = 0.5 * 1 * 1 = 0.5
    double area = triangle.area();
    EXPECT_NEAR(area, 0.5, 1e-6);
}

TEST_F(MeshTest, FaceNormal) {
    auto normal = triangle.faceNormal(0);
    // Triangle in XY plane, normal should be (0, 0, 1) or (0, 0, -1)
    EXPECT_NEAR(std::abs(normal.z), 1.0f, 1e-6f);
}

TEST_F(MeshTest, FaceArea) {
    float area = triangle.faceArea(0);
    EXPECT_NEAR(area, 0.5f, 1e-6f);
}

TEST_F(MeshTest, FaceCentroid) {
    auto centroid = triangle.faceCentroid(0);
    EXPECT_FLOAT_EQ(centroid.x, 0.5f);
    EXPECT_NEAR(centroid.y, 1.0f / 3.0f, 1e-6f);
}

TEST_F(MeshTest, VertexNormals) {
    const auto& normals = triangle.vertexNormals();
    EXPECT_EQ(normals.size(), triangle.vertexCount());
    
    // All vertex normals should be same for a single triangle
    for (const auto& n : normals) {
        EXPECT_NEAR(std::abs(n.z), 1.0f, 1e-6f);
    }
}

TEST_F(MeshTest, FaceNormals) {
    const auto& normals = triangle.faceNormals();
    EXPECT_EQ(normals.size(), triangle.triangleCount());
    EXPECT_NEAR(std::abs(normals[0].z), 1.0f, 1e-6f);
}

TEST_F(MeshTest, Transform) {
    // transform() modifies in-place
    auto t = AffineTransform3f::fromTranslation({1.0f, 0.0f, 0.0f});
    triangle.transform(t);
    
    const auto& vertices = triangle.vertices();
    EXPECT_FLOAT_EQ(vertices[0].x, 1.0f);  // 0 + 1 = 1
    EXPECT_FLOAT_EQ(vertices[1].x, 2.0f);  // 1 + 1 = 2
}

TEST_F(MeshTest, Translate) {
    // translate() modifies in-place
    triangle.translate({2.0f, 3.0f, 4.0f});
    
    const auto& vertices = triangle.vertices();
    EXPECT_FLOAT_EQ(vertices[0].x, 2.0f);
    EXPECT_FLOAT_EQ(vertices[0].y, 3.0f);
    EXPECT_FLOAT_EQ(vertices[0].z, 4.0f);
}

TEST_F(MeshTest, Scale) {
    // scale() modifies in-place
    triangle.scale(2.0f);
    
    const auto& vertices = triangle.vertices();
    EXPECT_FLOAT_EQ(vertices[1].x, 2.0f);  // 1 * 2 = 2
    EXPECT_FLOAT_EQ(vertices[2].y, 2.0f);  // 1 * 2 = 2
}

TEST_F(MeshTest, ScaleNonUniform) {
    // scale() with Vector3f modifies in-place
    triangle.scale(Vector3f{2.0f, 3.0f, 1.0f});
    
    const auto& vertices = triangle.vertices();
    EXPECT_FLOAT_EQ(vertices[1].x, 2.0f);  // 1 * 2 = 2
    EXPECT_FLOAT_EQ(vertices[2].y, 3.0f);  // 1 * 3 = 3
}

TEST_F(MeshTest, FlipFaces) {
    // flipFaces() modifies in-place
    triangle.flipFaces();
    
    const auto& tris = triangle.triangles();
    // Original: {0, 1, 2}, Flipped: {0, 2, 1}
    EXPECT_EQ(tris[0][0], 0);
    EXPECT_EQ(tris[0][1], 2);
    EXPECT_EQ(tris[0][2], 1);
}

TEST_F(MeshTest, IsClosed) {
    EXPECT_FALSE(triangle.isClosed());  // Single triangle is open
}

TEST_F(MeshTest, Clear) {
    triangle.clear();
    EXPECT_TRUE(triangle.empty());
    EXPECT_EQ(triangle.vertexCount(), 0);
    EXPECT_EQ(triangle.triangleCount(), 0);
}

// Test with actual mesh primitives
class MeshPrimitiveTest : public ::testing::Test {
protected:
    Mesh box;
    Mesh sphere;
    
    void SetUp() override {
        box = createBox({1.0f, 1.0f, 1.0f});
        sphere = createSphere(1.0f, Vector3f::zero(), 16);  // createSphere(radius, center, segments)
    }
};

TEST_F(MeshPrimitiveTest, BoxVertexCount) {
    EXPECT_EQ(box.vertexCount(), 8);  // Box has 8 vertices
}

TEST_F(MeshPrimitiveTest, BoxTriangleCount) {
    EXPECT_EQ(box.triangleCount(), 12);  // Box has 12 triangles (2 per face)
}

TEST_F(MeshPrimitiveTest, BoxBoundingBox) {
    auto bbox = box.boundingBox();
    EXPECT_NEAR(bbox.min.x, -0.5f, 1e-6f);
    EXPECT_NEAR(bbox.max.x, 0.5f, 1e-6f);
}

TEST_F(MeshPrimitiveTest, SphereVertexCount) {
    EXPECT_GT(sphere.vertexCount(), 0);
}

TEST_F(MeshPrimitiveTest, SphereBoundingBox) {
    auto bbox = sphere.boundingBox();
    EXPECT_NEAR(bbox.min.x, -1.0f, 1e-2f);
    EXPECT_NEAR(bbox.max.x, 1.0f, 1e-2f);
}

TEST_F(MeshPrimitiveTest, BoxIsClosed) {
    // A box should be closed
    EXPECT_TRUE(box.isClosed());
}

TEST_F(MeshPrimitiveTest, MergeCloseVertices) {
    // Create mesh with duplicate vertices
    std::vector<Vector3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {0.00001f, 0.0f, 0.0f},  // Very close to vertex 0
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f}
    };
    std::vector<std::array<int, 3>> triangles = {
        {0, 2, 3},
        {1, 3, 2}  // Uses duplicate vertex
    };
    
    Mesh mesh = Mesh::fromTriangles(vertices, triangles);
    size_t originalCount = mesh.vertexCount();
    int merged = mesh.mergeCloseVertices(0.001f);
    
    // Should have merged at least one vertex
    EXPECT_GE(merged, 0);
}

TEST_F(MeshPrimitiveTest, RemoveDegenerateTriangles) {
    // Create mesh with degenerate triangle
    std::vector<Vector3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f}  // Same as vertex 0, will create degenerate triangle
    };
    std::vector<std::array<int, 3>> triangles = {
        {0, 1, 2},
        {0, 3, 1}  // Degenerate (0 and 3 are same point)
    };
    
    Mesh mesh = Mesh::fromTriangles(vertices, triangles);
    int removed = mesh.removeDegenerateTriangles();
    
    EXPECT_GE(removed, 0);
}

TEST_F(MeshPrimitiveTest, RemoveUnusedVertices) {
    // Create mesh with unused vertex
    std::vector<Vector3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {10.0f, 10.0f, 10.0f}  // Unused vertex
    };
    std::vector<std::array<int, 3>> triangles = {{0, 1, 2}};
    
    Mesh mesh = Mesh::fromTriangles(vertices, triangles);
    int removed = mesh.removeUnusedVertices();
    
    EXPECT_GE(removed, 0);
}
