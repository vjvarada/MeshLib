/**
 * @file test_primitives.cpp
 * @brief Unit tests for mesh primitive creation functions
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <gtest/gtest.h>
#include "meshlib/mesh/Primitives.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace meshlib;

class PrimitivesTest : public ::testing::Test {
};

TEST_F(PrimitivesTest, CreateSphere) {
    // createSphere(float radius, Vector3f center, int segments)
    auto sphere = createSphere(1.0f, Vector3f::zero(), 16);
    
    EXPECT_FALSE(sphere.empty());
    EXPECT_GT(sphere.vertexCount(), 0);
    EXPECT_GT(sphere.triangleCount(), 0);
    
    auto bbox = sphere.boundingBox();
    EXPECT_NEAR(bbox.min.x, -1.0f, 1e-2f);
    EXPECT_NEAR(bbox.max.x, 1.0f, 1e-2f);
}

TEST_F(PrimitivesTest, CreateSphereRadius) {
    auto sphere1 = createSphere(1.0f, Vector3f::zero(), 16);
    auto sphere2 = createSphere(2.0f, Vector3f::zero(), 16);
    
    auto bbox1 = sphere1.boundingBox();
    auto bbox2 = sphere2.boundingBox();
    
    // Sphere2 should be twice the size
    EXPECT_NEAR(bbox2.diagonal(), bbox1.diagonal() * 2.0f, 0.1f);
}

TEST_F(PrimitivesTest, CreateSphereWithParams) {
    SphereParams params;
    params.radius = 1.0f;
    params.center = Vector3f::zero();
    params.numMeridians = 32;
    params.numParallels = 16;
    
    auto sphere = createSphere(params);
    
    EXPECT_FALSE(sphere.empty());
    auto bbox = sphere.boundingBox();
    EXPECT_NEAR(bbox.min.x, -1.0f, 1e-2f);
    EXPECT_NEAR(bbox.max.x, 1.0f, 1e-2f);
}

TEST_F(PrimitivesTest, CreateIcosphere) {
    auto ico = createIcosphere(1.0f, 2);
    
    EXPECT_FALSE(ico.empty());
    EXPECT_GT(ico.vertexCount(), 12);  // Icosahedron has 12 vertices
    EXPECT_GT(ico.triangleCount(), 20);  // Subdivided should have more
    
    // All vertices should be approximately on the sphere surface
    const auto& vertices = ico.vertices();
    for (const auto& v : vertices) {
        float dist = v.length();
        EXPECT_NEAR(dist, 1.0f, 1e-5f);
    }
}

TEST_F(PrimitivesTest, CreateBox) {
    auto box = createBox({2.0f, 3.0f, 4.0f});
    
    EXPECT_FALSE(box.empty());
    EXPECT_EQ(box.vertexCount(), 8);
    EXPECT_EQ(box.triangleCount(), 12);
    
    auto bbox = box.boundingBox();
    EXPECT_NEAR(bbox.size().x, 2.0f, 1e-6f);
    EXPECT_NEAR(bbox.size().y, 3.0f, 1e-6f);
    EXPECT_NEAR(bbox.size().z, 4.0f, 1e-6f);
}

TEST_F(PrimitivesTest, CreateBoxCentered) {
    auto box = createBox({2.0f, 2.0f, 2.0f});
    
    auto bbox = box.boundingBox();
    EXPECT_NEAR(bbox.center().x, 0.0f, 1e-6f);
    EXPECT_NEAR(bbox.center().y, 0.0f, 1e-6f);
    EXPECT_NEAR(bbox.center().z, 0.0f, 1e-6f);
}

TEST_F(PrimitivesTest, CreateBoxWithParams) {
    BoxParams params;
    params.size = {2.0f, 3.0f, 4.0f};
    params.center = {1.0f, 1.0f, 1.0f};
    
    auto box = createBox(params);
    
    EXPECT_FALSE(box.empty());
    auto bbox = box.boundingBox();
    EXPECT_NEAR(bbox.center().x, 1.0f, 1e-6f);
    EXPECT_NEAR(bbox.center().y, 1.0f, 1e-6f);
    EXPECT_NEAR(bbox.center().z, 1.0f, 1e-6f);
}

TEST_F(PrimitivesTest, CreateCube) {
    auto cube = createCube(2.0f);
    
    EXPECT_FALSE(cube.empty());
    auto bbox = cube.boundingBox();
    EXPECT_NEAR(bbox.size().x, 2.0f, 1e-6f);
    EXPECT_NEAR(bbox.size().y, 2.0f, 1e-6f);
    EXPECT_NEAR(bbox.size().z, 2.0f, 1e-6f);
}

TEST_F(PrimitivesTest, CreateCylinder) {
    // createCylinder(float radius, float height, int segments, bool capped)
    auto cylinder = createCylinder(1.0f, 2.0f, 16, true);
    
    EXPECT_FALSE(cylinder.empty());
    EXPECT_GT(cylinder.vertexCount(), 0);
    EXPECT_GT(cylinder.triangleCount(), 0);
    
    auto bbox = cylinder.boundingBox();
    EXPECT_NEAR(bbox.size().x, 2.0f, 0.1f);  // Diameter
    EXPECT_NEAR(bbox.size().y, 2.0f, 0.1f);  // Diameter
    EXPECT_NEAR(bbox.size().z, 2.0f, 1e-6f);  // Height
}

TEST_F(PrimitivesTest, CreateCone) {
    // createCone(float baseRadius, float height, int segments, bool capped)
    auto cone = createCone(1.0f, 2.0f, 16, true);
    
    EXPECT_FALSE(cone.empty());
    EXPECT_GT(cone.vertexCount(), 0);
    EXPECT_GT(cone.triangleCount(), 0);
    
    auto bbox = cone.boundingBox();
    EXPECT_NEAR(bbox.size().x, 2.0f, 0.1f);  // Diameter at base
    EXPECT_NEAR(bbox.size().y, 2.0f, 0.1f);  // Diameter at base
    EXPECT_NEAR(bbox.size().z, 2.0f, 1e-6f);  // Height
}

TEST_F(PrimitivesTest, CreateTorus) {
    // createTorus(float majorRadius, float minorRadius, int majorSegments, int minorSegments)
    auto torus = createTorus(2.0f, 0.5f, 16, 8);
    
    EXPECT_FALSE(torus.empty());
    EXPECT_GT(torus.vertexCount(), 0);
    EXPECT_GT(torus.triangleCount(), 0);
    
    // Torus in XZ plane: (majorR + minorR) * 2 in X and Z, minorR * 2 in Y
    auto bbox = torus.boundingBox();
    EXPECT_NEAR(bbox.size().x, 5.0f, 0.2f);  // (2 + 0.5) * 2
    EXPECT_NEAR(bbox.size().z, 5.0f, 0.2f);
    EXPECT_NEAR(bbox.size().y, 1.0f, 0.1f);  // minorR * 2
}

TEST_F(PrimitivesTest, CreatePlane) {
    // createPlane(float width, float height, int divisionsX, int divisionsY)
    // Creates plane in XZ (Y=0) with width in X and height in Z
    auto plane = createPlane(4.0f, 3.0f, 1, 1);
    
    EXPECT_FALSE(plane.empty());
    EXPECT_EQ(plane.vertexCount(), 4);
    EXPECT_EQ(plane.triangleCount(), 2);
    
    auto bbox = plane.boundingBox();
    EXPECT_NEAR(bbox.size().x, 4.0f, 1e-6f);
    EXPECT_NEAR(bbox.size().z, 3.0f, 1e-6f);  // height is in Z direction
    EXPECT_NEAR(bbox.size().y, 0.0f, 1e-6f);  // Flat in Y
}

TEST_F(PrimitivesTest, CreatePlaneDivisions) {
    auto plane = createPlane(4.0f, 4.0f, 4, 4);
    
    // 4x4 divisions = 5x5 = 25 vertices
    EXPECT_EQ(plane.vertexCount(), 25);
    
    // Each cell has 2 triangles, 4x4 = 16 cells, 32 triangles
    EXPECT_EQ(plane.triangleCount(), 32);
}

TEST_F(PrimitivesTest, CreateDisc) {
    // createDisc(float radius, int segments, Vector3f center)
    // Creates disc in XZ plane (Y=0)
    auto disc = createDisc(1.0f, 16, Vector3f::zero());
    
    EXPECT_FALSE(disc.empty());
    EXPECT_GT(disc.vertexCount(), 1);  // At least center + edge vertices
    EXPECT_GT(disc.triangleCount(), 0);
    
    auto bbox = disc.boundingBox();
    EXPECT_NEAR(bbox.size().x, 2.0f, 0.1f);  // Diameter in X
    EXPECT_NEAR(bbox.size().z, 2.0f, 0.1f);  // Diameter in Z
    EXPECT_NEAR(bbox.size().y, 0.0f, 1e-6f);  // Flat in Y
}

TEST_F(PrimitivesTest, PrimitivesMeshValidity) {
    // Test that all primitives create valid meshes
    auto sphere = createSphere(1.0f, Vector3f::zero(), 8);
    auto ico = createIcosphere(1.0f, 1);
    auto box = createBox({1.0f, 1.0f, 1.0f});
    auto cylinder = createCylinder(1.0f, 1.0f, 8, true);
    auto cone = createCone(1.0f, 1.0f, 8, true);
    auto torus = createTorus(1.0f, 0.3f, 8, 4);
    auto plane = createPlane(1.0f, 1.0f, 1, 1);
    auto disc = createDisc(1.0f, 8, Vector3f::zero());
    
    // All should have positive area
    EXPECT_GT(sphere.area(), 0.0);
    EXPECT_GT(ico.area(), 0.0);
    EXPECT_GT(box.area(), 0.0);
    EXPECT_GT(cylinder.area(), 0.0);
    EXPECT_GT(cone.area(), 0.0);
    EXPECT_GT(torus.area(), 0.0);
    EXPECT_GT(plane.area(), 0.0);
    EXPECT_GT(disc.area(), 0.0);
}

TEST_F(PrimitivesTest, SphereAreaApproximation) {
    // For high subdivision, sphere area should approach 4 * pi * r^2
    auto sphere = createSphere(1.0f, Vector3f::zero(), 64);
    double expectedArea = 4.0 * M_PI;
    
    EXPECT_NEAR(sphere.area(), expectedArea, 0.1);
}

TEST_F(PrimitivesTest, BoxVolume) {
    auto box = createBox({2.0f, 3.0f, 4.0f});
    double expectedVolume = 2.0 * 3.0 * 4.0;
    
    EXPECT_NEAR(std::abs(box.volume()), expectedVolume, 0.01);
}

TEST_F(PrimitivesTest, CreateCapsule) {
    // createCapsule(float radius, float height, int segments)
    auto capsule = createCapsule(0.5f, 2.0f, 16);
    
    EXPECT_FALSE(capsule.empty());
    EXPECT_GT(capsule.vertexCount(), 0);
    EXPECT_GT(capsule.triangleCount(), 0);
}

TEST_F(PrimitivesTest, CreateArrow) {
    // createArrow(Vector3f from, Vector3f to, float shaftRadius, float headRadius, float headLength)
    auto arrow = createArrow(Vector3f::zero(), Vector3f{0.0f, 0.0f, 1.0f}, 0.02f, 0.05f, 0.1f);
    
    EXPECT_FALSE(arrow.empty());
    EXPECT_GT(arrow.vertexCount(), 0);
    EXPECT_GT(arrow.triangleCount(), 0);
}

TEST_F(PrimitivesTest, CreateGrid) {
    // createGrid(float width, float height, int divisionsX, int divisionsY)
    auto grid = createGrid(4.0f, 4.0f, 4, 4);
    
    EXPECT_FALSE(grid.empty());
}
