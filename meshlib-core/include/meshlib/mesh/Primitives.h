#pragma once

/**
 * @file Primitives.h
 * @brief Primitive mesh creation functions
 */

#include "meshlib/config.h"
#include "Mesh.h"

namespace meshlib {

/**
 * @brief Parameters for sphere creation
 */
struct MESHLIB_API SphereParams {
    float radius = 1.0f;
    Vector3f center = Vector3f::zero();
    int numMeridians = 32;    // Longitude divisions
    int numParallels = 16;    // Latitude divisions
};

/**
 * @brief Parameters for box/cube creation
 */
struct MESHLIB_API BoxParams {
    Vector3f size = Vector3f::one();
    Vector3f center = Vector3f::zero();
};

/**
 * @brief Parameters for cylinder creation
 */
struct MESHLIB_API CylinderParams {
    float radius = 1.0f;
    float height = 2.0f;
    Vector3f baseCenter = Vector3f::zero();
    int numSegments = 32;
    bool capped = true;
};

/**
 * @brief Parameters for cone creation
 */
struct MESHLIB_API ConeParams {
    float baseRadius = 1.0f;
    float height = 2.0f;
    Vector3f baseCenter = Vector3f::zero();
    int numSegments = 32;
    bool capped = true;
};

/**
 * @brief Parameters for torus creation
 */
struct MESHLIB_API TorusParams {
    float majorRadius = 1.0f;  // Distance from center to tube center
    float minorRadius = 0.25f; // Tube radius
    Vector3f center = Vector3f::zero();
    int numMajorSegments = 32;
    int numMinorSegments = 16;
};

/**
 * @brief Parameters for plane creation
 */
struct MESHLIB_API PlaneParams {
    float width = 1.0f;
    float height = 1.0f;
    Vector3f center = Vector3f::zero();
    Vector3f normal = Vector3f::unitZ();
    int divisionsX = 1;
    int divisionsY = 1;
};

// ==================== Primitive Creation Functions ====================

/// Create a UV sphere
MESHLIB_API Mesh createSphere(const SphereParams& params = {});

/// Create a sphere with given radius and center
MESHLIB_API Mesh createSphere(float radius, const Vector3f& center = Vector3f::zero(), int segments = 32);

/// Create an icosphere (geodesic sphere)
MESHLIB_API Mesh createIcosphere(float radius, int subdivisions = 2, const Vector3f& center = Vector3f::zero());

/// Create a box/cuboid
MESHLIB_API Mesh createBox(const BoxParams& params = {});

/// Create a box with given size and center
MESHLIB_API Mesh createBox(const Vector3f& size, const Vector3f& center = Vector3f::zero());

/// Create a unit cube centered at origin
MESHLIB_API Mesh createCube(float size = 1.0f);

/// Create a cylinder
MESHLIB_API Mesh createCylinder(const CylinderParams& params = {});

/// Create a cylinder with given radius and height
MESHLIB_API Mesh createCylinder(float radius, float height, int segments = 32, bool capped = true);

/// Create a cone
MESHLIB_API Mesh createCone(const ConeParams& params = {});

/// Create a cone with given base radius and height
MESHLIB_API Mesh createCone(float baseRadius, float height, int segments = 32, bool capped = true);

/// Create a torus
MESHLIB_API Mesh createTorus(const TorusParams& params = {});

/// Create a torus with given radii
MESHLIB_API Mesh createTorus(float majorRadius, float minorRadius, int majorSegments = 32, int minorSegments = 16);

/// Create a plane (quad subdivided into triangles)
MESHLIB_API Mesh createPlane(const PlaneParams& params = {});

/// Create a simple plane with given dimensions
MESHLIB_API Mesh createPlane(float width, float height, int divisionsX = 1, int divisionsY = 1);

/// Create a grid of points as a mesh
MESHLIB_API Mesh createGrid(float width, float height, int divisionsX, int divisionsY);

/// Create a disc (filled circle)
MESHLIB_API Mesh createDisc(float radius, int segments = 32, const Vector3f& center = Vector3f::zero());

/// Create a capsule (cylinder with hemispherical caps)
MESHLIB_API Mesh createCapsule(float radius, float height, int segments = 32);

/// Create an arrow (for visualization)
MESHLIB_API Mesh createArrow(const Vector3f& from, const Vector3f& to, float shaftRadius = 0.02f, float headRadius = 0.05f, float headLength = 0.1f);

/// Create coordinate axes (for visualization)
MESHLIB_API Mesh createAxes(float length = 1.0f, float radius = 0.02f);

} // namespace meshlib
