/**
 * @file Primitives.cpp
 * @brief Implementation of primitive mesh creation functions
 */

#include "meshlib/mesh/Primitives.h"
#include <cmath>
#include <numbers>
#include <map>

namespace meshlib {

namespace {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;
}

Mesh createSphere(const SphereParams& params) {
    return createSphere(params.radius, params.center, params.numMeridians);
}

Mesh createSphere(float radius, const Vector3f& center, int segments) {
    std::vector<Vector3f> vertices;
    std::vector<std::array<int, 3>> triangles;
    
    int stacks = segments / 2;
    int slices = segments;
    
    // Generate vertices
    vertices.push_back(center + Vector3f(0, radius, 0)); // Top pole
    
    for (int i = 1; i < stacks; ++i) {
        float phi = PI * static_cast<float>(i) / static_cast<float>(stacks);
        float y = radius * std::cos(phi);
        float r = radius * std::sin(phi);
        
        for (int j = 0; j < slices; ++j) {
            float theta = TWO_PI * static_cast<float>(j) / static_cast<float>(slices);
            float x = r * std::cos(theta);
            float z = r * std::sin(theta);
            vertices.push_back(center + Vector3f(x, y, z));
        }
    }
    
    vertices.push_back(center + Vector3f(0, -radius, 0)); // Bottom pole
    
    // Generate triangles
    // Top cap
    for (int j = 0; j < slices; ++j) {
        int next = (j + 1) % slices;
        triangles.push_back({0, j + 1, next + 1});
    }
    
    // Middle bands
    for (int i = 0; i < stacks - 2; ++i) {
        int row1 = 1 + i * slices;
        int row2 = 1 + (i + 1) * slices;
        
        for (int j = 0; j < slices; ++j) {
            int next = (j + 1) % slices;
            
            triangles.push_back({row1 + j, row2 + j, row2 + next});
            triangles.push_back({row1 + j, row2 + next, row1 + next});
        }
    }
    
    // Bottom cap
    int bottomPole = static_cast<int>(vertices.size()) - 1;
    int lastRow = 1 + (stacks - 2) * slices;
    
    for (int j = 0; j < slices; ++j) {
        int next = (j + 1) % slices;
        triangles.push_back({lastRow + j, bottomPole, lastRow + next});
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createIcosphere(float radius, int subdivisions, const Vector3f& center) {
    // Start with icosahedron
    float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
    
    std::vector<Vector3f> vertices = {
        Vector3f(-1,  t,  0).normalized() * radius + center,
        Vector3f( 1,  t,  0).normalized() * radius + center,
        Vector3f(-1, -t,  0).normalized() * radius + center,
        Vector3f( 1, -t,  0).normalized() * radius + center,
        Vector3f( 0, -1,  t).normalized() * radius + center,
        Vector3f( 0,  1,  t).normalized() * radius + center,
        Vector3f( 0, -1, -t).normalized() * radius + center,
        Vector3f( 0,  1, -t).normalized() * radius + center,
        Vector3f( t,  0, -1).normalized() * radius + center,
        Vector3f( t,  0,  1).normalized() * radius + center,
        Vector3f(-t,  0, -1).normalized() * radius + center,
        Vector3f(-t,  0,  1).normalized() * radius + center
    };
    
    std::vector<std::array<int, 3>> triangles = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
        {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
        {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
        {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
    };
    
    // Subdivide
    for (int s = 0; s < subdivisions; ++s) {
        std::vector<std::array<int, 3>> newTriangles;
        std::map<std::pair<int, int>, int> midpointCache;
        
        auto getMidpoint = [&](int i1, int i2) -> int {
            auto key = std::make_pair(std::min(i1, i2), std::max(i1, i2));
            auto it = midpointCache.find(key);
            
            if (it != midpointCache.end()) {
                return it->second;
            }
            
            Vector3f mid = (vertices[i1] + vertices[i2]) * 0.5f;
            mid = (mid - center).normalized() * radius + center;
            int idx = static_cast<int>(vertices.size());
            vertices.push_back(mid);
            midpointCache[key] = idx;
            return idx;
        };
        
        for (const auto& tri : triangles) {
            int a = getMidpoint(tri[0], tri[1]);
            int b = getMidpoint(tri[1], tri[2]);
            int c = getMidpoint(tri[2], tri[0]);
            
            newTriangles.push_back({tri[0], a, c});
            newTriangles.push_back({tri[1], b, a});
            newTriangles.push_back({tri[2], c, b});
            newTriangles.push_back({a, b, c});
        }
        
        triangles = std::move(newTriangles);
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createBox(const BoxParams& params) {
    return createBox(params.size, params.center);
}

Mesh createBox(const Vector3f& size, const Vector3f& center) {
    Vector3f h = size * 0.5f;
    
    std::vector<Vector3f> vertices = {
        center + Vector3f(-h.x, -h.y, -h.z),
        center + Vector3f( h.x, -h.y, -h.z),
        center + Vector3f( h.x,  h.y, -h.z),
        center + Vector3f(-h.x,  h.y, -h.z),
        center + Vector3f(-h.x, -h.y,  h.z),
        center + Vector3f( h.x, -h.y,  h.z),
        center + Vector3f( h.x,  h.y,  h.z),
        center + Vector3f(-h.x,  h.y,  h.z)
    };
    
    std::vector<std::array<int, 3>> triangles = {
        // Front
        {0, 1, 2}, {0, 2, 3},
        // Back
        {5, 4, 7}, {5, 7, 6},
        // Left
        {4, 0, 3}, {4, 3, 7},
        // Right
        {1, 5, 6}, {1, 6, 2},
        // Top
        {3, 2, 6}, {3, 6, 7},
        // Bottom
        {4, 5, 1}, {4, 1, 0}
    };
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createCube(float size) {
    return createBox(Vector3f(size, size, size), Vector3f::zero());
}

Mesh createCylinder(const CylinderParams& params) {
    return createCylinder(params.radius, params.height, params.numSegments, params.capped);
}

Mesh createCylinder(float radius, float height, int segments, bool capped) {
    std::vector<Vector3f> vertices;
    std::vector<std::array<int, 3>> triangles;
    
    float halfHeight = height * 0.5f;
    
    // Generate side vertices
    for (int i = 0; i < segments; ++i) {
        float theta = TWO_PI * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * std::cos(theta);
        float z = radius * std::sin(theta);
        
        vertices.push_back(Vector3f(x, -halfHeight, z)); // Bottom
        vertices.push_back(Vector3f(x,  halfHeight, z)); // Top
    }
    
    // Side triangles
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        int bl = i * 2;
        int tl = i * 2 + 1;
        int br = next * 2;
        int tr = next * 2 + 1;
        
        triangles.push_back({bl, br, tr});
        triangles.push_back({bl, tr, tl});
    }
    
    if (capped) {
        // Add center vertices for caps
        int bottomCenter = static_cast<int>(vertices.size());
        vertices.push_back(Vector3f(0, -halfHeight, 0));
        
        int topCenter = static_cast<int>(vertices.size());
        vertices.push_back(Vector3f(0, halfHeight, 0));
        
        // Cap triangles
        for (int i = 0; i < segments; ++i) {
            int next = (i + 1) % segments;
            
            // Bottom cap (reversed winding)
            triangles.push_back({bottomCenter, next * 2, i * 2});
            
            // Top cap
            triangles.push_back({topCenter, i * 2 + 1, next * 2 + 1});
        }
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createCone(const ConeParams& params) {
    return createCone(params.baseRadius, params.height, params.numSegments, params.capped);
}

Mesh createCone(float baseRadius, float height, int segments, bool capped) {
    std::vector<Vector3f> vertices;
    std::vector<std::array<int, 3>> triangles;
    
    // Apex
    int apexIdx = 0;
    vertices.push_back(Vector3f(0, height, 0));
    
    // Base vertices
    for (int i = 0; i < segments; ++i) {
        float theta = TWO_PI * static_cast<float>(i) / static_cast<float>(segments);
        float x = baseRadius * std::cos(theta);
        float z = baseRadius * std::sin(theta);
        vertices.push_back(Vector3f(x, 0, z));
    }
    
    // Side triangles
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        triangles.push_back({apexIdx, i + 1, next + 1});
    }
    
    if (capped) {
        int centerIdx = static_cast<int>(vertices.size());
        vertices.push_back(Vector3f(0, 0, 0));
        
        for (int i = 0; i < segments; ++i) {
            int next = (i + 1) % segments;
            triangles.push_back({centerIdx, next + 1, i + 1});
        }
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createTorus(const TorusParams& params) {
    return createTorus(params.majorRadius, params.minorRadius, params.numMajorSegments, params.numMinorSegments);
}

Mesh createTorus(float majorRadius, float minorRadius, int majorSegments, int minorSegments) {
    std::vector<Vector3f> vertices;
    std::vector<std::array<int, 3>> triangles;
    
    for (int i = 0; i < majorSegments; ++i) {
        float theta = TWO_PI * static_cast<float>(i) / static_cast<float>(majorSegments);
        float ct = std::cos(theta);
        float st = std::sin(theta);
        
        for (int j = 0; j < minorSegments; ++j) {
            float phi = TWO_PI * static_cast<float>(j) / static_cast<float>(minorSegments);
            float cp = std::cos(phi);
            float sp = std::sin(phi);
            
            float x = (majorRadius + minorRadius * cp) * ct;
            float y = minorRadius * sp;
            float z = (majorRadius + minorRadius * cp) * st;
            
            vertices.push_back(Vector3f(x, y, z));
        }
    }
    
    for (int i = 0; i < majorSegments; ++i) {
        int nextI = (i + 1) % majorSegments;
        
        for (int j = 0; j < minorSegments; ++j) {
            int nextJ = (j + 1) % minorSegments;
            
            int v00 = i * minorSegments + j;
            int v01 = i * minorSegments + nextJ;
            int v10 = nextI * minorSegments + j;
            int v11 = nextI * minorSegments + nextJ;
            
            triangles.push_back({v00, v10, v11});
            triangles.push_back({v00, v11, v01});
        }
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createPlane(const PlaneParams& params) {
    return createPlane(params.width, params.height, params.divisionsX, params.divisionsY);
}

Mesh createPlane(float width, float height, int divisionsX, int divisionsY) {
    std::vector<Vector3f> vertices;
    std::vector<std::array<int, 3>> triangles;
    
    float halfW = width * 0.5f;
    float halfH = height * 0.5f;
    
    int nx = divisionsX + 1;
    int ny = divisionsY + 1;
    
    for (int y = 0; y < ny; ++y) {
        float v = static_cast<float>(y) / static_cast<float>(divisionsY);
        float py = -halfH + v * height;
        
        for (int x = 0; x < nx; ++x) {
            float u = static_cast<float>(x) / static_cast<float>(divisionsX);
            float px = -halfW + u * width;
            
            vertices.push_back(Vector3f(px, 0, py));
        }
    }
    
    for (int y = 0; y < divisionsY; ++y) {
        for (int x = 0; x < divisionsX; ++x) {
            int i00 = y * nx + x;
            int i10 = y * nx + (x + 1);
            int i01 = (y + 1) * nx + x;
            int i11 = (y + 1) * nx + (x + 1);
            
            triangles.push_back({i00, i01, i11});
            triangles.push_back({i00, i11, i10});
        }
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createGrid(float width, float height, int divisionsX, int divisionsY) {
    return createPlane(width, height, divisionsX, divisionsY);
}

Mesh createDisc(float radius, int segments, const Vector3f& center) {
    std::vector<Vector3f> vertices;
    std::vector<std::array<int, 3>> triangles;
    
    vertices.push_back(center); // Center
    
    for (int i = 0; i < segments; ++i) {
        float theta = TWO_PI * static_cast<float>(i) / static_cast<float>(segments);
        vertices.push_back(center + Vector3f(radius * std::cos(theta), 0, radius * std::sin(theta)));
    }
    
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        triangles.push_back({0, i + 1, next + 1});
    }
    
    return Mesh::fromTriangles(vertices, triangles);
}

Mesh createCapsule(float radius, float height, int segments) {
    // Create as cylinder + two hemispheres
    auto cylinder = createCylinder(radius, height, segments, false);
    
    // Would need to add hemisphere caps - simplified for now
    // Return cylinder with caps as approximation
    return createCylinder(radius, height + 2 * radius, segments, true);
}

Mesh createArrow(const Vector3f& from, const Vector3f& to, float shaftRadius, float headRadius, float headLength) {
    Vector3f dir = to - from;
    float length = dir.length();
    
    if (length < 1e-6f) {
        return Mesh();
    }
    
    dir = dir / length;
    
    // Create shaft cylinder
    float shaftLength = length - headLength;
    if (shaftLength < 0) {
        shaftLength = length * 0.8f;
        headLength = length * 0.2f;
    }
    
    auto shaft = createCylinder(shaftRadius, shaftLength, 12, false);
    auto head = createCone(headRadius, headLength, 12, true);
    
    // Transform and combine...
    // Simplified: just return the shaft for now
    return shaft;
}

Mesh createAxes(float length, float radius) {
    // Create three arrows for X, Y, Z axes
    // Simplified: create three cylinders
    auto xAxis = createCylinder(radius, length, 8, true);
    
    return xAxis;
}

} // namespace meshlib
