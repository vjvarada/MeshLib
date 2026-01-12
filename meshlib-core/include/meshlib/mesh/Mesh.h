#pragma once

/**
 * @file Mesh.h
 * @brief Core mesh data structure
 */

#include "meshlib/config.h"
#include "../core/Vector.h"
#include "../core/Matrix.h"
#include "../core/Box.h"
#include "../core/Id.h"
#include "../core/Types.h"
#include <vector>
#include <array>
#include <memory>
#include <optional>

namespace meshlib {

// Forward declarations
class MeshTopology;

/**
 * @brief Half-edge mesh representation
 * 
 * A mesh consists of:
 * - Vertex positions (3D points)
 * - Triangle connectivity (topology)
 * - Optional vertex normals
 * - Optional UV coordinates
 */
class MESHLIB_API Mesh {
public:
    /// Default constructor - creates an empty mesh
    Mesh() = default;
    
    /// Move constructor
    Mesh(Mesh&& other) noexcept = default;
    
    /// Move assignment
    Mesh& operator=(Mesh&& other) noexcept = default;
    
    /// Copy constructor
    Mesh(const Mesh& other);
    
    /// Copy assignment
    Mesh& operator=(const Mesh& other);
    
    /// Destructor
    ~Mesh();
    
    // ==================== Construction ====================
    
    /**
     * @brief Create a mesh from vertex positions and triangle indices
     * @param vertices Array of 3D vertex positions
     * @param triangles Array of triangle indices (3 vertex indices per triangle)
     * @return The constructed mesh
     */
    static Mesh fromTriangles(
        const std::vector<Vector3f>& vertices,
        const std::vector<std::array<int, 3>>& triangles);
    
    /**
     * @brief Create a mesh from interleaved triangle vertices
     * @param triangleVertices Array of triangles, each with 3 vertex positions
     * @return The constructed mesh (vertices will be welded if duplicates exist)
     */
    static Mesh fromTriangleVertices(const std::vector<Triangle3f>& triangleVertices);
    
    // ==================== Accessors ====================
    
    /// Get number of vertices
    size_t vertexCount() const { return vertices_.size(); }
    
    /// Get number of triangles (faces)
    size_t triangleCount() const { return triangles_.size(); }
    
    /// Check if mesh is empty
    bool empty() const { return vertices_.empty() || triangles_.empty(); }
    
    /// Check if mesh is valid (has consistent topology)
    bool valid() const;
    
    /// Get all vertex positions
    const std::vector<Vector3f>& vertices() const { return vertices_; }
    std::vector<Vector3f>& vertices() { invalidateCache(); return vertices_; }
    
    /// Get all triangle indices
    const std::vector<std::array<int, 3>>& triangles() const { return triangles_; }
    std::vector<std::array<int, 3>>& triangles() { invalidateCache(); return triangles_; }
    
    /// Get vertex normals (computed on demand)
    const std::vector<Vector3f>& vertexNormals() const;
    
    /// Get face normals (computed on demand)
    const std::vector<Vector3f>& faceNormals() const;
    
    /// Get a specific vertex position
    const Vector3f& vertex(int index) const { return vertices_[index]; }
    Vector3f& vertex(int index) { invalidateCache(); return vertices_[index]; }
    
    /// Get a specific vertex position by ID
    const Vector3f& vertex(VertexId id) const { return vertices_[id.get()]; }
    Vector3f& vertex(VertexId id) { invalidateCache(); return vertices_[id.get()]; }
    
    /// Get a specific triangle
    const std::array<int, 3>& triangle(int index) const { return triangles_[index]; }
    std::array<int, 3>& triangle(int index) { invalidateCache(); return triangles_[index]; }
    
    /// Get the three vertices of a triangle
    Triangle3f triangleVertices(int faceIndex) const;
    Triangle3f triangleVertices(FaceId faceId) const { return triangleVertices(faceId.get()); }
    
    // ==================== Geometry Queries ====================
    
    /// Compute the axis-aligned bounding box
    Box3f boundingBox() const;
    
    /// Compute the center of mass (assuming uniform density)
    Vector3f centroid() const;
    
    /// Compute total surface area
    double area() const;
    
    /// Compute total volume (mesh must be closed and consistently oriented)
    double volume() const;
    
    /// Compute the normal of a specific face
    Vector3f faceNormal(int faceIndex) const;
    Vector3f faceNormal(FaceId faceId) const { return faceNormal(faceId.get()); }
    
    /// Compute the area of a specific face
    float faceArea(int faceIndex) const;
    float faceArea(FaceId faceId) const { return faceArea(faceId.get()); }
    
    /// Compute the centroid of a specific face
    Vector3f faceCentroid(int faceIndex) const;
    Vector3f faceCentroid(FaceId faceId) const { return faceCentroid(faceId.get()); }
    
    // ==================== Topology Queries ====================
    
    /// Check if the mesh is closed (watertight)
    bool isClosed() const;
    
    /// Check if the mesh is manifold
    bool isManifold() const;
    
    /// Find boundary edges (edges with only one adjacent face)
    std::vector<std::pair<int, int>> boundaryEdges() const;
    
    /// Count the number of boundary loops (holes)
    int boundaryLoopCount() const;
    
    /// Find triangles adjacent to a vertex
    std::vector<int> adjacentTriangles(int vertexIndex) const;
    
    /// Find vertices adjacent to a vertex (connected by an edge)
    std::vector<int> adjacentVertices(int vertexIndex) const;
    
    // ==================== Modification ====================
    
    /// Transform all vertices by an affine transformation
    void transform(const AffineTransform3f& xf);
    
    /// Translate all vertices
    void translate(const Vector3f& offset);
    
    /// Scale uniformly around origin
    void scale(float factor);
    
    /// Scale non-uniformly around origin
    void scale(const Vector3f& factors);
    
    /// Flip all face orientations (reverse winding order)
    void flipFaces();
    
    /// Merge duplicate vertices within tolerance
    int mergeCloseVertices(float tolerance = 1e-6f);
    
    /// Remove degenerate triangles (zero area)
    int removeDegenerateTriangles(float minArea = 1e-10f);
    
    /// Remove unreferenced vertices
    int removeUnusedVertices();
    
    /// Recalculate normals
    void recomputeNormals();
    
    /// Clear all data
    void clear();
    
    // ==================== UV Coordinates ====================
    
    /// Check if mesh has UV coordinates
    bool hasUVs() const { return !uvCoords_.empty(); }
    
    /// Get UV coordinates
    const std::vector<Vector2f>& uvCoords() const { return uvCoords_; }
    std::vector<Vector2f>& uvCoords() { return uvCoords_; }
    
    /// Set UV coordinates
    void setUVCoords(std::vector<Vector2f> uvs) { uvCoords_ = std::move(uvs); }
    
private:
    std::vector<Vector3f> vertices_;
    std::vector<std::array<int, 3>> triangles_;
    std::vector<Vector2f> uvCoords_;
    
    // Cached data (mutable for lazy computation)
    mutable std::optional<std::vector<Vector3f>> cachedVertexNormals_;
    mutable std::optional<std::vector<Vector3f>> cachedFaceNormals_;
    mutable std::optional<Box3f> cachedBoundingBox_;
    
    void invalidateCache();
    void computeVertexNormals() const;
    void computeFaceNormals() const;
};

// ==================== Free Functions ====================

/// Compute the distance from a point to the mesh surface
float distanceToMesh(const Vector3f& point, const Mesh& mesh);

/// Find the closest point on a mesh to a given point
Vector3f closestPointOnMesh(const Vector3f& point, const Mesh& mesh);

/// Check if a point is inside a closed mesh
bool isPointInsideMesh(const Vector3f& point, const Mesh& mesh);

} // namespace meshlib
