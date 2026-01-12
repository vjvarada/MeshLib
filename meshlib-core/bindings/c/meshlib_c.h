/**
 * @file meshlib_c.h
 * @brief C API for MeshLib Core
 * 
 * This header provides a C-compatible interface to MeshLib Core functionality.
 * All functions are thread-safe unless otherwise noted.
 */

#ifndef MESHLIB_C_H
#define MESHLIB_C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Export/Import macros
// =============================================================================

#if defined(_WIN32)
    #if defined(MESHLIB_C_EXPORTS)
        #define MESHLIB_C_API __declspec(dllexport)
    #else
        #define MESHLIB_C_API __declspec(dllimport)
    #endif
#else
    #define MESHLIB_C_API __attribute__((visibility("default")))
#endif

// =============================================================================
// Opaque Types
// =============================================================================

typedef struct MeshLibMesh MeshLibMesh;
typedef struct MeshLibPointCloud MeshLibPointCloud;
typedef struct MeshLibAABBTree MeshLibAABBTree;

// =============================================================================
// Basic Types
// =============================================================================

typedef struct {
    float x, y, z;
} MeshLibVector3f;

typedef struct {
    MeshLibVector3f min;
    MeshLibVector3f max;
} MeshLibBox3f;

typedef struct {
    float m[3][3];  // 3x3 rotation/scale matrix
    MeshLibVector3f b;  // translation
} MeshLibAffineXf3f;

// =============================================================================
// Error Handling
// =============================================================================

typedef enum {
    MESHLIB_SUCCESS = 0,
    MESHLIB_ERROR_INVALID_ARGUMENT = 1,
    MESHLIB_ERROR_OUT_OF_MEMORY = 2,
    MESHLIB_ERROR_IO_FAILED = 3,
    MESHLIB_ERROR_OPERATION_FAILED = 4,
    MESHLIB_ERROR_NOT_IMPLEMENTED = 5,
} MeshLibError;

/// Get the last error message (thread-local)
MESHLIB_C_API const char* meshlib_get_last_error(void);

/// Clear the last error
MESHLIB_C_API void meshlib_clear_error(void);

// =============================================================================
// Version Info
// =============================================================================

MESHLIB_C_API const char* meshlib_version(void);
MESHLIB_C_API bool meshlib_has_parallel_support(void);
MESHLIB_C_API bool meshlib_has_voxel_support(void);

// =============================================================================
// Mesh Management
// =============================================================================

/// Create a new empty mesh
MESHLIB_C_API MeshLibMesh* meshlib_mesh_create(void);

/// Destroy a mesh
MESHLIB_C_API void meshlib_mesh_destroy(MeshLibMesh* mesh);

/// Clone a mesh
MESHLIB_C_API MeshLibMesh* meshlib_mesh_clone(const MeshLibMesh* mesh);

/// Get number of vertices
MESHLIB_C_API size_t meshlib_mesh_num_vertices(const MeshLibMesh* mesh);

/// Get number of faces
MESHLIB_C_API size_t meshlib_mesh_num_faces(const MeshLibMesh* mesh);

/// Get bounding box
MESHLIB_C_API MeshLibBox3f meshlib_mesh_bounding_box(const MeshLibMesh* mesh);

/// Get vertices as flat array (xyz, xyz, xyz...)
/// Returns the number of floats written (3 * num_vertices)
MESHLIB_C_API size_t meshlib_mesh_get_vertices(
    const MeshLibMesh* mesh,
    float* buffer,
    size_t buffer_size
);

/// Get face indices as flat array (v0, v1, v2, v0, v1, v2...)
/// Returns the number of ints written (3 * num_faces)
MESHLIB_C_API size_t meshlib_mesh_get_faces(
    const MeshLibMesh* mesh,
    int32_t* buffer,
    size_t buffer_size
);

/// Create mesh from vertices and faces
MESHLIB_C_API MeshLibMesh* meshlib_mesh_from_arrays(
    const float* vertices,
    size_t num_vertices,
    const int32_t* faces,
    size_t num_faces
);

/// Transform mesh in place
MESHLIB_C_API MeshLibError meshlib_mesh_transform(
    MeshLibMesh* mesh,
    const MeshLibAffineXf3f* transform
);

// =============================================================================
// Primitive Creation
// =============================================================================

MESHLIB_C_API MeshLibMesh* meshlib_make_uv_sphere(
    float radius,
    int h_segments,
    int v_segments
);

MESHLIB_C_API MeshLibMesh* meshlib_make_cube(
    float size_x,
    float size_y,
    float size_z
);

MESHLIB_C_API MeshLibMesh* meshlib_make_cylinder(
    float radius,
    float height,
    int segments
);

MESHLIB_C_API MeshLibMesh* meshlib_make_torus(
    float major_radius,
    float minor_radius,
    int major_segments,
    int minor_segments
);

// =============================================================================
// File I/O
// =============================================================================

MESHLIB_C_API MeshLibMesh* meshlib_mesh_load(const char* path);

MESHLIB_C_API MeshLibError meshlib_mesh_save(
    const MeshLibMesh* mesh,
    const char* path
);

MESHLIB_C_API MeshLibMesh* meshlib_mesh_load_from_memory(
    const uint8_t* data,
    size_t data_size,
    const char* format  // "stl", "obj", "ply", "off"
);

MESHLIB_C_API MeshLibError meshlib_mesh_save_to_memory(
    const MeshLibMesh* mesh,
    uint8_t** out_data,
    size_t* out_size,
    const char* format
);

/// Free memory allocated by meshlib_mesh_save_to_memory
MESHLIB_C_API void meshlib_free_buffer(uint8_t* buffer);

// =============================================================================
// Decimation
// =============================================================================

typedef enum {
    MESHLIB_DECIMATE_MINIMIZE_ERROR = 0,
    MESHLIB_DECIMATE_SHORTEST_EDGE_FIRST = 1
} MeshLibDecimateStrategy;

typedef struct {
    MeshLibDecimateStrategy strategy;
    float max_error;
    float max_edge_len;
    float max_triangle_aspect_ratio;
    int max_deleted_vertices;
    int max_deleted_faces;
    bool pack_mesh;
} MeshLibDecimateSettings;

/// Initialize settings with defaults
MESHLIB_C_API void meshlib_decimate_settings_init(MeshLibDecimateSettings* settings);

/// Decimate mesh in place
MESHLIB_C_API MeshLibError meshlib_decimate_mesh(
    MeshLibMesh* mesh,
    const MeshLibDecimateSettings* settings
);

// =============================================================================
// Subdivision
// =============================================================================

typedef struct {
    float max_edge_len;
    int max_edge_splits;
    float max_deviation_after_flip;
} MeshLibSubdivideSettings;

MESHLIB_C_API void meshlib_subdivide_settings_init(MeshLibSubdivideSettings* settings);

MESHLIB_C_API MeshLibError meshlib_subdivide_mesh(
    MeshLibMesh* mesh,
    const MeshLibSubdivideSettings* settings
);

// =============================================================================
// Boolean Operations
// =============================================================================

typedef enum {
    MESHLIB_BOOLEAN_UNION = 0,
    MESHLIB_BOOLEAN_INTERSECTION = 1,
    MESHLIB_BOOLEAN_DIFFERENCE_AB = 2,
    MESHLIB_BOOLEAN_DIFFERENCE_BA = 3
} MeshLibBooleanOperation;

typedef struct {
    MeshLibMesh* mesh;
    char error_string[256];
    bool valid;
} MeshLibBooleanResult;

MESHLIB_C_API MeshLibBooleanResult meshlib_boolean(
    const MeshLibMesh* mesh_a,
    const MeshLibMesh* mesh_b,
    MeshLibBooleanOperation operation,
    const MeshLibAffineXf3f* transform_b2a  // optional, NULL for identity
);

// =============================================================================
// Collision Detection
// =============================================================================

MESHLIB_C_API bool meshlib_meshes_collide(
    const MeshLibMesh* mesh_a,
    const MeshLibMesh* mesh_b,
    const MeshLibAffineXf3f* transform_b2a
);

MESHLIB_C_API bool meshlib_has_self_intersections(const MeshLibMesh* mesh);

// =============================================================================
// Mesh Relaxation
// =============================================================================

typedef struct {
    int iterations;
    float force;
} MeshLibRelaxParams;

MESHLIB_C_API void meshlib_relax_params_init(MeshLibRelaxParams* params);

MESHLIB_C_API MeshLibError meshlib_relax_mesh(
    MeshLibMesh* mesh,
    const MeshLibRelaxParams* params
);

// =============================================================================
// Point Cloud
// =============================================================================

MESHLIB_C_API MeshLibPointCloud* meshlib_pointcloud_create(void);
MESHLIB_C_API void meshlib_pointcloud_destroy(MeshLibPointCloud* pc);
MESHLIB_C_API size_t meshlib_pointcloud_num_points(const MeshLibPointCloud* pc);
MESHLIB_C_API bool meshlib_pointcloud_has_normals(const MeshLibPointCloud* pc);

MESHLIB_C_API MeshLibPointCloud* meshlib_pointcloud_from_array(
    const float* points,
    size_t num_points,
    const float* normals  // optional, NULL if no normals
);

/// Triangulate point cloud to mesh
MESHLIB_C_API MeshLibMesh* meshlib_triangulate_pointcloud(
    const MeshLibPointCloud* pc,
    int num_neighbours
);

// =============================================================================
// Transformation Helpers
// =============================================================================

MESHLIB_C_API MeshLibAffineXf3f meshlib_transform_identity(void);
MESHLIB_C_API MeshLibAffineXf3f meshlib_transform_translation(MeshLibVector3f v);
MESHLIB_C_API MeshLibAffineXf3f meshlib_transform_scale(float s);
MESHLIB_C_API MeshLibAffineXf3f meshlib_transform_multiply(
    const MeshLibAffineXf3f* a,
    const MeshLibAffineXf3f* b
);

#ifdef __cplusplus
}
#endif

#endif // MESHLIB_C_H
