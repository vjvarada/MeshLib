/**
 * @file meshlib_c.cpp
 * @brief C API implementation for MeshLib Core
 */

#include "meshlib_c.h"
#include <meshlib/meshlib.h>
#include <cstring>
#include <string>

using namespace meshlib;

// =============================================================================
// Thread-Local Error Handling
// =============================================================================

static thread_local std::string g_last_error;

static void set_error(const std::string& msg) {
    g_last_error = msg;
}

extern "C" {

const char* meshlib_get_last_error(void) {
    return g_last_error.c_str();
}

void meshlib_clear_error(void) {
    g_last_error.clear();
}

// =============================================================================
// Version Info
// =============================================================================

const char* meshlib_version(void) {
    return MESHLIB_CORE_VERSION;
}

bool meshlib_has_parallel_support(void) {
    return MESHLIB_PARALLEL_ENABLED;
}

bool meshlib_has_voxel_support(void) {
#ifdef MESHLIB_HAS_OPENVDB
    return true;
#else
    return false;
#endif
}

// =============================================================================
// Mesh Management
// =============================================================================

MeshLibMesh* meshlib_mesh_create(void) {
    return reinterpret_cast<MeshLibMesh*>(new Mesh());
}

void meshlib_mesh_destroy(MeshLibMesh* mesh) {
    delete reinterpret_cast<Mesh*>(mesh);
}

MeshLibMesh* meshlib_mesh_clone(const MeshLibMesh* mesh) {
    if (!mesh) {
        set_error("Null mesh pointer");
        return nullptr;
    }
    return reinterpret_cast<MeshLibMesh*>(
        new Mesh(*reinterpret_cast<const Mesh*>(mesh))
    );
}

size_t meshlib_mesh_num_vertices(const MeshLibMesh* mesh) {
    if (!mesh) return 0;
    return reinterpret_cast<const Mesh*>(mesh)->topology.numValidVerts();
}

size_t meshlib_mesh_num_faces(const MeshLibMesh* mesh) {
    if (!mesh) return 0;
    return reinterpret_cast<const Mesh*>(mesh)->topology.numValidFaces();
}

MeshLibBox3f meshlib_mesh_bounding_box(const MeshLibMesh* mesh) {
    MeshLibBox3f result = {};
    if (!mesh) return result;
    
    auto box = reinterpret_cast<const Mesh*>(mesh)->getBoundingBox();
    result.min = {box.min.x, box.min.y, box.min.z};
    result.max = {box.max.x, box.max.y, box.max.z};
    return result;
}

size_t meshlib_mesh_get_vertices(
    const MeshLibMesh* mesh,
    float* buffer,
    size_t buffer_size
) {
    if (!mesh || !buffer) return 0;
    
    const auto& m = *reinterpret_cast<const Mesh*>(mesh);
    const auto& points = m.points;
    
    size_t num_floats = points.size() * 3;
    if (buffer_size < num_floats) {
        set_error("Buffer too small");
        return 0;
    }
    
    size_t idx = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        buffer[idx++] = points[VertId(i)].x;
        buffer[idx++] = points[VertId(i)].y;
        buffer[idx++] = points[VertId(i)].z;
    }
    
    return num_floats;
}

size_t meshlib_mesh_get_faces(
    const MeshLibMesh* mesh,
    int32_t* buffer,
    size_t buffer_size
) {
    if (!mesh || !buffer) return 0;
    
    const auto& m = *reinterpret_cast<const Mesh*>(mesh);
    // TODO: Extract faces from topology
    // This requires iterating over the half-edge structure
    
    return 0; // Placeholder
}

MeshLibMesh* meshlib_mesh_from_arrays(
    const float* vertices,
    size_t num_vertices,
    const int32_t* faces,
    size_t num_faces
) {
    if (!vertices || !faces || num_vertices == 0 || num_faces == 0) {
        set_error("Invalid input arrays");
        return nullptr;
    }
    
    try {
        VertCoords points;
        points.resize(num_vertices);
        for (size_t i = 0; i < num_vertices; ++i) {
            points[VertId(i)] = Vector3f(
                vertices[i * 3],
                vertices[i * 3 + 1],
                vertices[i * 3 + 2]
            );
        }
        
        Triangulation tris;
        tris.reserve(num_faces);
        for (size_t i = 0; i < num_faces; ++i) {
            tris.push_back({
                VertId(faces[i * 3]),
                VertId(faces[i * 3 + 1]),
                VertId(faces[i * 3 + 2])
            });
        }
        
        auto mesh = Mesh::fromTriangles(std::move(points), tris);
        return reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(mesh)));
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

MeshLibError meshlib_mesh_transform(
    MeshLibMesh* mesh,
    const MeshLibAffineXf3f* transform
) {
    if (!mesh || !transform) {
        set_error("Null pointer");
        return MESHLIB_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        AffineXf3f xf;
        // Copy matrix
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                xf.A(i, j) = transform->m[i][j];
            }
        }
        xf.b = Vector3f(transform->b.x, transform->b.y, transform->b.z);
        
        reinterpret_cast<Mesh*>(mesh)->transform(xf);
        return MESHLIB_SUCCESS;
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return MESHLIB_ERROR_OPERATION_FAILED;
    }
}

// =============================================================================
// Primitive Creation
// =============================================================================

MeshLibMesh* meshlib_make_uv_sphere(float radius, int h_segments, int v_segments) {
    try {
        auto mesh = makeUVSphere(radius, h_segments, v_segments);
        return reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(mesh)));
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

MeshLibMesh* meshlib_make_cube(float size_x, float size_y, float size_z) {
    try {
        auto mesh = makeCube(Vector3f(size_x, size_y, size_z));
        return reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(mesh)));
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

MeshLibMesh* meshlib_make_cylinder(float radius, float height, int segments) {
    try {
        auto mesh = makeCylinder(radius, height, segments);
        return reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(mesh)));
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

MeshLibMesh* meshlib_make_torus(float major_radius, float minor_radius, int major_segments, int minor_segments) {
    try {
        auto mesh = makeTorus(major_radius, minor_radius, major_segments, minor_segments);
        return reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(mesh)));
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

// =============================================================================
// File I/O
// =============================================================================

MeshLibMesh* meshlib_mesh_load(const char* path) {
    if (!path) {
        set_error("Null path");
        return nullptr;
    }
    
    try {
        auto result = loadMesh(path);
        if (!result) {
            set_error(result.error());
            return nullptr;
        }
        return reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(*result)));
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

MeshLibError meshlib_mesh_save(const MeshLibMesh* mesh, const char* path) {
    if (!mesh || !path) {
        set_error("Null pointer");
        return MESHLIB_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto result = MeshSave::toAnySupportedFormat(
            *reinterpret_cast<const Mesh*>(mesh), path);
        if (!result) {
            set_error(result.error());
            return MESHLIB_ERROR_IO_FAILED;
        }
        return MESHLIB_SUCCESS;
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return MESHLIB_ERROR_IO_FAILED;
    }
}

MeshLibMesh* meshlib_mesh_load_from_memory(
    const uint8_t* data,
    size_t data_size,
    const char* format
) {
    if (!data || data_size == 0 || !format) {
        set_error("Invalid arguments");
        return nullptr;
    }
    
    // TODO: Implement memory-based loading
    set_error("Not implemented");
    return nullptr;
}

MeshLibError meshlib_mesh_save_to_memory(
    const MeshLibMesh* mesh,
    uint8_t** out_data,
    size_t* out_size,
    const char* format
) {
    if (!mesh || !out_data || !out_size || !format) {
        set_error("Invalid arguments");
        return MESHLIB_ERROR_INVALID_ARGUMENT;
    }
    
    // TODO: Implement memory-based saving
    set_error("Not implemented");
    return MESHLIB_ERROR_NOT_IMPLEMENTED;
}

void meshlib_free_buffer(uint8_t* buffer) {
    delete[] buffer;
}

// =============================================================================
// Decimation
// =============================================================================

void meshlib_decimate_settings_init(MeshLibDecimateSettings* settings) {
    if (!settings) return;
    
    settings->strategy = MESHLIB_DECIMATE_MINIMIZE_ERROR;
    settings->max_error = FLT_MAX;
    settings->max_edge_len = FLT_MAX;
    settings->max_triangle_aspect_ratio = 20.0f;
    settings->max_deleted_vertices = INT_MAX;
    settings->max_deleted_faces = INT_MAX;
    settings->pack_mesh = false;
}

MeshLibError meshlib_decimate_mesh(
    MeshLibMesh* mesh,
    const MeshLibDecimateSettings* settings
) {
    if (!mesh) {
        set_error("Null mesh");
        return MESHLIB_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        DecimateSettings s;
        if (settings) {
            s.strategy = static_cast<DecimateStrategy>(settings->strategy);
            s.maxError = settings->max_error;
            s.maxEdgeLen = settings->max_edge_len;
            s.maxTriangleAspectRatio = settings->max_triangle_aspect_ratio;
            s.maxDeletedVertices = settings->max_deleted_vertices;
            s.maxDeletedFaces = settings->max_deleted_faces;
            s.packMesh = settings->pack_mesh;
        }
        
        decimateMesh(*reinterpret_cast<Mesh*>(mesh), s);
        return MESHLIB_SUCCESS;
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return MESHLIB_ERROR_OPERATION_FAILED;
    }
}

// =============================================================================
// Subdivision
// =============================================================================

void meshlib_subdivide_settings_init(MeshLibSubdivideSettings* settings) {
    if (!settings) return;
    
    settings->max_edge_len = 0.0f;
    settings->max_edge_splits = 1000;
    settings->max_deviation_after_flip = 1.0f;
}

MeshLibError meshlib_subdivide_mesh(
    MeshLibMesh* mesh,
    const MeshLibSubdivideSettings* settings
) {
    if (!mesh) {
        set_error("Null mesh");
        return MESHLIB_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        SubdivideSettings s;
        if (settings) {
            s.maxEdgeLen = settings->max_edge_len;
            s.maxEdgeSplits = settings->max_edge_splits;
            s.maxDeviationAfterFlip = settings->max_deviation_after_flip;
        }
        
        subdivideMesh(*reinterpret_cast<Mesh*>(mesh), s);
        return MESHLIB_SUCCESS;
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return MESHLIB_ERROR_OPERATION_FAILED;
    }
}

// =============================================================================
// Boolean Operations
// =============================================================================

MeshLibBooleanResult meshlib_boolean(
    const MeshLibMesh* mesh_a,
    const MeshLibMesh* mesh_b,
    MeshLibBooleanOperation operation,
    const MeshLibAffineXf3f* transform_b2a
) {
    MeshLibBooleanResult result = {};
    
    if (!mesh_a || !mesh_b) {
        strncpy(result.error_string, "Null mesh pointer", sizeof(result.error_string) - 1);
        result.valid = false;
        return result;
    }
    
    try {
        const Mesh& a = *reinterpret_cast<const Mesh*>(mesh_a);
        const Mesh& b = *reinterpret_cast<const Mesh*>(mesh_b);
        
        const AffineXf3f* xf = nullptr;
        AffineXf3f xf_storage;
        if (transform_b2a) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    xf_storage.A(i, j) = transform_b2a->m[i][j];
                }
            }
            xf_storage.b = Vector3f(transform_b2a->b.x, transform_b2a->b.y, transform_b2a->b.z);
            xf = &xf_storage;
        }
        
        auto boolResult = boolean(a, b, static_cast<BooleanOperation>(operation), xf);
        
        if (boolResult.valid()) {
            result.mesh = reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(boolResult.mesh)));
            result.valid = true;
        } else {
            strncpy(result.error_string, boolResult.errorString.c_str(), sizeof(result.error_string) - 1);
            result.valid = false;
        }
    }
    catch (const std::exception& e) {
        strncpy(result.error_string, e.what(), sizeof(result.error_string) - 1);
        result.valid = false;
    }
    
    return result;
}

// =============================================================================
// Collision Detection
// =============================================================================

bool meshlib_meshes_collide(
    const MeshLibMesh* mesh_a,
    const MeshLibMesh* mesh_b,
    const MeshLibAffineXf3f* transform_b2a
) {
    if (!mesh_a || !mesh_b) return false;
    
    try {
        const Mesh& a = *reinterpret_cast<const Mesh*>(mesh_a);
        const Mesh& b = *reinterpret_cast<const Mesh*>(mesh_b);
        
        const AffineXf3f* xf = nullptr;
        AffineXf3f xf_storage;
        if (transform_b2a) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    xf_storage.A(i, j) = transform_b2a->m[i][j];
                }
            }
            xf_storage.b = Vector3f(transform_b2a->b.x, transform_b2a->b.y, transform_b2a->b.z);
            xf = &xf_storage;
        }
        
        auto result = findCollidingTriangles(MeshPart{a}, MeshPart{b}, xf, true);
        return !result.empty();
    }
    catch (...) {
        return false;
    }
}

bool meshlib_has_self_intersections(const MeshLibMesh* mesh) {
    if (!mesh) return false;
    
    try {
        const Mesh& m = *reinterpret_cast<const Mesh*>(mesh);
        auto result = findSelfCollidingTriangles(MeshPart{m}, nullptr, nullptr);
        return result.has_value() && result.value();
    }
    catch (...) {
        return false;
    }
}

// =============================================================================
// Mesh Relaxation
// =============================================================================

void meshlib_relax_params_init(MeshLibRelaxParams* params) {
    if (!params) return;
    params->iterations = 1;
    params->force = 0.5f;
}

MeshLibError meshlib_relax_mesh(
    MeshLibMesh* mesh,
    const MeshLibRelaxParams* params
) {
    if (!mesh) {
        set_error("Null mesh");
        return MESHLIB_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        MeshRelaxParams p;
        if (params) {
            p.iterations = params->iterations;
            p.force = params->force;
        }
        
        relax(*reinterpret_cast<Mesh*>(mesh), p);
        return MESHLIB_SUCCESS;
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return MESHLIB_ERROR_OPERATION_FAILED;
    }
}

// =============================================================================
// Point Cloud
// =============================================================================

MeshLibPointCloud* meshlib_pointcloud_create(void) {
    return reinterpret_cast<MeshLibPointCloud*>(new PointCloud());
}

void meshlib_pointcloud_destroy(MeshLibPointCloud* pc) {
    delete reinterpret_cast<PointCloud*>(pc);
}

size_t meshlib_pointcloud_num_points(const MeshLibPointCloud* pc) {
    if (!pc) return 0;
    return reinterpret_cast<const PointCloud*>(pc)->calcNumValidPoints();
}

bool meshlib_pointcloud_has_normals(const MeshLibPointCloud* pc) {
    if (!pc) return false;
    return reinterpret_cast<const PointCloud*>(pc)->hasNormals();
}

MeshLibPointCloud* meshlib_pointcloud_from_array(
    const float* points,
    size_t num_points,
    const float* normals
) {
    if (!points || num_points == 0) {
        set_error("Invalid arguments");
        return nullptr;
    }
    
    try {
        auto pc = new PointCloud();
        pc->points.resize(num_points);
        pc->validPoints.resize(num_points, true);
        
        for (size_t i = 0; i < num_points; ++i) {
            pc->points[VertId(i)] = Vector3f(
                points[i * 3],
                points[i * 3 + 1],
                points[i * 3 + 2]
            );
        }
        
        if (normals) {
            pc->normals.resize(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                pc->normals[VertId(i)] = Vector3f(
                    normals[i * 3],
                    normals[i * 3 + 1],
                    normals[i * 3 + 2]
                );
            }
        }
        
        return reinterpret_cast<MeshLibPointCloud*>(pc);
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

MeshLibMesh* meshlib_triangulate_pointcloud(
    const MeshLibPointCloud* pc,
    int num_neighbours
) {
    if (!pc) {
        set_error("Null point cloud");
        return nullptr;
    }
    
    try {
        TriangulationParameters params;
        params.numNeighbours = num_neighbours;
        
        auto result = triangulatePointCloud(
            *reinterpret_cast<const PointCloud*>(pc), params);
        
        if (!result) {
            set_error("Triangulation failed");
            return nullptr;
        }
        
        return reinterpret_cast<MeshLibMesh*>(new Mesh(std::move(*result)));
    }
    catch (const std::exception& e) {
        set_error(e.what());
        return nullptr;
    }
}

// =============================================================================
// Transformation Helpers
// =============================================================================

MeshLibAffineXf3f meshlib_transform_identity(void) {
    MeshLibAffineXf3f result;
    result.m[0][0] = 1; result.m[0][1] = 0; result.m[0][2] = 0;
    result.m[1][0] = 0; result.m[1][1] = 1; result.m[1][2] = 0;
    result.m[2][0] = 0; result.m[2][1] = 0; result.m[2][2] = 1;
    result.b = {0, 0, 0};
    return result;
}

MeshLibAffineXf3f meshlib_transform_translation(MeshLibVector3f v) {
    MeshLibAffineXf3f result = meshlib_transform_identity();
    result.b = v;
    return result;
}

MeshLibAffineXf3f meshlib_transform_scale(float s) {
    MeshLibAffineXf3f result;
    result.m[0][0] = s; result.m[0][1] = 0; result.m[0][2] = 0;
    result.m[1][0] = 0; result.m[1][1] = s; result.m[1][2] = 0;
    result.m[2][0] = 0; result.m[2][1] = 0; result.m[2][2] = s;
    result.b = {0, 0, 0};
    return result;
}

MeshLibAffineXf3f meshlib_transform_multiply(
    const MeshLibAffineXf3f* a,
    const MeshLibAffineXf3f* b
) {
    MeshLibAffineXf3f result;
    
    // Matrix multiplication: A * B
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                result.m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
    
    // Translation: A.A * B.b + A.b
    result.b.x = a->m[0][0] * b->b.x + a->m[0][1] * b->b.y + a->m[0][2] * b->b.z + a->b.x;
    result.b.y = a->m[1][0] * b->b.x + a->m[1][1] * b->b.y + a->m[1][2] * b->b.z + a->b.y;
    result.b.z = a->m[2][0] * b->b.x + a->m[2][1] * b->b.y + a->m[2][2] * b->b.z + a->b.z;
    
    return result;
}

} // extern "C"
