#pragma once

/**
 * @file MeshIO.h
 * @brief Mesh file I/O functions
 */

#include "meshlib/config.h"
#include "../mesh/Mesh.h"
#include "../mesh/PointCloud.h"
#include "../core/Types.h"
#include <string>
#include <vector>
#include <istream>
#include <ostream>
#include <cstdint>

namespace meshlib {

/**
 * @brief Supported mesh file formats
 */
enum class MeshFormat {
    Auto,       ///< Auto-detect from file extension
    OBJ,        ///< Wavefront OBJ
    STL,        ///< STereoLithography (binary or ASCII)
    STL_ASCII,  ///< STereoLithography (ASCII only)
    STL_BINARY, ///< STereoLithography (binary only)
    PLY,        ///< Polygon File Format
    PLY_ASCII,  ///< PLY ASCII
    PLY_BINARY, ///< PLY Binary
    OFF,        ///< Object File Format
    GLTF,       ///< glTF 2.0
    GLB         ///< glTF Binary
};

/**
 * @brief Options for loading meshes
 */
struct MESHLIB_API LoadOptions {
    /// Expected file format (Auto = detect from extension)
    MeshFormat format = MeshFormat::Auto;
    
    /// Load vertex normals if available
    bool loadNormals = true;
    
    /// Load texture coordinates if available
    bool loadUVs = true;
    
    /// Load vertex colors if available
    bool loadColors = true;
    
    /// Merge duplicate vertices
    bool mergeVertices = true;
    
    /// Tolerance for vertex merging
    float mergeTolerance = 1e-6f;
    
    /// Progress callback
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Options for saving meshes
 */
struct MESHLIB_API SaveOptions {
    /// Output file format (Auto = detect from extension)
    MeshFormat format = MeshFormat::Auto;
    
    /// Save vertex normals
    bool saveNormals = true;
    
    /// Save texture coordinates
    bool saveUVs = true;
    
    /// Save vertex colors
    bool saveColors = true;
    
    /// For binary formats: use binary encoding
    bool binary = true;
    
    /// Number of decimal places for ASCII formats
    int precision = 6;
    
    /// Progress callback
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Result of loading a mesh
 */
struct MESHLIB_API LoadResult {
    Mesh mesh;              ///< Loaded mesh
    Result status;          ///< Operation status
    std::string filename;   ///< Source filename
    
    bool ok() const { return status.ok(); }
    explicit operator bool() const { return ok(); }
};

// ==================== Mesh Loading ====================

/**
 * @brief Load a mesh from a file
 * @param filename Path to the mesh file
 * @param options Loading options
 * @return Result containing loaded mesh
 */
MESHLIB_API LoadResult loadMesh(const std::string& filename, const LoadOptions& options = {});

/**
 * @brief Load a mesh from memory buffer
 * @param data Pointer to mesh data
 * @param size Size of data in bytes
 * @param format Format of the data
 * @param options Loading options
 * @return Result containing loaded mesh
 */
MESHLIB_API LoadResult loadMeshFromMemory(
    const uint8_t* data, 
    size_t size, 
    MeshFormat format,
    const LoadOptions& options = {});

/**
 * @brief Load a mesh from a stream
 * @param stream Input stream
 * @param format Format of the data
 * @param options Loading options
 * @return Result containing loaded mesh
 */
MESHLIB_API LoadResult loadMeshFromStream(
    std::istream& stream,
    MeshFormat format,
    const LoadOptions& options = {});

// ==================== Mesh Saving ====================

/**
 * @brief Save a mesh to a file
 * @param mesh Mesh to save
 * @param filename Output filename
 * @param options Saving options
 * @return Result of the operation
 */
MESHLIB_API Result saveMesh(const Mesh& mesh, const std::string& filename, const SaveOptions& options = {});

/**
 * @brief Save a mesh to memory buffer
 * @param mesh Mesh to save
 * @param format Output format
 * @param options Saving options
 * @return Vector containing mesh data, or empty on error
 */
MESHLIB_API std::vector<uint8_t> saveMeshToMemory(
    const Mesh& mesh,
    MeshFormat format,
    const SaveOptions& options = {});

/**
 * @brief Save a mesh to a stream
 * @param mesh Mesh to save
 * @param stream Output stream
 * @param format Output format
 * @param options Saving options
 * @return Result of the operation
 */
MESHLIB_API Result saveMeshToStream(
    const Mesh& mesh,
    std::ostream& stream,
    MeshFormat format,
    const SaveOptions& options = {});

// ==================== Point Cloud I/O ====================

/**
 * @brief Load a point cloud from file
 * @param filename Path to file
 * @return Loaded point cloud (empty on error)
 */
MESHLIB_API PointCloud loadPointCloud(const std::string& filename);

/**
 * @brief Save a point cloud to file
 * @param cloud Point cloud to save
 * @param filename Output filename
 * @return Result of the operation
 */
MESHLIB_API Result savePointCloud(const PointCloud& cloud, const std::string& filename);

// ==================== Format Detection ====================

/**
 * @brief Detect mesh format from filename extension
 * @param filename Filename or path
 * @return Detected format, or Auto if unknown
 */
MESHLIB_API MeshFormat detectFormat(const std::string& filename);

/**
 * @brief Get file extension for a format
 * @param format Mesh format
 * @return File extension (e.g., ".obj")
 */
MESHLIB_API std::string formatExtension(MeshFormat format);

/**
 * @brief Get list of supported file extensions for loading
 * @return Vector of extensions (e.g., {".obj", ".stl", ".ply"})
 */
MESHLIB_API std::vector<std::string> supportedLoadExtensions();

/**
 * @brief Get list of supported file extensions for saving
 * @return Vector of extensions
 */
MESHLIB_API std::vector<std::string> supportedSaveExtensions();

} // namespace meshlib
