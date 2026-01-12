#pragma once

/**
 * @file Types.h
 * @brief Core type definitions for meshlib
 * 
 * This file defines the fundamental types used throughout the meshlib library.
 */

#include <cstddef>
#include <cstdint>
#include <vector>
#include <array>
#include <string>

namespace meshlib {

/// Forward declarations
template <typename T> struct Vector2;
template <typename T> struct Vector3;
template <typename T> struct Matrix3;
template <typename T> struct Box3;
template <typename T> class Id;

/// Common vector types
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;
using Vector2i = Vector2<int>;

using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;
using Vector3i = Vector3<int>;

/// Common matrix types
using Matrix3f = Matrix3<float>;
using Matrix3d = Matrix3<double>;

/// Common box types
using Box3f = Box3<float>;
using Box3d = Box3<double>;

/// Tag types for type-safe IDs
struct VertexTag {};
struct FaceTag {};
struct EdgeTag {};
struct UndirectedEdgeTag {};

/// Type-safe IDs
using VertexId = Id<VertexTag>;
using FaceId = Id<FaceTag>;
using EdgeId = Id<EdgeTag>;
using UndirectedEdgeId = Id<UndirectedEdgeTag>;

/// Common containers
using VertexCoords = std::vector<Vector3f>;
using FaceIndices = std::vector<std::array<int, 3>>;
using Normals = std::vector<Vector3f>;

/// Three vertices forming a triangle
using ThreeVertIds = std::array<VertexId, 3>;

/// Triangle index triplet (3 vertex indices)
using Triangle = std::array<int, 3>;

/// Triangle representation with 3D points
using Triangle3f = std::array<Vector3f, 3>;

/// Result type for operations that can fail
enum class ErrorCode {
    Success = 0,
    InvalidInput,
    FileNotFound,
    ParseError,
    UnsupportedFormat,
    OutOfMemory,
    OperationFailed,
    InvalidMesh,
    NonManifold,
    EmptyMesh
};

/// Simple error result
struct Result {
    ErrorCode code = ErrorCode::Success;
    std::string message;
    
    bool ok() const { return code == ErrorCode::Success; }
    explicit operator bool() const { return ok(); }
};

/// Callback for reporting progress (0.0 to 1.0)
using ProgressCallback = bool(*)(float progress);

} // namespace meshlib
