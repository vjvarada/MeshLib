#pragma once

/**
 * @file Boolean.h
 * @brief Boolean operations on meshes (CSG)
 */

#include "meshlib/config.h"
#include "../mesh/Mesh.h"
#include "../core/Types.h"
#include <variant>

namespace meshlib {

/**
 * @brief Type of boolean operation
 */
enum class BooleanOperation {
    Union,        ///< A ∪ B - combines both volumes
    Intersection, ///< A ∩ B - keeps only overlapping volume
    Difference,   ///< A - B - subtracts B from A
    SymDifference ///< A △ B - keeps non-overlapping volumes (XOR)
};

/**
 * @brief Parameters for boolean operations
 */
struct MESHLIB_API BooleanParams {
    /// The type of boolean operation
    BooleanOperation operation = BooleanOperation::Union;
    
    /// Tolerance for considering points coincident
    float tolerance = 1e-6f;
    
    /// Whether to merge close vertices in the result
    bool mergeCloseVertices = true;
    
    /// Progress callback (can be null)
    ProgressCallback progressCallback = nullptr;
};

/**
 * @brief Result of a boolean operation
 */
struct MESHLIB_API BooleanResult {
    Mesh mesh;           ///< Resulting mesh
    Result status;       ///< Operation status
    
    /// Check if operation succeeded
    bool ok() const { return status.ok(); }
    explicit operator bool() const { return ok(); }
};

// ==================== Boolean Operations ====================

/**
 * @brief Perform a boolean operation on two meshes
 * @param meshA First input mesh
 * @param meshB Second input mesh
 * @param params Operation parameters
 * @return Result containing the output mesh or error
 */
MESHLIB_API BooleanResult boolean(
    const Mesh& meshA,
    const Mesh& meshB,
    const BooleanParams& params = {});

/**
 * @brief Compute the union of two meshes (A ∪ B)
 */
MESHLIB_API BooleanResult booleanUnion(const Mesh& meshA, const Mesh& meshB);

/**
 * @brief Compute the intersection of two meshes (A ∩ B)
 */
MESHLIB_API BooleanResult booleanIntersection(const Mesh& meshA, const Mesh& meshB);

/**
 * @brief Compute the difference of two meshes (A - B)
 */
MESHLIB_API MESHLIB_API BooleanResult booleanDifference(const Mesh& meshA, const Mesh& meshB);

/**
 * @brief Compute the symmetric difference of two meshes (XOR)
 */
MESHLIB_API BooleanResult booleanSymDifference(const Mesh& meshA, const Mesh& meshB);

// ==================== Convenience Functions ====================

/**
 * @brief Merge multiple meshes into one
 * @param meshes Vector of meshes to merge
 * @return Combined mesh (simple merge, not boolean union)
 */
MESHLIB_API Mesh mergeMeshes(const std::vector<Mesh>& meshes);

/**
 * @brief Merge multiple meshes with boolean union
 * @param meshes Vector of meshes to union
 * @return Result of unioning all meshes
 */
MESHLIB_API BooleanResult unionMeshes(const std::vector<Mesh>& meshes);

} // namespace meshlib
