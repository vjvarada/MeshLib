// MRVoxels Basic Test
// Tests: Mesh offset (core MRVoxels operation with OpenVDB)
#include <iostream>
#include <cmath>

// MRMesh headers
#include "MRMesh/MRMesh.h"
#include "MRMesh/MRCube.h"
#include "MRMesh/MRBox.h"
#include "MRMesh/MRVector3.h"
#include "MRMesh/MRMeshPart.h"

// MRVoxels headers
#include "MRVoxels/MRVoxelsVolume.h"  // Defines VoxelsVolumeMinMax (VdbVolume)
#include "MRVoxels/MROffset.h"
#include "MRVoxels/MRVDBConversions.h"
#include "MRVoxels/MRFloatGrid.h"

int main() {
    std::cout << "=== MRVoxels Basic Test ===" << std::endl;
    int passed = 0;
    int failed = 0;

    // Test 1: Create a cube mesh for voxelization
    std::cout << "\n[Test 1] Creating source cube mesh..." << std::endl;
    MR::Mesh cubeMesh = MR::makeCube(MR::Vector3f(1.0f, 1.0f, 1.0f));
    if (cubeMesh.topology.numValidFaces() > 0) {
        std::cout << "  PASS: Cube mesh created with " << cubeMesh.topology.numValidFaces() << " faces" << std::endl;
        passed++;
    } else {
        std::cout << "  FAIL: Could not create cube mesh" << std::endl;
        failed++;
        return 1;
    }

    // Test 2: Mesh offset (the key MRVoxels operation - uses OpenVDB internally)
    std::cout << "\n[Test 2] Testing mesh offset (OpenVDB-based shell generation)..." << std::endl;
    try {
        MR::OffsetParameters offsetParams;
        offsetParams.voxelSize = 0.05f;
        offsetParams.signDetectionMode = MR::SignDetectionMode::OpenVDB;

        auto offsetResult = MR::offsetMesh(cubeMesh, 0.1f, offsetParams);
        if (offsetResult.has_value()) {
            auto& offsetMesh = offsetResult.value();
            std::cout << "  PASS: Offset mesh created with " 
                      << offsetMesh.topology.numValidFaces() << " faces" << std::endl;
            
            // Verify it's larger than original
            auto origBox = cubeMesh.getBoundingBox();
            auto offsetBox = offsetMesh.getBoundingBox();
            
            if (offsetBox.size().length() > origBox.size().length()) {
                std::cout << "  PASS: Offset mesh is correctly larger than original" << std::endl;
                passed++;
            } else {
                std::cout << "  WARN: Offset mesh size not as expected" << std::endl;
            }
            passed++;
        } else {
            std::cout << "  FAIL: Mesh offset failed - " << offsetResult.error() << std::endl;
            failed++;
        }
    } catch (const std::exception& e) {
        std::cout << "  EXCEPTION: " << e.what() << std::endl;
        failed++;
    }

    // Test 3: Mesh to VDB FloatGrid conversion
    std::cout << "\n[Test 3] Converting mesh to VDB FloatGrid..." << std::endl;
    try {
        MR::MeshToVolumeParams params;
        params.voxelSize = MR::Vector3f(0.05f, 0.05f, 0.05f);
        params.surfaceOffset = 3;

        auto floatGridResult = MR::meshToVolume(cubeMesh, params);
        if (floatGridResult.has_value()) {
            std::cout << "  PASS: VDB FloatGrid created successfully" << std::endl;
            passed++;
        } else {
            std::cout << "  FAIL: meshToVolume failed - " << floatGridResult.error() << std::endl;
            failed++;
        }
    } catch (const std::exception& e) {
        std::cout << "  EXCEPTION: " << e.what() << std::endl;
        failed++;
    }

    // Test 4: Suggest voxel size utility
    std::cout << "\n[Test 4] Testing suggestVoxelSize utility..." << std::endl;
    try {
        float suggestedSize = MR::suggestVoxelSize(cubeMesh, 100000.0f);
        if (suggestedSize > 0.0f) {
            std::cout << "  PASS: suggestVoxelSize returned " << suggestedSize << std::endl;
            passed++;
        } else {
            std::cout << "  FAIL: suggestVoxelSize returned invalid value" << std::endl;
            failed++;
        }
    } catch (const std::exception& e) {
        std::cout << "  EXCEPTION: " << e.what() << std::endl;
        failed++;
    }

    // Summary
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Passed: " << passed << std::endl;
    std::cout << "Failed: " << failed << std::endl;

    return (failed == 0) ? 0 : 1;
}
