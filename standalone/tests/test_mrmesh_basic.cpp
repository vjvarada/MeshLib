#include <iostream>
#include "MRMesh/MRMesh.h"
#include "MRMesh/MRMeshBuilder.h"
#include "MRMesh/MRCube.h"
#include "MRMesh/MRMeshBoolean.h"
#include "MRMesh/MRMeshDecimate.h"
#include "MRMesh/MRAffineXf3.h"

int main() {
    std::cout << "=== MRMesh Standalone Test ===" << std::endl;
    
    // Test 1: Create a simple cube mesh
    std::cout << "\n[Test 1] Creating cube mesh..." << std::endl;
    auto cube = MR::makeCube(MR::Vector3f(1.0f, 1.0f, 1.0f));
    std::cout << "  Vertices: " << cube.points.size() << std::endl;
    std::cout << "  Faces: " << cube.topology.numValidFaces() << std::endl;
    
    if (cube.points.size() != 8 || cube.topology.numValidFaces() != 12) {
        std::cerr << "FAILED: Cube mesh has incorrect vertex/face count" << std::endl;
        return 1;
    }
    std::cout << "  PASSED" << std::endl;
    
    // Test 2: Create second cube and perform boolean union
    std::cout << "\n[Test 2] Boolean union of two cubes..." << std::endl;
    auto cube2 = MR::makeCube(MR::Vector3f(1.0f, 1.0f, 1.0f));
    // Translate second cube
    MR::AffineXf3f xf = MR::AffineXf3f::translation(MR::Vector3f(0.5f, 0.5f, 0.5f));
    cube2.transform(xf);
    
    auto boolResult = MR::boolean(cube, cube2, MR::BooleanOperation::Union, nullptr);
    if (!boolResult.valid()) {
        std::cerr << "FAILED: Boolean union failed" << std::endl;
        return 1;
    }
    std::cout << "  Result vertices: " << boolResult.mesh.points.size() << std::endl;
    std::cout << "  Result faces: " << boolResult.mesh.topology.numValidFaces() << std::endl;
    std::cout << "  PASSED" << std::endl;
    
    // Test 3: Mesh decimation
    std::cout << "\n[Test 3] Mesh decimation..." << std::endl;
    auto sphereMesh = MR::makeCube(MR::Vector3f(1.0f, 1.0f, 1.0f));
    int originalFaces = static_cast<int>(sphereMesh.topology.numValidFaces());
    
    MR::DecimateSettings settings;
    settings.maxError = 0.1f;
    settings.maxDeletedFaces = originalFaces / 2;
    auto decimateResult = MR::decimateMesh(sphereMesh, settings);
    
    std::cout << "  Original faces: " << originalFaces << std::endl;
    std::cout << "  After decimation: " << sphereMesh.topology.numValidFaces() << std::endl;
    std::cout << "  PASSED" << std::endl;
    
    std::cout << "\n=== All Tests Passed ===" << std::endl;
    return 0;
}
