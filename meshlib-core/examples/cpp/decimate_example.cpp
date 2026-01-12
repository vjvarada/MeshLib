/**
 * @file decimate_example.cpp
 * @brief Example: Mesh decimation with MeshLib Core
 */

#include <meshlib/meshlib.h>
#include <iostream>

using namespace meshlib;

int main() {
    std::cout << "MeshLib Core Decimation Example\n";
    std::cout << "Version: " << MESHLIB_CORE_VERSION << "\n\n";
    
    // Create a high-resolution sphere
    std::cout << "Creating high-resolution sphere...\n";
    Mesh mesh = makeUVSphere(1.0f, 64, 64);
    
    std::cout << "Original mesh:\n";
    std::cout << "  Vertices: " << mesh.topology.numValidVerts() << "\n";
    std::cout << "  Faces: " << mesh.topology.numValidFaces() << "\n\n";
    
    // Save original
    MeshSave::toAnySupportedFormat(mesh, "original.stl");
    
    // Decimate with different error thresholds
    float errors[] = {0.01f, 0.05f, 0.1f};
    
    for (float maxError : errors) {
        // Clone the mesh
        Mesh decimated = mesh;
        
        // Configure decimation
        DecimateSettings settings;
        settings.strategy = DecimateStrategy::MinimizeError;
        settings.maxError = maxError;
        settings.packMesh = true;
        
        std::cout << "Decimating with maxError = " << maxError << "...\n";
        
        DecimateResult result = decimateMesh(decimated, settings);
        
        std::cout << "  Vertices deleted: " << result.vertsDeleted << "\n";
        std::cout << "  Faces deleted: " << result.facesDeleted << "\n";
        std::cout << "  Final vertices: " << decimated.topology.numValidVerts() << "\n";
        std::cout << "  Final faces: " << decimated.topology.numValidFaces() << "\n";
        std::cout << "  Error introduced: " << result.errorIntroduced << "\n\n";
        
        // Save decimated mesh
        std::string filename = "decimated_" + std::to_string(maxError) + ".stl";
        MeshSave::toAnySupportedFormat(decimated, filename);
        std::cout << "  Saved to " << filename << "\n\n";
    }
    
    std::cout << "Done!\n";
    return 0;
}
