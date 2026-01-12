/**
 * @file icp_example.cpp
 * @brief Example: ICP (Iterative Closest Point) alignment with MeshLib Core
 */

#include <meshlib/meshlib.h>
#include <iostream>
#include <cmath>

using namespace meshlib;

int main() {
    std::cout << "MeshLib Core ICP Alignment Example\n";
    std::cout << "Version: " << MESHLIB_CORE_VERSION << "\n\n";
    
    // Create a source mesh (sphere)
    std::cout << "Creating source and target meshes...\n";
    Mesh source = makeUVSphere(1.0f, 32, 32);
    Mesh target = source;  // Clone
    
    // Apply a known transformation to source
    // Rotation around Y axis by 30 degrees + translation
    float angle = 30.0f * 3.14159f / 180.0f;
    Matrix3f rotation;
    rotation(0, 0) = std::cos(angle);  rotation(0, 1) = 0;  rotation(0, 2) = std::sin(angle);
    rotation(1, 0) = 0;                 rotation(1, 1) = 1;  rotation(1, 2) = 0;
    rotation(2, 0) = -std::sin(angle); rotation(2, 1) = 0;  rotation(2, 2) = std::cos(angle);
    
    AffineXf3f knownTransform;
    knownTransform.A = rotation;
    knownTransform.b = Vector3f(0.1f, 0.2f, -0.15f);
    
    source.transform(knownTransform);
    
    std::cout << "Applied transformation to source:\n";
    std::cout << "  Rotation: 30 degrees around Y\n";
    std::cout << "  Translation: (0.1, 0.2, -0.15)\n\n";
    
    // Save initial state
    MeshSave::toAnySupportedFormat(source, "icp_source_before.stl");
    MeshSave::toAnySupportedFormat(target, "icp_target.stl");
    
    // Configure ICP
    ICPProperties icpParams;
    icpParams.method = ICPMethod::PointToPlane;
    icpParams.iterLimit = 100;
    icpParams.cosTreshold = 0.7f;
    icpParams.distTresholdSq = 1.0f;
    
    std::cout << "Running ICP alignment...\n";
    
    // Create ICP object
    MeshOrPoints sourceOrPoints(source);
    MeshOrPoints targetOrPoints(target);
    
    ICP icp(sourceOrPoints, targetOrPoints, AffineXf3f(), AffineXf3f());
    icp.setParams(icpParams);
    
    // Run ICP
    AffineXf3f resultXf = icp.calculateTransformation();
    
    std::cout << "ICP completed.\n";
    std::cout << "  Final translation: (" 
              << resultXf.b.x << ", " 
              << resultXf.b.y << ", " 
              << resultXf.b.z << ")\n";
    
    // Get ICP quality metrics
    float rms = icp.getMeanSqDistToPoint();
    std::cout << "  RMS error: " << std::sqrt(rms) << "\n\n";
    
    // Apply result transformation
    source.transform(resultXf);
    
    // Save aligned result
    MeshSave::toAnySupportedFormat(source, "icp_source_after.stl");
    
    std::cout << "Saved results:\n";
    std::cout << "  icp_source_before.stl - source before alignment\n";
    std::cout << "  icp_target.stl - target mesh\n";
    std::cout << "  icp_source_after.stl - source after ICP alignment\n\n";
    
    std::cout << "Done!\n";
    return 0;
}
