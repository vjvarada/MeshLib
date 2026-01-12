/**
 * @file boolean_example.cpp
 * @brief Example: Boolean operations with MeshLib Core
 */

#include <meshlib/meshlib.h>
#include <iostream>

using namespace meshlib;

int main() {
    std::cout << "MeshLib Core Boolean Example\n";
    std::cout << "Version: " << MESHLIB_CORE_VERSION << "\n\n";
    
    // Create two spheres
    std::cout << "Creating two spheres...\n";
    Mesh sphere1 = makeUVSphere(1.0f, 32, 32);
    Mesh sphere2 = makeUVSphere(1.0f, 32, 32);
    
    // Translate second sphere
    AffineXf3f xf = AffineXf3f::translation(Vector3f(0.7f, 0.0f, 0.0f));
    sphere2.transform(xf);
    
    std::cout << "Sphere 1: " << sphere1.topology.numValidVerts() << " vertices, "
              << sphere1.topology.numValidFaces() << " faces\n";
    std::cout << "Sphere 2: " << sphere2.topology.numValidVerts() << " vertices, "
              << sphere2.topology.numValidFaces() << " faces\n\n";
    
    // Perform boolean operations
    std::cout << "Performing Boolean Union...\n";
    BooleanResult unionResult = boolean(sphere1, sphere2, BooleanOperation::Union);
    if (unionResult.valid()) {
        std::cout << "Union result: " << unionResult.mesh.topology.numValidVerts() << " vertices, "
                  << unionResult.mesh.topology.numValidFaces() << " faces\n";
        
        // Save result
        auto saveResult = MeshSave::toAnySupportedFormat(unionResult.mesh, "union_result.stl");
        if (saveResult) {
            std::cout << "Saved to union_result.stl\n";
        }
    } else {
        std::cerr << "Union failed: " << unionResult.errorString << "\n";
    }
    
    std::cout << "\nPerforming Boolean Intersection...\n";
    BooleanResult intersectResult = boolean(sphere1, sphere2, BooleanOperation::Intersection);
    if (intersectResult.valid()) {
        std::cout << "Intersection result: " << intersectResult.mesh.topology.numValidVerts() << " vertices, "
                  << intersectResult.mesh.topology.numValidFaces() << " faces\n";
        
        auto saveResult = MeshSave::toAnySupportedFormat(intersectResult.mesh, "intersection_result.stl");
        if (saveResult) {
            std::cout << "Saved to intersection_result.stl\n";
        }
    } else {
        std::cerr << "Intersection failed: " << intersectResult.errorString << "\n";
    }
    
    std::cout << "\nPerforming Boolean Difference (A - B)...\n";
    BooleanResult diffResult = boolean(sphere1, sphere2, BooleanOperation::DifferenceAB);
    if (diffResult.valid()) {
        std::cout << "Difference result: " << diffResult.mesh.topology.numValidVerts() << " vertices, "
                  << diffResult.mesh.topology.numValidFaces() << " faces\n";
        
        auto saveResult = MeshSave::toAnySupportedFormat(diffResult.mesh, "difference_result.stl");
        if (saveResult) {
            std::cout << "Saved to difference_result.stl\n";
        }
    } else {
        std::cerr << "Difference failed: " << diffResult.errorString << "\n";
    }
    
    std::cout << "\nDone!\n";
    return 0;
}
