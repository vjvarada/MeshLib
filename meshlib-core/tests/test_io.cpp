/**
 * @file test_io.cpp
 * @brief Unit tests for mesh I/O functions
 */

#include <gtest/gtest.h>
#include "meshlib/io/MeshIO.h"
#include "meshlib/mesh/Primitives.h"
#include <fstream>
#include <filesystem>
#include <cstdio>

using namespace meshlib;
namespace fs = std::filesystem;

class MeshIOTest : public ::testing::Test {
protected:
    Mesh testMesh;
    std::string tempDir;
    
    void SetUp() override {
        testMesh = createBox({1.0f, 1.0f, 1.0f});
        tempDir = fs::temp_directory_path().string();
    }
    
    void TearDown() override {
        // Clean up temp files
        auto cleanup = [](const std::string& path) {
            if (fs::exists(path)) {
                fs::remove(path);
            }
        };
        
        cleanup(tempDir + "/test.obj");
        cleanup(tempDir + "/test.stl");
        cleanup(tempDir + "/test_ascii.stl");
        cleanup(tempDir + "/test.ply");
        cleanup(tempDir + "/test2.obj");
    }
};

TEST_F(MeshIOTest, DetectFormatOBJ) {
    EXPECT_EQ(detectFormat("model.obj"), MeshFormat::OBJ);
    EXPECT_EQ(detectFormat("path/to/model.OBJ"), MeshFormat::OBJ);
}

TEST_F(MeshIOTest, DetectFormatSTL) {
    EXPECT_EQ(detectFormat("model.stl"), MeshFormat::STL);
    EXPECT_EQ(detectFormat("model.STL"), MeshFormat::STL);
}

TEST_F(MeshIOTest, DetectFormatPLY) {
    EXPECT_EQ(detectFormat("model.ply"), MeshFormat::PLY);
}

TEST_F(MeshIOTest, DetectFormatAuto) {
    // Unknown extensions should return Auto
    EXPECT_EQ(detectFormat("model.xyz"), MeshFormat::Auto);
    EXPECT_EQ(detectFormat("no_extension"), MeshFormat::Auto);
}

TEST_F(MeshIOTest, SaveLoadOBJ) {
    std::string path = tempDir + "/test.obj";
    
    // Save - saveMesh returns Result
    Result saveResult = saveMesh(testMesh, path);
    EXPECT_TRUE(saveResult.ok());
    EXPECT_TRUE(fs::exists(path));
    
    // Load - loadMesh returns LoadResult
    LoadResult loadResult = loadMesh(path);
    EXPECT_TRUE(loadResult.ok());
    EXPECT_FALSE(loadResult.mesh.empty());
    
    // Verify
    EXPECT_EQ(loadResult.mesh.vertexCount(), testMesh.vertexCount());
    EXPECT_EQ(loadResult.mesh.triangleCount(), testMesh.triangleCount());
}

TEST_F(MeshIOTest, SaveLoadSTLBinary) {
    std::string path = tempDir + "/test.stl";
    
    SaveOptions options;
    options.binary = true;  // Use binary = true for binary STL
    options.format = MeshFormat::STL_BINARY;
    
    // Save
    Result saveResult = saveMesh(testMesh, path, options);
    EXPECT_TRUE(saveResult.ok());
    EXPECT_TRUE(fs::exists(path));
    
    // Load
    LoadResult loadResult = loadMesh(path);
    EXPECT_TRUE(loadResult.ok());
    EXPECT_FALSE(loadResult.mesh.empty());
    
    // Note: STL doesn't share vertices, so vertex count may differ
    EXPECT_EQ(loadResult.mesh.triangleCount(), testMesh.triangleCount());
}

TEST_F(MeshIOTest, SaveLoadSTLAscii) {
    std::string path = tempDir + "/test_ascii.stl";
    
    SaveOptions options;
    options.binary = false;  // ASCII STL
    options.format = MeshFormat::STL_ASCII;
    
    // Save
    Result saveResult = saveMesh(testMesh, path, options);
    EXPECT_TRUE(saveResult.ok());
    
    // Verify it's ASCII (should start with "solid")
    std::ifstream file(path);
    std::string firstLine;
    std::getline(file, firstLine);
    EXPECT_TRUE(firstLine.find("solid") == 0);
    file.close();
    
    // Load
    LoadResult loadResult = loadMesh(path);
    EXPECT_TRUE(loadResult.ok());
    EXPECT_EQ(loadResult.mesh.triangleCount(), testMesh.triangleCount());
}

TEST_F(MeshIOTest, SaveLoadPLY) {
    std::string path = tempDir + "/test.ply";
    
    // Save
    Result saveResult = saveMesh(testMesh, path);
    EXPECT_TRUE(saveResult.ok());
    EXPECT_TRUE(fs::exists(path));
    
    // Load
    LoadResult loadResult = loadMesh(path);
    EXPECT_TRUE(loadResult.ok());
    EXPECT_FALSE(loadResult.mesh.empty());
    
    EXPECT_EQ(loadResult.mesh.vertexCount(), testMesh.vertexCount());
    EXPECT_EQ(loadResult.mesh.triangleCount(), testMesh.triangleCount());
}

TEST_F(MeshIOTest, LoadNonExistentFile) {
    LoadResult result = loadMesh("nonexistent_file.obj");
    EXPECT_FALSE(result.ok());
    EXPECT_TRUE(result.mesh.empty());
}

TEST_F(MeshIOTest, SaveToInvalidPath) {
    Result result = saveMesh(testMesh, "/invalid/path/that/does/not/exist/mesh.obj");
    EXPECT_FALSE(result.ok());
}

TEST_F(MeshIOTest, LoadWithOptions) {
    std::string path = tempDir + "/test.obj";
    saveMesh(testMesh, path);
    
    LoadOptions options;
    // Default options should work
    LoadResult result = loadMesh(path, options);
    EXPECT_TRUE(result.ok());
}

TEST_F(MeshIOTest, MeshDataIntegrity) {
    std::string path = tempDir + "/test.obj";
    
    // Create a simple mesh with known values
    std::vector<Vector3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f}
    };
    std::vector<std::array<int, 3>> triangles = {{0, 1, 2}};
    Mesh simpleMesh = Mesh::fromTriangles(vertices, triangles);
    
    // Save
    saveMesh(simpleMesh, path);
    
    // Load
    LoadResult result = loadMesh(path);
    EXPECT_TRUE(result.ok());
    
    // Verify vertex positions
    const auto& loadedVerts = result.mesh.vertices();
    EXPECT_EQ(loadedVerts.size(), 3);
    
    // Check first vertex
    EXPECT_NEAR(loadedVerts[0].x, 0.0f, 1e-5f);
    EXPECT_NEAR(loadedVerts[0].y, 0.0f, 1e-5f);
    EXPECT_NEAR(loadedVerts[0].z, 0.0f, 1e-5f);
}

TEST_F(MeshIOTest, RoundTrip) {
    // Test that save->load->save produces consistent results
    std::string path1 = tempDir + "/test.obj";
    std::string path2 = tempDir + "/test2.obj";
    
    saveMesh(testMesh, path1);
    LoadResult result1 = loadMesh(path1);
    saveMesh(result1.mesh, path2);
    LoadResult result2 = loadMesh(path2);
    
    EXPECT_EQ(result1.mesh.vertexCount(), result2.mesh.vertexCount());
    EXPECT_EQ(result1.mesh.triangleCount(), result2.mesh.triangleCount());
    
    // Bounding boxes should match
    auto bbox1 = result1.mesh.boundingBox();
    auto bbox2 = result2.mesh.boundingBox();
    EXPECT_NEAR(bbox1.diagonal(), bbox2.diagonal(), 1e-5f);
}

TEST_F(MeshIOTest, LoadResultFilename) {
    std::string path = tempDir + "/test.obj";
    saveMesh(testMesh, path);
    
    LoadResult result = loadMesh(path);
    EXPECT_TRUE(result.ok());
    // LoadResult has filename member
    EXPECT_FALSE(result.filename.empty());
}

TEST_F(MeshIOTest, SaveOptionsFormat) {
    std::string path = tempDir + "/test.stl";
    
    SaveOptions options;
    options.format = MeshFormat::STL_BINARY;
    options.saveNormals = true;
    
    Result result = saveMesh(testMesh, path, options);
    EXPECT_TRUE(result.ok());
}

TEST_F(MeshIOTest, LoadOptionsFormat) {
    std::string path = tempDir + "/test.obj";
    saveMesh(testMesh, path);
    
    LoadOptions options;
    options.format = MeshFormat::OBJ;
    options.loadNormals = true;
    options.loadUVs = true;
    options.mergeVertices = true;
    
    LoadResult result = loadMesh(path, options);
    EXPECT_TRUE(result.ok());
}
