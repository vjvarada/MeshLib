/**
 * @file MeshIO.cpp
 * @brief Implementation of mesh file I/O functions
 */

#include "meshlib/io/MeshIO.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <cctype>

namespace meshlib {

namespace {

std::string toLowerCase(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), 
                   [](unsigned char c) { return std::tolower(c); });
    return result;
}

std::string getExtension(const std::string& filename) {
    size_t pos = filename.rfind('.');
    if (pos == std::string::npos) {
        return "";
    }
    return toLowerCase(filename.substr(pos));
}

// ==================== OBJ I/O ====================

LoadResult loadOBJ(std::istream& stream, const LoadOptions& options) {
    LoadResult result;
    
    std::vector<Vector3f> positions;
    std::vector<Vector3f> normals;
    std::vector<Vector2f> texCoords;
    std::vector<std::array<int, 3>> triangles;
    
    std::string line;
    while (std::getline(stream, line)) {
        // Remove comments
        size_t commentPos = line.find('#');
        if (commentPos != std::string::npos) {
            line = line.substr(0, commentPos);
        }
        
        // Skip empty lines
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "v") {
            // Vertex position
            float x, y, z;
            if (iss >> x >> y >> z) {
                positions.push_back(Vector3f(x, y, z));
            }
        } else if (prefix == "vn" && options.loadNormals) {
            // Vertex normal
            float x, y, z;
            if (iss >> x >> y >> z) {
                normals.push_back(Vector3f(x, y, z));
            }
        } else if (prefix == "vt" && options.loadUVs) {
            // Texture coordinate
            float u, v;
            if (iss >> u >> v) {
                texCoords.push_back(Vector2f(u, v));
            }
        } else if (prefix == "f") {
            // Face
            std::vector<int> faceVerts;
            std::string vertex;
            
            while (iss >> vertex) {
                // Parse vertex index (format: v, v/vt, v/vt/vn, v//vn)
                int vi = 0;
                
                size_t slashPos = vertex.find('/');
                if (slashPos != std::string::npos) {
                    vi = std::stoi(vertex.substr(0, slashPos));
                } else {
                    vi = std::stoi(vertex);
                }
                
                // OBJ indices are 1-based, negative means relative to end
                if (vi < 0) {
                    vi = static_cast<int>(positions.size()) + vi + 1;
                }
                
                faceVerts.push_back(vi - 1); // Convert to 0-based
            }
            
            // Triangulate face
            if (faceVerts.size() >= 3) {
                for (size_t i = 1; i + 1 < faceVerts.size(); ++i) {
                    triangles.push_back({faceVerts[0], faceVerts[i], faceVerts[i + 1]});
                }
            }
        }
    }
    
    if (positions.empty() || triangles.empty()) {
        result.status.code = ErrorCode::ParseError;
        result.status.message = "No valid geometry found in OBJ file";
        return result;
    }
    
    result.mesh = Mesh::fromTriangles(positions, triangles);
    result.status.code = ErrorCode::Success;
    
    return result;
}

Result saveOBJ(const Mesh& mesh, std::ostream& stream, const SaveOptions& options) {
    stream << "# MeshLib Core OBJ Export\n";
    stream << "# Vertices: " << mesh.vertexCount() << "\n";
    stream << "# Triangles: " << mesh.triangleCount() << "\n\n";
    
    stream << std::fixed;
    stream.precision(options.precision);
    
    // Write vertices
    for (const auto& v : mesh.vertices()) {
        stream << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    
    // Write normals
    if (options.saveNormals) {
        stream << "\n";
        const auto& normals = mesh.vertexNormals();
        for (const auto& n : normals) {
            stream << "vn " << n.x << " " << n.y << " " << n.z << "\n";
        }
    }
    
    // Write UVs
    if (options.saveUVs && mesh.hasUVs()) {
        stream << "\n";
        for (const auto& uv : mesh.uvCoords()) {
            stream << "vt " << uv.x << " " << uv.y << "\n";
        }
    }
    
    // Write faces
    stream << "\n";
    for (const auto& tri : mesh.triangles()) {
        // OBJ uses 1-based indices
        stream << "f " << (tri[0] + 1) << " " << (tri[1] + 1) << " " << (tri[2] + 1) << "\n";
    }
    
    return Result{ErrorCode::Success, ""};
}

// ==================== STL I/O ====================

LoadResult loadSTL(std::istream& stream, const LoadOptions& options, bool binary) {
    LoadResult result;
    
    if (binary) {
        // Binary STL
        char header[80];
        stream.read(header, 80);
        
        uint32_t numTriangles;
        stream.read(reinterpret_cast<char*>(&numTriangles), 4);
        
        std::vector<Triangle3f> triangleVerts;
        triangleVerts.reserve(numTriangles);
        
        for (uint32_t i = 0; i < numTriangles; ++i) {
            float normal[3], v1[3], v2[3], v3[3];
            uint16_t attrib;
            
            stream.read(reinterpret_cast<char*>(normal), 12);
            stream.read(reinterpret_cast<char*>(v1), 12);
            stream.read(reinterpret_cast<char*>(v2), 12);
            stream.read(reinterpret_cast<char*>(v3), 12);
            stream.read(reinterpret_cast<char*>(&attrib), 2);
            
            triangleVerts.push_back({
                Vector3f(v1[0], v1[1], v1[2]),
                Vector3f(v2[0], v2[1], v2[2]),
                Vector3f(v3[0], v3[1], v3[2])
            });
        }
        
        result.mesh = Mesh::fromTriangleVertices(triangleVerts);
    } else {
        // ASCII STL
        std::vector<Triangle3f> triangleVerts;
        
        std::string line;
        while (std::getline(stream, line)) {
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;
            
            if (keyword == "facet") {
                Triangle3f tri;
                int vertexIdx = 0;
                
                while (std::getline(stream, line) && vertexIdx < 3) {
                    std::istringstream vertIss(line);
                    std::string vKeyword;
                    vertIss >> vKeyword;
                    
                    if (vKeyword == "vertex") {
                        float x, y, z;
                        if (vertIss >> x >> y >> z) {
                            tri[vertexIdx++] = Vector3f(x, y, z);
                        }
                    } else if (vKeyword == "endfacet") {
                        break;
                    }
                }
                
                if (vertexIdx == 3) {
                    triangleVerts.push_back(tri);
                }
            }
        }
        
        result.mesh = Mesh::fromTriangleVertices(triangleVerts);
    }
    
    if (result.mesh.empty()) {
        result.status.code = ErrorCode::ParseError;
        result.status.message = "No valid geometry found in STL file";
    } else {
        result.status.code = ErrorCode::Success;
    }
    
    return result;
}

Result saveSTL(const Mesh& mesh, std::ostream& stream, const SaveOptions& options) {
    if (options.binary) {
        // Binary STL
        char header[80] = "MeshLib Core Binary STL";
        stream.write(header, 80);
        
        uint32_t numTriangles = static_cast<uint32_t>(mesh.triangleCount());
        stream.write(reinterpret_cast<const char*>(&numTriangles), 4);
        
        const auto& normals = mesh.faceNormals();
        
        for (size_t i = 0; i < mesh.triangleCount(); ++i) {
            auto tri = mesh.triangleVertices(static_cast<int>(i));
            Vector3f normal = normals[i];
            
            float n[3] = {normal.x, normal.y, normal.z};
            float v1[3] = {tri[0].x, tri[0].y, tri[0].z};
            float v2[3] = {tri[1].x, tri[1].y, tri[1].z};
            float v3[3] = {tri[2].x, tri[2].y, tri[2].z};
            uint16_t attrib = 0;
            
            stream.write(reinterpret_cast<const char*>(n), 12);
            stream.write(reinterpret_cast<const char*>(v1), 12);
            stream.write(reinterpret_cast<const char*>(v2), 12);
            stream.write(reinterpret_cast<const char*>(v3), 12);
            stream.write(reinterpret_cast<const char*>(&attrib), 2);
        }
    } else {
        // ASCII STL
        stream << "solid mesh\n";
        
        const auto& normals = mesh.faceNormals();
        stream << std::fixed;
        stream.precision(options.precision);
        
        for (size_t i = 0; i < mesh.triangleCount(); ++i) {
            auto tri = mesh.triangleVertices(static_cast<int>(i));
            const Vector3f& n = normals[i];
            
            stream << "  facet normal " << n.x << " " << n.y << " " << n.z << "\n";
            stream << "    outer loop\n";
            stream << "      vertex " << tri[0].x << " " << tri[0].y << " " << tri[0].z << "\n";
            stream << "      vertex " << tri[1].x << " " << tri[1].y << " " << tri[1].z << "\n";
            stream << "      vertex " << tri[2].x << " " << tri[2].y << " " << tri[2].z << "\n";
            stream << "    endloop\n";
            stream << "  endfacet\n";
        }
        
        stream << "endsolid mesh\n";
    }
    
    return Result{ErrorCode::Success, ""};
}

// ==================== PLY I/O ====================

LoadResult loadPLY(std::istream& stream, const LoadOptions& options) {
    LoadResult result;
    
    std::string line;
    std::getline(stream, line);
    
    if (line != "ply") {
        result.status.code = ErrorCode::ParseError;
        result.status.message = "Not a valid PLY file";
        return result;
    }
    
    bool binary = false;
    bool littleEndian = true;
    int numVertices = 0;
    int numFaces = 0;
    
    // Parse header
    while (std::getline(stream, line)) {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        
        if (keyword == "format") {
            std::string format;
            iss >> format;
            binary = (format != "ascii");
            littleEndian = (format == "binary_little_endian");
        } else if (keyword == "element") {
            std::string elemType;
            int count;
            iss >> elemType >> count;
            
            if (elemType == "vertex") {
                numVertices = count;
            } else if (elemType == "face") {
                numFaces = count;
            }
        } else if (keyword == "end_header") {
            break;
        }
    }
    
    // Read vertices
    std::vector<Vector3f> vertices;
    vertices.reserve(numVertices);
    
    if (binary) {
        for (int i = 0; i < numVertices; ++i) {
            float x, y, z;
            stream.read(reinterpret_cast<char*>(&x), 4);
            stream.read(reinterpret_cast<char*>(&y), 4);
            stream.read(reinterpret_cast<char*>(&z), 4);
            vertices.push_back(Vector3f(x, y, z));
        }
    } else {
        for (int i = 0; i < numVertices; ++i) {
            float x, y, z;
            stream >> x >> y >> z;
            vertices.push_back(Vector3f(x, y, z));
            std::getline(stream, line); // Skip rest of line
        }
    }
    
    // Read faces
    std::vector<std::array<int, 3>> triangles;
    triangles.reserve(numFaces);
    
    if (binary) {
        for (int i = 0; i < numFaces; ++i) {
            uint8_t count;
            stream.read(reinterpret_cast<char*>(&count), 1);
            
            std::vector<int> indices(count);
            for (int j = 0; j < count; ++j) {
                int32_t idx;
                stream.read(reinterpret_cast<char*>(&idx), 4);
                indices[j] = idx;
            }
            
            // Triangulate
            for (size_t j = 1; j + 1 < indices.size(); ++j) {
                triangles.push_back({indices[0], indices[j], indices[j + 1]});
            }
        }
    } else {
        for (int i = 0; i < numFaces; ++i) {
            int count;
            stream >> count;
            
            std::vector<int> indices(count);
            for (int j = 0; j < count; ++j) {
                stream >> indices[j];
            }
            
            // Triangulate
            for (size_t j = 1; j + 1 < indices.size(); ++j) {
                triangles.push_back({indices[0], indices[j], indices[j + 1]});
            }
        }
    }
    
    result.mesh = Mesh::fromTriangles(vertices, triangles);
    result.status.code = ErrorCode::Success;
    
    return result;
}

Result savePLY(const Mesh& mesh, std::ostream& stream, const SaveOptions& options) {
    stream << "ply\n";
    stream << "format ascii 1.0\n";
    stream << "element vertex " << mesh.vertexCount() << "\n";
    stream << "property float x\n";
    stream << "property float y\n";
    stream << "property float z\n";
    stream << "element face " << mesh.triangleCount() << "\n";
    stream << "property list uchar int vertex_indices\n";
    stream << "end_header\n";
    
    stream << std::fixed;
    stream.precision(options.precision);
    
    for (const auto& v : mesh.vertices()) {
        stream << v.x << " " << v.y << " " << v.z << "\n";
    }
    
    for (const auto& tri : mesh.triangles()) {
        stream << "3 " << tri[0] << " " << tri[1] << " " << tri[2] << "\n";
    }
    
    return Result{ErrorCode::Success, ""};
}

} // anonymous namespace

// ==================== Public API ====================

MeshFormat detectFormat(const std::string& filename) {
    std::string ext = getExtension(filename);
    
    if (ext == ".obj") return MeshFormat::OBJ;
    if (ext == ".stl") return MeshFormat::STL;
    if (ext == ".ply") return MeshFormat::PLY;
    if (ext == ".off") return MeshFormat::OFF;
    if (ext == ".gltf") return MeshFormat::GLTF;
    if (ext == ".glb") return MeshFormat::GLB;
    
    return MeshFormat::Auto;
}

std::string formatExtension(MeshFormat format) {
    switch (format) {
        case MeshFormat::OBJ: return ".obj";
        case MeshFormat::STL:
        case MeshFormat::STL_ASCII:
        case MeshFormat::STL_BINARY: return ".stl";
        case MeshFormat::PLY:
        case MeshFormat::PLY_ASCII:
        case MeshFormat::PLY_BINARY: return ".ply";
        case MeshFormat::OFF: return ".off";
        case MeshFormat::GLTF: return ".gltf";
        case MeshFormat::GLB: return ".glb";
        default: return "";
    }
}

std::vector<std::string> supportedLoadExtensions() {
    return {".obj", ".stl", ".ply"};
}

std::vector<std::string> supportedSaveExtensions() {
    return {".obj", ".stl", ".ply"};
}

LoadResult loadMesh(const std::string& filename, const LoadOptions& options) {
    LoadResult result;
    result.filename = filename;
    
    MeshFormat format = options.format;
    if (format == MeshFormat::Auto) {
        format = detectFormat(filename);
        if (format == MeshFormat::Auto) {
            result.status.code = ErrorCode::UnsupportedFormat;
            result.status.message = "Could not detect file format from extension";
            return result;
        }
    }
    
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        result.status.code = ErrorCode::FileNotFound;
        result.status.message = "Could not open file: " + filename;
        return result;
    }
    
    result = loadMeshFromStream(file, format, options);
    result.filename = filename;  // Preserve the filename after stream load
    return result;
}

LoadResult loadMeshFromMemory(const uint8_t* data, size_t size, MeshFormat format, const LoadOptions& options) {
    std::string str(reinterpret_cast<const char*>(data), size);
    std::istringstream stream(str);
    return loadMeshFromStream(stream, format, options);
}

LoadResult loadMeshFromStream(std::istream& stream, MeshFormat format, const LoadOptions& options) {
    switch (format) {
        case MeshFormat::OBJ:
            return loadOBJ(stream, options);
        
        case MeshFormat::STL:
        case MeshFormat::STL_BINARY: {
            // Try to detect if binary or ASCII
            char header[6];
            stream.read(header, 5);
            header[5] = '\0';
            stream.seekg(0);
            
            bool isBinary = (std::string(header) != "solid");
            return loadSTL(stream, options, isBinary);
        }
        
        case MeshFormat::STL_ASCII:
            return loadSTL(stream, options, false);
        
        case MeshFormat::PLY:
        case MeshFormat::PLY_ASCII:
        case MeshFormat::PLY_BINARY:
            return loadPLY(stream, options);
        
        default: {
            LoadResult result;
            result.status.code = ErrorCode::UnsupportedFormat;
            result.status.message = "Unsupported mesh format";
            return result;
        }
    }
}

Result saveMesh(const Mesh& mesh, const std::string& filename, const SaveOptions& options) {
    MeshFormat format = options.format;
    if (format == MeshFormat::Auto) {
        format = detectFormat(filename);
        if (format == MeshFormat::Auto) {
            return Result{ErrorCode::UnsupportedFormat, "Could not detect file format from extension"};
        }
    }
    
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        return Result{ErrorCode::OperationFailed, "Could not create file: " + filename};
    }
    
    return saveMeshToStream(mesh, file, format, options);
}

std::vector<uint8_t> saveMeshToMemory(const Mesh& mesh, MeshFormat format, const SaveOptions& options) {
    std::ostringstream stream;
    Result result = saveMeshToStream(mesh, stream, format, options);
    
    if (!result.ok()) {
        return {};
    }
    
    std::string str = stream.str();
    return std::vector<uint8_t>(str.begin(), str.end());
}

Result saveMeshToStream(const Mesh& mesh, std::ostream& stream, MeshFormat format, const SaveOptions& options) {
    switch (format) {
        case MeshFormat::OBJ:
            return saveOBJ(mesh, stream, options);
        
        case MeshFormat::STL:
        case MeshFormat::STL_BINARY:
            return saveSTL(mesh, stream, SaveOptions{options.format, options.saveNormals, options.saveUVs, options.saveColors, true, options.precision, options.progressCallback});
        
        case MeshFormat::STL_ASCII:
            return saveSTL(mesh, stream, SaveOptions{options.format, options.saveNormals, options.saveUVs, options.saveColors, false, options.precision, options.progressCallback});
        
        case MeshFormat::PLY:
        case MeshFormat::PLY_ASCII:
            return savePLY(mesh, stream, options);
        
        default:
            return Result{ErrorCode::UnsupportedFormat, "Unsupported mesh format for saving"};
    }
}

PointCloud loadPointCloud(const std::string& filename) {
    // For now, load as mesh and extract points
    auto result = loadMesh(filename);
    if (!result.ok()) {
        return PointCloud();
    }
    
    PointCloud cloud;
    cloud.points() = result.mesh.vertices();
    
    return cloud;
}

Result savePointCloud(const PointCloud& cloud, const std::string& filename) {
    // Save as PLY
    std::ofstream file(filename);
    if (!file) {
        return Result{ErrorCode::OperationFailed, "Could not create file"};
    }
    
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    
    if (cloud.hasNormals()) {
        file << "property float nx\n";
        file << "property float ny\n";
        file << "property float nz\n";
    }
    
    file << "end_header\n";
    
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud.points()[i];
        file << p.x << " " << p.y << " " << p.z;
        
        if (cloud.hasNormals()) {
            const auto& n = cloud.normals()[i];
            file << " " << n.x << " " << n.y << " " << n.z;
        }
        
        file << "\n";
    }
    
    return Result{ErrorCode::Success, ""};
}

} // namespace meshlib
