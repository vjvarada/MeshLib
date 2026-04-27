# MeshLib Standalone - Python Test Script
# This demonstrates the Python bindings for industrial-grade mesh processing

import sys
import os

# Add the dist directory to the path
dist_path = os.path.join(os.path.dirname(__file__), 'dist', 'meshlib')
sys.path.insert(0, dist_path)
os.environ['PATH'] = dist_path + ';' + os.environ['PATH']

import mrmeshpy as mr

print('=' * 60)
print('MeshLib Standalone - Python Bindings Test')
print('=' * 60)

# 1. Create primitives
print('\n1. Creating primitive meshes...')
cube = mr.makeCube()
print(f'   Cube: {cube.topology.numValidFaces()} faces, {cube.topology.numValidVerts()} vertices')
print(f'   Volume: {cube.volume():.4f}, Area: {cube.area():.4f}')

torus = mr.makeTorus(1.0, 0.3, 32, 16)
print(f'   Torus: {torus.topology.numValidFaces()} faces, {torus.topology.numValidVerts()} vertices')

sphere = mr.makeUVSphere(1.0, 16, 16)
print(f'   Sphere: {sphere.topology.numValidFaces()} faces, {sphere.topology.numValidVerts()} vertices')

# 2. Vector operations
print('\n2. Vector operations...')
v1 = mr.Vector3f(1.0, 0.0, 0.0)
v2 = mr.Vector3f(0.0, 1.0, 0.0)
cross = mr.cross(v1, v2)
dot = mr.dot(v1, v2)
print(f'   Cross product of (1,0,0) x (0,1,0) = {cross}')
print(f'   Dot product: {dot}')

# 3. Boolean operations
print('\n3. Boolean operations...')
cube1 = mr.makeCube(mr.Vector3f(1.0, 1.0, 1.0), mr.Vector3f(0.0, 0.0, 0.0))
cube2 = mr.makeCube(mr.Vector3f(1.0, 1.0, 1.0), mr.Vector3f(0.5, 0.5, 0.5))

union = mr.boolean(cube1, cube2, mr.BooleanOperation.Union)
intersection = mr.boolean(cube1, cube2, mr.BooleanOperation.Intersection)
difference = mr.boolean(cube1, cube2, mr.BooleanOperation.DifferenceAB)

print(f'   Union: {union.topology.numValidFaces()} faces')
print(f'   Intersection: {intersection.topology.numValidFaces()} faces')
print(f'   Difference: {difference.topology.numValidFaces()} faces')

# 4. Mesh decimation
print('\n4. Mesh decimation...')
high_res = mr.makeTorus(1.0, 0.3, 64, 32)
original_faces = high_res.topology.numValidFaces()
print(f'   Original torus: {original_faces} faces')

settings = mr.DecimateSettings()
settings.maxError = 0.05
result = mr.decimateMesh(high_res, settings)
print(f'   After decimation: {high_res.topology.numValidFaces()} faces')
print(f'   Removed {result.facesDeleted} faces, {result.vertsDeleted} vertices')

# 5. Convex hull
print('\n5. Convex hull...')
hull = mr.makeConvexHull(sphere)
print(f'   Sphere convex hull: {hull.topology.numValidFaces()} faces')

print('\n' + '=' * 60)
print('All tests completed successfully!')
print('=' * 60)
