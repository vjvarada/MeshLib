import sys
sys.path.insert(0, 'build/bin')
import mrmeshpy as mr

mesh = mr.makeUVSphere(1.0, 8, 8)
print(f"Mesh: {mesh.topology.numValidFaces()} faces")

f0 = mr.FaceId(0)
edge = mesh.topology.edgeWithLeft(f0)
print(f"Edge ID: {int(edge)}, Has valid(): {hasattr(edge, 'valid')}")

# Try to get vertices
v0 = mesh.topology.org(edge)
v1 = mesh.topology.dest(edge)
v2_edge = mesh.topology.next(edge)
v2 = mesh.topology.dest(v2_edge)

p0 = mesh.points[int(v0)]
p1 = mesh.points[int(v1)]  
p2 = mesh.points[int(v2)]

print(f"Triangle: ({p0.x:.3f},{p0.y:.3f},{p0.z:.3f}) ({p1.x:.3f},{p1.y:.3f},{p1.z:.3f}) ({p2.x:.3f},{p2.y:.3f},{p2.z:.3f})")
