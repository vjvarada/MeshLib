import sys
sys.path.insert(0, r'c:\Users\VijayRaghavVarada\Documents\Github\MeshLib\standalone\build\bin')
import mrmeshpy
import mrvoxelspy

# Create a small sphere
mesh = mrmeshpy.makeUVSphere(1.0, 16, 16)
print(f"Created mesh: {mesh.topology.numValidFaces()} faces")

# Check boundaries
boundaries = mrmeshpy.findRightBoundary(mesh.topology)
print(f"Boundaries: {len(boundaries)}")

# Setup offset
bbox = mesh.getBoundingBox()
voxel = (bbox.max - bbox.min).length() * 5e-3
params = mrvoxelspy.OffsetParameters()
params.voxelSize = voxel

if len(boundaries) > 0:
    params.signDetectionMode = mrvoxelspy.SignDetectionMode.HoleWindingRule
    print("Using HoleWindingRule (mesh has boundaries)")
else:
    print("Using default sign detection (closed mesh)")

print(f"Voxel size: {voxel:.6f}")
print("Applying offset...")

result = mrvoxelspy.offsetMesh(mesh, 0.05, params)
print(f"SUCCESS! Result: {result.topology.numValidFaces()} faces")
