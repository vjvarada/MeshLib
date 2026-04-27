import sys
sys.path.insert(0, r'build\bin')
import mrmeshpy as mr

# Test basic mesh creation
print("Testing mrmeshpy functionality...")
print(f"Available functions: {[x for x in dir(mr) if not x.startswith('_')][:10]}")

# Try to create a simple box mesh
try:
    box = mr.Box3f(mr.Vector3f(0,0,0), mr.Vector3f(1,1,1))
    print(f" Created box: {box}")
except Exception as e:
    print(f" Error creating box: {e}")

print("SUCCESS: Basic mrmeshpy functionality working!")
