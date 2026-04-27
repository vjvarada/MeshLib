"""
MeshLib Python Bindings - Standalone Version

This package provides Python bindings for MeshLib mesh processing library.
It includes both mesh operations (mrmeshpy) and voxel operations (mrvoxelspy).

Features:
- Mesh creation (cube, sphere, torus, cylinder)
- Mesh I/O (STL, OBJ, PLY, etc.)
- Boolean operations (union, intersection, difference)
- Mesh decimation and subdivision
- Mesh offset/shell operations (via OpenVDB)
- Hole filling and mesh repair
- Mesh relaxation/smoothing
- ICP alignment
- And more!

Example:
    >>> import meshlib
    >>> mesh = meshlib.mrmeshpy.makeCube()
    >>> print(f"Vertices: {mesh.topology.numValidVerts()}")
"""

__version__ = "1.0.0"
__author__ = "MeshLib Team"

# Import the main modules
try:
    from . import mrmeshpy
    from .mrmeshpy import *
except ImportError as e:
    import warnings
    warnings.warn(f"Could not import mrmeshpy: {e}")

try:
    from . import mrvoxelspy
    from .mrvoxelspy import *
except ImportError:
    # MRVoxels may not be available in all builds
    pass

# Convenience functions
def make_cube(size=1.0):
    """Create a unit cube mesh centered at origin."""
    from .mrmeshpy import makeCube, Vector3f
    return makeCube(Vector3f(-size/2, -size/2, -size/2), Vector3f(size, size, size))

def make_sphere(radius=1.0, detail=3):
    """Create a sphere mesh."""
    from .mrmeshpy import makeUVSphere
    return makeUVSphere(radius, detail * 16, detail * 8)

def make_torus(primary_radius=1.0, secondary_radius=0.25):
    """Create a torus mesh."""
    from .mrmeshpy import makeTorus
    return makeTorus(primary_radius, secondary_radius)

def load_mesh(filepath):
    """Load a mesh from file."""
    from .mrmeshpy import loadMesh
    return loadMesh(filepath)

def save_mesh(mesh, filepath):
    """Save a mesh to file."""
    from .mrmeshpy import saveMesh
    return saveMesh(mesh, filepath)

__all__ = [
    "mrmeshpy",
    "make_cube",
    "make_sphere", 
    "make_torus",
    "load_mesh",
    "save_mesh",
]
