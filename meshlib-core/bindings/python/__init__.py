"""
MeshLib Core - 3D Mesh Processing Library

A standalone, cross-platform library for 3D mesh processing with support for:
- Boolean operations (Union, Intersection, Difference)
- Mesh simplification (decimation)
- Mesh subdivision
- Mesh smoothing
- Hole filling
- ICP alignment
- File I/O (STL, OBJ, PLY)

Example:
    >>> import meshlib_core as ml
    >>> sphere = ml.create_sphere(1.0, 32, 16)
    >>> print(f"Vertices: {sphere.vertex_count()}, Triangles: {sphere.triangle_count()}")
"""

from .meshlib_core import *

__all__ = [
    # Version
    '__version__',
    'VERSION_MAJOR',
    'VERSION_MINOR',
    'VERSION_PATCH',
    
    # Core types
    'Vector2f',
    'Vector3f',
    'Box3f',
    'Matrix3f',
    'AffineTransform3f',
    'ErrorCode',
    'Status',
    
    # Mesh and PointCloud
    'Mesh',
    'PointCloud',
    
    # Primitives
    'create_sphere',
    'create_icosphere',
    'create_box',
    'create_cylinder',
    'create_cone',
    'create_torus',
    'create_plane',
    'create_disc',
    
    # I/O
    'MeshFormat',
    'detect_format',
    'load_mesh',
    'save_mesh',
    
    # Decimation
    'DecimateParams',
    'DecimateResult',
    'decimate',
    'decimate_to_count',
    'decimate_by_ratio',
    'generate_lods',
    
    # Smoothing
    'SmoothingMethod',
    'SmoothParams',
    'smooth',
    'smooth_laplacian',
    'smooth_taubin',
    'relax',
    
    # Subdivision
    'SubdivisionScheme',
    'SubdivideParams',
    'subdivide',
    'subdivide_loop',
    'subdivide_midpoint',
    'subdivide_adaptive',
    
    # Hole filling
    'FillHoleMethod',
    'HoleInfo',
    'FillHoleParams',
    'FillHoleResult',
    'find_holes',
    'fill_holes',
    
    # ICP
    'ICPMethod',
    'ICPSampling',
    'ICPParams',
    'ICPResult',
    'icp',
    'align_mesh',
    'align_point_cloud',
    
    # Boolean
    'BooleanOperation',
    'BooleanParams',
    'BooleanResult',
    'boolean',
    'boolean_union',
    'boolean_intersection',
    'boolean_difference',
]
