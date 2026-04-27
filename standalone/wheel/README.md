# MeshLib Standalone

Standalone distribution of MeshLib 3D mesh processing library.

## Features

- Core mesh operations
- Industrial I/O format support: PDF, STEP, E57, LAS, glTF, 3MF, OpenCTM
- Standard formats: STL, OBJ, PLY, OFF

## Installation

```bash
pip install meshlib-standalone-0.1.0-py3-none-win_amd64.whl
```

## Usage

```python
import meshlib_standalone as ml

# Create a box
box = ml.Box3f(ml.Vector3f(0,0,0), ml.Vector3f(1,1,1))
print(box)
```

## Size

~52 MB (includes OpenCASCADE for STEP support)

