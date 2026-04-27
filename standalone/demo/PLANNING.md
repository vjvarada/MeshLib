# MeshLib Standalone Demo Application - Planning Document

##  Project Overview

This document outlines the design and implementation plan for a Python + Qt demo application that demonstrates all major features of the MeshLib Standalone implementation.

### Goals
- Provide a visual, interactive demonstration of MeshLib capabilities
- Allow users to import, manipulate, and process 3D meshes
- Support multi-model operations (boolean, collision detection)
- Serve as a reference implementation for MeshLib Python bindings

---

##  Feature Requirements

### Core Requirements (Must Have)
1. **Model Import/Export**
   - Load mesh files (STL, OBJ, PLY) - *Note: May need custom STL parser if I/O not in bindings*
   - Export processed meshes to file

2. **Model Selection**
   - Click to select models in 3D viewport
   - Multi-select support for operations requiring 2+ models
   - Visual highlighting of selected models

3. **Model Transformation**
   - Move models (translate X/Y/Z)
   - Rotate models
   - Scale models
   - Reset to original position

4. **Single-Model Operations**
   - Decimation (mesh simplification)
   - Convex Hull generation
   - Mesh statistics (vertices, faces, area, volume)
   - Transform application

5. **Multi-Model Operations**
   - Boolean Union (A  B)
   - Boolean Intersection (A  B)
   - Boolean Difference (A - B, B - A)
   - Collision Detection

6. **Voxel-Based Operations**
   - Offset Mesh (shell/thicken)
   - Double Offset (smooth)
   - General Offset (advanced)
   - Thicken Mesh

### Nice-to-Have Features
- Undo/Redo support
- Mesh wireframe toggle
- Multiple viewports
- Measurement tools
- Mesh repair suggestions

---

##  Architecture

### Technology Stack
| Component | Technology | Justification |
|-----------|------------|---------------|
| Language | Python 3.10+ | Direct MeshLib bindings access |
| GUI Framework | PyQt6 / PySide6 | Modern Qt bindings, OpenGL support |
| 3D Rendering | PyOpenGL + Qt OpenGL | Native integration, hardware acceleration |
| Mesh Processing | mrmeshpy, mrvoxelspy | Our standalone MeshLib bindings |

### Application Structure

```
demo/
 PLANNING.md              # This document
 requirements.txt         # Python dependencies
 main.py                  # Application entry point
 app/
    __init__.py
    main_window.py       # Main application window
    viewport.py          # 3D OpenGL viewport widget
    scene.py             # Scene management (models, selection)
    model.py             # Model wrapper class
    operations.py        # MeshLib operation wrappers
 ui/
    sidebar.py           # Operations sidebar
    toolbar.py           # Main toolbar
    dialogs/
       import_dialog.py
       export_dialog.py
       decimate_dialog.py
       offset_dialog.py
       boolean_dialog.py
    widgets/
        model_list.py    # Model selection list
        transform_panel.py
        stats_panel.py
 rendering/
    __init__.py
    mesh_renderer.py     # OpenGL mesh rendering
    grid_renderer.py     # Ground grid
    axes_renderer.py     # Coordinate axes
    shaders/
        mesh.vert
        mesh.frag
        wireframe.vert
        wireframe.frag
 utils/
     __init__.py
     stl_loader.py        # Custom STL parser
     obj_loader.py        # Custom OBJ parser
     math_utils.py        # Matrix/vector helpers
```

---

##  UI Design

### Main Window Layout

```

  File   Edit   View   Operations   Help                      [][][]

 [Import] [Export] [] [Sphere] [Cube] [Torus] [] [Select] [Move]   

                                                                   
   MODEL                                              OPERATIONS   
   LIST                3D VIEWPORT                                 
                                                        
   Sphere             [OpenGL Canvas]                Transform  
   Cube                                                
   Torus                                               X: 0.0   
                                                        Y: 0.0   
                                                        Z: 0.0   
                                                        
                                                                   
                                                        
  [Add ]                                               Stats    
  [Remove]                                              
                                                      Verts:1k   
                                                      Faces:2k   

  Ready  Models: 3  Selected: Cube  Vertices: 8  Faces: 12       

```

### Color Scheme
- Background: Dark (#1a1a2e)
- Accent: Cyan (#4cc9f0)
- Secondary: Purple (#7209b7)
- Selection Highlight: Yellow (#ffd60a)
- Grid: Subtle blue (#0f3460)

---

##  Available MeshLib Features

### mrmeshpy Module

| Feature | Function/Class | Description |
|---------|----------------|-------------|
| **Primitives** | | |
| Sphere | `makeUVSphere(radius, hRes, vRes)` | UV-mapped sphere |
| Cube | `makeCube(size, base)` | Box primitive |
| Torus | `makeTorus(primaryR, secondaryR, ...)` | Torus/donut shape |
| Convex Hull | `makeConvexHull(mesh)` | Compute convex hull |
| **Boolean Ops** | | |
| Union | `boolean(a, b, BooleanOperation.Union)` | Combine meshes |
| Intersection | `boolean(a, b, BooleanOperation.Intersection)` | Common volume |
| Difference A-B | `boolean(a, b, BooleanOperation.DifferenceAB)` | Subtract B from A |
| Difference B-A | `boolean(a, b, BooleanOperation.DifferenceBA)` | Subtract A from B |
| **Processing** | | |
| Decimation | `decimateMesh(mesh, settings)` | Reduce polygon count |
| Collision | `findCollidingTriangles(a, b)` | Find intersections |
| **Transforms** | | |
| Apply Transform | `mesh.transform(AffineXf3f)` | Apply matrix transform |
| Translation | `AffineXf3f.translation` | Position offset |
| Linear (Rotation/Scale) | `AffineXf3f.linear` | 3x3 matrix |
| **Mesh Info** | | |
| Bounding Box | `mesh.getBoundingBox()` | AABB bounds |
| Surface Area | `mesh.area()` | Total surface area |
| Volume | `mesh.volume()` | Enclosed volume |
| Vertex Count | `mesh.topology.numValidVerts()` | Number of vertices |
| Face Count | `mesh.topology.numValidFaces()` | Number of triangles |
| **Data Types** | | |
| 3D Vector | `Vector3f(x, y, z)` | Position/direction |
| 2D Vector | `Vector2f(x, y)` | UV coordinates |
| Bounding Box | `Box3f` | Axis-aligned box |
| Transform | `AffineXf3f` | 4x4 affine matrix |

### mrvoxelspy Module

| Feature | Function/Class | Description |
|---------|----------------|-------------|
| **Offset Operations** | | |
| Simple Offset | `offsetMesh(mesh, distance, params)` | Shell/thicken |
| Double Offset | `doubleOffsetMesh(mesh, d1, d2, params)` | Smooth offset |
| General Offset | `generalOffsetMesh(mesh, distance, params)` | Advanced control |
| Thicken | `thickenMesh(mesh, thickness, params)` | Wall thickness |
| **Volume Operations** | | |
| Mesh to Volume | `meshToVolume(mesh, params)` | Voxelize mesh |
| **Utilities** | | |
| Suggest Voxel Size | `suggestVoxelSize(mesh, targetVoxels)` | Auto voxel size |
| **Parameters** | | |
| OffsetParameters | `.voxelSize`, `.signDetectionMode` | Basic params |
| GeneralOffsetParameters | Above + `.minNewVertDev`, etc. | Advanced params |
| SignDetectionMode | `OpenVDB`, `WindingRule`, `Unsigned`, etc. | Detection algorithm |

---

##  Implementation Plan

### Phase 1: Foundation (Day 1)
- [ ] Set up project structure and dependencies
- [ ] Create main window with basic layout
- [ ] Implement basic OpenGL viewport
- [ ] Add camera controls (orbit, pan, zoom)
- [ ] Render coordinate axes and grid

### Phase 2: Model Management (Day 1-2)
- [ ] Create Model wrapper class
- [ ] Implement Scene manager
- [ ] Add STL file loader (custom parser)
- [ ] Add OBJ file loader (custom parser)
- [ ] Implement model list widget
- [ ] Add primitive creation (sphere, cube, torus)

### Phase 3: Selection & Transform (Day 2)
- [ ] Implement ray-casting for model selection
- [ ] Add visual selection highlighting
- [ ] Create transform panel (translate/rotate/scale)
- [ ] Implement model movement with sliders
- [ ] Add keyboard shortcuts

### Phase 4: Single-Model Operations (Day 2-3)
- [ ] Implement decimation dialog
- [ ] Add convex hull operation
- [ ] Show mesh statistics panel
- [ ] Add transform reset button

### Phase 5: Multi-Model Operations (Day 3)
- [ ] Implement model multi-selection
- [ ] Create boolean operations dialog
- [ ] Add collision detection visualization
- [ ] Handle operation results (new mesh creation)

### Phase 6: Voxel Operations (Day 3-4)
- [ ] Create offset mesh dialog
- [ ] Implement double offset
- [ ] Add thicken mesh operation
- [ ] Add voxel size auto-suggestion

### Phase 7: Polish & Testing (Day 4)
- [ ] Add export functionality
- [ ] Improve error handling
- [ ] Add status bar updates
- [ ] Write usage documentation
- [ ] Test all operations thoroughly

---

##  User Interactions

### Viewport Controls
| Input | Action |
|-------|--------|
| Left Drag | Rotate camera |
| Right Drag | Pan camera |
| Scroll | Zoom in/out |
| Click | Select model |
| Ctrl+Click | Add to selection |
| Delete | Remove selected model |

### Keyboard Shortcuts
| Key | Action |
|-----|--------|
| `Ctrl+O` | Open/Import file |
| `Ctrl+S` | Save/Export file |
| `Ctrl+A` | Select all |
| `Escape` | Clear selection |
| `Delete` | Delete selected |
| `W` | Toggle wireframe |
| `G` | Toggle grid |
| `R` | Reset camera |
| `1-4` | Create primitives |

---

##  Dependencies

```txt
# requirements.txt
PyQt6>=6.4.0
PyOpenGL>=3.1.6
numpy>=1.24.0
```

**Note:** mrmeshpy and mrvoxelspy are loaded from the build directory.

---

##  Technical Challenges

### 1. Mesh File I/O
**Problem:** The standalone bindings may not include full I/O functions.
**Solution:** Implement custom STL/OBJ parsers and convert to MeshLib format.

### 2. OpenGL Rendering
**Problem:** Need to extract vertex/face data from MeshLib meshes.
**Solution:** Use `mesh.points` and `mesh.topology` to build OpenGL buffers.

### 3. Ray Casting for Selection
**Problem:** Need to intersect mouse ray with meshes.
**Solution:** Use bounding box first, then triangle intersection.

### 4. Memory Management
**Problem:** Large meshes can consume significant memory.
**Solution:** Implement mesh LOD for viewport, full mesh for operations.

---

##  Testing Strategy

### Unit Tests
- Mesh creation and manipulation
- File loading/saving
- Operation parameter validation

### Integration Tests
- Boolean operations with various meshes
- Decimation at different target levels
- Offset operations with different voxel sizes

### Manual Tests
- Import real-world STL files
- Perform boolean on complex geometries
- Stress test with high-poly models

---

##  References

- [PyQt6 Documentation](https://www.riverbankcomputing.com/static/Docs/PyQt6/)
- [OpenGL in Python](https://pyopengl.sourceforge.net/)
- [MeshLib Documentation](https://meshlib.io/docs)
- [STL File Format](https://en.wikipedia.org/wiki/STL_(file_format))
- [OBJ File Format](https://en.wikipedia.org/wiki/Wavefront_.obj_file)

---

##  Success Criteria

The demo application will be considered complete when:

1.  Can import STL files and display them in 3D viewport
2.  Can create primitive shapes (sphere, cube, torus)
3.  Can select individual models by clicking
4.  Can select multiple models for multi-model operations
5.  Can move/transform selected models
6.  Can perform all 4 boolean operations
7.  Can decimate meshes with configurable settings
8.  Can perform voxel-based offset operations
9.  Can export results to STL file
10.  UI is responsive and provides feedback on operations

---

##  Timeline

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| Phase 1 | 4 hours | Basic window + viewport |
| Phase 2 | 4 hours | Model loading + primitives |
| Phase 3 | 3 hours | Selection + transforms |
| Phase 4 | 3 hours | Single-model operations |
| Phase 5 | 3 hours | Boolean operations |
| Phase 6 | 3 hours | Voxel operations |
| Phase 7 | 4 hours | Polish + documentation |
| **Total** | **~24 hours** | **Complete demo app** |

---

*Document created: January 14, 2026*
*Author: MeshLib Standalone Demo Team*
