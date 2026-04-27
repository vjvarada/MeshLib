"""
MeshLib Standalone Demo Application
A simple PyQt6 app demonstrating STL import and mesh offset operations.
Uses VBO-based GPU rendering for performance.
"""

import sys
import os
import struct
import numpy as np
import ctypes
from pathlib import Path

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFileDialog, QSlider, QGroupBox, QStatusBar,
    QDoubleSpinBox, QMessageBox, QSplitter, QProgressDialog
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QAction

from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.arrays import vbo

# Add the build directory to path for mrmeshpy and mrvoxelspy
BUILD_BIN_PATH = os.path.join(os.path.dirname(__file__), '..', 'build', 'bin')
sys.path.insert(0, os.path.abspath(BUILD_BIN_PATH))

try:
    import mrmeshpy
    import mrvoxelspy
    MESHLIB_AVAILABLE = True
    print("MeshLib modules loaded successfully!")
except ImportError as e:
    MESHLIB_AVAILABLE = False
    print(f"Warning: MeshLib modules not available: {e}")


class MeshData:
    """Simple mesh data container with VBO support"""
    def __init__(self, vertices=None, faces=None, normals=None, name="Mesh"):
        self.vertices = vertices if vertices is not None else np.array([], dtype=np.float32)
        self.faces = faces if faces is not None else np.array([], dtype=np.int32)
        self.normals = normals if normals is not None else np.array([], dtype=np.float32)
        self.name = name
        self.mr_mesh = None  # MeshLib mesh object
        
        # VBO handles (created when added to scene)
        self.vbo_vertices = None
        self.vbo_normals = None
        self.vertex_count = 0
        
    def create_vbos(self):
        """Create VBOs for GPU rendering"""
        if len(self.vertices) == 0:
            return
            
        # Ensure data is contiguous float32
        verts = np.ascontiguousarray(self.vertices, dtype=np.float32)
        self.vertex_count = len(verts) // 3
        
        # Create vertex VBO
        self.vbo_vertices = vbo.VBO(verts)
        
        # Create normal VBO if normals exist
        if len(self.normals) > 0:
            norms = np.ascontiguousarray(self.normals, dtype=np.float32)
            self.vbo_normals = vbo.VBO(norms)
        else:
            # Generate flat normals if not provided
            self.generate_normals()
            
    def generate_normals(self):
        """Generate flat normals for the mesh"""
        if len(self.vertices) == 0:
            return
            
        verts = self.vertices.reshape(-1, 3)
        normals = []
        
        for i in range(0, len(verts), 3):
            if i + 2 < len(verts):
                v0, v1, v2 = verts[i], verts[i+1], verts[i+2]
                e1 = v1 - v0
                e2 = v2 - v0
                n = np.cross(e1, e2)
                norm = np.linalg.norm(n)
                if norm > 0:
                    n = n / norm
                else:
                    n = np.array([0, 0, 1], dtype=np.float32)
                normals.extend([n, n, n])
        
        if normals:
            self.normals = np.array(normals, dtype=np.float32).flatten()
            self.vbo_normals = vbo.VBO(np.ascontiguousarray(self.normals, dtype=np.float32))
            
    def delete_vbos(self):
        """Clean up VBOs"""
        if self.vbo_vertices:
            self.vbo_vertices.delete()
            self.vbo_vertices = None
        if self.vbo_normals:
            self.vbo_normals.delete()
            self.vbo_normals = None
            
    @property
    def num_vertices(self):
        return len(self.vertices) // 3
    
    @property
    def num_faces(self):
        return self.num_vertices // 3


def load_stl_binary(filepath):
    """Load a binary STL file and return MeshData"""
    with open(filepath, 'rb') as f:
        # Skip 80-byte header
        f.read(80)
        # Read number of triangles
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        # Pre-allocate arrays for speed
        vertices = np.zeros(num_triangles * 9, dtype=np.float32)
        normals = np.zeros(num_triangles * 9, dtype=np.float32)
        
        for i in range(num_triangles):
            # Read normal (3 floats)
            nx, ny, nz = struct.unpack('<3f', f.read(12))
            # Read 3 vertices (9 floats)
            v1 = struct.unpack('<3f', f.read(12))
            v2 = struct.unpack('<3f', f.read(12))
            v3 = struct.unpack('<3f', f.read(12))
            # Skip attribute byte count
            f.read(2)
            
            base = i * 9
            vertices[base:base+3] = v1
            vertices[base+3:base+6] = v2
            vertices[base+6:base+9] = v3
            normals[base:base+3] = [nx, ny, nz]
            normals[base+3:base+6] = [nx, ny, nz]
            normals[base+6:base+9] = [nx, ny, nz]
        
        mesh = MeshData(
            vertices=vertices,
            normals=normals,
            name=os.path.basename(filepath)
        )
        return mesh


def load_stl_ascii(filepath):
    """Load an ASCII STL file and return MeshData"""
    vertices = []
    normals = []
    current_normal = [0, 0, 0]
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip().lower()
            if line.startswith('facet normal'):
                parts = line.split()
                current_normal = [float(parts[2]), float(parts[3]), float(parts[4])]
            elif line.startswith('vertex'):
                parts = line.split()
                vertices.extend([float(parts[1]), float(parts[2]), float(parts[3])])
                normals.extend(current_normal)
    
    mesh = MeshData(
        vertices=np.array(vertices, dtype=np.float32),
        normals=np.array(normals, dtype=np.float32),
        name=os.path.basename(filepath)
    )
    return mesh


def load_stl(filepath):
    """Load STL file (auto-detect binary vs ASCII)"""
    with open(filepath, 'rb') as f:
        header = f.read(80)
        try:
            header_str = header.decode('ascii').strip()
            if header_str.startswith('solid'):
                f.seek(0)
                content = f.read(1000).decode('ascii', errors='ignore')
                if 'facet' in content.lower():
                    return load_stl_ascii(filepath)
        except:
            pass
    return load_stl_binary(filepath)


class GLWidget(QOpenGLWidget):
    """OpenGL widget for 3D mesh rendering using VBOs (GPU-accelerated)"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.meshes = []  # List of MeshData objects
        self.rotation_x = 30
        self.rotation_y = 45
        self.zoom = 5.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.last_pos = None
        self.wireframe = False
        
        # Grid VBO and scaling (Z-up coordinate system)
        self.grid_vbo = None
        self.grid_vertex_count = 0
        self.grid_size = 10.0  # Dynamic grid size
        self.grid_divisions = 10  # Number of grid lines
        
        # Axes VBO
        self.axes_vbo = None
        self.axes_size = 2.0  # Dynamic axes size
        
    def initializeGL(self):
        glClearColor(0.1, 0.1, 0.15, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHT1)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Better lighting setup
        glLight(GL_LIGHT0, GL_POSITION, [5.0, 10.0, 5.0, 0.0])
        glLight(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLight(GL_LIGHT0, GL_DIFFUSE, [0.7, 0.7, 0.7, 1.0])
        glLight(GL_LIGHT0, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])
        
        glLight(GL_LIGHT1, GL_POSITION, [-5.0, -5.0, 10.0, 0.0])
        glLight(GL_LIGHT1, GL_DIFFUSE, [0.3, 0.3, 0.4, 1.0])
        
        # Material properties
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.3, 0.3, 0.3, 1.0])
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 30.0)
        
        # Enable smooth shading
        glShadeModel(GL_SMOOTH)
        
        # Create static VBOs for grid and axes
        self.create_grid_vbo()
        self.create_axes_vbo()
        
    def create_grid_vbo(self):
        """Create VBO for the ground grid (XY plane, Z-up coordinate system)"""
        grid_lines = []
        half_size = self.grid_size / 2
        step = self.grid_size / self.grid_divisions
        
        # Create grid on XY plane (Z=0) for Z-up coordinate system
        for i in range(self.grid_divisions + 1):
            pos = -half_size + i * step
            # Lines parallel to Y axis
            grid_lines.extend([pos, -half_size, 0, pos, half_size, 0])
            # Lines parallel to X axis
            grid_lines.extend([-half_size, pos, 0, half_size, pos, 0])
        
        if self.grid_vbo:
            self.grid_vbo.delete()
        
        grid_data = np.array(grid_lines, dtype=np.float32)
        self.grid_vbo = vbo.VBO(grid_data)
        self.grid_vertex_count = len(grid_lines) // 3
        
    def create_axes_vbo(self):
        """Create VBO for coordinate axes (Z-up coordinate system)"""
        # X, Y, Z axes with colors interleaved: x,y,z,r,g,b
        # Z-up: X=red, Y=green, Z=blue (pointing up)
        if self.axes_vbo:
            self.axes_vbo.delete()
            
        axes_data = np.array([
            # X axis - red (right)
            0, 0, 0, 1, 0, 0,
            self.axes_size, 0, 0, 1, 0, 0,
            # Y axis - green (forward)
            0, 0, 0, 0, 1, 0,
            0, self.axes_size, 0, 0, 1, 0,
            # Z axis - blue (up)
            0, 0, 0, 0, 0, 1,
            0, 0, self.axes_size, 0, 0, 1,
        ], dtype=np.float32)
        self.axes_vbo = vbo.VBO(axes_data)
        
    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / h if h > 0 else 1
        gluPerspective(45, aspect, 0.1, 1000.0)
        glMatrixMode(GL_MODELVIEW)
        
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Camera transform for Z-up coordinate system
        glTranslatef(self.pan_x, self.pan_y, -self.zoom)
        # Rotate around X axis (tilt)
        glRotatef(self.rotation_x, 1, 0, 0)
        # Rotate around Z axis (spin) - Z is up
        glRotatef(self.rotation_y, 0, 0, 1)
        
        # Draw grid (VBO)
        self.draw_grid_vbo()
        
        # Draw axes (VBO)
        self.draw_axes_vbo()
        
        # Draw meshes (VBO)
        for i, mesh in enumerate(self.meshes):
            self.draw_mesh_vbo(mesh, i)
            
    def draw_grid_vbo(self):
        """Draw grid using VBO"""
        if self.grid_vbo is None:
            return
            
        glDisable(GL_LIGHTING)
        glColor3f(0.2, 0.2, 0.3)
        
        self.grid_vbo.bind()
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(3, GL_FLOAT, 0, self.grid_vbo)
        glDrawArrays(GL_LINES, 0, self.grid_vertex_count)
        glDisableClientState(GL_VERTEX_ARRAY)
        self.grid_vbo.unbind()
        
        glEnable(GL_LIGHTING)
        
    def draw_axes_vbo(self):
        """Draw coordinate axes using VBO"""
        if self.axes_vbo is None:
            return
            
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        
        self.axes_vbo.bind()
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        
        stride = 6 * 4  # 6 floats * 4 bytes
        glVertexPointer(3, GL_FLOAT, stride, self.axes_vbo)
        glColorPointer(3, GL_FLOAT, stride, self.axes_vbo + 12)  # offset by 3 floats
        
        glDrawArrays(GL_LINES, 0, 6)
        
        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        self.axes_vbo.unbind()
        
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)
        
    def draw_mesh_vbo(self, mesh, index=0):
        """Draw mesh using VBOs (GPU-accelerated)"""
        if mesh.vbo_vertices is None or mesh.vertex_count == 0:
            return
            
        # Different colors for different meshes
        colors = [
            (0.3, 0.8, 0.95),  # Cyan
            (0.95, 0.4, 0.6),  # Pink
            (0.4, 0.95, 0.4),  # Green
            (0.95, 0.8, 0.3),  # Yellow
        ]
        color = colors[index % len(colors)]
        glColor3f(*color)
        
        if self.wireframe:
            glDisable(GL_LIGHTING)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        else:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        
        # Bind and draw vertices
        mesh.vbo_vertices.bind()
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(3, GL_FLOAT, 0, mesh.vbo_vertices)
        
        # Bind normals if available
        if mesh.vbo_normals is not None:
            mesh.vbo_normals.bind()
            glEnableClientState(GL_NORMAL_ARRAY)
            glNormalPointer(GL_FLOAT, 0, mesh.vbo_normals)
        
        # Draw all triangles in one call!
        glDrawArrays(GL_TRIANGLES, 0, mesh.vertex_count)
        
        # Cleanup
        glDisableClientState(GL_VERTEX_ARRAY)
        if mesh.vbo_normals is not None:
            glDisableClientState(GL_NORMAL_ARRAY)
            mesh.vbo_normals.unbind()
        mesh.vbo_vertices.unbind()
        
        if self.wireframe:
            glEnable(GL_LIGHTING)
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            
    def add_mesh(self, mesh):
        """Add mesh to scene and create its VBOs"""
        mesh.create_vbos()
        self.meshes.append(mesh)
        self.update_scene_scale()
        self.auto_zoom()
        self.update()
        
    def clear_meshes(self):
        """Clear all meshes and delete their VBOs"""
        for mesh in self.meshes:
            mesh.delete_vbos()
        self.meshes = []
        self.update()
        
    def update_scene_scale(self):
        """Update grid and axes size based on mesh bounding box"""
        if not self.meshes:
            self.grid_size = 10.0
            self.axes_size = 2.0
        else:
            # Calculate bounding box of all meshes
            all_verts = []
            for mesh in self.meshes:
                if len(mesh.vertices) > 0:
                    all_verts.extend(mesh.vertices)
            
            if all_verts:
                verts = np.array(all_verts).reshape(-1, 3)
                bbox_min = np.min(verts, axis=0)
                bbox_max = np.max(verts, axis=0)
                bbox_size = bbox_max - bbox_min
                max_dim = np.max(bbox_size)
                
                # Scale grid to be 2x the largest dimension
                self.grid_size = max(max_dim * 2, 1.0)
                # Scale axes to be 1/4 of grid size
                self.axes_size = self.grid_size * 0.25
        
        # Recreate grid and axes with new sizes
        self.create_grid_vbo()
        self.create_axes_vbo()
    
    def auto_zoom(self):
        """Auto-adjust zoom to fit all meshes"""
        if not self.meshes:
            self.zoom = 5.0
            return
        all_verts = []
        for mesh in self.meshes:
            if len(mesh.vertices) > 0:
                all_verts.extend(mesh.vertices)
        if all_verts:
            verts = np.array(all_verts).reshape(-1, 3)
            bbox_min = np.min(verts, axis=0)
            bbox_max = np.max(verts, axis=0)
            bbox_center = (bbox_min + bbox_max) / 2
            bbox_size = bbox_max - bbox_min
            max_dim = np.max(bbox_size)
            
            # Set zoom to view entire mesh with some margin
            self.zoom = max(max_dim * 2.5, 2.0)
            
            # Center the view on the mesh (optional)
            # self.pan_x = -bbox_center[0] * 0.1
            # self.pan_y = -bbox_center[1] * 0.1
            
    def mousePressEvent(self, event):
        self.last_pos = event.position()
        
    def mouseMoveEvent(self, event):
        if self.last_pos is None:
            return
        dx = event.position().x() - self.last_pos.x()
        dy = event.position().y() - self.last_pos.y()
        
        if event.buttons() & Qt.MouseButton.LeftButton:
            self.rotation_y += dx * 0.5
            self.rotation_x += dy * 0.5
        elif event.buttons() & Qt.MouseButton.RightButton:
            self.pan_x += dx * 0.01
            self.pan_y -= dy * 0.01
            
        self.last_pos = event.position()
        self.update()
        
    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self.zoom *= 0.9 if delta > 0 else 1.1
        self.zoom = max(0.5, min(100, self.zoom))
        self.update()


class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MeshLib Demo - STL Import & Offset (GPU Accelerated)")
        self.setMinimumSize(1200, 800)
        
        self.current_mesh = None
        self.original_mesh = None
        
        self.setup_ui()
        self.setup_menu()
        self.update_status()
        
    def setup_ui(self):
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        
        # Main layout with splitter
        layout = QHBoxLayout(central)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(splitter)
        
        # Left sidebar
        sidebar = QWidget()
        sidebar.setMaximumWidth(300)
        sidebar.setMinimumWidth(250)
        sidebar_layout = QVBoxLayout(sidebar)
        
        # File operations
        file_group = QGroupBox("File Operations")
        file_layout = QVBoxLayout(file_group)
        
        import_btn = QPushButton("Import STL")
        import_btn.clicked.connect(self.import_stl)
        file_layout.addWidget(import_btn)
        
        export_btn = QPushButton("Export STL")
        export_btn.clicked.connect(self.export_stl)
        file_layout.addWidget(export_btn)
        
        clear_btn = QPushButton("Clear Scene")
        clear_btn.clicked.connect(self.clear_scene)
        file_layout.addWidget(clear_btn)
        
        sidebar_layout.addWidget(file_group)
        
        # Primitives
        prim_group = QGroupBox("Create Primitives")
        prim_layout = QVBoxLayout(prim_group)
        
        sphere_btn = QPushButton("Create Sphere")
        sphere_btn.clicked.connect(self.create_sphere)
        prim_layout.addWidget(sphere_btn)
        
        cube_btn = QPushButton("Create Cube")
        cube_btn.clicked.connect(self.create_cube)
        prim_layout.addWidget(cube_btn)
        
        torus_btn = QPushButton("Create Torus")
        torus_btn.clicked.connect(self.create_torus)
        prim_layout.addWidget(torus_btn)
        
        sidebar_layout.addWidget(prim_group)
        
        # Offset operation
        offset_group = QGroupBox("Mesh Offset")
        offset_layout = QVBoxLayout(offset_group)
        
        # Offset distance
        dist_layout = QHBoxLayout()
        dist_layout.addWidget(QLabel("Distance:"))
        self.offset_spin = QDoubleSpinBox()
        self.offset_spin.setRange(-10.0, 10.0)
        self.offset_spin.setValue(0.1)
        self.offset_spin.setSingleStep(0.01)
        self.offset_spin.setDecimals(3)
        dist_layout.addWidget(self.offset_spin)
        offset_layout.addLayout(dist_layout)
        
        # Voxel size (auto)
        voxel_layout = QHBoxLayout()
        voxel_layout.addWidget(QLabel("Voxel Size:"))
        self.voxel_spin = QDoubleSpinBox()
        self.voxel_spin.setRange(0.001, 1.0)
        self.voxel_spin.setValue(0.01)
        self.voxel_spin.setSingleStep(0.001)
        self.voxel_spin.setDecimals(4)
        voxel_layout.addWidget(self.voxel_spin)
        offset_layout.addLayout(voxel_layout)
        
        auto_voxel_btn = QPushButton("Auto Voxel Size")
        auto_voxel_btn.clicked.connect(self.auto_voxel_size)
        offset_layout.addWidget(auto_voxel_btn)
        
        offset_btn = QPushButton("Apply Offset")
        offset_btn.setStyleSheet("background-color: #4cc9f0; font-weight: bold;")
        offset_btn.clicked.connect(self.apply_offset)
        offset_layout.addWidget(offset_btn)
        
        sidebar_layout.addWidget(offset_group)
        
        # Mesh info
        info_group = QGroupBox("Mesh Info")
        info_layout = QVBoxLayout(info_group)
        
        self.info_name = QLabel("Name: -")
        self.info_verts = QLabel("Vertices: -")
        self.info_faces = QLabel("Faces: -")
        self.info_area = QLabel("Area: -")
        self.info_volume = QLabel("Volume: -")
        
        info_layout.addWidget(self.info_name)
        info_layout.addWidget(self.info_verts)
        info_layout.addWidget(self.info_faces)
        info_layout.addWidget(self.info_area)
        info_layout.addWidget(self.info_volume)
        
        sidebar_layout.addWidget(info_group)
        
        # View options
        view_group = QGroupBox("View")
        view_layout = QVBoxLayout(view_group)
        
        wireframe_btn = QPushButton("Toggle Wireframe")
        wireframe_btn.clicked.connect(self.toggle_wireframe)
        view_layout.addWidget(wireframe_btn)
        
        reset_btn = QPushButton("Reset View")
        reset_btn.clicked.connect(self.reset_view)
        view_layout.addWidget(reset_btn)
        
        sidebar_layout.addWidget(view_group)
        
        sidebar_layout.addStretch()
        
        # OpenGL viewport
        self.gl_widget = GLWidget()
        
        splitter.addWidget(sidebar)
        splitter.addWidget(self.gl_widget)
        splitter.setSizes([250, 950])
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
    def setup_menu(self):
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        
        import_action = QAction("Import STL...", self)
        import_action.setShortcut("Ctrl+O")
        import_action.triggered.connect(self.import_stl)
        file_menu.addAction(import_action)
        
        export_action = QAction("Export STL...", self)
        export_action.setShortcut("Ctrl+S")
        export_action.triggered.connect(self.export_stl)
        file_menu.addAction(export_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Help menu
        help_menu = menubar.addMenu("Help")
        
        about_action = QAction("About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
        
    def import_stl(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Import STL File", "", "STL Files (*.stl);;All Files (*)"
        )
        if filepath:
            try:
                self.status_bar.showMessage("Loading STL...")
                QApplication.processEvents()
                
                if MESHLIB_AVAILABLE:
                    # Load using MeshLib for full functionality
                    mr_mesh = mrmeshpy.loadStl(filepath)
                    mesh = self.mrmesh_to_render_data(mr_mesh, Path(filepath).stem)
                    mesh.mr_mesh = mr_mesh
                    
                    # Auto-calculate recommended voxel size based on mesh size
                    bbox = mr_mesh.getBoundingBox()
                    diagonal = (bbox.max - bbox.min).length()
                    recommended_voxel = diagonal * 5e-3  # 0.5% of diagonal as per MeshLib recommendation
                    self.voxel_spin.setValue(recommended_voxel)
                    self.status_bar.showMessage(f"Loaded: {mesh.name} ({mesh.num_faces:,} triangles) - Voxel size: {recommended_voxel:.4f}")
                else:
                    # Fallback to basic STL loader
                    mesh = load_stl(filepath)
                    self.status_bar.showMessage(f"Loaded: {mesh.name} ({mesh.num_faces:,} triangles)")
                
                self.current_mesh = mesh
                self.original_mesh = mesh
                
                self.gl_widget.clear_meshes()
                self.gl_widget.add_mesh(mesh)
                self.update_mesh_info(mesh)
            except Exception as e:
                import traceback
                traceback.print_exc()
                QMessageBox.critical(self, "Error", f"Failed to load STL: {e}")
                
    def export_stl(self):
        if not self.current_mesh or len(self.current_mesh.vertices) == 0:
            QMessageBox.warning(self, "Warning", "No mesh to export!")
            return
            
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export STL File", "mesh.stl", "STL Files (*.stl)"
        )
        if filepath:
            try:
                self.save_stl_binary(filepath, self.current_mesh)
                self.status_bar.showMessage(f"Exported: {filepath}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to export STL: {e}")
                
    def save_stl_binary(self, filepath, mesh):
        """Save mesh as binary STL"""
        vertices = mesh.vertices.reshape(-1, 3)
        normals = mesh.normals.reshape(-1, 3) if len(mesh.normals) > 0 else None
        num_triangles = len(vertices) // 3
        
        with open(filepath, 'wb') as f:
            # 80-byte header
            header = b'Binary STL exported from MeshLib Demo'
            f.write(header + b'\x00' * (80 - len(header)))
            # Number of triangles
            f.write(struct.pack('<I', num_triangles))
            
            for i in range(0, len(vertices), 3):
                v1, v2, v3 = vertices[i], vertices[i+1], vertices[i+2]
                
                # Use stored normal or calculate
                if normals is not None and i < len(normals):
                    normal = normals[i]
                else:
                    edge1 = v2 - v1
                    edge2 = v3 - v1
                    normal = np.cross(edge1, edge2)
                    norm = np.linalg.norm(normal)
                    if norm > 0:
                        normal = normal / norm
                    else:
                        normal = np.array([0, 0, 1])
                
                # Write normal and vertices
                f.write(struct.pack('<3f', *normal))
                f.write(struct.pack('<3f', *v1))
                f.write(struct.pack('<3f', *v2))
                f.write(struct.pack('<3f', *v3))
                f.write(struct.pack('<H', 0))
                
    def clear_scene(self):
        self.gl_widget.clear_meshes()
        self.current_mesh = None
        self.original_mesh = None
        self.update_mesh_info(None)
        self.status_bar.showMessage("Scene cleared")
        
    def create_sphere(self):
        if not MESHLIB_AVAILABLE:
            QMessageBox.warning(self, "Warning", "MeshLib not available!")
            return
            
        try:
            mr_mesh = mrmeshpy.makeUVSphere(1.0, 64, 64)
            print(f"Created sphere mesh: {mr_mesh.topology.numValidVerts()} vertices, {mr_mesh.topology.numValidFaces()} faces")
            mesh_data = self.mrmesh_to_render_data(mr_mesh, "Sphere")
            print(f"Converted to render data: {mesh_data.num_vertices} vertices, {mesh_data.num_faces} faces")
            mesh_data.mr_mesh = mr_mesh
            
            # Auto-set recommended voxel size
            bbox = mr_mesh.getBoundingBox()
            diagonal = (bbox.max - bbox.min).length()
            self.voxel_spin.setValue(diagonal * 5e-3)
            
            self.current_mesh = mesh_data
            self.original_mesh = mesh_data
            self.gl_widget.clear_meshes()
            self.gl_widget.add_mesh(mesh_data)
            self.update_mesh_info(mesh_data)
            self.status_bar.showMessage(f"Created sphere ({mesh_data.num_faces:,} triangles)")
        except Exception as e:
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "Error", f"Failed to create sphere: {e}")
            
    def create_cube(self):
        if not MESHLIB_AVAILABLE:
            QMessageBox.warning(self, "Warning", "MeshLib not available!")
            return
            
        try:
            size = mrmeshpy.Vector3f(1.0, 1.0, 1.0)
            base = mrmeshpy.Vector3f(-0.5, -0.5, -0.5)
            mr_mesh = mrmeshpy.makeCube(size, base)
            mesh_data = self.mrmesh_to_render_data(mr_mesh, "Cube")
            mesh_data.mr_mesh = mr_mesh
            
            # Auto-set recommended voxel size
            bbox = mr_mesh.getBoundingBox()
            diagonal = (bbox.max - bbox.min).length()
            self.voxel_spin.setValue(diagonal * 5e-3)
            
            self.current_mesh = mesh_data
            self.original_mesh = mesh_data
            self.gl_widget.clear_meshes()
            self.gl_widget.add_mesh(mesh_data)
            self.update_mesh_info(mesh_data)
            self.status_bar.showMessage(f"Created cube ({mesh_data.num_faces:,} triangles)")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to create cube: {e}")
            
    def create_torus(self):
        if not MESHLIB_AVAILABLE:
            QMessageBox.warning(self, "Warning", "MeshLib not available!")
            return
            
        try:
            mr_mesh = mrmeshpy.makeTorus(1.0, 0.3, 64, 32)
            mesh_data = self.mrmesh_to_render_data(mr_mesh, "Torus")
            mesh_data.mr_mesh = mr_mesh
            
            # Auto-set recommended voxel size
            bbox = mr_mesh.getBoundingBox()
            diagonal = (bbox.max - bbox.min).length()
            self.voxel_spin.setValue(diagonal * 5e-3)
            
            self.current_mesh = mesh_data
            self.original_mesh = mesh_data
            self.gl_widget.clear_meshes()
            self.gl_widget.add_mesh(mesh_data)
            self.update_mesh_info(mesh_data)
            self.status_bar.showMessage(f"Created torus ({mesh_data.num_faces:,} triangles)")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to create torus: {e}")
            
    def mrmesh_to_render_data(self, mr_mesh, name="Mesh"):
        """Convert MeshLib mesh to renderable MeshData"""
        try:
            num_verts = mr_mesh.topology.numValidVerts()
            num_faces = mr_mesh.topology.numValidFaces()
            
            print(f"Converting mesh: {num_verts} verts, {num_faces} faces")
            
            topology = mr_mesh.topology
            
            # Pre-allocate arrays
            triangles = np.zeros(num_faces * 9, dtype=np.float32)
            normals = np.zeros(num_faces * 9, dtype=np.float32)
            
            tri_idx = 0
            # Iterate through all possible face IDs
            for fid in range(num_faces * 2):  # Generous upper bound
                try:
                    face_id = mrmeshpy.FaceId(fid)
                    if not topology.hasFace(face_id):
                        continue
                        
                    edge = topology.edgeWithLeft(face_id)
                    
                    if not edge.valid():
                        continue
                    
                    v0_id = topology.org(edge)
                    v1_id = topology.dest(edge)
                    next_edge = topology.next(edge)
                    v2_id = topology.dest(next_edge)
                    
                    # Get vertex positions
                    p0 = mr_mesh.points[int(v0_id)]
                    p1 = mr_mesh.points[int(v1_id)]
                    p2 = mr_mesh.points[int(v2_id)]
                    
                    base = tri_idx * 9
                    triangles[base:base+3] = [p0.x, p0.y, p0.z]
                    triangles[base+3:base+6] = [p1.x, p1.y, p1.z]
                    triangles[base+6:base+9] = [p2.x, p2.y, p2.z]
                    
                    # Calculate normal
                    e1 = np.array([p1.x - p0.x, p1.y - p0.y, p1.z - p0.z])
                    e2 = np.array([p2.x - p0.x, p2.y - p0.y, p2.z - p0.z])
                    n = np.cross(e1, e2)
                    norm = np.linalg.norm(n)
                    if norm > 0:
                        n = n / norm
                    else:
                        n = np.array([0, 0, 1])
                    
                    normals[base:base+3] = n
                    normals[base+3:base+6] = n
                    normals[base+6:base+9] = n
                    
                    tri_idx += 1
                    
                    # Stop once we've found all valid faces
                    if tri_idx >= num_faces:
                        break
                        
                except Exception as ex:
                    # Skip invalid faces
                    continue
            
            # Trim arrays to actual size
            actual_size = tri_idx * 9
            triangles = triangles[:actual_size]
            normals = normals[:actual_size]
            
            print(f"Conversion complete: {tri_idx} triangles extracted, {actual_size} float values")
            
            mesh_data = MeshData(
                vertices=triangles,
                normals=normals,
                name=name
            )
            mesh_data.mr_mesh = mr_mesh
            return mesh_data
        except Exception as e:
            import traceback
            traceback.print_exc()
            raise Exception(f"Failed to convert MRMesh: {e}")
        
    def auto_voxel_size(self):
        if not MESHLIB_AVAILABLE:
            QMessageBox.warning(self, "Warning", "MeshLib not available!")
            return
            
        if not self.current_mesh or not self.current_mesh.mr_mesh:
            QMessageBox.warning(self, "Warning", "No MeshLib mesh available! Create a primitive first.")
            return
            
        try:
            voxel_size = mrvoxelspy.suggestVoxelSize(self.current_mesh.mr_mesh, 1000000)
            self.voxel_spin.setValue(voxel_size)
            self.status_bar.showMessage(f"Suggested voxel size: {voxel_size:.4f}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to calculate voxel size: {e}")
            
    def apply_offset(self):
        if not MESHLIB_AVAILABLE:
            QMessageBox.warning(self, "Warning", "MeshLib not available!")
            return
            
        if not self.current_mesh or not self.current_mesh.mr_mesh:
            QMessageBox.warning(self, "Warning", "No MeshLib mesh available! Create a primitive first.")
            return
            
        distance = self.offset_spin.value()
        voxel_size = self.voxel_spin.value()
        
        try:
            mesh = self.current_mesh.mr_mesh
            
            # Show progress
            num_faces = mesh.topology.numValidFaces()
            self.status_bar.showMessage(f"Applying offset to {num_faces:,} faces... (voxel size: {voxel_size:.4f})")
            QApplication.processEvents()
            
            params = mrvoxelspy.OffsetParameters()
            params.voxelSize = voxel_size
            
            # Check for holes and set sign detection mode (recommended by MeshLib)
            boundaries = mrmeshpy.findRightBoundary(mesh.topology)
            if len(boundaries) > 0:  # Mesh has boundaries/holes
                params.signDetectionMode = mrvoxelspy.SignDetectionMode.HoleWindingRule
            
            result = mrvoxelspy.offsetMesh(mesh, distance, params)
            
            result_data = self.mrmesh_to_render_data(result, f"Offset ({distance:.3f})")
            result_data.mr_mesh = result
            
            self.current_mesh = result_data
            self.gl_widget.clear_meshes()
            self.gl_widget.add_mesh(result_data)
            self.update_mesh_info(result_data)
            
            self.status_bar.showMessage(
                f"Offset applied: {distance:.3f} ({result_data.num_faces:,} triangles)"
            )
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to apply offset: {e}")
            self.status_bar.showMessage("Offset failed")
            
    def update_mesh_info(self, mesh):
        if mesh is None:
            self.info_name.setText("Name: -")
            self.info_verts.setText("Vertices: -")
            self.info_faces.setText("Faces: -")
            self.info_area.setText("Area: -")
            self.info_volume.setText("Volume: -")
            return
            
        self.info_name.setText(f"Name: {mesh.name}")
        
        if mesh.mr_mesh:
            verts = mesh.mr_mesh.topology.numValidVerts()
            faces = mesh.mr_mesh.topology.numValidFaces()
            area = mesh.mr_mesh.area()
            volume = mesh.mr_mesh.volume()
            
            self.info_verts.setText(f"Vertices: {verts:,}")
            self.info_faces.setText(f"Faces: {faces:,}")
            self.info_area.setText(f"Area: {area:.4f}")
            self.info_volume.setText(f"Volume: {volume:.4f}")
        else:
            verts = mesh.num_vertices
            faces = mesh.num_faces
            self.info_verts.setText(f"Vertices: {verts:,}")
            self.info_faces.setText(f"Faces: {faces:,}")
            self.info_area.setText("Area: N/A")
            self.info_volume.setText("Volume: N/A")
            
    def toggle_wireframe(self):
        self.gl_widget.wireframe = not self.gl_widget.wireframe
        self.gl_widget.update()
        mode = "Wireframe" if self.gl_widget.wireframe else "Solid"
        self.status_bar.showMessage(f"View mode: {mode}")
        
    def reset_view(self):
        self.gl_widget.rotation_x = 30
        self.gl_widget.rotation_y = 45
        self.gl_widget.pan_x = 0
        self.gl_widget.pan_y = 0
        self.gl_widget.update_scene_scale()
        self.gl_widget.auto_zoom()
        self.gl_widget.update()
        self.status_bar.showMessage("View reset")
        
    def update_status(self):
        status = "MeshLib: " + ("Available" if MESHLIB_AVAILABLE else "Not Available")
        status += " | GPU Rendering: VBO"
        self.status_bar.showMessage(status)
        
    def show_about(self):
        QMessageBox.about(
            self,
            "About MeshLib Demo",
            "MeshLib Standalone Demo Application\n\n"
            "Features:\n"
            "- Import/Export STL files\n"
            "- Create primitives (Sphere, Cube, Torus)\n"
            "- Mesh offset operations\n"
            "- GPU-accelerated VBO rendering\n\n"
            f"MeshLib Status: {'Available' if MESHLIB_AVAILABLE else 'Not Available'}\n\n"
            "Controls:\n"
            "- Left drag: Rotate\n"
            "- Right drag: Pan\n"
            "- Scroll: Zoom"
        )


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # Dark theme
    from PyQt6.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(30, 30, 40))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 35))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(35, 35, 45))
    palette.setColor(QPalette.ColorRole.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.ColorRole.Button, QColor(45, 45, 55))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(220, 220, 220))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(76, 201, 240))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
