import sys
sys.path.insert(0, 'build/bin')
sys.path.insert(0, 'demo')
from main import MainWindow, mrmeshpy
import numpy as np

# Create app instance (minimal, won't show GUI)
class FakeApp:
    def mrmesh_to_render_data(self, mr_mesh, name="Mesh"):
        num_verts = mr_mesh.topology.numValidVerts()
        num_faces = mr_mesh.topology.numValidFaces()
        print(f"Converting: {num_verts} verts, {num_faces} faces")
        
        topology = mr_mesh.topology
        triangles = np.zeros(num_faces * 9, dtype=np.float32)
        normals = np.zeros(num_faces * 9, dtype=np.float32)
        
        tri_idx = 0
        for fid in range(num_faces * 2):
            try:
                face_id = mrmeshpy.FaceId(fid)
                if not topology.hasFace(face_id):
                    continue
                    
                edge = topology.edgeWithLeft(face_id)
                if not edge.valid():
                    continue
                
                v0 = topology.org(edge)
                v1 = topology.dest(edge)
                v2_edge = topology.next(edge)
                v2 = topology.dest(v2_edge)
                
                p0 = mr_mesh.points[int(v0)]
                p1 = mr_mesh.points[int(v1)]
                p2 = mr_mesh.points[int(v2)]
                
                base = tri_idx * 9
                triangles[base:base+3] = [p0.x, p0.y, p0.z]
                triangles[base+3:base+6] = [p1.x, p1.y, p1.z]
                triangles[base+6:base+9] = [p2.x, p2.y, p2.z]
                
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
                
                if tri_idx >= num_faces:
                    break
            except Exception as ex:
                continue
        
        print(f"Extracted {tri_idx} triangles")
        return triangles[:tri_idx*9], normals[:tri_idx*9]

app = FakeApp()
mesh = mrmeshpy.makeUVSphere(1.0, 32, 32)
verts, norms = app.mrmesh_to_render_data(mesh, "test")
print(f"Result: {len(verts)} vertex floats, {len(verts)//9} triangles")
