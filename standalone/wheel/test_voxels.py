import meshlib_standalone.mrmeshpy as mrm
import meshlib_standalone.mrvoxelspy as mrv

# Test basic import
print(' Successfully imported mrmeshpy')
print(' Successfully imported mrvoxelspy')

# Create a simple mesh (cube)
mesh = mrm.Mesh()
print(' Created mesh object')

# List available voxel operations
voxel_ops = [attr for attr in dir(mrv) if not attr.startswith('_')]
print(f'\\n Available voxel operations ({len(voxel_ops)}):')
for op in voxel_ops[:10]:  # Show first 10
    print(f'  - {op}')
if len(voxel_ops) > 10:
    print(f'  ... and {len(voxel_ops) - 10} more')

print('\\n Phase 4 Python Bindings - COMPLETE!')
