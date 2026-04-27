import sys
sys.path.insert(0, r'build\bin')
import mrmeshpy
print(' mrmeshpy imported successfully')
print(f'Module: {mrmeshpy.__file__}')
print(f'Attributes: {len(dir(mrmeshpy))}')
