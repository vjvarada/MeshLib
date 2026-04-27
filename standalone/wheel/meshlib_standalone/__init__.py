import os, sys
if sys.platform == 'win32':
    os.add_dll_directory(os.path.dirname(__file__))
from .mrmeshpy import *
__version__ = '0.1.0'
