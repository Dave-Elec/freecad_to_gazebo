#!/usr/bin/env python3

import os

# check os types to search for freecad libraries
if 'posix' in os.name:
    # TODO: Test the result of os.name in different systems
    _so = os.popen('which freecad')
    FREECAD_PATH = _so.readline().replace('\n', '')
    if not FREECAD_PATH:
        _so.close()
        _so = os.popen('which FreeCAD')
        FREECAD_PATH = _so.readline().replace('\n', '')
        _so.close()

    FREECAD_PATH = FREECAD_PATH.replace('bin', 'lib')

    if not FREECAD_PATH:
        raise ModuleNotFoundError('FreeCAD not installed.')

elif 'nt' in os.name:
    # TODO: Find freecad libs on windows
    pass
else:
    raise Exception("Platform not supported")

# Extend sys.path to include freecad python libraries (including workbenches)
os.sys.path.extend(d.path for d in os.scandir(FREECAD_PATH))

from .exporter import export
from .model import *

