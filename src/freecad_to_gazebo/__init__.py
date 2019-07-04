#!/usr/bin/env python3

import os
import platform

# check os types to search for freecad libraries
if 'linux' in platform.system().lower():
    if 'ubuntu' in platform.dist().lower():
        FREECAD_PATH = '/usr/lib/freecad'
    elif 'fedora' in platform.dist().lower():
        FREECAD_PATH = '/usr/lib/freecad'
elif 'nt' in platform.dist().lower():
    # TODO: Find freecad libs on windows
else:
    raise Exception("Platform not supported")

# Extend sys.path to include freecad python libraries (including workbenches)
os.sys.path.extend(d.path for d in os.scandir(FREECAD_PATH))

from freecad_to_gazebo.mesh_exporter import export
from freecad_to_gazebo.model import *
from freecad_to_gazebo.freecad_exporter import *

