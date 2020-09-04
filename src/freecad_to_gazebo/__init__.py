import os, platform
import distro

FREECAD_PATH = ''

# check os types to search for freecad libraries
if 'linux' in platform.system().lower():
    dist = distro.linux_distribution(full_distribution_name=False)[0].lower()
    # TODO: check freecad libs on different distros
    if dist in ['ubuntu', 'debian', 'fedora', 'arch']:
        FREECAD_PATH = '/usr/lib/freecad'
    else:
        # fallback to default path
        FREECAD_PATH = '/usr/lib/freecad'
    print(dist, FREECAD_PATH)
elif 'nt' in platform.dist().lower():
    pass
    # TODO: Find freecad libs on windows
else:
    raise Exception("Platform not supported")

# Extend sys.path to include freecad python libraries (including workbenches)
os.sys.path.extend(os.path.join(FREECAD_PATH, d) for d in os.listdir(FREECAD_PATH))

from freecad_to_gazebo.mesh_exporter import export
from freecad_to_gazebo.model import *
from freecad_to_gazebo.freecad_exporter import *

