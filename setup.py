from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

args = generate_distutils_setup(
    packages=['freecad_to_gazebo'],
    package_dir={'': 'src'},
    scripts=['scripts/freecad2gazebo'],
)

setup(**args)

