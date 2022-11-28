from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

modules = generate_distutils_setup(
    packages=['robot_behavior'],
    package_dir={'': 'scripts'}
)
setup(**modules)
