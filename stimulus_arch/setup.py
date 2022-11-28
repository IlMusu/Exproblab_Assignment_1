from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

modules = generate_distutils_setup(
    packages=['stimulus_arch'],
    package_dir={'': 'scripts'}
)
setup(**modules)
