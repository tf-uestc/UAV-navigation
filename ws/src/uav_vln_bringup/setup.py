from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['navigation_core','planning'],
    package_dir={'': 'scripts'}, py_modules=['base_detector','qwen_vl_grounding','test_backend']))
