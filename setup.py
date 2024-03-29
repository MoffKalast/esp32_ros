from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['esp32_ros'],
    package_dir={'': 'nodes'},
    requires=['std_msgs', 'rospy']
)

setup(**setup_args)
