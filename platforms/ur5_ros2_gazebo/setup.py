#!/usr/bin/env python3
import os
from glob import glob
from setuptools import setup
    
package_name = 'ur5_ros2_gazebo'

# def get_model_files():
#     model_files = []
#     for root, dirs, files in os.walk('models'):
#         for file in files:
#             model_files.append(os.path.join(root, file))
#     return model_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config','*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config','*.yaml'))),       
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds','*.world'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf','*.xacro'))),
        (os.path.join('share', package_name, 'sdf'), glob(os.path.join('sdf','*.sdf'))),
        # (os.path.join('share', package_name, 'models'), get_model_files()),     
        # *get_model_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daehyung Park',
    tests_require=['pytest'],
    entry_points={
    },
)


# #!/usr/bin/env python
# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# d = generate_distutils_setup()
# d['packages'] = ['manip_challenge']
# d['package_dir'] = {'': 'src'}

# setup(**d)

