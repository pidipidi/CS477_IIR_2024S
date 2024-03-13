#!/usr/bin/env python3
import os
from glob import glob
from setuptools import setup
    
package_name = 'manip_challenge'

# def get_model_files():
#     model_files = []
#     for root, dirs, files in os.walk('models'):
#         for file in files:
#             model_files.append(os.path.join(root, file))
#     return model_files

def get_model_files():
    model_files = []
    for root, dirs, files in os.walk('models'):
        for file in files:
            file_path = os.path.join(root, file)
            # 'models' 디렉토리 이후의 상대 경로 계산
            relative_path = os.path.relpath(os.path.dirname(file_path), 'models')
            model_files.append((os.path.join('share', package_name, 'models', relative_path), glob(file_path)))
            # print(package_name, 'models', relative_path, file_path)
    return model_files

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
        # (os.path.join('share', package_name, 'models'), get_model_files()),     
        *get_model_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daehyung Park',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_object = manip_challenge.add_object:main',
            'init_joints = manip_challenge.init_joints:main',
            ],
    },
)


# #!/usr/bin/env python
# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# d = generate_distutils_setup()
# d['packages'] = ['manip_challenge']
# d['package_dir'] = {'': 'src'}

# setup(**d)

