#!/usr/bin/env python3
import os
from glob import glob
from setuptools import setup
from pathlib import Path
    
package_name = 'manip_challenge'


data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):

    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=package_files(data_files, ['data/models/', 'launch/', 'data/worlds/', 'config']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daehyung Park',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_object = manip_challenge.add_object:main',
            'init_joints = manip_challenge.init_joints:main',
            'world_model_gazebo = manip_challenge.world_model_gazebo:main',
            'get_joint    = manip_challenge.get_joint:main',            
            'get_pose     = manip_challenge.get_pose:main',            
            'move_gripper = manip_challenge.move_gripper:main',
            'move_joint   = manip_challenge.move_joint:main',            
            'example1   = manip_challenge.example1:main',            
            'example2   = manip_challenge.example2:main',
            'item_list_pub = manip_challenge.item_list_pub:main',
            'item_list_sub = manip_challenge.item_list_sub:main',
            ],
    },
)

