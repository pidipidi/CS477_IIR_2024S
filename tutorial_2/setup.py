import os
from glob import glob
from setuptools import setup

package_name = 'tutorial_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('tutorial_2','cube.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keonyoung',
    maintainer_email='kkyjusikhoisa@kaist.ac.kr',
    description='Basic Tutorial of ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '1_printout = tutorial_2.1_printout:main',
            '2_posestamp = tutorial_2.2_posestamp:main',
            '3_posearray = tutorial_2.3_posearray:main',
            '4_gazebo_object = tutorial_2.4_gazebo_object:main',
            '5_camera_image_saver = tutorial_2.5_camera_image_saver:main'
        ],
    },
)
