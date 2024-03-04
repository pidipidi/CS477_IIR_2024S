from setuptools import setup

package_name = 'tutorial_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minu',
    maintainer_email='cmw9903@kaist.ac.kr',
    description='Basic Tutorial of ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = tutorial_1.talker:main',
            'listener = tutorial_1.listener:main',
            'service_server = tutorial_1.service_server:main',
            'service_client = tutorial_1.service_client:main',
            'ros_parameter = tutorial_1.ros_parameter:main'
        ],
    },
)
