from setuptools import setup

package_name = 'assignment_3'

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
    maintainer='daehyung',
    maintainer_email='daehyung@kaist.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'min_jerk = assignment_3.solution.min_jerk:main',
            'move_joint = assignment_3.solution.move_joint:main',
            'add_object = assignment_3.add_object:main',
            'move_astar = assignment_3.move_astar:main',
        ],
    },
)
