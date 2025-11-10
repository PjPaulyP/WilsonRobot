from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pandas',
        'scipy'
    ],
    zip_safe=True,
    maintainer='pjpoon',
    maintainer_email='paulpoon09@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gait_controller = robot_pkg.gait_controller_node:main',
            'mode_manager = robot_pkg.mode_node:main',
            'servo_bridge = robot_pkg.servo_bridge:main',
        ],
    },
)
