import os
from glob import glob
from setuptools import setup

package_name = 'teleop_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Include launch files
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Teleoperation for UR5 in Gazebo',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = teleop_arm.keyboard_teleop:main',
        ],
    },
)
