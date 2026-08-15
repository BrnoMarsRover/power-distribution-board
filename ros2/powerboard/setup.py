import os
from glob import glob
from setuptools import setup

package_name = 'powerboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='standa',
    maintainer_email='standa.svediroh@gmail.com',
    description='ROS 2 driver for the BrnoMarsRover power distribution board',
    license='MIT',
    entry_points={
        'console_scripts': [
            'powerboard_node = powerboard.powerboard_node:main',
        ],
    },
)
