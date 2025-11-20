from setuptools import setup
import os
from glob import glob

package_name = 'sdl_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdl',
    maintainer_email='sdl@example.com',
    description='Bringup package for full SDL system',
    license='Apache License 2.0',
    tests_require=['pytest'],
)
