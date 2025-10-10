from setuptools import find_packages, setup

package_name = 'ot2_ros2'


from setuptools import setup
import os
from glob import glob

package_name = 'ot2_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'custom_labware'), glob('ot2_ros2/custom_labware/*.json')),
        ('share/' + package_name + '/action', ['ot2_ros2/action/RunProtocol.action']),
        ('share/' + package_name + '/srv', ['ot2_ros2/srv/StopProtocol.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdl',
    maintainer_email='sdl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ot2_node = ot2_ros2.ot2_node:main'
        ],
    },
)
