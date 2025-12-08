from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'livox_converter'

setup(
    name=package_name,
    version='0.0.1',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Launcher package for multiple ROS2 packages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    packages=find_packages(include=['livox_converter', 'livox_converter.*']),
    entry_points={
        'console_scripts': [
            'converter = livox_converter.converter_node:main',
        ]
    },
)