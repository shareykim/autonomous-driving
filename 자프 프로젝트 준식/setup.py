from setuptools import setup
import os
from glob import glob

package_name = 'raceline_generator'

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
    maintainer='misys',
    maintainer_email='you@example.com',
    description='Raceline generation nodes',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'path_logger = raceline_generator.path_logger_node:main',
            'cubic_spline = raceline_generator.cubic_spline_node:main',
            'bspline = raceline_generator.bspline_node:main',
            'catmullrom = raceline_generator.catmullrom_node:main',
            'clothoid = raceline_generator.clothoid_node:main',
        ],
    },
)
