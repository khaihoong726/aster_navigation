import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'aster_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ushan Fernando',
    maintainer_email='ushanfernando123@gmail.com',
    description='Robot navigation package for ASTER',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aster_go_to_goal = aster_navigation.aster_go_to_goal:main',
        ],
    },
)
