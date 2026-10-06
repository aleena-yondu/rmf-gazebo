import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fleet_adapter_r1'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Open-RMF fleet adapter for R1 bi-manual picking robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter = fleet_adapter_r1.fleet_adapter:main',
            'fake_robot = fleet_adapter_r1.fake_robot:main',
        ],
    },
)
