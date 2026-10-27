"""Setuptools configuration for the patrol_amr_bridge ROS 2 package."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'patrol_amr_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Control Team',
    maintainer_email='control@example.com',
    description='Bridge from Control Patrol interface to genius_patrol',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'amr_bridge_node = patrol_amr_bridge.amr_bridge_node:main',
        ],
    },
)
