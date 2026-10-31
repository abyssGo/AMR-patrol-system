from setuptools import setup

package_name = 'patrol_amr_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/amr.launch.py']),
        ('share/' + package_name + '/config',
         ['config/site.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='조정묵',
    maintainer_email='kj270477@gmail.com',
    description='AMR mission, local safety and detection nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_node_v2 = patrol_amr_safety.vision_node_v2:main',
            'mission_supervisor = patrol_amr_safety.mission_supervisor:main',
            'local_safety_supervisor = '
            'patrol_amr_safety.local_safety_supervisor:main',
        ],
    },
)
