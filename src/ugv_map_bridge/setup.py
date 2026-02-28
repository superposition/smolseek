from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_map_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'msgpack', 'websockets'],
    zip_safe=True,
    maintainer='Eric Manganaro',
    maintainer_email='eric@example.com',
    description='Bridge between stella_vslam and Babylon.js 3D viewer',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_bridge_node = ugv_map_bridge.map_bridge_node:main',
            'point_cloud_ws = ugv_map_bridge.point_cloud_ws:main',
            'nav2_goal_sender = ugv_map_bridge.nav2_goal_sender:main',
        ],
    },
)
