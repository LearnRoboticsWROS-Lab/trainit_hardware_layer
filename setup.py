from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'trainit_hardware_layer'

setup(
    name=package_name,
    version='0.2.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'contracts'), glob('contracts/*.md')),
        (os.path.join('share', package_name, 'profiles', 'robot'), glob('profiles/robot/*.yaml')),
        (os.path.join('share', package_name, 'profiles', 'eef'), glob('profiles/eef/*.yaml')),
        (os.path.join('share', package_name, 'profiles', 'camera'), glob('profiles/camera/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fra',
    maintainer_email='ros.master.ai@gmail.com',
    description='TrainIt hardware abstraction layer: contracts, profiles, adapters.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Adapters are console_scripts, never install(PROGRAMS) scripts: entry
            # points are generated executable, so they cannot hit the permission bug
            # that broke mode:=isaac on 2026-08-24.
            'mock_camera = trainit_hardware_layer.camera.mock_camera_node:main',
            # The canonical end-effector adapters (ADR-0022 D8): the rows trainit_mock and
            # trainit_isaac_surface_gripper.
            'mock_eef_adapter = trainit_hardware_layer.eef.mock_eef_adapter:main',
            'isaac_eef_adapter = trainit_hardware_layer.eef.isaac_eef_adapter:main',
        ],
    },
)
