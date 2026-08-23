from glob import glob

from setuptools import find_packages, setup

package_name = 'so_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.srdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adityakamath',
    maintainer_email='adityakamath@live.com',
    description='Launch and controller configuration for SO-ARM100 (SO100 and SO101) robot arms',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_trajectory_bridge = so_arm_control.joint_trajectory_bridge_node:main',
            'bool_toggle_node = so_arm_control.bool_toggle_node:main',
            'teleop_ik_node = so_arm_control.teleop_ik_node:main',
            'target_visualizer_node = so_arm_control.target_visualizer_node:main',
            'joint_state_switch_node = so_arm_control.joint_state_switch_node:main',
            'teleop_support_node = so_arm_control.teleop_support_node:main',
            'generate_srdf = so_arm_control.scripts.generate_srdf:main',
        ],
    },
)
