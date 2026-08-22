from glob import glob

from setuptools import find_packages, setup

package_name = 'so_arm_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/recordings', ['recordings/.gitkeep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adityakamath',
    maintainer_email='adityakamath@live.com',
    description=(
        'Top-level orchestration for SO-ARM100/101: composes so_arm_control\'s control and '
        'teleop stacks into single-arm and leader-follower dual-arm launches, and hosts '
        'record/replay and the wrist camera.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_replay_node = so_arm_bringup.record_replay_node:main',
            'opencv_camera_node = so_arm_bringup.opencv_camera_node:main',
        ],
    },
)
