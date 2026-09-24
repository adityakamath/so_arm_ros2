from glob import glob

from setuptools import find_packages, setup

package_name = 'so_arm_mujoco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/mjcf', glob('mjcf/*.xacro') + glob('mjcf/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adityakamath',
    maintainer_email='adityakamath@live.com',
    description=(
        'Standalone MuJoCo simulation (no ROS required) for SO-ARM100 (SO100 and SO101), '
        'with optional Gymnasium single-arm and vectorized N-arm training envs'
    ),
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
        'gym': ['gymnasium'],
    },
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_preview = so_arm_mujoco.cli:preview_main',
            'build_mujoco_models = so_arm_mujoco.cli:build_main',
            'teleop_ik = so_arm_mujoco.teleop_ik:teleop_main',
            'benchmark_mujoco = so_arm_mujoco.benchmark_mujoco:main',
        ],
    },
)
