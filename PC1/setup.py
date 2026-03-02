from setuptools import setup
import os
from glob import glob

package_name = 'pc3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 아래 라인들이 정확히 있어야 합니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'map'), glob('map/map_final.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='Integrated PC3 Robot Control with Depth and Navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc3_stage1_2_node = pc3.pc3_stage1_2_node:main',
            'pc3_stage3_4_node = pc3.pc3_stage3_4_node:main',
            'pc3_stage5_6_node = pc3.pc3_stage5_6_node:main',
        ],
    },
)
