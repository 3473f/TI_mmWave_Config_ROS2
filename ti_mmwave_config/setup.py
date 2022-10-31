from setuptools import setup
import os
from glob import glob

package_name = 'ti_mmwave_config'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Abdelaziz',
    maintainer_email='mohamed.abdelaziz@zal.aero',
    description='ROS2 package to configure and and control TI mmWave radar sensors',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ti_mmwave_config_node = ti_mmwave_config.ti_mmwave_config_node:main'
        ],
    },
)
