from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bumperbot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', ['config/hardware.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Hardware interface for BumperBot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = bumperbot_hardware.motor_driver:main',
            'encoder_reader = bumperbot_hardware.encoder_reader:main',
            'pid = bumperbot_hardware.pid:main',
            'odometry = bumperbot_hardware.odometry:main',
            'water_actuator = bumperbot_hardware.water_actuator:main',
            'calibrate_encoders = bumperbot_hardware.calibrate_encoders:main',
            'diagnose_slam = bumperbot_hardware.diagnose_slam:main',
        ],
    },
)