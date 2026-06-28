from setuptools import find_packages, setup

package_name = 'bumperbot_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Autonomous area coverage behaviors for BumperBot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spiral_coverage = bumperbot_coverage.spiral_coverage:main',
            'stadium_coverage = bumperbot_coverage.stadium_coverage:main',
            'wall_follow_coverage = bumperbot_coverage.wall_follow_coverage:main',
        ],
    },
)