import os
from glob import glob
from setuptools import setup

package_name = 'drone_landing'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'quadrotor'),
            glob('models/quadrotor/*')),
        (os.path.join('share', package_name, 'models', 'landing_platform'),
            glob('models/landing_platform/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@university.edu',
    description='Autonomous drone landing on a moving platform',
    license='MIT',
    entry_points={
        'console_scripts': [
            'platform_mover = drone_landing.platform_mover:main',
            'state_estimator = drone_landing.state_estimator:main',
            'landing_controller = drone_landing.landing_controller:main',
        ],
    },
)
