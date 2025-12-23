from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'trial_1'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files (Python and shell scripts)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.sh')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include map files
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anbarasan',
    maintainer_email='anbarasan@todo.todo',
    description='TurtleBot3 Navigation with Teleop Recovery',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_recovery_node = trial_1.teleop_recovery_node:main',
            'teleop_keyboard_recovery = trial_1.teleop_keyboard_recovery:main',
            'trigger_teleop = trial_1.teleop_trigger:main',
        ],
    },
)
