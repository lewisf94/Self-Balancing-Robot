import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sbr_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lewis Fowler',
    maintainer_email='85638536+lewisf94@users.noreply.github.com',
    description='Top-level launch files and parameters for the self-balancing robot.',
    license='MIT',
    entry_points={'console_scripts': []},
)
