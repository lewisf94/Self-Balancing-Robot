from setuptools import find_packages, setup

package_name = 'sbr_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lewis Fowler',
    maintainer_email='85638536+lewisf94@users.noreply.github.com',
    description='Hardware driver nodes (MPU6050 IMU, DC motors) for the self-balancing robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = sbr_drivers.imu_node:main',
            'motor_node = sbr_drivers.motor_node:main',
        ],
    },
)
