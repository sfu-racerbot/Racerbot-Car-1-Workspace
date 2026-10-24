from setuptools import find_packages, setup

package_name = 'racerbot_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_launch.py']),
        ('share/' + package_name + '/config', ['config/imu_conditioner.yaml', 'config/imu_rb1.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-1',
    maintainer_email='bryanmaubc@gmail.com',
    description='IMU conditioning and the two-IMU cross-check',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_conditioner_node = racerbot_imu.imu_conditioner_node:main',
            'imu_select_node = racerbot_imu.imu_select_node:main',
            'calibrate_axes = racerbot_imu.calibrate_axes:main',
        ],
    },
)
