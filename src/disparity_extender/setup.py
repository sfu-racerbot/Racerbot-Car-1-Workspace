from setuptools import find_packages, setup

package_name = 'disparity_trajectory'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/disparity_trajectory_launch.py']),
        ('share/' + package_name + '/config', ['config/disparity_trajectory.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SFU Racerbot',
    maintainer_email='respondtocyrus@gmail.com',
    description='Method D reactive driving: footprint-checked steering arcs, LB deadman',
    license='MIT',
    entry_points={
        'console_scripts': [
            'disparity_trajectory_node = disparity_trajectory.disparity_trajectory_node:main',
        ],
    },
)
