import os

from setuptools import find_packages, setup

package_name = 'racerbot_car'

car_files = [
    (f'share/{package_name}/{dirpath}',
     [os.path.join(dirpath, filename) for filename in filenames])
    for dirpath, _, filenames in os.walk('cars') if filenames
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + car_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-1',
    maintainer_email='bryanmaubc@gmail.com',
    description="Which car this is (rb1/rb2), and where that car's profile and data live",
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
