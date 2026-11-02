from glob import glob
from setuptools import find_packages, setup

setup(
    name='opponent_perception', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/opponent_perception']),
        ('share/opponent_perception', ['package.xml']),
        ('share/opponent_perception/config', glob('config/*.yaml')),
        ('share/opponent_perception/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True, maintainer='SFU Racerbot', maintainer_email='contact@sfuracerbot.ca',
    description='Offline-first AprilTag opponent perception; no driving outputs.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'opponent_perception_node = opponent_perception.perception_node:main',
        'opponent_perception_offline = opponent_perception.offline:main',
    ]},
)
