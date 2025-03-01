from setuptools import find_packages, setup
import os 
from glob import glob  
package_name = 'little_helper_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='alfchr21@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = little_helper_commander.little_helper_commander:main',
            'item_tf_broadcaster = little_helper_commander.item_tf_broadcaster:main',
            'velocity_estimator = little_helper_commander.velocity_estimator:main',
            'speed_limiter = little_helper_commander.grasping_path_speed_limit:main'
        ],

    },
)
