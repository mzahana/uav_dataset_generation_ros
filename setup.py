import os
from glob import glob
from setuptools import setup

package_name = 'uav_dataset_generation_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/mavros/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khaled Gabr',
    maintainer_email='khaledgabr77@gmail.com',
    description='ROS 2 package for automated generating of large UAV datasets',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execute_random_trajectories = uav_dataset_generation_ros.execute_random_trajectories_node:main',
        ],
    },
)
