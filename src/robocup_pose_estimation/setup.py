from setuptools import setup
import os
from glob import glob

package_name = 'robocup_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Añade esta línea:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='tu_nombre',
    maintainer_email='tu@email.com',
    description='MediaPipe pose estimation for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_node = robocup_pose_estimation.pose_node:main',
        ],
    },
)
