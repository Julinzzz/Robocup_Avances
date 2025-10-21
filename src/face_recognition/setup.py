from setuptools import setup
import os
from glob import glob

package_name = 'face_recognition'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # index para ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # scripts de launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # modelos
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('face_recognition', 'models', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu.email@example.com',
    description='Reconocimiento facial con cámara USB y FaceNet en ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'face_recognition = face_recognition.face_recognition_node:main',
        ],
    },
)
