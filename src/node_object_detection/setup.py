from setuptools import setup

package_name = 'node_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julian',
    maintainer_email='tu@email.com',
    description='Simulador de detección de objetos',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector = node_object_detection.detector:main',
        ],
    },
)
