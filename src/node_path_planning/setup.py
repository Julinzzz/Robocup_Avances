from setuptools import setup

package_name = 'node_path_planning'

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
    description='Nodo de planificación de ruta',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner = node_path_planning.planner:main',
        ],
    },
)
