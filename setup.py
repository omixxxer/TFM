from setuptools import setup
import os
from glob import glob

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=['person_follower'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Añade esta línea para incluir todos los archivos de lanzamiento
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omixer',
    maintainer_email='al364109@uji.es',
    description='Package for person-following robot project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = person_follower.control_node.control_node:main',
            'camera_node = person_follower.camera_node.camera_node:main',
            'detection_node = person_follower.detection_node.detection_node:main',
            'tracking_node = person_follower.tracking_node.tracking_node:main',
            'collision_handling_node = person_follower.collision_handling_node.collision_handling_node:main',
            'user_interface_node = person_follower.user_interface_node.user_interface_node:main',
        ],
    },
)
