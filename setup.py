from setuptools import setup
import os
from glob import glob

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('person_follower/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('person_follower/config/*.yaml')),  # Incluye la carpeta config
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
            'visual_detection_node = person_follower.visual_detection_node.visual_detection_node:main',
            'detection_node = person_follower.detection_node.detection_node:main',
            'tracking_node = person_follower.tracking_node.tracking_node:main',
            'user_interface_node = person_follower.user_interface_node.user_interface_node:main',
        ],
    },
)
