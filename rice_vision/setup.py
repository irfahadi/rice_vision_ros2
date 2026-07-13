import os
from glob import glob
from setuptools import setup

package_name = 'rice_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.pt') + glob('models/README*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raka',
    maintainer_email='inverse.aya@gmail.com',
    description='Integrasi computer vision (YOLOv8) dengan ROS 2 Foxy untuk '
                'deteksi objek real-time dari webcam/USB camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = rice_vision.camera_node:main',
            'detector_node = rice_vision.detector_node:main',
            'viewer_node = rice_vision.viewer_node:main',
        ],
    },
)
