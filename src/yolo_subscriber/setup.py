import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yolo_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This is the new line that includes the model file
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishwas',
    maintainer_email='vishwas@todo.todo',
    description='A ROS 2 subscriber for YOLOv8 inference.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This is the new line that creates the executable
            'subscriber = yolo_subscriber.subscriber_node:main',
        ],
    },
)

