from setuptools import find_packages, setup

package_name = 'webcam_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishwas',
    maintainer_email='vishwas@todo.todo',
    description='A ROS 2 node that publishes webcam video.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This is the crucial line you need to add
            'publisher = webcam_publisher.publisher_node:main',
        ],
    },
)

