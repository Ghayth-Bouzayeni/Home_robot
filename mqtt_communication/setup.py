from setuptools import find_packages, setup

package_name = 'mqtt_communication'

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
    maintainer='ghayth',
    maintainer_email='ghayth@todo.todo',
    description='Package for MQTT communication with ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = mqtt_communication.pose_publisher:main',
            'command_subscriber = mqtt_communication.command_subscriber:main',
        ],
    },
)
